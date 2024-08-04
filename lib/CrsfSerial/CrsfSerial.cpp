#include "CrsfSerial.h"

// static void hexdump(void *p, size_t len)
// {
//     char *data = (char *)p;
//     while (len > 0)
//     {
//         uint8_t linepos = 0;
//         char* linestart = data;
//         // Binary part
//         while (len > 0 && linepos < 16)
//         {
//             if (*data < 0x0f)
//             Serial.write('0');
//             Serial.print(*data, HEX);
//             Serial.write(' ');
//             ++data;
//             ++linepos;
//             --len;
//         }

//         // Spacer to align last line
//         for (uint8_t i = linepos; i < 16; ++i)
//             Serial.print("   ");

//         // ASCII part
//         for (uint8_t i = 0; i < linepos; ++i)
//             Serial.write((linestart[i] < ' ') ? '.' : linestart[i]);
//         Serial.println();
//     }
// }

CrsfSerial::CrsfSerial(HardwareSerial &port, uint32_t baud) :
    _port(port), _crc(0xd5), _baud(baud),
    _lastReceive(0), _lastChannelsPacket(0), _linkIsUp(false),
    _passthroughBaud(0)
{}

void CrsfSerial::begin(uint32_t baud)
{
    if (baud != 0)
        _port.begin(baud);
    else
        _port.begin(_baud);
}

// Call from main loop to update
void CrsfSerial::loop()
{
    handleSerialIn();
}

void CrsfSerial::handleSerialIn()
{
    while (_port.available())
    {
        uint8_t b = _port.read();   // 串口读1byte数据
        _lastReceive = millis();    // 记录上一个byte读到数据的时间

        // 如果开启了串口透传模式，且onOobData()函数被用户定义了，那么就进到onOobData()函数中。
        // onOobData()函数 一般就是用来串口透传，把接收到的数据 b 从另一个串口原封不动地发出去
        if (getPassthroughMode())
        {
            if (onOobData)
                onOobData(b);
            continue;
        }

        // 如果没启用串口透传模式，就正常按照CRSF协议处理接收到的数据
        _rxBuf[_rxBufPos++] = b;    // 把接收到的字节存到 _rxBuf 里面，之后 _rxBufPos 自增
        handleByteReceived();       // 处理_rxBuf里面的所有数据

        if (_rxBufPos == (sizeof(_rxBuf)/sizeof(_rxBuf[0])))
        {
            // Packet buffer filled and no valid packet found, dump the whole thing
            _rxBufPos = 0;
        }
    }

    checkPacketTimeout();   // 检查串口接收字节是否超时
    checkLinkDown();        // 检查通信链路是否正常（主要是检查遥控帧是否超时）
}

/**
 * @brief: 处理 _rxBuf[] 里面的数据，关键是从数据流里面找出来帧头，这样才能找出来完整的一帧.
 * 主要通过两种方式找出来第一个完整的一帧：1.帧长字节符合要求 2.CRC校验通过
 * 只要找出来了第一帧，之后的帧就快了，因为主机肯定是一帧一帧挨着发的。
 * @return {*}
 */
void CrsfSerial::handleByteReceived()
{
    bool reprocess;
    do
    {
        reprocess = false;
        if (_rxBufPos > 1)  // _rxBuf 中有多于2个的数据，因为 `帧长`字节 存到了一帧的第二个byte
        {
            uint8_t len = _rxBuf[1];    // 我们假定现在的_rxBuf[0]是帧头，那么 _rxBuf[1] 就是帧长
            // Sanity check the declared length isn't outside Type + X{1,CRSF_MAX_PAYLOAD_LEN} + CRC
            // assumes there never will be a CRSF message that just has a type and no data (X)
            if (len < 3 || len > (CRSF_MAX_PAYLOAD_LEN + 2))
            {
                // 如果现在“可能对的一帧” 它的帧长超出了CRSF协议中帧长的范围，那么就说明现在的_rxBuf[0]肯定不是帧头，我们上面的假设错了
                shiftRxBuffer(1);   //把 _rxBuf[] 右移一个元素（原来的 _rxBuf[0]就被扔掉了）
                reprocess = true;
            }

            // 现在的_rxBuf[1]（也就是假定的帧长len）在crsf协议的大小范围内的话，
            // 那么，我们很有可能找到正确的帧头了，即以_rxBuf[0]为开始的len + 2个数据很有可能是完整的连续的一帧
            // 所以就向后接收连续的 len + 2 个数据，收集完这一帧。虽然它有可能是错的。
            else if (_rxBufPos >= (len + 2))
            {
                // 我们只是通过len在范围内就认为这len + 2个字节有可能是一帧，这很可能不对。
                // 但是如果它真的是一帧的话，最后一字节(_rxBuf[2 + len - 1])应该就是CRC校验码
                // 如果它真的是一帧，那它必须是能通过CRC校验的
                uint8_t inCrc = _rxBuf[2 + len - 1];
                uint8_t crc = _crc.calc(&_rxBuf[2], len - 1);   // 执行CRC校验
                if (crc == inCrc)
                {
                    // 通过了CRC校验，以_rxBuf[0]为帧头的这（len + 2）个字节确实是一帧没错
                    processPacketIn(len);   // 处理这一帧数据
                    shiftRxBuffer(len + 2); // 这一帧数据处理完了，已经没用了，把这一帧移出 buffer
                    reprocess = true;       // buffer里可能还有剩的数据，所以不能写false。(分析一下？有可能是上一次CRC不通过，而且上一次假定的帧长比这次真实的一帧还长，导致Buffer里面积累了比这次帧还长的数据)
                }
                else
                {
                    // 如果CRC校验不通过，说明我们找的帧头找错了（也有可能是传输过程中误码了）
                    shiftRxBuffer(1);   // 左移一位。并不能清空，我们可能只是帧头找错了而已，其他数据还是有用的
                    reprocess = true;   // 回到循环里面，从buffer里剩下的数据重找帧头就是了
                }
            }  // if complete packet  如果 buffer 里数据的长度不够 len + 2 说明我们还没收集齐这一帧的数据，那就返回到 handleSerialIn()  的 while 里面继续接收数据吧
        } // if pos > 1 如果_rxBuf 中没有多于2个数据，那就返回到 handleSerialIn()  的 while 里面继续接收数据吧
    } while (reprocess);
}

/**
 * @brief: 检查串口发送数据包是否超时，如果距离接收到上一个字节的时间太久了，说明数据超时了，清空 _rxBufPos
 * @return {*}
 */
void CrsfSerial::checkPacketTimeout()
{
    // If we haven't received data in a long time, flush the buffer a byte at a time (to trigger shiftyByte)
    if (_rxBufPos > 0 && millis() - _lastReceive > CRSF_PACKET_TIMEOUT_MS)
        while (_rxBufPos)
            shiftRxBuffer(1);
}

/**
 * @brief: 只检查RC遥控帧是否超时，为了保证遥控的安全。
 * @return {*}
 */
void CrsfSerial::checkLinkDown()
{
    //如果距离上一次得到RC数据的时间太久了，就认为链路异常，触发onLinkDown()，并将 链路正常标志位 置false
    if (_linkIsUp && millis() - _lastChannelsPacket > CRSF_FAILSAFE_STAGE1_MS)
    {
        if (onLinkDown)
            onLinkDown();
        _linkIsUp = false;
    }
}

/**
 * @brief: 处理一帧数据，
 * @param {uint8_t} len: <Type> + <Payload> + <CRC>的长度
 * @return {*}
 */
void CrsfSerial::processPacketIn(uint8_t len)
{
    const crsf_header_t *hdr = (crsf_header_t *)_rxBuf;     // 强制类型转换，使用位域操作，直接更改那一块内存的数据解释方式
    if (hdr->device_addr == CRSF_ADDRESS_FLIGHT_CONTROLLER)
    {
        switch (hdr->type)
        {
        case CRSF_FRAMETYPE_GPS:    // 如果收到的CRSF帧类型是GPS数据
            packetGps(hdr);
            break;
        case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:     // 如果收到的是遥控信号
            packetChannelsPacked(hdr);
            break;
        case CRSF_FRAMETYPE_LINK_STATISTICS:        // 如果收到的是连接质量
            packetLinkStatistics(hdr);
            break;
        }
    } // CRSF_ADDRESS_FLIGHT_CONTROLLER
}

// Shift the bytes in the RxBuf down by cnt bytes
/**
 * @brief: 将RxBuffer整体右移 cnt 个元素。
 * @param {uint8_t} cnt
 * @return {*}
 */
void CrsfSerial::shiftRxBuffer(uint8_t cnt)
{
    // If removing the whole thing, just set pos to 0
    // 如果右移的个数比现在 Buffer 暂存的数据还多或者相等
    // 干脆就直接把标记数据存到哪里的变量 `_rxBufPos` 清零就好咯。意思就等于是整个Buffer里面都是空的。
    if (cnt >= _rxBufPos)
    {
        _rxBufPos = 0;
        return;
    }

    // 如果是右移一个元素，并且我们还定义了onOobData()函数，那么就把这个 数据 作为带外数据处理（这里是从另一个串口发出去）
    if (cnt == 1 && onOobData)
        onOobData(_rxBuf[0]);

    // Otherwise do the slow shift down
    // 右移 cnt 个元素
    uint8_t *src = &_rxBuf[cnt];
    uint8_t *dst = &_rxBuf[0];
    _rxBufPos -= cnt;
    uint8_t left = _rxBufPos;
    while (left--)
        *dst++ = *src++;
}

/**
 * @brief: 解包16个chennel的遥控数据。仍然是使用结构体位域操作，直接改变一块内存的数据解释方式。
 * @param {crsf_header_t} *p
 * @return {*}
 */
void CrsfSerial::packetChannelsPacked(const crsf_header_t *p)
{
    // 强制类型转换，使用结构体位域操作，直接改变 p->data 这块内存的数据解释方式，按照 crsf_channels_t 的方式来划分这块内存。
    crsf_channels_t *ch = (crsf_channels_t *)&p->data;
    _channels[0] = ch->ch0;
    _channels[1] = ch->ch1;
    _channels[2] = ch->ch2;
    _channels[3] = ch->ch3;
    _channels[4] = ch->ch4;
    _channels[5] = ch->ch5;
    _channels[6] = ch->ch6;
    _channels[7] = ch->ch7;
    _channels[8] = ch->ch8;
    _channels[9] = ch->ch9;
    _channels[10] = ch->ch10;
    _channels[11] = ch->ch11;
    _channels[12] = ch->ch12;
    _channels[13] = ch->ch13;
    _channels[14] = ch->ch14;
    _channels[15] = ch->ch15;

    // 将每个 channel 的原始数据 0 ~ 2^11 映射到 1000us 到 2000us之间
    for (unsigned int i=0; i<CRSF_NUM_CHANNELS; ++i)
        _channels[i] = map(_channels[i], CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000, 1000, 2000);

    // 如果我们定义了onLinkUp()函数，并且数据链路状态指示位为false，那就执行onLinkUp()
    if (!_linkIsUp && onLinkUp)
        onLinkUp();
    _linkIsUp = true;   //数据链路状态指示位 置为true 表示通讯正常建立(或者恢复正常)
    _lastChannelsPacket = millis();     // 记录上一次解包遥控数据的时间

    // 如果我们定义了onPacketChannels()函数，那就执行onPacketChannels()。
    if (onPacketChannels)
        onPacketChannels();
}

void CrsfSerial::packetLinkStatistics(const crsf_header_t *p)
{
    const crsfLinkStatistics_t *link = (crsfLinkStatistics_t *)p->data;
    memcpy(&_linkStatistics, link, sizeof(_linkStatistics));

    if (onPacketLinkStatistics)
        onPacketLinkStatistics(&_linkStatistics);
}

void CrsfSerial::packetGps(const crsf_header_t *p)
{
    const crsf_sensor_gps_t *gps = (crsf_sensor_gps_t *)p->data;
    _gpsSensor.latitude = be32toh(gps->latitude);
    _gpsSensor.longitude = be32toh(gps->longitude);
    _gpsSensor.groundspeed = be16toh(gps->groundspeed);
    _gpsSensor.heading = be16toh(gps->heading);
    _gpsSensor.altitude = be16toh(gps->altitude);
    _gpsSensor.satellites = gps->satellites;

    if (onPacketGps)
        onPacketGps(&_gpsSensor);
}

void CrsfSerial::write(uint8_t b)
{
    _port.write(b);
}

void CrsfSerial::write(const uint8_t *buf, size_t len)
{
    _port.write(buf, len);
}

void CrsfSerial::queuePacket(uint8_t addr, uint8_t type, const void *payload, uint8_t len)
{
    if (getPassthroughMode())
        return;
    if (len > CRSF_MAX_PAYLOAD_LEN)
        return;

    uint8_t buf[CRSF_MAX_PACKET_SIZE];
    buf[0] = addr;
    buf[1] = len + 2; // type + payload + crc
    buf[2] = type;
    memcpy(&buf[3], payload, len);
    buf[len+3] = _crc.calc(&buf[2], len + 1);

    write(buf, len + 4);
}

/**
 * @brief   Enter passthrough mode (serial sent directly to shiftybyte),
 *          optionally changing the baud rate used during passthrough mode
 * @param val
 *          True to start passthrough mode, false to resume processing CRSF
 * @param passthroughBaud
 *          New baud rate for passthrough mode, or 0 to not change baud
 *          Not used if disabling passthough
*/
void CrsfSerial::setPassthroughMode(bool val, uint32_t passthroughBaud)
{
    if (val)
    {
        // If not requesting any baud change
        if (passthroughBaud == 0)
        {
            // Enter passthrough mode if not yet
            if (_passthroughBaud == 0)
                _passthroughBaud = _baud;
            return;
        }

        _passthroughBaud = passthroughBaud;
    }
    else
    {
        // Not in passthrough, can't leave it any harder than we already are
        if (_passthroughBaud == 0)
            return;

        // Leaving passthrough, but going back to same baud, just disable
        if (_passthroughBaud == _baud)
        {
            _passthroughBaud = 0;
            return;
        }

        _passthroughBaud = 0;
    }

    // Can only get here if baud is changing, close and reopen the port
    _port.end(); // assumes flush()
    begin(_passthroughBaud);
}
