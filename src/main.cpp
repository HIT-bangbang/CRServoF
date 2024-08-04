#include <Arduino.h>
#include <CrsfSerial.h>
#include <median.h>
#include "target.h"

#define NUM_OUTPUTS 8

// Configuration
// Map input CRSF channels (1-based, up to 16 for CRSF, 12 for ELRS) to outputs 1-8
// use a negative number to invert the signal (i.e. +100% becomes -100%)
constexpr int OUTPUT_MAP[NUM_OUTPUTS] = { 1, 2, 3, 4, 6, 7, 8, 12 };
/**   
 * The failsafe action for each channel (fsaNoPulses, fsaHold, or microseconds) 
 * 标记为 fsaHold 表示该通道在失控后保持失控前的值
 * 标记为 fsaNoPulses 表示该通道失控后不输出pwm
 * 其它值表示失控后默认输出的pwm脉宽
**/
constexpr int OUTPUT_FAILSAFE[NUM_OUTPUTS] = {
    1500, 1500, 988, 1500,                  // ch1-ch4
    fsaHold, fsaHold, fsaHold, fsaNoPulses  // ch5-ch8
    };
// Define the pins used to output servo PWM, must use hardware PWM,
// and change HardwareTimer targets below if the timers change
constexpr PinName OUTPUT_PINS[NUM_OUTPUTS] = { OUTPUT_PIN_MAP };

#define PWM_FREQ_HZ     50
#define VBAT_INTERVAL   500
#define VBAT_SMOOTH     5
// Scale used to calibrate or change to CRSF standard 0.1 scale
#define VBAT_SCALE      1.0

// Optimal safety and performance: Arm switch on AUX1 (channel 5)
// It is not recommended to change the channel
// See https://www.expresslrs.org/software/switch-config/
// Only used if USE_ARMSWITCH defined
#define ELRS_ARM_CHANNEL 5

// Local Variables
#if defined(ARDUINO_ARCH_STM32)
static HardwareSerial CrsfSerialStream(USART_INPUT);
#elif defined(TARGET_RASPBERRY_PI_PICO)
static UART CrsfSerialStream(DPIN_CRSF_TX, DPIN_CRSF_RX);
#endif
static CrsfSerial crsf(CrsfSerialStream);
static int g_OutputsUs[NUM_OUTPUTS];
#if defined(TARGET_RASPBERRY_PI_PICO)
#include <Servo.h>
static Servo *g_Servos[NUM_OUTPUTS];
#endif
static struct tagConnectionState {
    uint32_t lastVbatRead;
    MedianAvgFilter<unsigned int, VBAT_SMOOTH>vbatSmooth;
    unsigned int vbatValue;

    char serialInBuff[64];
    uint8_t serialInBuffLen;
    bool serialEcho;
} g_State;

/**
 * @brief: OOBData 通常指的是“Out-of-Band Data”，即带外数据。
 * 带外数据指的是不通过正常的CRSF通信通道传输的数据，而是通过另一条独立的通道来传输。
 * 例如，串口透传模式下的数据，也就是设备从串口收到了什么就再从另一个串口原封不动得发出去。
 * 飞控串口透传给接收机刷固件估计也是这么搞的。这里我们留着，必要的时候拿来做debug用
 * @param {uint8_t} b
 * @return {*}
 */
static void crsfOobData(uint8_t b)
{
    // A shifty byte is usually just log messages from ELRS
    Serial.write(b);
}

/**
 * @brief: Initialize a servo pin output for the first time
*/
static void servoPlatformBegin(unsigned int servo)
{
#if defined(ARDUINO_ARCH_STM32)
    // vv pinMode(p, OUTPUT) vv
    pin_function(OUTPUT_PINS[servo], STM_PIN_DATA(STM_MODE_OUTPUT_PP, GPIO_NOPULL, 0));
#endif
#if defined(TARGET_RASPBERRY_PI_PICO)
    // Pi Pico waits for the value before attaching the servo
    Servo *s = new Servo();
    s->attach(OUTPUT_PINS[servo], CRSF_ELIMIT_US_MIN, CRSF_ELIMIT_US_MAX);
    g_Servos[servo] = s;
#endif
}

/**
 * @brief: Set an already initialized servo to a usec value
*/
static void servoPlatformSet(unsigned int servo, int usec)
{
#if defined(ARDUINO_ARCH_STM32)
    pwm_start(OUTPUT_PINS[servo], PWM_FREQ_HZ, usec, MICROSEC_COMPARE_FORMAT);
#endif
#if defined(TARGET_RASPBERRY_PI_PICO)
    if (g_Servos[servo] == nullptr)
    {
        Servo *s = new Servo();
        s->attach(OUTPUT_PINS[servo], CRSF_ELIMIT_US_MIN, CRSF_ELIMIT_US_MAX, usec);
        g_Servos[servo] = s;
    }
    else
        g_Servos[servo]->writeMicroseconds(usec);
#endif
}

static void servoPlatformEnd(unsigned int servo)
{
#if defined(PLATFORM_STM32)
    pwm_stop(OUTPUT_PINS[servo]);
    // vv pinMode(p, INPUT_PULLDOWN) vv
    pin_function(OUTPUT_PINS[servo], STM_PIN_DATA(STM_MODE_INPUT, GPIO_PULLDOWN, 0));
#endif
#if defined(TARGET_RASPBERRY_PI_PICO)
     Servo *s = g_Servos[servo];
     g_Servos[servo] = nullptr;
     s->detach();
     delete s;
#endif
}

/**
 * @brief: 输出PWM值到舵机。
 * @param {unsigned int} servo
 * @param {int} usec 微秒，pwm 高电平时间 范围 ：1000us ~ 2000us
 * @return {*}
 */
static void servoSetUs(unsigned int servo, int usec)
{
    if (usec > 0)
    {
        // 0 means it was disabled previously, enable OUTPUT mode
        // 如果g_OutputsUs[servo]里面记录的数值是0，就说明上一次该引脚被 de-init(disabled)了。
        // 那么这次输出PWM之前就要重新 init 一下该引脚
        if (g_OutputsUs[servo] == 0)
            servoPlatformBegin(servo);
        servoPlatformSet(servo, usec);
    }
    else
    {
        // 如果 usec <= 0 就说明这个引脚用不到了，就 disabled 这个引脚。
        servoPlatformEnd(servo);
    }
    g_OutputsUs[servo] = usec;  // 记录上一次 servo_x 的pwm值到(g_OutputsUs[servo])
}


/**
 * @brief: 失控保护，按照 OUTPUT_FAILSAFE[] 数组内存储的模式 配置失控之后各个PWM引脚的输出
 * @return {*}
 */
static void outputFailsafeValues()
 {
    // 遍历8个要输出PWM的通道
    for (unsigned int out=0; out<NUM_OUTPUTS; ++out)
    {
        if (OUTPUT_FAILSAFE[out] == fsaNoPulses)
            servoSetUs(out, 0);
        else if (OUTPUT_FAILSAFE[out] != fsaHold)
            servoSetUs(out, OUTPUT_FAILSAFE[out]);
        // else fsaHold does nothing, keep the same value
    }
}


#if defined(USE_ARMSWITCH)
// If USE_ARMSWITCH flag is given during compilation, isArmed
// checks if the arm signal was sent on channel defined by CRSF_ELRS_ARM_CHANNEL.
// The arm signal has to be *over* 1500us
static bool isArmed()
{
    // Static variable to store arm count, initialized to 0
    static uint8_t armCount = 0;

    if (crsf.getChannel(ELRS_ARM_CHANNEL) <= 1500)
    {
        armCount = 0;
        return false;
    }
    // Require at least 4 packets with "arm" signal, in order to
    // prevent accidental arming due to corrupt signals, similar
    // to Betaflight
    if (armCount < 4)
    {
        armCount++;
        return false;
    }
    return true;
}
#endif

/**
 * @brief: 这个函数指针被赋值给 crsf.onPacketChannels 。用于处理收到的RC通道，这里是转换为PWM信号输出到引脚
 * @return {*}
 */
static void packetChannels()
{
    #if defined(USE_ARMSWITCH)
        if (!isArmed())
        {
            outputFailsafeValues();
            return;
        }
    #endif

    for (unsigned int out=0; out<NUM_OUTPUTS; ++out)
    {
        const int chInput = OUTPUT_MAP[out];    // 遍历 输出PWM的引脚和通道映射表 OUTPUT_MAP[out]
        int usOutput;
        if (chInput > 0)
            usOutput = crsf.getChannel(chInput);
        else
        {
            // 通道值反相。如果我们在引脚和通道映射表 OUTPUT_MAP[] 里面填入 `-2`。意思是 GPIO 2 输出关于1500对称的通道值，也就是3000-usOutput
            // if chInput is negative, invert the channel output
            usOutput = crsf.getChannel(-chInput);
            // (1500 - usOutput) + 1500
            usOutput = 3000 - usOutput;
        }
        servoSetUs(out, usOutput);
    }

    // for (unsigned int ch=1; ch<=4; ++ch)
    // {
    //     Serial.write(ch < 10 ? '0' + ch : 'A' + ch - 10);
    //     Serial.write('=');
    //     Serial.print(crsf.getChannel(ch), DEC);
    //     Serial.write(' ');
    // }
    // Serial.println();
}

static void packetLinkStatistics(crsfLinkStatistics_t *link)
{
  //Serial.print(link->uplink_RSSI_1, DEC);
  //Serial.println("dBm");
}

static void crsfLinkUp()
{
    digitalWrite(DPIN_LED, HIGH ^ LED_INVERTED);
}

/**
 * @brief: 数据链路异常（失控），触发失控保护
 * @return {*}
 */
static void crsfLinkDown()
{
    digitalWrite(DPIN_LED, LOW ^ LED_INVERTED);
    outputFailsafeValues();
 }

/**
 * @brief: 检查供电电压
 * @return {*}
 */
static void checkVbatt()
{
#if defined(APIN_VBAT)
    if (millis() - g_State.lastVbatRead < (VBAT_INTERVAL / VBAT_SMOOTH))
        return; // 不到检测电压的时候，直接退出
    g_State.lastVbatRead = millis();    // 上一次检查供电电压的时刻

    unsigned int idx = g_State.vbatSmooth.add(analogRead(APIN_VBAT));
    if (idx != 0)
        return;

    unsigned int adc = g_State.vbatSmooth;
    g_State.vbatValue = 330U * adc * (VBAT_R1 + VBAT_R2) / VBAT_R2 / ((1 << 12) - 1);

    crsf_sensor_battery_t crsfbatt = { 0 };
    uint16_t scaledVoltage = g_State.vbatValue * VBAT_SCALE;
    // Values are MSB first (BigEndian)
    crsfbatt.voltage = htobe16(scaledVoltage);
    crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_BATTERY_SENSOR, &crsfbatt, sizeof(crsfbatt));

    //Serial.print("ADC="); Serial.print(adc, DEC);
    //Serial.print(" "); Serial.print(g_State.vbatValue, DEC); Serial.println("V");
#endif // APIN_VBAT
}

static void passthroughBegin(uint32_t baud)
{
    if (baud != crsf.getBaud())
    {
        // Force a reboot command since we want to send the reboot
        // at 420000 then switch to what the user wanted
        const uint8_t rebootpayload[] = { 'b', 'l' };
        crsf.queuePacket(CRSF_ADDRESS_CRSF_RECEIVER, CRSF_FRAMETYPE_COMMAND, &rebootpayload, sizeof(rebootpayload));
    }

    crsf.setPassthroughMode(true, baud);
    g_State.serialEcho = false;
}

/***
 * @brief Processes a text command like we're some sort of CLI or something
 * @return true if CrsfSerial was placed into passthrough mode, false otherwise
*/
static bool handleSerialCommand(char *cmd)
{
    // Fake a CRSF RX on UART6
    bool prompt = true;
    if (strcmp(cmd, "#") == 0)
    {
        Serial.println("Fake CLI Mode, type 'exit' or 'help' to do nothing\r\n");
        g_State.serialEcho = true;
    }

    else if (strcmp(cmd, "serial") == 0)
        Serial.println("serial 5 64 0 0 0 0\r\n");

    else if (strcmp(cmd, "get serialrx_provider") == 0)
        Serial.println("serialrx_provider = CRSF\r\n");

    else if (strcmp(cmd, "get serialrx_inverted") == 0)
        Serial.println("serialrx_inverted = OFF\r\n");

    else if (strcmp(cmd, "get serialrx_halfduplex") == 0)
        Serial.println("serialrx_halfduplex = OFF\r\n");

    else if (strncmp(cmd, "serialpassthrough 5 ", 20) == 0)
    {
        // Just echo the command back, BF and iNav both send
        // difference blocks of text as they start passthrough
        Serial.println(cmd);

        unsigned int baud = atoi(&cmd[20]);
        passthroughBegin(baud);

        return true;
    }

    else
        prompt = false;

    if (prompt)
        Serial.print("# ");

    return false;
}

/**
 * @brief: 串口透传模式下，处理串口输入的数据。从串口in接收，直接从串口crsf发出去
 * @return {*}
 */
static void checkSerialInPassthrough()
{
    static uint32_t lastData = 0;
    static bool LED = false;
    bool gotData = false;

    // Simple data passthrough from in to crsf
    unsigned int avail;
    while ((avail = Serial.available()) != 0)
    {
        uint8_t buf[16];
        avail = Serial.readBytes((char *)buf, min(sizeof(buf), avail));
        crsf.write(buf, avail);
        digitalWrite(DPIN_LED, LED);
        LED = !LED;
        gotData = true;
    }

    //如果太长时间都没收到串口in的数据了，就先关闭串口透传模式，等待下一次 串口in 的指令重新开启串口透传吧
    // If longer than X seconds since last data, switch out of passthrough
    if (gotData || !lastData)
        lastData = millis();
    // Turn off LED 1s after last data
    else if (LED && (millis() - lastData > 1000))
    {
        LED = false;
        digitalWrite(DPIN_LED, LOW ^ LED_INVERTED);
    }
    // Short blink LED after timeout
    else if (millis() - lastData > 5000)
    {
        lastData = 0;
        digitalWrite(DPIN_LED, HIGH ^ LED_INVERTED);
        delay(200);
        digitalWrite(DPIN_LED, LOW ^ LED_INVERTED);
        crsf.setPassthroughMode(false);
    }
}

static void checkSerialInNormal()
{
    while (Serial.available())
    {
        char c = Serial.read();     //  串口读字节
        if (g_State.serialEcho && c != '\n')
            Serial.write(c);    // 发回读到的字符

        if (c == '\r' || c == '\n')     //收到了一次发送的结束符
        {
            if (g_State.serialInBuffLen != 0)
            {
                Serial.write('\n'); // 发回去一个换行符，让串口助手换行

                g_State.serialInBuff[g_State.serialInBuffLen] = '\0'; // 最后面要拼一个\0 因为c语言中字符串最后是\0字符
                g_State.serialInBuffLen = 0;

                bool goToPassthrough = handleSerialCommand(g_State.serialInBuff);   // 看一下指令是不是开启了串口透传模式(Passthrough mode)
                // If need to go to passthrough, get outta here before we dequeue any bytes
                if (goToPassthrough)    //如果串口发来的指令让 我们的设备进入了串口透传模式，直接返回
                    return;
            }
        }
        else
        {
            g_State.serialInBuff[g_State.serialInBuffLen++] = c;    // 收到的字符存到buffer里面
            // if the buffer fills without getting a newline, just reset
            if (g_State.serialInBuffLen >= sizeof(g_State.serialInBuff))
                g_State.serialInBuffLen = 0;
        }
    }  /* while Serial */
}

/**
 * @brief: 检查是否有串口输入，注意这个串口不是CRSF的那个串口，是另一个用户串口。用于用户的指令输入或者串口透传
 * @return {*}
 */
static void checkSerialIn()
{
    if (crsf.getPassthroughMode())      // 如果开启了串口透传模式
        checkSerialInPassthrough();
    else
        checkSerialInNormal();          // 检查串口输入
}

/**
 * @brief: 1、Crsf通信初始化，包括向类中的函数指针传入对应的函数（用户可以灵活得自定义这些行为）2、开启通信
 * @return {*}
 */
static void setupCrsf()
{
    crsf.onLinkUp = &crsfLinkUp;        // 向crsf类传入通讯链路正常时的行为
    crsf.onLinkDown = &crsfLinkDown;    // 通讯链路异常时的行为
    crsf.onOobData = &crsfOobData;      // 如何处理带外数据的行为
    crsf.onPacketChannels = &packetChannels;    // 处理收到的通道数据的行为。
    crsf.onPacketLinkStatistics = &packetLinkStatistics;
    crsf.begin();       // 开启串口，通信正式开始
}

static void setupGpio()
{
    pinMode(DPIN_LED, OUTPUT);  //LED 
    digitalWrite(DPIN_LED, LOW ^ LED_INVERTED); // 上电时LED先拉低，数据通讯链路正常建立时 再调用crsfLinkUp()拉高LED
    analogReadResolution(12);
    
    // 在这里我们并没有初始化 所有的OUTPUT_MAP[NUM_OUTPUTS]中记录的与servo outputs相关的引脚，
    // 而是在后续过程中，当接收到了第一个有效数据包时，再将用到的引脚进行对应的初始化。
    // The servo outputs are initialized when the
    // first channels packet comes in and sets the PWM
    // output value, to prevent them from jerking around
    // on startup
}

void setup()
{
    Serial.begin(115200);

    setupGpio();
    setupCrsf();
}

void loop()
{
    crsf.loop();
    checkVbatt();
    checkSerialIn();    // 检查是否有串口输入，注意是另一个的串口，不是连接接收机CRSF的那个串口
}

/**
 * 串口透传模式 PassthroughMode
 * 假设我们现在有两个串口，serial_A serial_B B连接CRSF设备例如接收机，A连接到了电脑
 * 串口透传就是 单片机从A接收到的数据原封不动地再从B串口发出去，从B串口接收到的数据再从A串口发出去
 * 作者在这里其实是想通过串口透传，把stm32f103的crsf-pwm转接板当成一个USB转ttl模块，直接插把转接板插usb到电脑上就可以给elrs接收机刷固件了
 */

