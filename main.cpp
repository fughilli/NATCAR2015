/*
 * main.c
 */
#include <main.h>

#include <stdbool.h>
#include <stdint.h>

#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/rom_map.h"
#include "driverlib/rom.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "inc/hw_adc.h"
#include "inc/hw_gpio.h"
#include "inc/hw_pwm.h"
#include "inc/hw_uart.h"
#include "inc/hw_types.h"
#include "driverlib/pin_map.h"
#include "inc/hw_memmap.h"

#include "fast_utils.h"
#include <stdlib.h>
#include <string.h>

#include "drivers/camera/driver_camera.h"
#include "drivers/driver_accel.h"
#include "drivers/driver_batmon.h"
#include "drivers/driver_bt.h"
#include "drivers/driver_pin.h"
#include "drivers/driver_serial.h"
#include "drivers/driver_system.h"
#include "drivers/motor/driver_motor.h"
#include "programs/shell/prog_getset.h"
#include "programs/shell/shell.h"

#include "programs/kill.h"
#include "programs/drive.h"
#include "programs/batmon.h"
#include "programs/reboot.h"

#include "registers/registers_gpio.h"
#include "settings/defaults.h"
#include "utils/base64enc.hpp"
#include "utils/bezier.h"
#include "utils/circular_buffer.hpp"
#include "utils/PID/PIDControl.h"
#include "utils/streammux.h"
#include "utils/streamutils.h"
#include "utils/sym_utils.h"

#include "utils/camera_utils.hpp"

#include <math.h>

#include "global_settings.h"


struct ShellStreamReceiverAdapter : StreamMux::MuxStreamReader<0>
{
    static CircularBuffer<uint8_t, 64> rxbuffer;

    static void rxbegincallback()
    {

    }

    static void rxendcallback()
    {

    }

    static void rxcallback(uint8_t data)
    {
        rxbuffer.push(data);
    }

    static int get()
    {
        if(!rxbuffer.empty())
        {
            uint8_t front = rxbuffer.front();
            rxbuffer.pop();
            return front;
        }

        return 0;
    }

    static bool avail()
    {
        return !rxbuffer.empty();
    }
};

CircularBuffer<uint8_t, 64> ShellStreamReceiverAdapter::rxbuffer;

template<class UWStreamAdapter>
struct ShellStreamSenderAdapter
{
    static void putc(char c)
    {
        UWStreamAdapter::begin();
        UWStreamAdapter::write(static_cast<uint8_t>(c));
        UWStreamAdapter::end();
    }

    static void puts(const char* s)
    {
        UWStreamAdapter::begin();
        while(*s)
        {
            UWStreamAdapter::write(static_cast<uint8_t>(*s++));
        }
        UWStreamAdapter::end();
    }

    static void writebuf(const uint8_t* buf, uint32_t len)
    {
        UWStreamAdapter::begin();
        while(len--)
        {
            UWStreamAdapter::write(*buf++);
        }
        UWStreamAdapter::end();
    }

    static void flush()
    {

    }
};

struct SerialStreamAdapter
{
    static void write(char c)
    {
        Serial_putc(SERIAL_MODULE, c);
        Serial_flush(SERIAL_MODULE);
    }
};

typedef CameraUtils::LinecamProcessor<camera_sample_t, 128> LineProcessor;

static accel_t accelData;

typedef Base64::Base64StreamWriter<SerialStreamAdapter,
                                   Base64::Base64LookupMapper> B64SerialWriter;
typedef Base64::Base64StreamReader<
            StreamMux::StreamReaderMultiplexer<ShellStreamReceiverAdapter>,
            Base64::Base64LookupMapper> B64SerialReader;

typedef Pin::DigitalPin<SYSCTL_PERIPH_GPIOF, GPIO_PORTF_BASE, 0> SW1;
typedef Pin::DigitalPin<SYSCTL_PERIPH_GPIOF, GPIO_PORTF_BASE, 4> SW2;

struct centers_t
{
    camera_sample_t lcenter, rcenter;
};

static centers_t centers;

typedef StreamMux::MuxStreamWriter<B64SerialWriter, 0> ShellStream;
typedef StreamMux::MuxStreamWriter<B64SerialWriter, 1> CameraStream0;
typedef StreamMux::MuxStreamWriter<B64SerialWriter, 2> CameraStream1;
typedef StreamMux::MuxStreamWriter<B64SerialWriter, 3> AccelStream;
typedef StreamMux::MuxStreamWriter<B64SerialWriter, 4> CenterStream;
typedef StreamMux::MuxStreamWriter<B64SerialWriter, 5> DerivativeStream;

typedef StreamUtils::StructSender<CameraStream0> Camera0DataSender;
typedef StreamUtils::StructSender<CameraStream1> Camera1DataSender;
typedef StreamUtils::StaticStructSender<AccelStream, accel_t, &accelData, 6>
        AccelDataSender;
typedef StreamUtils::StaticStructSender<CenterStream, centers_t, &centers, 4>
        CentersSender;
typedef StreamUtils::StaticStructSender<DerivativeStream, camera_sample_t, LineProcessor::derivative_buffer, 128-1>
        DerivativeSender;

typedef ShellStreamSenderAdapter<ShellStream> ShellStreamSender;

typedef Pin::DigitalPin<SYSCTL_PERIPH_GPIOF, GPIO_PORTF_BASE, 1> RedPin;
typedef Pin::DigitalPin<SYSCTL_PERIPH_GPIOF, GPIO_PORTF_BASE, 3> GreenPin;
typedef Pin::DigitalPin<SYSCTL_PERIPH_GPIOF, GPIO_PORTF_BASE, 2> BluePin;

static int s_getc()
{
    return Serial_getc(SERIAL_MODULE);
}

static void s_putc(char c)
{
    Serial_putc(SERIAL_MODULE, c);
}

static bool s_avail()
{
    return Serial_avail(SERIAL_MODULE);
}

static void s_puts(const char* str)
{
    return Serial_puts(SERIAL_MODULE, str);
}

static void s_writebuf(const uint8_t* buf, uint32_t len)
{
    Serial_writebuf(SERIAL_MODULE, buf, len);
}

static void s_flush()
{
    Serial_flush(SERIAL_MODULE);
}

const shell_stream_adapter_t shell_stream_adapter =
{
    .getc = ShellStreamReceiverAdapter::get,
    .putc = ShellStreamSender::putc,
    .puts = ShellStreamSender::puts,
    .writebuf = ShellStreamSender::writebuf,
    .flush = ShellStreamSender::flush,
    .avail = ShellStreamReceiverAdapter::avail
};

float speed;

// Coefficients
float threshold_ratio;
bool speed_control;
float speed_multiplier;
bool killswitch;
static PIDController_t servopid;
uint32_t edgeclip_count;
float min_speed;

float turn_threshold;
float left_setpoint, right_setpoint;
float setpoint_decay;

float speedboost_decay;

bool red_led, green_led, blue_led;

bool be_still;


strategy_t strategy = SAFE;
float bezier_strat_lookahead = 0.7;

inline uint32_t map_val(uint32_t val, uint32_t inmin, uint32_t inmax,
        uint32_t outmin, uint32_t outmax)
{
    uint32_t retval = ((val - inmin) * (outmax - outmin));
    uint32_t divisor = (inmax - inmin);
    if (divisor != 0)
        return retval / divisor;
    return 0;
}


speedcont_policy_t speedcont_policy;

camera_buffer_index_t buffer_switch;

set_entry_t set_entries[MAX_SET_ENTRIES];




void shell_keepalive()
{
    static int i = 0;

    i++;

    if(i >= 100)
    {
        shell_stream_adapter.putc(0);
        i = 0;
    }
}

camera_sample_t derivative_buffer[128];

#define THRESHOLD_RATIO (0.65f)

//#define DEBUG_B64_OUT
//#define REMOTE_CONTROL
//#define DEBUG_LINE_TO_CONSOLE
//
// Speed scaling for PID coefficients
bool ss_pid;
float ss_p_coeff, ss_i_coeff, ss_d_coeff;
float base_p_coeff, base_i_coeff, base_d_coeff;

int main(void)
{
    speed_control = DEFAULT_SPEED_CONT;
    speed_multiplier = DEFAULT_SPEED_MULT;
    threshold_ratio = DEFAULT_THRESHOLD;
    PID_PIDController(&servopid, DEFAULT_STEER_P, DEFAULT_STEER_I, DEFAULT_STEER_D, 2.0f);
    edgeclip_count = DEFAULT_EDGECLIP_COUNT;
    be_still = DEFAULT_BE_STILL;
    min_speed = DEFAULT_MIN_SPEED;
    killswitch = false;
    red_led = false;
    green_led = false;
    blue_led = false;
    speed = 0.0f;
    buffer_switch = CAMERA_BUFFER_A;
    speedcont_policy = DEFAULT_SPEEDCONT_POLICY;
    strategy = DEFAULT_STRATEGY;

    setpoint_decay = DEFAULT_SETPOINT_DECAY;
    left_setpoint = DEFAULT_LEFT_SETPOINT;
    right_setpoint = DEFAULT_RIGHT_SETPOINT;
    turn_threshold = DEFAULT_TURN_THRESH;

    speedboost_decay = DEFAULT_SPEEDBOOST_DECAY;

    ss_pid = DEFAULT_SS_PID;
    ss_p_coeff = DEFAULT_SS_P_COEFF;
    ss_i_coeff = DEFAULT_SS_I_COEFF;
    ss_d_coeff = DEFAULT_SS_D_COEFF;
    base_p_coeff = DEFAULT_STEER_P;
    base_i_coeff = DEFAULT_STEER_I;
    base_d_coeff = DEFAULT_STEER_D;

    SysCtlClockSet(
            SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN
                    | SYSCTL_XTAL_16MHZ);

    Serial_init(SERIAL_MODULE, 1382400);

    //bt_init(NULL, BT_BAUD_1382400, SERIAL_MODULE);

    //while(1)
    //{
    //    while(Serial_avail(SERIAL_MODULE))
    //    {
    //        Serial_putc(SERIAL_MODULE, Serial_getc(SERIAL_MODULE));
    //    }
    //}

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;

    SW1::init();
    SW2::init();

    SW1::setMode(Pin::Mode_e::MODE_INPUT);
    SW2::setMode(Pin::Mode_e::MODE_INPUT);

    SW1::setPullMode(Pin::PullMode_e::PULLMODE_UP);
    SW2::setPullMode(Pin::PullMode_e::PULLMODE_UP);

    GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4 | GPIO_PIN_0, GPIO_DIR_MODE_IN);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4 | GPIO_PIN_0,
            GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);

    Serial_puts(SERIAL_MODULE, "DankWheels 2.0 Booting...\r\n");

    Serial_puts(SERIAL_MODULE, "Initializing motor control...\r\n");
    motor_init();

    Serial_puts(SERIAL_MODULE, "Initializing camera...\r\n");
    camera_init();

    Serial_puts(SERIAL_MODULE, "Initializing battery monitor...\r\n");
    //batmon_init();

    Serial_puts(SERIAL_MODULE, "Starting main control loop...\r\n");

    //accel_init();

    set_init_table();
    set_register_variable("smaxb", VARTYPE_UINT, &servo_max_highband);
    set_register_variable("sminb", VARTYPE_UINT, &servo_min_highband);
    set_register_variable("sp", VARTYPE_FLOAT, &base_p_coeff);
    set_register_variable("si", VARTYPE_FLOAT, &base_i_coeff);
    set_register_variable("sd", VARTYPE_FLOAT, &base_d_coeff);

    set_register_variable("ssp", VARTYPE_FLOAT, &ss_p_coeff);
    set_register_variable("ssi", VARTYPE_FLOAT, &ss_i_coeff);
    set_register_variable("ssd", VARTYPE_FLOAT, &ss_d_coeff);

    set_register_variable("ssen", VARTYPE_BOOL, &ss_pid);

    set_register_variable("thresh", VARTYPE_FLOAT, &threshold_ratio);
    set_register_variable("scont", VARTYPE_BOOL, &speed_control);
    set_register_variable("smult", VARTYPE_FLOAT, &speed_multiplier);
    set_register_variable("edgec", VARTYPE_UINT, &edgeclip_count);
    set_register_variable("still", VARTYPE_BOOL, &be_still);
    set_register_variable("minsp", VARTYPE_FLOAT, &min_speed);
    set_register_variable("scpol", VARTYPE_UINT, &speedcont_policy);
    set_register_variable("strat", VARTYPE_UINT, &strategy);

    set_register_variable("spdec", VARTYPE_FLOAT, &setpoint_decay);
    set_register_variable("lsp", VARTYPE_FLOAT, &left_setpoint);
    set_register_variable("rsp", VARTYPE_FLOAT, &right_setpoint);
    set_register_variable("tth", VARTYPE_FLOAT, &turn_threshold);

    set_register_variable("sbdec", VARTYPE_FLOAT, &speedboost_decay);

    set_register_variable("red", VARTYPE_BOOL, &red_led);
    set_register_variable("green", VARTYPE_BOOL, &green_led);
    set_register_variable("blue", VARTYPE_BOOL, &blue_led);

    uint32_t loopdelay = SysCtlClockGet() / 100 / 3;

    shell_init();
    shell_registerProgram("get", get_main);
    shell_registerProgram("set", set_main);
    shell_registerProgram("drive", drive_main);
    shell_registerProgram("k", kill_main);
    shell_registerProgram("bat", batmon_main);
    shell_registerProgram("reboot", reboot_main);

    motor_setSpeedi(0);
    servo_setPosf(0.5f);

    float setpoint = 0.5;

    while (1)
    {
        int i;

        float avgpos = 0.5f;

        LineProcessor::update(camera_buffers[buffer_switch]);

        avgpos = LineProcessor::getPos();

        // Scale and apply with PID to servo
        //avgpos /= CAMERA_SAMPLES;

        // Separate variables to keep track of near and far viewpoints
        static float nearPos = 0;
        static float farPos = 0;

        switch(buffer_switch)
        {
            case CAMERA_BUFFER_A:
                nearPos = nearPos*0.3 + (1-avgpos)*0.7;
                centers.lcenter = nearPos * 128;
                break;
            case CAMERA_BUFFER_B:
                farPos = farPos*0.3 + (1-avgpos)*0.7;
                centers.rcenter = farPos * 128;
                break;
        }
//        char debugposbuf[64];
//        int dpbcursor = 0;
//        fast_strcpy(debugposbuf, "NEAR: ");
//        dpbcursor += 6;
//        dpbcursor += fast_snfmtf(debugposbuf + dpbcursor, 64 - dpbcursor,
//                nearPos, 6, 10);
//        fast_strcpy(debugposbuf + dpbcursor, " FAR: ");
//        dpbcursor += 6;
//        dpbcursor += fast_snfmtf(debugposbuf + dpbcursor, 64 - dpbcursor,
//                farPos, 6, 10);
//        fast_strcpy(debugposbuf + dpbcursor, "\r\n");
//        Serial_puts(SERIAL_MODULE, debugposbuf, 64);

        //servo_setPosf(farPos);

        float travelTo = 0;
        if (strategy == BEZIER)
        {
            // a more complicated bezier interpolation
            // 64 and 128 were chosen because I figured they'd be similar in
            // magnitude to nearPos and farPos
            Point p1, p2, pGo;
            p1.x = (nearPos-setpoint);
            p1.y = 2.0f;
            p2.x = (farPos-setpoint);
            p2.y = 1.0f;
            Bezier(bezier_strat_lookahead, &p1, &p2, &pGo);
            travelTo = pGo.x + setpoint;
        }
        else if (strategy == SAFE)
        {
            // only use near camera for steering
            // far camera used for speed scaling (which is down outside of the
            // if-statement)
            travelTo = nearPos;
        }
        else if (strategy == AVERAGE)
        {
            // simple linear interpolation
            travelTo = (nearPos + farPos)/2.0;
        }
        else if (strategy == WEIGHTED)
        {

        travelTo = ((0.75f)*nearPos + (0.25f)*farPos);

        }
        float servopos = PID_calculate(&servopid, travelTo, setpoint, 0.01f);

//           char debugposbuf[64];
//            int dpbcursor = 0;
//            fast_strcpy(debugposbuf, "TRTO: ");
//            dpbcursor += 6;
//            dpbcursor += fast_snfmtf(debugposbuf + dpbcursor, 64 - dpbcursor,
//                    travelTo, 6, 10);
//            fast_strcpy(debugposbuf + dpbcursor, " SPO: ");
//            dpbcursor += 6;
//            dpbcursor += fast_snfmtf(debugposbuf + dpbcursor, 64 - dpbcursor,
//                    servopos, 6, 10);
//            fast_strcpy(debugposbuf + dpbcursor, "\r\n");
//            Serial_puts(SERIAL_MODULE, debugposbuf, 64);

        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, (red_led ? GPIO_PIN_1 : 0) | (blue_led ? GPIO_PIN_2 : 0) | (green_led ? GPIO_PIN_3 : 0));

#ifndef REMOTE_CONTROL
        if (be_still)
        {
            servo_setPosf(0.5f);
            motor_setSpeedi(0);
        }
        else
        {
            servo_setPosf(servopos + 0.5f);

            float speedboost = 0.0f;

            int turndir = 0;

            // Default turnthreshold = 0.05
            if((farPos - nearPos) > turn_threshold)
                turndir = 1;
            if((farPos - nearPos) < -turn_threshold)
                turndir = 2;

            // Default setpointdecay = 0.995
            // Default left_setpoint = 0.2
            // Default right_setpoint = 0.8
            switch(turndir)
            {
                case 0:
                    setpoint = (setpoint*setpoint_decay + 0.5f*(1.0f - setpoint_decay));
                    //shell_stream_adapter.puts("C\r\n");
                    break;
                case 1:
                    setpoint = (setpoint*setpoint_decay + left_setpoint*(1.0f - setpoint_decay));
                    //shell_stream_adapter.puts("L\r\n");
                    break;
                case 2:
                    setpoint = (setpoint*setpoint_decay + right_setpoint*(1.0f - setpoint_decay));
                    //shell_stream_adapter.puts("R\r\n");
                    break;
            }

            if (speed_control)
            {
                // changed to use far camera's position
                float ctr_farPos = farPos - setpoint;
                float ctr_nearPos = nearPos - setpoint;
                float ctr_avgPos = (ctr_farPos + ctr_nearPos)/2;
                float ctr_weightedPos = ((0.25f)*ctr_nearPos + (0.75f)*ctr_farPos);
                float newspeedboost = 0;
                switch(speedcont_policy)
                {
                case SPEEDCONT_AVG_INVSQ:
                    newspeedboost = ((speed_multiplier / (ctr_avgPos * ctr_avgPos)));
                    break;
                case SPEEDCONT_FAR_INVSQ:
                    newspeedboost = ((speed_multiplier / (ctr_farPos * ctr_farPos)));
                    break;
                case SPEEDCONT_NEAR_INVSQ:
                    newspeedboost = ((speed_multiplier / (ctr_nearPos * ctr_nearPos)));
                    break;
                case SPEEDCONT_WEIGHTED_INVSQ:
                    newspeedboost = ((speed_multiplier / (ctr_weightedPos * ctr_weightedPos)));
                    break;
                case SPEEDCONT_AVG_LINEAR:
                    newspeedboost = (speed_multiplier * (0.5 - fabs(ctr_avgPos)));
                    break;
                case SPEEDCONT_FAR_LINEAR:
                    newspeedboost = (speed_multiplier * (0.5 - fabs(ctr_farPos)));
                    break;
                case SPEEDCONT_NEAR_LINEAR:
                    newspeedboost = (speed_multiplier * (0.5 - fabs(ctr_nearPos)));
                    break;
                case SPEEDCONT_WEIGHTED_LINEAR:
                    newspeedboost = (speed_multiplier * (0.5 - fabs(ctr_weightedPos)));
                    break;
                }

                speedboost = speedboost*speedboost_decay + newspeedboost*(1.0f - speedboost_decay);

                motor_setSpeedf(min_speed + speedboost);

            }

            if (ss_pid)
            {
                servopid.coefficients.p = base_p_coeff + speedboost * ss_p_coeff;
                servopid.coefficients.i = base_i_coeff + speedboost * ss_p_coeff;
                servopid.coefficients.d = base_d_coeff + speedboost * ss_p_coeff;
            }
            else
            {
                servopid.coefficients.p = base_p_coeff;
                servopid.coefficients.i = base_i_coeff;
                servopid.coefficients.d = base_d_coeff;
            }

        }
#endif

#ifdef REMOTE_CONTROL

        char cchar = ' ', lchar = ' ', llchar = ' ';

        while((cchar = Serial_getc(SERIAL_MODULE)) != '\n')
        {
            llchar = lchar;
            lchar = cchar;
        }

        if (llchar == 'L')
            servo_setPosf(0.0f);
        else if (llchar == 'R')
            servo_setPosf(1.0f);
        else
            servo_setPosf(0.5f);

        if (lchar == 'U')
            motor_setSpeedf(1.0f);
        else if (lchar == 'D')
            motor_brake();
        else
            motor_setSpeedf(0.0f);

#else

#if !(defined(DEBUG_B64_OUT) || defined(DEBUG_LINE_TO_CONSOLE))

        while(Serial_avail(SERIAL_MODULE))
            B64SerialReader::rxcallback(Serial_getc(SERIAL_MODULE));

        //accel_poll(&accelData);

        Camera0DataSender::registerData(camera_buffers[0], 256);
        Camera1DataSender::registerData(camera_buffers[1], 256);
        Camera0DataSender::send();
        Camera1DataSender::send();
        DerivativeSender::send();
        //AccelDataSender::send();
        //
        CentersSender::send();

        if(SW1::getState() == Pin::State_e::STATE_LOW)
        {
            speed_control = true;
            //green_led = true;
            ////speed_control = !speed_control;
            //while(SW1::getState() == Pin::State_e::STATE_LOW);

            //green_led = false;
        }

        if(SW2::getState() == Pin::State_e::STATE_LOW)
        {
            kill_main(nullptr, 0);
            //speed_control = false;
            //red_led = true;

            //while(SW2::getState() == Pin::State_e::STATE_LOW);

            //red_led = false;
        }
        
        //red_led = SW1::getState() == Pin::State_e::STATE_LOW;
        //green_led = SW2::getState() == Pin::State_e::STATE_LOW;

        shell_poll();

        shell_keepalive();

#endif

#endif

#ifdef DEBUG_B64_OUT
        for (i = 0; i < 128; i++)
        {
            Serial_putc(SERIAL_MODULE, b64str[(camera_buffers[buffer_switch][i] >> 6) & 0x3F]);
            Serial_putc(SERIAL_MODULE, b64str[camera_buffers[buffer_switch][i] & 0x3F]);
        }
        Serial_puts(SERIAL_MODULE, "\r\n");
#endif

#ifdef DEBUG_LINE_TO_CONSOLE

        asm volatile (
                    "dsb\n\t"
                    "isb" :::
            );

        char numprintbuf[64];

        //fast_snprintf(numprintbuf, 64, "Buffer A: 0x%08x B: 0x%08x\r\n", (unsigned int)camera_buffers[0], (unsigned int)camera_buffers[1]);
        //Serial_puts(SERIAL_MODULE, numprintbuf, 100);

        Serial_putc(SERIAL_MODULE, buffer_switch + 'A');
        for (i = 1; i < 128; i++)
        {
            derivative_buffer[i - 1] = abs(((int16_t)camera_buffers[buffer_switch][i]) - ((int16_t)camera_buffers[buffer_switch][i - 1]));
            Serial_putc(SERIAL_MODULE, darkness_charset[derivative_buffer[i - 1] >> 9]);
        }
        Serial_puts(SERIAL_MODULE, "\r\n", 2);
#endif

        switch(buffer_switch)
        {
        case CAMERA_BUFFER_A:
            buffer_switch = CAMERA_BUFFER_B;
            break;
        case CAMERA_BUFFER_B:
            buffer_switch = CAMERA_BUFFER_A;
            break;
        }

        SysCtlDelay(loopdelay);
    }

}

