/*
 * main.c
 */
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
#include <stdio.h>

#include "Motor/motor.h"
#include "Camera/camera.h"
#include "Gyro/gyro.h"

#include "PID/PIDControl.h"

#include "ShellInterface/shell.h"

#include "ShellInterface/prog_getset.h"

#include "debug_serial.h"

#include "Settings/defaults.h"

#include "bezier.h"

float speed;

// Coefficients
float threshold_ratio;
bool speed_control;
float speed_multiplier;
bool killswitch;
static PIDController_t servopid;
uint32_t edgeclip_count;
float min_speed;

bool red_led, green_led, blue_led;

bool be_still;

// some variables which control the strategy used by the car
enum strategy_t
{
    BEZIER = 1,
    SAFE = 2,
    AVERAGE = 3,
};
enum strategy_t strategy = AVERAGE;
float bezier_strat_lookahead = 0.7;

const char darkness_charset[] = " .,:;!?#";

const char b64str[] =
		"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789/+";

inline uint32_t map_val(uint32_t val, uint32_t inmin, uint32_t inmax,
		uint32_t outmin, uint32_t outmax)
{
	uint32_t retval = ((val - inmin) * (outmax - outmin));
	uint32_t divisor = (inmax - inmin);
	if (divisor != 0)
		return retval / divisor;
	return 0;
}

set_entry_t set_entries[MAX_SET_ENTRIES];

int kill_main(char* argv[], int argc)
{
	killswitch = true;
	speed_control = false;
	motor_setSpeedf(0.0f);
	motor_brake();

	if(argc == 2 && fast_strcmp(argv[1], "unkill") == 0)
	{
		killswitch = false;
		motor_releaseBrake();
		Serial_puts(UART_DEBUG_MODULE, "Released brake.\r\n", 100);
	}
	else
	{
		Serial_puts(UART_DEBUG_MODULE, "KILLSWITCH ENGAGED!\r\n", 100);
	}

	return 0;
}

int drive_main(char* argv[], int argc)
{
	char printbuf[64];
	if(argc != 2)
	{
		return -1;
	}

	bool succ = false;
	float val = fast_sntof(argv[1], fast_strlen(argv[1]), 10, &succ);

	if(succ)
	{
		motor_setSpeedf(val);
		fast_snprintf(printbuf, 64, "Speed=%d.%05d\r\n", (long)val,(long)(100000*(val - (long)val)));
	}
	else
	{
		Serial_puts(UART_DEBUG_MODULE, "Invalid argument!\r\n", 100);
		return -1;
	}

	Serial_puts(UART_DEBUG_MODULE, printbuf, 100);

	return 0;
}

camera_sample_t derivative_buffer[128];

#define THRESHOLD_RATIO (0.65f)



//#define DEBUG_B64_OUT
//#define REMOTE_CONTROL

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

	SysCtlClockSet(
			SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN
					| SYSCTL_XTAL_16MHZ);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;

	GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4 | GPIO_PIN_0, GPIO_DIR_MODE_IN);
	GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4 | GPIO_PIN_0,
			GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);

	Serial_init(UART_DEBUG_MODULE, 9600);

	Serial_puts(UART_DEBUG_MODULE, "Hello there!\r\n", 100);

	Serial_puts(UART_DEBUG_MODULE, "Initializing motor control...\r\n", 100);
	motor_init();

	Serial_puts(UART_DEBUG_MODULE, "Initializing camera...\r\n", 100);
	camera_init();

	Serial_puts(UART_DEBUG_MODULE, "Starting main control loop...\r\n", 100);

	set_init_table();
	set_register_variable("smaxb", VARTYPE_UINT, &servo_max_highband);
	set_register_variable("sminb", VARTYPE_UINT, &servo_min_highband);
	set_register_variable("sp", VARTYPE_FLOAT, &servopid.coefficients.p);
	set_register_variable("si", VARTYPE_FLOAT, &servopid.coefficients.i);
	set_register_variable("sd", VARTYPE_FLOAT, &servopid.coefficients.d);
	set_register_variable("thresh", VARTYPE_FLOAT, &threshold_ratio);
	set_register_variable("scont", VARTYPE_BOOL, &speed_control);
	set_register_variable("smult", VARTYPE_FLOAT, &speed_multiplier);
	set_register_variable("edgec", VARTYPE_UINT, &edgeclip_count);
	set_register_variable("still", VARTYPE_BOOL, &be_still);
	set_register_variable("minsp", VARTYPE_FLOAT, &min_speed);

	set_register_variable("red", VARTYPE_BOOL, &red_led);
	set_register_variable("green", VARTYPE_BOOL, &green_led);
	set_register_variable("blue", VARTYPE_BOOL, &blue_led);

	uint32_t loopdelay = SysCtlClockGet() / 100 / 3;

	shell_init();
	shell_registerProgram("get", get_main);
	shell_registerProgram("set", set_main);
	shell_registerProgram("drive", drive_main);
	shell_registerProgram("k", kill_main);

	motor_setSpeedi(0);
	servo_setPosf(0.5f);

	while (1)
	{
		int i;
		static camera_sample_t minderiv, maxderiv;

		minderiv = 1 << 13;
		maxderiv = 0;

		// Compute the derivative of the linecam data
		for (i = 1; i < CAMERA_SAMPLES; i++)
		{
			if(i < (edgeclip_count + 1) || i > (CAMERA_SAMPLES - edgeclip_count))
			{
				derivative_buffer[i - 1] = 0;
				continue;
			}
			derivative_buffer[i - 1] = abs(
					((int16_t) camera_buffer[i])
							- ((int16_t) camera_buffer[i - 1]));
		}

		// Get the min and max derivative values
		for (i = 0; i < (CAMERA_SAMPLES - 1); i++)
		{
			if (derivative_buffer[i] > maxderiv)
				maxderiv = derivative_buffer[i];
			if (derivative_buffer[i] < minderiv)
				minderiv = derivative_buffer[i];
		}

		// Compute the average x value of the threshold values
		int numpositions = 0;
		float avgpos = 0;

		for (i = 0; i < (CAMERA_SAMPLES - 1); i++)
		{
            // why use the derivative for this?
			float absval = derivative_buffer[i] - minderiv;
			float range = maxderiv - minderiv;

			if(absval > (range*threshold_ratio))
			{
				avgpos += i;
				numpositions++;
			}
		}

		if(numpositions != 0)
			avgpos /= numpositions;
		else
			avgpos = CAMERA_SAMPLES / 2;

		// Scale and apply with PID to servo
		avgpos /= CAMERA_SAMPLES;

        // Separate variables to keep track of near and far viewpoints
        static float nearPos = 0;
        static float farPos = 0;

        switch(current_Camera)
        {
            case NEAR:
                nearPos = avgpos;
                break;
            case FAR:
                farPos = avgpos;
                break;
        }

        float travelTo = 0;
        if (strategy == BEZIER)
        {
            // a more complicated bezier interpolation
            // 64 and 128 were chosen because I figured they'd be similar in
            // magnitude to nearPos and farPos
            Point p1, p2, pGo;
            p1.x = nearPos;
            p1.y = 64;
            p2.x = farPos;
            p2.y = 128;
            bezier(bezier_strat_lookahead, &p1, &p2, &pGo);
            travelTo = arctan(pGo.x/pGo.y);
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

		float servopos = PID_calculate(&servopid, travelTo, 0.5f, 0.01f);

		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, (red_led ? GPIO_PIN_1 : 0) | (blue_led ? GPIO_PIN_2 : 0) | (green_led ? GPIO_PIN_3 : 0));

#ifndef REMOTE_CONTROL
		if (be_still)
		{
			servo_setPosf(0.5f);
			motor_setSpeedi(0);
		}
		else
		{
			servo_setPosf(0.5f - servopos);

			if (speed_control)
			{
                // changed to use far camera's position
                float ctr_farPos = farPos - CAMERA_SAMPLES/2;
				motor_setSpeedf(min_speed + (speed_multiplier / (ctr_farPos * ctr_farPos)));
			}
		}
#endif

#ifdef REMOTE_CONTROL

		char cchar = ' ', lchar = ' ', llchar = ' ';

		while((cchar = Serial_getc(UART_DEBUG_MODULE)) != '\n')
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

#ifndef DEBUG_B64_OUT

		shell_poll();

#endif

#endif

#ifdef DEBUG_B64_OUT
		for (i = 0; i < 128; i++)
		{
			Serial_putc(UART_DEBUG_MODULE, b64str[(camera_buffer[i] >> 6) & 0x3F]);
			Serial_putc(UART_DEBUG_MODULE, b64str[camera_buffer[i] & 0x3F]);
		}
		Serial_puts(UART_DEBUG_MODULE, "\r\n", 2);
#endif

#ifdef DEBUG_LINE_TO_CONSOLE
		for (i = 1; i < 128; i++)
		{
			derivative_buffer[i - 1] = abs(((int16_t)camera_buffer[i]) - ((int16_t)camera_buffer[i - 1]));
			Serial_putc(UART_DEBUG_MODULE, darkness_charset[derivative_buffer[i - 1] >> 9]);
		}
		Serial_puts(UART_DEBUG_MODULE, "\r\n", 2);
#endif


		SysCtlDelay(loopdelay);
	}

}

