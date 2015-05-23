/*
 * camera.c
 *
 *  Created on: Apr 17, 2015
 *      Author: Kevin
 */

#include "camera.h"

#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/udma.h"
#include "driverlib/interrupt.h"

#include "inc/hw_gpio.h"
#include "inc/hw_pwm.h"
#include "inc/hw_adc.h"
#include "inc/hw_udma.h"
#include "inc/hw_ints.h"

#include "driverlib/pin_map.h"
#include "inc/hw_memmap.h"

#include "../debug_serial.h"

uint8_t uDMAControlTable[1024] __attribute__ ((aligned(1024)));

#define CAMERA_CLOCK_DIV (32)

#define CAMERA_SAMPLE_PERIOD (64)

uint32_t camera_PWMClockFreq;

uint32_t intcounter;

volatile uint32_t testval;

camera_sample_t camera_DoubleBuffer[2][CAMERA_SAMPLES];
camera_sample_t *camera_buffer;
uint32_t camera_DBSelected;
volatile Camera_t camera_CurCamera;


#define _CT_ATOI(val) #val
#define CT_ATOI(val) _CT_ATOI(val)
#define DEBUG_LINE(_label_) Serial_puts(UART_DEBUG_MODULE,"inside \"" _label_ "\" @ " __FILE__ ":" CT_ATOI(__LINE__) "\r\n", 100)

void PWMGen1Handler()
{
	PWMGenIntClear(PWM0_BASE, PWM_GEN_1, PWM_INT_CNT_LOAD);

	camera_buffer = camera_DoubleBuffer[camera_DBSelected];

	if (!camera_DBSelected)
	{
		camera_DBSelected = 1;
		uDMAChannelTransferSet(UDMA_CHANNEL_ADC3 | UDMA_ALT_SELECT,
		UDMA_MODE_PINGPONG, (void*) (ADC0_BASE + ADC_O_SSFIFO3),
				camera_DoubleBuffer[camera_DBSelected], CAMERA_SAMPLES);
        // switch camera input to near camera
	    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH1 | ADC_CTL_IE | ADC_CTL_END);
        camera_CurCamera = FAR;
	}
	else
	{
		camera_DBSelected = 0;
		uDMAChannelTransferSet(UDMA_CHANNEL_ADC3 | UDMA_PRI_SELECT,
		UDMA_MODE_PINGPONG, (void*) (ADC0_BASE + ADC_O_SSFIFO3),
				camera_DoubleBuffer[camera_DBSelected], CAMERA_SAMPLES);
        // switch camera input to far camera
	    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
        camera_CurCamera = NEAR;
	}
}

void camera_init()
{
	Serial_puts(UART_DEBUG_MODULE, "inside \"camera_init\"\r\n", 100);
	// Calculate the PWM clock frequency
	camera_PWMClockFreq = SysCtlClockGet() / CAMERA_CLOCK_DIV;

	DEBUG_LINE("camera_init");

	// Enable the PWM peripheral
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

	// Enable the GPIO port
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	// Configure PD0 as the PWM output for the drive motor
	GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);
	GPIOPinConfigure(GPIO_PB7_M0PWM1);
	GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4);
	GPIOPinConfigure(GPIO_PB4_M0PWM2);

	// Set the camera clock pulse period
	PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, CAMERA_SAMPLE_PERIOD - 1);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, (CAMERA_SAMPLE_PERIOD / 2) - 1);

	// Set the camera enable pulse period
	PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, (CAMERA_SAMPLE_PERIOD * CAMERA_SAMPLES) - 1);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, ((CAMERA_SAMPLE_PERIOD / 2) * 2) - 1);

	DEBUG_LINE("camera_init");

	// Enable the PWM output
	PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT | PWM_OUT_2_BIT, true);
	PWMGenEnable(PWM0_BASE, PWM_GEN_0);
	PWMGenEnable(PWM0_BASE, PWM_GEN_1);

	PWMSyncTimeBase(PWM0_BASE, PWM_GEN_0_BIT | PWM_GEN_1_BIT);

	// Enable PWM trigger on zero count on Generator 0
	PWMGenIntTrigEnable(PWM0_BASE, PWM_GEN_0, PWM_TR_CNT_ZERO); // PWM_TR_CNT_ZERO/PWM_TR_CNT_LOAD

	// Trigger an interrupt on GEN1 load (to setup the uDMA transfer on a consistent time boundary)
	PWMGenIntTrigEnable(PWM0_BASE, PWM_GEN_1, PWM_INT_CNT_LOAD);

	DEBUG_LINE("camera_init");
	
    /********************************************
	 * 			  ADC CONFIGURATION			    *
	 ********************************************
	 */

	// Enable ADC0 module
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

	DEBUG_LINE("camera_init");

	ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PIOSC | ADC_CLOCK_RATE_FULL, 1);

	DEBUG_LINE("camera_init");

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    // Camera Far
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
    // Camera Near
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);

	DEBUG_LINE("camera_init");

	// Configure and enable the ADC sequence; single sample
	ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PWM0, 0);
	ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE, 3);

	DEBUG_LINE("camera_init");

	ADCSequenceDMAEnable(ADC0_BASE, 3);

	DEBUG_LINE("camera_init");

	// Start writing into the first buffer
	camera_DBSelected = 0;
    camera_CurCamera = FAR;
	// Expose the other buffer
	camera_buffer = camera_DoubleBuffer[1];

	/********************************************
	 * 			  uDMA CONFIGURATION			*
	 ********************************************
	 */

	// Enable the uDMA for normal and sleep operation
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
	SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UDMA);
	uDMAEnable();

	DEBUG_LINE("camera_init");

	// Set the position of the uDMA control table
	uDMAControlBaseSet(uDMAControlTable);

	// Put the uDMA table entry for ADC3 into a known state
	uDMAChannelAttributeDisable(UDMA_CHANNEL_ADC3,
		UDMA_ATTR_USEBURST |
		UDMA_ATTR_ALTSELECT |
		UDMA_ATTR_HIGH_PRIORITY |
		UDMA_ATTR_REQMASK);

	// Configure the primary and alternate uDMA channel structures
	uDMAChannelControlSet(UDMA_CHANNEL_ADC3 | UDMA_PRI_SELECT, UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);
	uDMAChannelControlSet(UDMA_CHANNEL_ADC3 | UDMA_ALT_SELECT, UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);

	// Configure the primary and alternate transfers for ping-pong operation
	uDMAChannelTransferSet(UDMA_CHANNEL_ADC3 | UDMA_PRI_SELECT, UDMA_MODE_PINGPONG, (void*) (ADC0_BASE + ADC_O_SSFIFO3), camera_DoubleBuffer[0], CAMERA_SAMPLES);
	uDMAChannelTransferSet(UDMA_CHANNEL_ADC3 | UDMA_ALT_SELECT, UDMA_MODE_PINGPONG, (void*) (ADC0_BASE + ADC_O_SSFIFO3), camera_DoubleBuffer[1], CAMERA_SAMPLES);

	DEBUG_LINE("camera_init");

	// Enable the ADC3 uDMA channel
	uDMAChannelEnable(UDMA_CHANNEL_ADC3);

	// Enable interrupts
	// IntEnable(INT_ADC0SS3);
	// ADCIntEnableEx(ADC0_BASE, ADC_INT_DMA_SS3);

	IntEnable(INT_PWM0_1);
	PWMIntEnable(PWM0_BASE, PWM_INT_GEN_1);


	DEBUG_LINE("camera_init");

}


void camera_registerCallback(camera_callback_t callback)
{

}
