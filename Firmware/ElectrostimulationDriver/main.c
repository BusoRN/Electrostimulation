#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <inc/hw_memmap.h>
#include "inc/hw_ssi.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include <driverlib/interrupt.h>
#include <driverlib/sysctl.h>
#include "driverlib/pin_map.h"
#include <driverlib/timer.h>
#include <inc/tm4c123gh6pm.h>
#include <driverlib/systick.h>
#include "driverlib/adc.h"
#include "driverlib/uart.h"
#include "driverlib/eeprom.h"
#include "driverlib/flash.h"
#include "utils/uartstdio.h"


#define FREQUENCY_INTERRUPT 1
#define PULSE 2
#define PHASE 1
#define NUM_SSI_DATA 8
#define CS1 GPIO_PIN_3
#define SHDWN GPIO_PIN_4
#define RS GPIO_PIN_6
#define CS2 GPIO_PIN_7

/*******************************************************************************************/
/*        																				   */
/*										   PROTOTYPE									   */
/*																						   */
/*******************************************************************************************/

void SendToFirstPotentiometer(uint16_t data);
void SendToSecondPotentiometer(uint16_t data);
//void PortFIntHandler(void);

volatile unsigned long FallingEdges = 0;
volatile uint32_t ui32Period;
int pulse_width = 1;
int frequency_interrupt = 1;
int phase = 1;


unsigned char cState;

// represents a State of the FSM
struct State{
	unsigned char out;     // PB7-4 to right motor, PB3-0 to left
	unsigned short wait;   // in ms units
};

typedef const struct State StateType;
StateType Fsm[3] = {
		{0x02, 1}, // Positive_Pulse
		{0x08, 1}, // Negative_Pulse
		{0x00, 0}, // Zero_Pulse
};

void SysTick_Wait(uint32_t delay){
	NVIC_ST_RELOAD_R = delay-1;  // number of counts to wait
	NVIC_ST_CURRENT_R = 0;       // any value written to CURRENT clears
	while((NVIC_ST_CTRL_R&0x00010000)==0){ // wait for count flag
	}

}

void SysTick_Wait1ms(uint32_t delay){
	uint32_t i;
	for(i=0; i<delay; i++){
		SysTick_Wait(40000);  // wait 1ms
	}
}



void SysTick_Init(void)
{
	NVIC_ST_CTRL_R = 0;               // disable SysTick during setup
	NVIC_ST_CTRL_R = 0x00000005;      // enable SysTick with core clock
}




int main(void)
{
	uint32_t read;
	volatile uint32_t ui32VoltageValue;
	//int i = 0;
	//char buffer[4];
	uint32_t pui32Read[6];
	uint32_t pui32Data[6];
	uint32_t returnCode;
	bool start_flag_01 = false;
	bool start_flag_02 = false;
	bool start_flag_03 = false;
	bool skip_flag = false;

	/*
	 * Set the clock @ 40 MHz
	 */
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

	/*
	 * Enable clock for the used peripheral:
	 * - PORTF
	 * - PORTA
	 * - SSI0
	 * - TIMER0
	 * - ADC0
	 * - UART0
	 * - EEPROM
	 */
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);


	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	/*
	 * Set the output PINs for the PORTF
	 * - PIN1 = RED LED
	 * - PIN2 = BLUE LED
	 * - PIN3 = GREEN LED
	 */

	/*
	 * Set the input for SW1
	 */


	GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock PortF PF0
	GPIO_PORTF_CR_R |= 0x1F;           // allow changes to PF4-0
	GPIO_PORTF_AMSEL_R &= 0x00;        // 3) disable analog function
	GPIO_PORTF_PCTL_R &= 0x00000000;   // 4) GPIO clear bit PCTL
	GPIO_PORTF_DIR_R &= ~0x11;          // 5.1) PF4,PF0 input,
	GPIO_PORTF_DIR_R |= 0x0E;          // 5.2) PF3-PF2-PF1 output
	GPIO_PORTF_AFSEL_R &= 0x00;        // 6) no alternate function
	GPIO_PORTF_PUR_R |= 0x11;          // enable pullup resistors on PF4,PF0
	GPIO_PORTF_DEN_R |= 0x1F;          // 7) enable digital pins PF4-PF0



	//
	// Register the port-level interrupt handler. This handler is the first
	// level interrupt handler for all the pin interrupts.
	//

	GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	//GPIOIntRegister(GPIO_PORTF_BASE, &PortFIntHandler);
	GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE);
	GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_4);




	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, CS1|CS2|SHDWN|RS);

	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE,GPIO_PIN_1|GPIO_PIN_3);

	/*
	 * Configure the UART peripheral
	 */
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	/*
	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
	       (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	 */
	UARTStdioConfig(0, 9600, 40000000);

	/*
	 * Configure the I2C peripheral
	 */
	GPIOPinConfigure(GPIO_PA6_I2C1SCL);
	GPIOPinConfigure(GPIO_PA7_I2C1SDA);
	GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7);

	//I2CMasterInitExpClk(I2C1_BASE, SysCtlClockGet(), false);
	//while(1){
	//send_command_LCD (0b00011111);
	//}

	//BacklightLCD (TURN_ON_LED_LCD);
	//WriteStringLCD ("  Hello World");






	/*
	 * Enable the SysTick timer, used for generating delays
	 */
	SysTickEnable();
	do
	{
		returnCode = EEPROMInit();
	}while (returnCode != EEPROM_INIT_OK);



	EEPROMRead((uint32_t *)&pui32Read, 0x0000, sizeof(pui32Read));
	pulse_width = pui32Read[0];
	frequency_interrupt = pui32Read[1];
	phase = pui32Read[2];
	uint32_t pulse_width2 = pui32Read[3];
	uint32_t frequency_interrupt2 = pui32Read[4];
	uint32_t phase2 = pui32Read[5];

	if (pulse_width != pulse_width2)
		pulse_width = PULSE;
	if ( frequency_interrupt != frequency_interrupt2)
		frequency_interrupt = FREQUENCY_INTERRUPT;
	if (phase != phase2 )
		phase = PHASE;


//	pulse_width = 2;
//	frequency_interrupt = 1;
//	phase = 0;



	/*
	 * Enable interrupt for from TimerA
	 */

	IntEnable(INT_TIMER0A);
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	IntMasterEnable();
	UARTprintf("Press a button to Start with the stored values\n");
	/*
	UARTprintf("Enter the voltage value:\n");

	char serial_buffer[5];
	do{
		while(UARTCharsAvail(UART0_BASE))
		{
			UARTgets(serial_buffer,5);
			goFlag = true;
		}
		if((GPIO_PORTF_DATA_R&0x11) != 0x11)
			read = 5; // PF0 into read
	}while(read !=5 & (!goFlag));
	if(read == 5)
		SysTick_Wait1ms(500);
	else
	{
		ui32OutputValue = (uint32_t) atoi(serial_buffer);
		UARTprintf("Taken value:");
		UARTprintf(serial_buffer);
	}
	*/

	UARTprintf("\nEnter the width value:\n");


		char serial_buffer_char[4];
		//i= 0;
		//char data[3];
		do{
			while(UARTCharsAvail(UART0_BASE))
			{
				UARTgets(serial_buffer_char,4);
				UARTprintf("Taken value:");
				UARTprintf(serial_buffer_char);
				UARTprintf("\nPress a button to Start or insert frequency\n");
				pulse_width = (uint32_t) atoi(serial_buffer_char);
				start_flag_01 = true;
				break;

			}
			read = GPIO_PORTF_DATA_R&0x11; // PF0 into read
			if(read != 0x11)
				skip_flag = true;

		}while((skip_flag == false) && (start_flag_01 == false));

		do{
					while(UARTCharsAvail(UART0_BASE))
					{
						UARTgets(serial_buffer_char,4);
						UARTprintf("Taken value:");
						UARTprintf(serial_buffer_char);
						UARTprintf("\nPress a button to Start or insert phasic\n");
						frequency_interrupt = (uint32_t) atoi(serial_buffer_char);
						start_flag_02 = true;
						break;

					}
					read = GPIO_PORTF_DATA_R&0x11; // PF0 into read
					if(read != 0x11)
						skip_flag = true;

				}while((skip_flag == false) && (start_flag_02 == false));

		do{
							while(UARTCharsAvail(UART0_BASE))
							{
								UARTgets(serial_buffer_char,2);
								UARTprintf("Taken value:");
								UARTprintf(serial_buffer_char);
								UARTprintf("\nPress a button to Start\n");
								phase = (uint32_t) atoi(serial_buffer_char);
								start_flag_03 = true;
								break;

							}
							read = GPIO_PORTF_DATA_R&0x11; // PF0 into read
							if(read != 0x11)
								skip_flag = true;

						}while((skip_flag == false) && (start_flag_03 == false));




		//pulse_width = (uint32_t)atoi(data);

		//EEProm
		pui32Data[0] = pulse_width;
		pui32Data[1] = frequency_interrupt;
		pui32Data[2] = phase;
		pui32Data[3] = pulse_width;
		pui32Data[4] = frequency_interrupt;
		pui32Data[5] = phase;
		EEPROMProgram((uint32_t *)&pui32Data, 0x00000, sizeof(pui32Data));
		SysTick_Wait1ms(500);

		/*
			 * Configure the TimerA0 in order to generate a periodic interrupt every seconds
			 */

			TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
			ui32Period = (SysCtlClockGet()/frequency_interrupt);
			TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period -1);




	UARTprintf("Pulses generator started\n");
	UARTprintf("Press a button to Stop\n");
	/*
	//EEProm
	pui32Data[0] = ui32OutputValue;
	pui32Data[1] = pulse_width;
	EEPROMProgram((uint32_t *)&pui32Data, 0x00000, sizeof(pui32Data));
	SysTick_Wait1ms(500);
	 */

	/*
	 * Start Timer0 to count
	 */

	TimerEnable(TIMER0_BASE, TIMER_A);


	/*
	 * Interrupt for GPIO
	 */
	//GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE);
	//GPIOIntEnable(GPIO_PORTF_BASE,GPIO_PIN_4);


	while(1)
	{

		read = GPIO_PORTF_DATA_R&0x11; // PF0 into read
		if(read != 0x11){
			if(FallingEdges%2)
			{
				TimerEnable(TIMER0_BASE, TIMER_A);
			}
			else
			{
				TimerDisable(TIMER0_BASE, TIMER_A);
			}
			FallingEdges++;
			SysTick_Wait1ms(500);
			//GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3,GPIO_PIN_3);


		}
	}
}

void Timer0AIntHandler(void)
{
	// Clear the timer interrupt
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

	for(cState = 0; cState <3; cState++)
	{
		if(cState == 1){
			// output based on current state
			GPIO_PORTB_DATA_R = Fsm[cState].out*phase;
			GPIO_PORTF_DATA_R = Fsm[cState].out*phase;
		}
		else
		{
			// output based on current state
			GPIO_PORTB_DATA_R = Fsm[cState].out;
			GPIO_PORTF_DATA_R = Fsm[cState].out;
		}

		// wait for time according to state
		SysTick_Wait1ms(Fsm[cState].wait*pulse_width);
	}
}


void PortFIntHandler(void){
	if(FallingEdges%2)
				{
					TimerEnable(TIMER0_BASE, TIMER_A);
				}
				else
				{
					TimerDisable(TIMER0_BASE, TIMER_A);
				}
				FallingEdges++;
				SysTick_Wait1ms(500);
}
