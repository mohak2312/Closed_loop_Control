/*
 * ECE_544_Project_2.c
 *
 *  Created on: May 16, 2018
 *      Author: Archit and Mohak
 */
// Description:

// Implemented Entry point function (main) which initializes Platform
// and all Peripherals such as PmodENC, PmodOLEDrgb, AXI Timer, GPIO Ports,
// Connects Interrupts to Interrupt Controller.
// There are in all Four Tasks:
// 1.Parameter Task--Take all inputs and update the values of parameter and send those values through queue.
// 2.Display Task-- Receive the input from queue and display all parameter's updated value on PMODOLEDrgb.
// 3.ADC_ input Task-- Read the GPIO for signal detection of Hall sensor.
// 4.PID algorithm Task--Receives the input from queue and implements the PID control logic.

/*********************************************************************************************************/
/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

/* BSP includes. */
#include "xtmrctr.h"
#include "xgpio.h"
#include "sleep.h"

#include "mb_interface.h"
#include "nexys4IO.h"
#include "PmodENC.h"
#include "PmodOLEDrgb.h"

#include<stdbool.h>
#include<stdio.h>
#include<stdlib.h>
#include<sys/_stdint.h>

#include "xil_types.h"
#include "xintc.h"
#include "xstatus.h"

#include "xtmrctr_l.h"
#include "platform.h"
#include "xtmrctr.h"

#include "xil_printf.h"
#include "xparameters.h"
#include "pwm_tmrctr.h"
#include "xwdttb.h"

/************************Constant Definitions******************/
// Clock frequencies
#define CPU_CLOCK_FREQ_HZ		XPAR_CPU_CORE_CLOCK_FREQ_HZ
#define AXI_CLOCK_FREQ_HZ		XPAR_CPU_M_AXI_DP_FREQ_HZ


// PWM and pulse detect timer parameters
#define PWM_TIMER_DEVICE_ID		XPAR_TMRCTR_2_DEVICE_ID



// AXI timer parameters
#define AXI_TIMER_0_DEVICE_ID		XPAR_AXI_TIMER_0_DEVICE_ID
#define AXI_TIMER_0_BASEADDR		XPAR_AXI_TIMER_0_BASEADDR
#define AXI_TIMER_0_HIGHADDR		XPAR_AXI_TIMER_0_HIGHADDR
#define TmrCtrNumber			0


// AXI timer parameters
#define AXI_TIMER_1_DEVICE_ID		XPAR_AXI_TIMER_1_DEVICE_ID
#define AXI_TIMER_1_BASEADDR		XPAR_AXI_TIMER_1_BASEADDR
#define AXI_TIMER_1_HIGHADDR		XPAR_AXI_TIMER_1_HIGHADDR
#define TmrCtrNumber_1				0


// AXI timer parameters
#define AXI_TIMER_2_DEVICE_ID		XPAR_AXI_TIMER_2_DEVICE_ID
#define AXI_TIMER_2_BASEADDR		XPAR_AXI_TIMER_2_BASEADDR
#define AXI_TIMER_2_HIGHADDR		XPAR_AXI_TIMER_2_HIGHADDR
#define TmrCtrNumber_2				0


// Definitions for peripheral NEXYS4IO
#define NX4IO_DEVICE_ID		XPAR_NEXYS4IO_0_DEVICE_ID
#define NX4IO_BASEADDR		XPAR_NEXYS4IO_0_S00_AXI_BASEADDR
#define NX4IO_HIGHADDR		XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR

// Definitions for peripheral PMODOLEDRGB
#define RGBDSPLY_DEVICE_ID		XPAR_PMODOLEDRGB_0_DEVICE_ID
#define RGBDSPLY_GPIO_BASEADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_GPIO_BASEADDR
#define RGBDSPLY_GPIO_HIGHADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_GPIO_HIGHADD
#define RGBDSPLY_SPI_BASEADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_SPI_BASEADDR
#define RGBDSPLY_SPI_HIGHADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_SPI_HIGHADDR

// Definitions for peripheral PMODENC
#define PMODENC_DEVICE_ID		XPAR_PMODENC_0_DEVICE_ID
#define PMODENC_BASEADDR		XPAR_PMODENC_0_AXI_LITE_GPIO_BASEADDR
#define PMODENC_HIGHADDR		XPAR_PMODENC_0_AXI_LITE_GPIO_HIGHADDR

#define WATCHDOG_TIMER_ID		XPAR_MICROBLAZE_0_AXI_INTC_AXI_TIMEBASE_WDT_0_WDT_INTERRUPT_INTR

// WatchDOG Timer parameters
#define WATCHDOG_DEVICE_ID		XPAR_WDTTB_0_DEVICE_ID
#define WATCHDOG_BASE_ADDR		XPAR_WDTTB_0_BASEADDR
#define WATCHDOG_HIGH_ADDR		XPAR_WDTTB_0_HIGHADDR
#define WATCHDOG_WND_TIMEOUT	XPAR_WDTTB_0_ENABLE_WINDOW_WDT
#define WATCHDOG_MAX_CNT		XPAR_WDTTB_0_MAX_COUNT_WIDTH

// Fixed Interval timer - 100 MHz input clock, 40KHz output clock
// FIT_COUNT_1MSEC = FIT_CLOCK_FREQ_HZ * .001
#define FIT_IN_CLOCK_FREQ_HZ	CPU_CLOCK_FREQ_HZ
#define FIT_CLOCK_FREQ_HZ		40000
#define FIT_COUNT				(FIT_IN_CLOCK_FREQ_HZ / FIT_CLOCK_FREQ_HZ)
#define FIT_COUNT_1MSEC			40

// GPIO parameters
#define GPIO_0_DEVICE_ID			XPAR_AXI_GPIO_0_DEVICE_ID
#define GPIO_0_OUTPUT_0_CHANNEL		1
#define GPIO_0_OUTPUT_1_CHANNEL		2
#define GPIO_1_DEVICE_ID            XPAR_AXI_GPIO_1_DEVICE_ID
#define GPIO_1_INPUT_0_CHANNEL		1
#define GPIO_1_INPUT_1_CHANNEL		2
#define GPIO_2_DEVICE_ID            XPAR_AXI_GPIO_2_DEVICE_ID
#define GPIO_2_INPUT_0_CHANNEL		1
#define GPIO_2_INPUT_1_CHANNEL		2

// Interrupt Controller parameters
#define INTC_DEVICE_ID			XPAR_INTC_0_DEVICE_ID
#define FIT_INTERRUPT_ID		XPAR_MICROBLAZE_0_AXI_INTC_FIT_TIMER_0_INTERRUPT_INTR
/*********************************************************/
#define mainQUEUE_LENGTH					( 1 )
#define taskQUEUE_LENGTH					( 3 )
/* A block time of 0 simply means, "don't block". */
#define mainDONT_BLOCK						( portTickType ) 0

uint64_t 	timestamp = 0L;
PmodENC 	pmodENC_inst;
PmodOLEDrgb	pmodOLEDrgb_inst;
PmodENC 	pmodENC_inst;
static XGpio		GPIOInst0;// GPIO instance

static XGpio		GPIOInst1;// GPIO instance
static XGpio		GPIOInst2;// GPIO instance

XIntc 		IntrptCtlrInst;				// Interrupt Controller instance
XTmrCtr		AXITimerInst;				//Timer0 instance
XTmrCtr		AXITimerInst1;				//Timer1 instance
XTmrCtr		AXITimerInst2;				//Timer2 instance
XTmrCtr		PWMTimerInst;				// PWM timer instance
XWdtTb		Watchdog;					//Watchdog timer instance


volatile u32			gpio_in;
int speed,mtr_var,freq;
//Function Declarations
void PMDIO_itoa(int32_t value, char *string, int32_t radix);
void PMDIO_puthex(PmodOLEDrgb* InstancePtr, uint32_t num);
void PMDIO_putnum(PmodOLEDrgb* InstancePtr, int32_t num, int32_t radix);
int	 do_init(void);											// initialize system
void FIT_Handler(void);										// fixed interval timer interrupt handler
int AXI_Timer_initialize(void);
int AXI_Timer_1_initialize(void);
int AXI_Timer_2_initialize(void);
void parameter_input_thread(void *p);                       // Input parameter thread
void adc_input_thread(void *p);								// ADC_input thread for Hall sensor signal detection
void set_PWM(int);											// TO set the PWM

/* The queue used by the queue send and queue receive tasks. */
static xQueueHandle xQueue1 = NULL;   //Queue to transfer data between parameter_input_thread and display_thread
static xQueueHandle xQueue2 = NULL;   //Queue to transfer data between parameter_input_thread and pid_thread
static xQueueHandle xQueue3 = NULL;   //Queue to transfer data between pid_thread and display_thread
static xQueueHandle xQueue4 = NULL;   //Queue to transfer data between pid_thread and adc_input

/* The structures used to transfer data via queue for IPC*/
typedef struct{     //Structure to pass the input states between different threads via queue
u16 switches;   //Dip switches value
int Kp;
int Ki;
int Kd;
int Speed;
}inputs_struct, *inputs_struct_ptr;
inputs_struct xInputs,xInputs2;				//Global struct variable


typedef struct {     //Structure to pass the pid parameters and states between different functions
int kp;
int ki;
int kd;
int mtr_speed;
}pid_struct;

typedef struct {     //Structure to pass the adc_input parameter to Pid thread
int freq;
}adc_struct;
adc_struct xInputs3;


//Parameter thread for all the INPUTS
void parameter_input_thread(void *p)
{	//local variables
	u32 state,laststate; //comparing current and previous state to detect edges on GPIO pins.
	int ticks = 0, lastticks = 0;
	u16 sw;
	int update_val=0;
	//int mtr_var=0;
	int param_var=0;
	int KP=0,KI=0,KD=0,speed=4000; //set the parameters for PID algorithm and set setpoint to 4000 rpm
	xInputs.Kp=0;
	xInputs.Ki=0;
	xInputs.Kd=0;

	// get the previous state
	laststate = ENC_getState(&pmodENC_inst);

	while(1)
	{
		// get the PmodENC state
		state = ENC_getState(&pmodENC_inst);

		// Update the speed value
		ticks += ENC_getRotation(state, laststate);

		//Read switches to set different parameters
		sw = NX4IO_getSwitches();

		if ((sw & 0x0003)==(0x0002|0x0003))
		{     													//Check if switches[1:0] are 10 or 11
			mtr_var = 100;           							//Increase the motor speed by 10
		}
		else if((sw & 0x0003)==0x0001){     					//Check if switches[1:0] are 01
			mtr_var = 5;             							//Increase the motor speed by 5
		}
		else if((sw & 0x0003)==0x0000){   					  //Check if switches[1:0] are 00
			mtr_var = 1;             						//Increase the motor speed by 1
		}

		if(ticks > lastticks)								//If new ticks is greater than old ticks
		{
			speed = speed + mtr_var;						// increase the speed
		}
		else if(ticks < lastticks)							//If new ticks is less than old ticks
		{
			speed = speed - mtr_var;						//Decrease the speed
		}

		//check BTNL and BTNR count if it's pressed
		//update the speed
		//BTNL to increase the speed and BTNR to decrease the speed
		if(NX4IO_isPressed(BTNL))
		{
			speed = speed + mtr_var;
		}
		else if(NX4IO_isPressed(BTNR))
		{
			speed = speed - mtr_var;
		}

		if((sw & 0x0030)==(0x0020|0x0030)){     				 //Check if switches[5:4] are 10 or 11
			param_var = 10;      								 //Increase the parameter value by 10
		}
		else if((sw & 0x0030)==0x0010){          				 //Check if switches[5:4] are 01
			param_var = 5;         								//Increase the parameter value by 5
		}
		else if((sw & 0x0030)==0x0000){           				//Check if switches[5:4] are 00
			param_var = 1;         								//Increase the parameter value by 1
		}

		if((sw & 0x000C)==(0x0008|0x000C)){      				//Check if switches[3:2] are 10 or 11
			update_val=1;        								//Update the value of proportional constant kp
		}
		else if((sw & 0x000C)==0x0004){         				//Check if switches[3:2] are 01
			update_val=2;           							//Update the value of integral constant ki
		}
		else if((sw & 0x000C)==0x0000){          				//Check if switches[3:2] are 00
			update_val=3;        								//Update the value of derivative constant kd
		}


		//check BTND and BTNU count if it's pressed
		//update the KP,PI,KD
		//BTNU to increase the Value and BTND to decrease the Value

		if (NX4IO_isPressed(BTNU))
		{
			if(update_val==1)KP=KP+param_var;
			if(update_val==2)KI=KI+param_var;
			if(update_val==3)KD=KD+param_var;
			xInputs.Kp=KP;
			xInputs.Ki=KI;
			xInputs.Kd=KD;

		}
		if (NX4IO_isPressed(BTND)) {
			if(update_val==1) KP=KP-param_var;
			if(update_val==2) KI=KI-param_var;
			if(update_val==3) KD=KD-param_var;
			xInputs.Kp=KP;
			xInputs.Ki=KI;
			xInputs.Kd=KD;

		}

		//if BTNC is pressed reset the all parameters
		if (NX4IO_isPressed(BTNC)) {
			KP=0;
			KD=0;
			KI=0;
			speed=0;
		}

		xInputs.switches = sw;
		xInputs.Speed = speed;
		//Send Input parameter to display
		xQueueSend( xQueue1, &xInputs, mainDONT_BLOCK);
		vTaskDelay(15);

		//Check ENC Switch
		//Change the direction of motor if it is Turn on ->(Counter Clock wise)
		// else Clock wise
		state = ENC_getState(&pmodENC_inst);

		if(ENC_switchOn(state) == 0){
			XGpio_DiscreteWrite( &GPIOInst0, GPIO_0_OUTPUT_0_CHANNEL, 0);
		}
		else{
			XGpio_DiscreteWrite( &GPIOInst0, GPIO_0_OUTPUT_0_CHANNEL, 1);
		}

		xInputs2.Kp=KP;
		xInputs2.Ki=KI;
		xInputs2.Kd=KD;
		xInputs2.Speed = speed;

		//Send Input parameters to PID algorithm for calculation of motor speed
		xQueueSend( xQueue2, &xInputs2, mainDONT_BLOCK);

		//Assign new values to previous value
		laststate = state;
		lastticks = ticks;

	}
}


//adc_input thread to read the GPIO port and calculate the RPM
//Send RPM to PID_thread for calculation
void adc_input_thread(void *p){

	//Local variable
	float count,freq_count= 0;
	int prev =0;
	while(1)
	{
		//Check for High to low and Low to high detection of signal
		//Read GPIO port for signal detection
		if(XGpio_DiscreteRead(&GPIOInst1, GPIO_1_INPUT_0_CHANNEL) == 0 && prev == 1)
		{
			freq_count = count;
			xInputs3.freq = 60*12/(freq_count*0.85*0.001); 					//Calculate the frequency of sensor pulses(RPM)
			xQueueSend( xQueue4, &xInputs3, mainDONT_BLOCK);				//Send RPM to PID thread
			vTaskDelay(10);
			count = 0;														//Reset the counter
			prev = 0;														//Set Previous value to 0
			vTaskDelay(1);
		}
		else
		{
			prev = XGpio_DiscreteRead(&GPIOInst1, GPIO_1_INPUT_0_CHANNEL);	//Read GPIO port for signal detection
			count++;														//increment the counter
		}
	}
}

//PID_algorithm to calculate the error
float pid_algorithm(pid_struct *pid, int target){

	pid_struct pid_param;
	//Local variable
	int i = 0,d = 0 ;
	int error = 0,preverr = 0;
	int sensor_input;
	float output1;
	pid_param = *pid;

	//Receive RPM from adc_input thread
	xQueueReceive( xQueue4, &xInputs3, portMAX_DELAY );
	sensor_input = xInputs3.freq;
	error = target - sensor_input;      								//Calculate the error
	d = error - preverr;          										//Calculate the value for derivative
	preverr = error;
	if (error <(target/10))
	{
		i = i + error;                								//Calculate the integral
	}
	else{
		i = 0;
	}
	output1 = (pid_param.kp * error) + (pid_param.kd * d) + (pid_param.ki * i);     //calculate the output.
	return output1;
}

//set_PWM to set the PWM to drive motor
void set_PWM(int pwm_duty)
{
	PWM_SetParams(&PWMTimerInst, 4000, pwm_duty);        //Set the TCR register of Timer
	PWM_Start(&PWMTimerInst);             				 //Start PWM

}

//
void pid_thread(void *p)
{
	//Initialize PID_stuct values to 0
	pid_struct pid_param;
	pid_param.kp = 0;
	pid_param.ki = 0;
	pid_param.kd = 0;
	pid_param.mtr_speed = 0;

	//Variable for mapping of duty cycle
	//Initialize all
	int input_start=1;
	int input_end=255;
	int output_end=98;
	int output_start=65;
	int duty_cycle=0;
	//Local variable
	int target=0,output=0;
	int mtr_speed =0;
	int offset=0;

	while(1){

		xQueueReceive( xQueue2, &xInputs2, portMAX_DELAY );    		//Get the data from the parameter_input_thread

		//assign values to PID_thread
		pid_param.kp = xInputs2.Kp;
		pid_param.ki = xInputs2.Ki;
		pid_param.kd = xInputs2.Kd;
		target = xInputs2.Speed;
		output = pid_algorithm(&pid_param,target);       			//Call PID algorithm function
		mtr_speed = output + offset;               					// Calculate the desired motor speed
		if(mtr_speed <1){                             				 //Checks to keep the speed between PWM module range
		mtr_speed = 1;
		}
		else if(mtr_speed >254){
		mtr_speed = 254;
		}

		//Map the motor speed (1-255) to duty cycle  (60-100)
		duty_cycle= output_start + ((output_end - output_start) / (input_end - input_start)) * (mtr_speed - input_start);
		//Set PWM to Map duty cycle
		set_PWM(duty_cycle);

	}
}

//Display all the Outputs
void display_thread(void *p)
{
	while(1)
	{
		//Receive the INPUT calues from parameter thread to display
		xQueueReceive(xQueue1, &xInputs, portMAX_DELAY);

		//Turn on the LEDs[15:0]
		NX4IO_setLEDs(xInputs.switches);

		//Display KP,KI,KD (PID-values) and targeted speed on PMODOLEDrgb
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 0);
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"     ");
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 0);
		PMDIO_putnum(&pmodOLEDrgb_inst,xInputs.Kp, 10);

		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 1);
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"     ");
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 1);
		PMDIO_putnum(&pmodOLEDrgb_inst,xInputs.Kd, 10);

		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 2);
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"     ");
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 2);
		PMDIO_putnum(&pmodOLEDrgb_inst,xInputs.Ki, 10);

		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 3);
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"     ");
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 3);
		PMDIO_putnum(&pmodOLEDrgb_inst,xInputs.Speed, 10);

	}
}


int main(void)
{
	uint32_t sts;

	sts = do_init();
	if (XST_SUCCESS != sts)
	{
		exit(1);
	}
	//Set LEDs[15:0]
	NX4IO_setLEDs(0x0000);

	// Set up the display output
	OLEDrgb_Clear(&pmodOLEDrgb_inst);
	OLEDrgb_SetFontColor(&pmodOLEDrgb_inst,OLEDrgb_BuildRGB(200, 12, 44));
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 0);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"KP:");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 1);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"KI:");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 2);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"KD:");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 3);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"SP:");

	/* Create the queue */
	xQueue1 = xQueueCreate( mainQUEUE_LENGTH, sizeof(u32) );
	xQueue2 = xQueueCreate( mainQUEUE_LENGTH, sizeof( u32 ) );
	xQueue3 = xQueueCreate( mainQUEUE_LENGTH, sizeof( u32) );
	xQueue4 = xQueueCreate( mainQUEUE_LENGTH, sizeof( u32) );

	/* Sanity check that the queue was created. */
	configASSERT(xQueue1);
	configASSERT(xQueue2);
	configASSERT(xQueue3);
	configASSERT(xQueue4);

	//Create Task1 for Input parameters
	xTaskCreate( parameter_input_thread,
				 ( const char * ) "Parameter",
				 2048,
				 NULL,
				 1,
				 NULL );

	//Create Task2 to display all parameters
	xTaskCreate( display_thread,
			( const char * )"Display",
				2048,
				NULL,
				1,
				NULL );

	//Create Task 3 for PID algorithm
	xTaskCreate( pid_thread,
			( const char * )"Pid",
				2048,
				NULL,
				1,
				NULL );

	//Create Task 4 for signal detection of hall sensor
	xTaskCreate( adc_input_thread,
				( const char * )"Adc",
					2048,
					NULL,
					2,
					NULL );

	//Start the Scheduler
	vTaskStartScheduler();
	return -1;
}



/**************************** HELPER FUNCTIONS ******************************/

/****************************************************************************/
/**
* initialize the system
*
* This function is executed once at start-up and after resets.  It initializes
* the peripherals and registers the interrupt handler(s)
*****************************************************************************/

int	 do_init(void)
{
	uint32_t status;				// status from Xilinx Lib calls

	// initialize the Nexys4 driver and (some of)the devices
	status = (uint32_t) NX4IO_initialize(NX4IO_BASEADDR);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// set all of the display digits to blanks and turn off
	// the decimal points using the "raw" set functions.
	// These registers are formatted according to the spec
	// and should remain unchanged when written to Nexys4IO...
	// something else to check w/ the debugger when we bring the
	// drivers up for the first time
	NX4IO_SSEG_setSSEG_DATA(SSEGHI, 0x0058E30E);
	NX4IO_SSEG_setSSEG_DATA(SSEGLO, 0x00144116);

	NX4IO_setLEDs(0x00000000);

	OLEDrgb_begin(&pmodOLEDrgb_inst, RGBDSPLY_GPIO_BASEADDR, RGBDSPLY_SPI_BASEADDR);

	// initialize the pmodENC and hardware
	ENC_begin(&pmodENC_inst, PMODENC_BASEADDR);

	// initialize the GPIO instances
	status = XGpio_Initialize(&GPIOInst0, GPIO_0_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}
	// GPIO0 channel 1 is an 8-bit input port.
	// GPIO0 channel 2 is an 8-bit output port.
	XGpio_SetDataDirection(&GPIOInst0, GPIO_0_OUTPUT_0_CHANNEL, 0x00);
	XGpio_SetDataDirection(&GPIOInst0, GPIO_0_OUTPUT_1_CHANNEL, 0x00);

	status = XGpio_Initialize(&GPIOInst1, GPIO_1_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}
	// GPIO1 channel 1 is an 32-bit input port.
	// GPIO1 channel 2 is an 32-bit output port.
	XGpio_SetDataDirection(&GPIOInst1, GPIO_1_INPUT_0_CHANNEL, 0xFFFFFFFF);
	XGpio_SetDataDirection(&GPIOInst1, GPIO_1_INPUT_1_CHANNEL, 0xFFFFFFFF);


	status = XGpio_Initialize(&GPIOInst2, GPIO_2_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}
	// GPIO1 channel 1 is an 32-bit input port.
	// GPIO1 channel 2 is an 32-bit output port.
	XGpio_SetDataDirection(&GPIOInst2, GPIO_2_INPUT_0_CHANNEL, 0xFFFFFFFF);
	XGpio_SetDataDirection(&GPIOInst2, GPIO_2_INPUT_1_CHANNEL, 0xFFFFFFFF);

	// initialize the PWM timer/counter instance but do not start it
	// do not enable PWM interrupts.  Clock frequency is the AXI clock frequency
	status = PWM_Initialize(&PWMTimerInst, PWM_TIMER_DEVICE_ID, false, AXI_CLOCK_FREQ_HZ);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	//AXI timer initialization
	status = AXI_Timer_initialize();
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	status = AXI_Timer_1_initialize();
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// initialize the interrupt controller
	status = XIntc_Initialize(&IntrptCtlrInst, INTC_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
	   return XST_FAILURE;
	}

	// connect the fixed interval timer (FIT) handler to the interrupt
	status = XIntc_Connect(&IntrptCtlrInst, FIT_INTERRUPT_ID,
						   (XInterruptHandler)FIT_Handler,
						   (void *)0);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;

	}

	//Watchdog timer Initialization
	 status = XWdtTb_Initialize(&Watchdog, WATCHDOG_TIMER_ID);
	 if (status != XST_SUCCESS)
	 {
		 return XST_FAILURE;
	  }

	// start the interrupt controller such that interrupts are enabled for
	// all devices that cause interrupts.
	status = XIntc_Start(&IntrptCtlrInst, XIN_REAL_MODE);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// enable the FIT interrupt
	XIntc_Enable(&IntrptCtlrInst, FIT_INTERRUPT_ID);
	return XST_SUCCESS;
}


/*
 * AXI timer initializes it to generate out a 4Khz signal, Which is given to the Nexys4IO module as clock input.
 * DO NOT MODIFY
 *
 */
int AXI_Timer_initialize(void){

	uint32_t status;				// status from Xilinx Lib calls
	u32		ctlsts;		// control/status register or mask

	status = XTmrCtr_Initialize(&AXITimerInst,AXI_TIMER_0_DEVICE_ID);
		if (status != XST_SUCCESS) {
			return XST_FAILURE;
		}
	status = XTmrCtr_SelfTest(&AXITimerInst, TmrCtrNumber);
		if (status != XST_SUCCESS) {
			return XST_FAILURE;
		}
	ctlsts = XTC_CSR_AUTO_RELOAD_MASK | XTC_CSR_EXT_GENERATE_MASK | XTC_CSR_LOAD_MASK |XTC_CSR_DOWN_COUNT_MASK ;
	XTmrCtr_SetControlStatusReg(AXI_TIMER_0_BASEADDR, TmrCtrNumber,ctlsts);

	//Set the value that is loaded into the timer counter and cause it to be loaded into the timer counter
	XTmrCtr_SetLoadReg(AXI_TIMER_0_BASEADDR, TmrCtrNumber, 24998);
	XTmrCtr_LoadTimerCounterReg(AXI_TIMER_0_BASEADDR, TmrCtrNumber);
	ctlsts = XTmrCtr_GetControlStatusReg(AXI_TIMER_0_BASEADDR, TmrCtrNumber);
	ctlsts &= (~XTC_CSR_LOAD_MASK);
	XTmrCtr_SetControlStatusReg(AXI_TIMER_0_BASEADDR, TmrCtrNumber, ctlsts);

	ctlsts = XTmrCtr_GetControlStatusReg(AXI_TIMER_0_BASEADDR, TmrCtrNumber);
	ctlsts |= XTC_CSR_ENABLE_TMR_MASK;
	XTmrCtr_SetControlStatusReg(AXI_TIMER_0_BASEADDR, TmrCtrNumber, ctlsts);

	XTmrCtr_Enable(AXI_TIMER_0_BASEADDR, TmrCtrNumber);
	return XST_SUCCESS;

}

/*
 * AXI timer initializes it to generate out a 4Khz signal, Which is given to the Nexys4IO module as clock input.
 * DO NOT MODIFY
 */

int AXI_Timer_2_initialize(void){

	uint32_t status;
	u32		ctlsts;

	status = XTmrCtr_Initialize(&AXITimerInst2,AXI_TIMER_2_DEVICE_ID);
		if (status != XST_SUCCESS) {
			return XST_FAILURE;
		}
	status = XTmrCtr_SelfTest(&AXITimerInst2, TmrCtrNumber_2);
		if (status != XST_SUCCESS) {
			return XST_FAILURE;
		}
	ctlsts = XTC_CSR_AUTO_RELOAD_MASK | XTC_CSR_EXT_GENERATE_MASK | XTC_CSR_LOAD_MASK |XTC_CSR_DOWN_COUNT_MASK ;
	XTmrCtr_SetControlStatusReg(AXI_TIMER_2_BASEADDR, TmrCtrNumber_2,ctlsts);

	XTmrCtr_SetLoadReg(AXI_TIMER_2_BASEADDR, TmrCtrNumber_2, 24998);
	XTmrCtr_LoadTimerCounterReg(AXI_TIMER_2_BASEADDR, TmrCtrNumber_2);
	ctlsts = XTmrCtr_GetControlStatusReg(AXI_TIMER_2_BASEADDR, TmrCtrNumber_2);
	ctlsts &= (~XTC_CSR_LOAD_MASK);
	XTmrCtr_SetControlStatusReg(AXI_TIMER_2_BASEADDR, TmrCtrNumber_2, ctlsts);

	ctlsts = XTmrCtr_GetControlStatusReg(AXI_TIMER_2_BASEADDR, TmrCtrNumber_2);
	ctlsts |= XTC_CSR_ENABLE_TMR_MASK;
	XTmrCtr_SetControlStatusReg(AXI_TIMER_2_BASEADDR, TmrCtrNumber_2, ctlsts);

	XTmrCtr_Enable(AXI_TIMER_2_BASEADDR, TmrCtrNumber_2);
	return XST_SUCCESS;

}

/*
 * AXI timer initializes it to generate out a 4Khz signal, Which is given to the Nexys4IO module as clock input.
 * DO NOT MODIFY
 */
int AXI_Timer_1_initialize(void){

	uint32_t status;
	u32		ctlsts;

	status = XTmrCtr_Initialize(&AXITimerInst1,AXI_TIMER_1_DEVICE_ID);
		if (status != XST_SUCCESS) {
			return XST_FAILURE;
		}
	status = XTmrCtr_SelfTest(&AXITimerInst1, TmrCtrNumber_1);
		if (status != XST_SUCCESS) {
			return XST_FAILURE;
		}
	ctlsts = XTC_CSR_AUTO_RELOAD_MASK | XTC_CSR_EXT_GENERATE_MASK | XTC_CSR_LOAD_MASK |XTC_CSR_DOWN_COUNT_MASK ;
	XTmrCtr_SetControlStatusReg(AXI_TIMER_1_BASEADDR, TmrCtrNumber_1,ctlsts);

	XTmrCtr_SetLoadReg(AXI_TIMER_1_BASEADDR, TmrCtrNumber_1, 24998);
	XTmrCtr_LoadTimerCounterReg(AXI_TIMER_1_BASEADDR, TmrCtrNumber_1);
	ctlsts = XTmrCtr_GetControlStatusReg(AXI_TIMER_1_BASEADDR, TmrCtrNumber_1);
	ctlsts &= (~XTC_CSR_LOAD_MASK);
	XTmrCtr_SetControlStatusReg(AXI_TIMER_1_BASEADDR, TmrCtrNumber_1, ctlsts);

	ctlsts = XTmrCtr_GetControlStatusReg(AXI_TIMER_1_BASEADDR, TmrCtrNumber_1);
	ctlsts |= XTC_CSR_ENABLE_TMR_MASK;
	XTmrCtr_SetControlStatusReg(AXI_TIMER_1_BASEADDR, TmrCtrNumber_1, ctlsts);

	XTmrCtr_Enable(AXI_TIMER_1_BASEADDR, TmrCtrNumber_1);
	return XST_SUCCESS;

}


/*********************** DISPLAY-RELATED FUNCTIONS ***********************************/

/****************************************************************************/
/**
* Converts an integer to ASCII characters
*
* algorithm borrowed from ReactOS system libraries
*
* Converts an integer to ASCII in the specified base.  Assumes string[] is
* long enough to hold the result plus the terminating null
*
* @param 	value is the integer to convert
* @param 	*string is a pointer to a buffer large enough to hold the converted number plus
*  			the terminating null
* @param	radix is the base to use in conversion,
*
* @return  *NONE*

*****************************************************************************/
void PMDIO_itoa(int32_t value, char *string, int32_t radix)
{
	char tmp[33];
	char *tp = tmp;
	int32_t i;
	uint32_t v;
	int32_t  sign;
	char *sp;

	if (radix > 36 || radix <= 1)
	{
		return;
	}

	sign = ((10 == radix) && (value < 0));
	if (sign)
	{
		v = -value;
	}
	else
	{
		v = (uint32_t) value;
	}

  	while (v || tp == tmp)
  	{
		i = v % radix;
		v = v / radix;
		if (i < 10)
		{
			*tp++ = i+'0';
		}
		else
		{
			*tp++ = i + 'a' - 10;
		}
	}
	sp = string;

	if (sign)
		*sp++ = '-';

	while (tp > tmp)
		*sp++ = *--tp;
	*sp = 0;

  	return;
}


/****************************************************************************/
/**
* Write a 32-bit unsigned hex number to PmodOLEDrgb in Hex
*
* Writes  32-bit unsigned number to the pmodOLEDrgb display starting at the current
* cursor position.
*
* @param num is the number to display as a hex value
*
* @return  *NONE*

*****************************************************************************/

void PMDIO_puthex(PmodOLEDrgb* InstancePtr, uint32_t num)
{
  char  buf[9];
  int32_t   cnt;
  char  *ptr;
  int32_t  digit;

  ptr = buf;
  for (cnt = 7; cnt >= 0; cnt--) {
    digit = (num >> (cnt * 4)) & 0xF;

    if (digit <= 9)
	{
      *ptr++ = (char) ('0' + digit);
	}
    else
	{
      *ptr++ = (char) ('a' - 10 + digit);
	}
  }

  *ptr = (char) 0;
  OLEDrgb_PutString(InstancePtr,buf);

  return;
}


/****************************************************************************/
/**
* Write a 32-bit number in Radix "radix" to LCD display
*
* Writes a 32-bit number to the LCD display starting at the current
* cursor position. "radix" is the base to output the number in.
*
* @param num is the number to display
*
* @param radix is the radix to display number in
*
* @return *NONE*

*****************************************************************************/
void PMDIO_putnum(PmodOLEDrgb* InstancePtr, int32_t num, int32_t radix)
{
  char  buf[16];

  PMDIO_itoa(num, buf, radix);
  OLEDrgb_PutString(InstancePtr,buf);

  return;
}



void FIT_Handler(void)
{
	// Read the GPIO port to read back the generated PWM signal for RGB led's
	gpio_in = XGpio_DiscreteRead(&GPIOInst2, GPIO_2_INPUT_0_CHANNEL);

}
