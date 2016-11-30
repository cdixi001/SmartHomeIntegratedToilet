#include <avr/io.h>

//FreeRTOS include files
#include "FreeRTOS.h"
#include "task.h"
#include "croutine.h"
#include "lcd.h"
#include "usart_ATmega1284.h"

//http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
unsigned long lastTime;
double ITerm, lastInput;
float kp, ki, kd;
int SampleTime = 1000;   //this should be period of the pid sensor SM thing
char Compute(int Input, int Setpoint)
{
	//How long since we last calculated
	
	/*Compute all the working error variables*/
	int error = Setpoint - Input;                //proportional
	if(ITerm > 100) ITerm = 0;
	ITerm += (ki * error);                 //integral
	int dInput = (Input - lastInput);   //derivative
	
	/*Compute PID Output*/
	char Output = kp * error + ITerm - kd * dInput;
	
	/*Remember some variables for next time*/
	lastInput = Input;
	
	return Output;
}

long errSum = 0;
int lasterror = 0;
char kakapwos(int Input, int Setpoint)
{
	//How long since we last calculated
	
	/*Compute all the working error variables*/
	int error = Setpoint - Input;                //proportional
	errSum += (error * 1000);                 //integral
	double dErr = (error - lasterror);   //derivative
	dErr = dErr/1000;
	
	/*Compute PID Output*/
	char Output = kp * error + ki * errSum + kd*dErr;
	
	/*Remember some variables for next time*/
	lasterror = error;
	
	return Output;
}


void SetTunings(float Kp, float Ki, float Kd)
{
	double SampleTimeInSec = ((double)SampleTime)/1000;
	kp = Kp;
	ki = Ki * SampleTimeInSec;
	kd = Kd / SampleTimeInSec;
}

void SetTunings2(float Kp, float Ki, float Kd)
{
	kp = Kp;
	ki = Ki;
	kd = Kd;
}

void set_PWM(char pwm) {
	//OCR1A = pwm;
	OCR3A = pwm;
}
void PWM_on() {
	//TCCR1A = (1<<COM1A1) | (1<<WGM10);
	TCCR3A = (1 << COM3A1) | (1<<WGM30);
	//TCCR1B = (1<<CS10) | (1<<WGM12);
	TCCR3B = (1<<CS30) | (1<<WGM32);
	set_PWM(0);
}
void PWM_off() {
	//TCCR1A = 0x00;
	TCCR3A = 0x00;
	//TCCR1B = 0x00;
	TCCR3B = 0x00;
}

void A2D_init() {
	ADCSRA |= (1 << ADEN) | (1 << ADSC) | (1 << ADATE);
	// ADEN: Enables analog-to-digital conversion
	// ADSC: Starts analog-to-digital conversion
	// ADATE: Enables auto-triggering, allowing for constant
	//      analog to digital conversions.
}
void Set_A2D_Pin(unsigned char pinNum) {
	ADMUX = (pinNum <= 0x07) ? pinNum : ADMUX;
	// Allow channel to stabilize
	static unsigned char i = 0;
	for ( i=0; i<25; i++ ) { asm("nop"); }
}

int getTemperature(unsigned char pinNum) {
	Set_A2D_Pin(pinNum);
	int celsius = ((ADC * 4.8828) - 550.0) * 10.0;
	int fahrenheit = (celsius * (9.0 / 500.0) + 32.0) * 100;
	return fahrenheit;
}

void inttodisplay(int dasq) {
	LCD_WriteData(dasq/1000 + '0');
	dasq = dasq % 1000;
	LCD_WriteData(dasq/100 + '0');
	LCD_WriteData('.');
	dasq = dasq % 100;
	LCD_WriteData(dasq/10 + '0');
	dasq = dasq % 10;
	LCD_WriteData(dasq + '0');
}


enum DisplayTempState{init,displaytemp, resetpid} displaytemp_state;
int currentTemp = 0;

char pot = 0;
int photo = 0;
char outthing = 0;

void DisplayTemp_Init(){
	displaytemp_state = init;
}

void DisplayTemp(){
	//Actions
	switch(displaytemp_state){
		case init:
			Set_A2D_Pin(0);
			LCD_ClearScreen();
			LCD_DisplayString(1, "Temp:           Phot: ");
			ITerm = 0;
			lastInput = 0;
			//SetTunings(3, 1, 2);
			SetTunings2(1.5, .001, .5);
			PWM_on();
			set_PWM(0);
		break;
		case displaytemp:
			
			Set_A2D_Pin(0);
  			currentTemp = getTemperature(0);
			
			LCD_Cursor(7);
			inttodisplay(currentTemp);
			LCD_WriteData('F');
			
			pot = 200;
			Set_A2D_Pin(1);
			photo = ADC;
			
			photo = ADC/4;		//convert range from 1024 to 256
			
			LCD_Cursor(23);
			inttodisplay(photo);
			
			//outthing = kakapwos(photo, pot);	//fancy legit pid
			
			outthing = outthing + (pot - photo);	//basic
			
			set_PWM(outthing);
		break;
		case resetpid:
			PWM_off();
			ITerm = 0;
			lastInput = 0;
			SetTunings(2, 0, 2);
			PWM_on();
			set_PWM(0);
		break;
	}
	//Transitions
	switch(displaytemp_state){
		case init:
		displaytemp_state = displaytemp;
		break;
		case displaytemp:
				displaytemp_state = displaytemp;
		break;
		case resetpid:
			displaytemp_state = displaytemp;
		default:
		displaytemp_state = init;
		break;
	}
}

void DisplayTemp_Task()
{
	DisplayTemp_Init();
	for(;;)
	{
		DisplayTemp();
		vTaskDelay(1000);
	}
}


enum LightsState{init2,lightson,lightsoff} lights_state;
unsigned short lightsphoto = 0;
unsigned short lightsirdetect = 0;

void Lights_Init(){
	lights_state = init2;
}

void Lights(){
	//Actions
	switch(lights_state){
		case init2:
			initUSART(0);
			USART_Flush(0);
		break;
		case lightsoff:
			
			if(USART_IsSendReady(0)){
				USART_Send(0x00,0);
				if(USART_HasTransmitted(0)) PORTD &= ~(0x10);	//lightsoff
			}
			
			Set_A2D_Pin(3);
			lightsphoto = ADC;
			Set_A2D_Pin(2);
			lightsirdetect = ADC;
			
		break;
		case lightson:
			if(USART_IsSendReady(0)){
				USART_Send(0x01,0);
				if(USART_HasTransmitted(0)) PORTD |= 0x10;	//lightson
			}
			
			Set_A2D_Pin(3);
			lightsphoto = ADC;
			Set_A2D_Pin(2);
			lightsirdetect = ADC;
			
		break;
		default:
			PORTD &= ~(0x10);
		break;
	}
	//Transitions
	switch(lights_state) {
		case init2:
			lights_state = lightsoff;
		break;
		case lightson:
			if(lightsphoto < 350 && lightsirdetect > 750) {
				
				lights_state = lightson;
			} else {
				lights_state = lightsoff;
			}
		break;
		case lightsoff:
			if(lightsphoto < 350 && lightsirdetect > 750) {
					lights_state = lightson;
			} else {
					lights_state = lightsoff;
			}
		break;
		default:
			lights_state = init2;
		break;
	}
}

void Lights_Task()
{
	Lights_Init();
	for(;;)
	{
		Lights();
		vTaskDelay(1500);
	}
}


void StartSecPulse(unsigned portBASE_TYPE Priority)
{
	xTaskCreate(DisplayTemp_Task, (signed portCHAR *)"DisplayTemp_Task", configMINIMAL_STACK_SIZE, NULL, Priority, NULL );
	xTaskCreate(Lights_Task, (signed portCHAR *)"Lights_Task", configMINIMAL_STACK_SIZE, NULL, Priority, NULL );
}

int main(void)
{
	DDRB = 0xFF; PORTB = 0x00;
	DDRD = 0xFF; PORTD = 0x00;       //outputs
	DDRA = 0x00; PORTA = 0x00;      //inputs
	DDRC = 0xFF; PORTC = 0x00;
	A2D_init();
	Set_A2D_Pin(0);
	LCD_init();
	//Start Tasks
	StartSecPulse(1);
	//RunSchedular
	vTaskStartScheduler();
	
	return 0;
}
