#include <avr/io.h>

//FreeRTOS include files
#include "FreeRTOS.h"
#include "task.h"
#include "croutine.h"
#include "lcd.h"
#include "usart_ATmega1284.h"

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

enum Lights2State{init,saimureceiver} lights2_state;

void Lights2_Init(){
	lights2_state = init;
}

char inferior = 0x00;

void Lights2(){
	//Actions
	switch(lights2_state){
		case init:
			initUSART(0);
			USART_Flush(0);
			PORTB = 0x00;
			Set_A2D_Pin(0);
			set_PWM(0);
		break;
		
		case saimureceiver:
			if(USART_HasReceived(0)) {
				if(USART_Receive(0) == 0x00) {
					set_PWM(0);
				} else set_PWM(ADC/4);
				USART_Flush(0);
			}
			
		break;
				
		default:
			set_PWM(0);
			PORTB = 0x00;
		break;
	}
	//Transitions
	switch(lights2_state) {
		case init:
			lights2_state = saimureceiver;
		break;
		
		case saimureceiver:
			lights2_state = saimureceiver;
		break;
				
		default:
			lights2_state = init;
		break;
	}
}

void Lights2_Task()
{
	Lights2_Init();
	for(;;)
	{
		Lights2();
		vTaskDelay(100);
	}
}


void StartSecPulse(unsigned portBASE_TYPE Priority)
{
	xTaskCreate(Lights2_Task, (signed portCHAR *)"Lights2_Task", configMINIMAL_STACK_SIZE, NULL, Priority, NULL );
}

int main(void)
{
	DDRB = 0xFF; PORTB = 0x00;
	DDRC = 0xFF; PORTC = 0x00;       //outputs
	DDRA = 0x00; PORTA = 0x00;      //inputs
	A2D_init();
	Set_A2D_Pin(0);
	PWM_on();
	//Start Tasks
	StartSecPulse(1);
	//RunSchedular
	vTaskStartScheduler();
	
	return 0;
}












// 
// // A PWM example for the ATmega328P using the 8-Bit Fast PWM mode.
// #include <avr/io.h>
// #include <avr/interrupt.h>
// #include <stdbool.h>
// #include <util/delay.h>
// 
// int main (void) {
// 
// 	/**
// 	 * We will be using OCR1A as our PWM output which is the
// 	 * same pin as PB1.
// 	 */
// 	DDRD = 0xFF;
// 	DDRB = 0xFF;
// 
// 	/**
// 	 * There are quite a number of PWM modes available but for the
// 	 * sake of simplicity we'll just use the 8-bit Fast PWM mode.
// 	 * This is done by setting the WGM10 and WGM12 bits.  We 
// 	 * Setting COM1A1 tells the microcontroller to set the 
// 	 * output of the OCR1A pin low when the timer's counter reaches
// 	 * a compare value (which will be explained below).  CS10 being
// 	 * set simply turns the timer on without a prescaler (so at full
// 	 * speed).  The timer is used to determine when the PWM pin should be
// 	 * on and when it should be off.
// 	 */
// 	TCCR1A = (1<<COM1A1) | (1<<WGM10);
// 	TCCR1B = (1<<CS10) | (1<<WGM12);
// 
// 	/**
// 	 *  This loop is used to change the value in the OCR1A register.
// 	 *  What that means is we're telling the timer waveform generator
// 	 *  the point when it should change the state of the PWM pin.
// 	 *  The way we configured it (with _BV(COM1A1) above) tells the
// 	 *  generator to have the pin be on when the timer is at zero and then
// 	 *  to turn it off once it reaches the value in the OCR1A register.
// 	 *
// 	 *  Given that we are using an 8-bit mode the timer will reset to zero
// 	 *  after it reaches 0xff, so we have 255 ticks of the timer until it
// 	 *  resets.  The value stored in OCR1A is the point within those 255
// 	 *  ticks of the timer when the output pin should be turned off
// 	 *  (remember, it starts on).
// 	 *
// 	 *  Effectively this means that the ratio of pwm / 255 is the percentage
// 	 *  of time that the pin will be high.  Given this it isn't too hard
// 	 *  to see what when the pwm value is at 0x00 the LED will be off
// 	 *  and when it is 0xff the LED will be at its brightest.
// 	 */
// 	uint8_t pwm = 0x00;
// 	bool up = true;
// 	for(;;) {
// 
// 		OCR1A = pwm;
// 
// 		pwm += up ? 1 : -1;
// 		if (pwm == 0xff)
// 			up = false;
// 		else if (pwm == 0x00)
// 			up = true;
// 
// 		_delay_ms(10);
// 	}
// 
// }

