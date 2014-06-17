/**
 * @mainpage package Collaborative_Cleaner
 * @author Group 11: Chinmay Vaishampayan(09305918), Jitendra Sahu(09305059), Ashutosh Patel(09305003)
 */

/********************************************************************************

   Copyright (c) 2010, ERTS Lab IIT Bombay erts@cse.iitb.ac.in               -*- c -*-
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

   * Source code can be used for academic purpose. 
	 For commercial use permission form the author needs to be taken.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. 

  Software released under Creative Commence cc by-nc-sa licence.
  For legal information refer to: 
  http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode

********************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lcd.c"
#include "motion.c"

unsigned int rotate = 0; //!< enable/disable rotation of servo motor
unsigned int sendDistance=0; //!< enable robot to send the distance to matlab
unsigned int sendAngle=0; //!< enable robot to send the angle to matlab
//unsigned int recvOK=0;
//unsigned int distance=0;
unsigned int prevI=0; //!< used to save the angle from which servo motor starts rotating
unsigned int saveNumber=1000; //!<save the number which comes after 'n' signal
unsigned int phase=0; //!<locate/clean
unsigned char ADC_Conversion(unsigned char);
unsigned char prevSignal='~'; //!< saves what was the previous signal

//for measuring distance
void adc_pin_config (void)
{
	DDRF = 0x00; 
	PORTF = 0x00;
	DDRK = 0x00;
	PORTK = 0x00;
}

void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

unsigned char ADC_Conversion(unsigned char Ch)
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;			// select the ch. > 7
	}
	Ch = Ch & 0x07;  			
	ADMUX= 0x20| Ch;	   		//do not disturb the left adjustment
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for ADC conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; 		//clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}

unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading)
{
	float distance;
	unsigned int distanceInt;
	distance = (int)(10.00*(2799.6*(1.00/(pow(adc_reading,1.1546)))));
	distanceInt = (int)distance;
	if(distanceInt>800)
	{
		distanceInt=800;
	}
	return distanceInt;
}

//for showing distance ON LCD
void lcd_port_config (void)
{
	DDRC = DDRC | 0xF7; 	//all the LCD pin's direction set as output
	PORTC = PORTC & 0x80; 	// all the LCD pins are set to logic 0 except PORTC 7
}


//for zigbee communication
void uart0_init(void)
{
 UCSR0B = 0x00; //disable while setting baud rate
 UCSR0A = 0x00;
 UCSR0C = 0x06;
 UBRR0L = 0x47; //set baud rate lo
 UBRR0H = 0x00; //set baud rate hi
 UCSR0B = 0x98;
}

//Configure PORTB 5 pin for servo motor 1 operation
void servo1_pin_config (void)
{
	DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
	PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}


//Initialize the ports
void port_init(void)
{
	servo1_pin_config(); //Configure PORTB 5 pin for servo motor 1 operation
	adc_pin_config();	
	lcd_port_config();
	motion_pin_config(); //robot motion pins config
//	left_encoder_pin_config(); //left encoder pin config
// 	right_encoder_pin_config(); //right encoder pin config	

}

//TIMER1 initialization in 10 bit fast PWM mode  
//prescale:256
// WGM: 7) PWM 10bit fast, TOP=0x03FF
// actual value: 42.187Hz 
void timer1_init(void)
{
	TCCR1B = 0x00; //stop
	TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
	TCNT1L = 0x01;	//Counter low value to which OCR1xH value is to be compared with
	OCR1AH = 0x03;	//Output compare eegister high value for servo 1
	OCR1AL = 0xFF;	//Output Compare Register low Value For servo 1
	OCR1BH = 0x03;	//Output compare eegister high value for servo 2
	OCR1BL = 0xFF;	//Output Compare Register low Value For servo 2
	OCR1CH = 0x03;	///Output compare eegister high value for servo 3
	OCR1CL = 0xFF;	//Output Compare Register low Value For servo 3
	ICR1H  = 0x03;	
	ICR1L  = 0xFF;
	TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
					 For Overriding normal port functionalit to OCRnA outputs.
					 {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
	TCCR1C = 0x00;
	TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}


//Function to initialize all the peripherals
void init_devices(void)
{
	cli(); //disable all interrupts
	port_init();
	timer1_init();
	adc_init();
	uart0_init(); 
//	left_position_encoder_interrupt_init();
// 	right_position_encoder_interrupt_init();
	sei(); //re-enable interrupts 
}


//Function to rotate Servo 1 by a specified angle in the multiples of 2.25 degrees
void servo_1(unsigned char degrees)  
{
	float PositionPanServo = 0;
	PositionPanServo = ((float)degrees / 2.25) + 21.0;
	OCR1AH = 0x00;
	OCR1AL = (unsigned char) PositionPanServo;
}


/** 
 * Servo_free functions unlocks the servo motors from the any angle 
 * and make them free by giving 100% duty cycle at the PWM. This function can be used to 
 * reduce the power consumption of the motor if it is holding load against the gravity.
 */

void servo_1_free (void) //makes servo 1 free rotating
{
	OCR1AH = 0x03; 
	OCR1AL = 0xFF; //Servo 1 off
}

/**
 * Sends the number through zigbee. 
 * When it is sending the number, matlab may or may not execute read command so sent 50 times.
 * @param number  - unsigned number to be sent
 */
void sendNumber(unsigned int number)
{
	int i;
	for(i=0; i<50; ++i) {
		UDR0 = number;
		_delay_ms(10);
	}
}

/**
 * Receives the data through zigbee.
 * Automatically called.
 */
SIGNAL(SIG_USART0_RECV)
{
	unsigned char signal = '|'; ///< save received in signal variable
	signal = UDR0;
	/*
	 * if previous signal was 'n' then a number will come now.
	 */
	if (prevSignal == 'n') {
		prevSignal = '~';
		saveNumber = signal; //if prev signal was n, the number is coming
		signal = '|';
	}

	/*
	 * if-else-if ladder decides what to do on which signal.
	 * 'a' = command for robot to send angle
	 * 'd' = command for robot to send distance
	 * 'f' = move forward continously, testing purpose only
	 * 'h' = halt robot
	 * 'm' = set servo motor at specifed angle (sent in previous signal)
	 * 'n' = means whatever comes next will be a number
	 * 'r' = start rotation
	 * 'q' = reset prevI, the angle from which motor starts rotating
	 * '2/4/5/6/8' = navigation keys
	 * '1/3' = soft left/right rotation
	 * '7/9' = hard left/right rotation
	 */
	if (signal == 's') { //stop rotation
		rotate = 0;
	} else if (signal == 'r') { //start rotation
		rotate = 1;
	} else if (signal == 'a') { // continue sending angle
		sendAngle = 1;
	} else if (signal == 'd') { // continue sending next
		sendDistance = 1;
	} else if (signal == 'q') { // reset
		prevI = 0;
	} else if (signal == 'f') { //forward
		motion_pin_config();
		forward();
	} else if (signal == 'h') { //halt
		motion_pin_config();
		stop();
	} else if (signal == 'n') { //number will come next
			prevSignal = 'n';
			saveNumber = 1000;
	} else if (signal == 'p') {
		if(saveNumber < 800) {
			phase = saveNumber;
			saveNumber = 1000;
		}
	} else if (signal == 'm') {
		if(saveNumber < 800) {
			servo_1(saveNumber);
			saveNumber = 1000;
		}
	} else if (signal == '1') {
		if(saveNumber < 800) {
			motion_pin_config();
			turn_soft_left(saveNumber);
			saveNumber = 1000;
		}
	} else if (signal == '2') {
		if(saveNumber < 800) {
			motion_pin_config();
			move_backward(saveNumber*10);
			saveNumber = 1000;
		}
	} else if (signal == '3') {
		if(saveNumber < 800) {
			motion_pin_config();
			turn_soft_right(saveNumber);
			saveNumber = 1000;
		}
	} else if (signal == '4') {
		if(saveNumber < 800) {
			motion_pin_config();
			turn_hard_right(90);
			motion_pin_config();
			move_forward(saveNumber*10);
			saveNumber = 1000;
		}
	} else if (signal == '5') {
		if(saveNumber < 800) {
			motion_pin_config();
			stop();
			saveNumber = 1000;
		}
	} else if (signal == '6') {
		if(saveNumber < 800) {
			motion_pin_config();
			turn_hard_right(90);
			motion_pin_config();
			move_forward(saveNumber*10);
			saveNumber = 1000;
		}
	} else if (signal == '7') {
		if(saveNumber < 800) {
			motion_pin_config();
			turn_hard_left(saveNumber);
			saveNumber = 1000;
		}
	} else if (signal == '8') {
		if(saveNumber < 800) {
			motion_pin_config();
			move_forward(saveNumber*10);
			saveNumber = 1000;
		}
	} else if (signal == '9') {
		if(saveNumber < 800) {
			motion_pin_config();
			turn_hard_right(saveNumber);
			saveNumber = 1000;
		}
	}
}

/**
 * Function called using matlab, used to detect red/green/blue poles.
 */
int locate()
{
	unsigned char i = 0, sharp;
	unsigned int value=0, cm;
	if (rotate == 1) {
		for (i = prevI; i < 180; i++) {
			servo_1(i);
			_delay_ms(300);
			sharp = ADC_Conversion(11); //Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
			value = Sharp_GP2D12_estimation(sharp); //Stores Distance calculated in a variable "value".
			lcd_print(2,1,value,3);
			lcd_print(2,6,i,3);
			if (rotate == 0) {
				prevI = i;
				/*
				 * Distance measured by sharp sensor is in mm, convert to cm for sending send.
				 * for example 230 is send as 23; 233 as 230; 238 as 24 so max error is 5 mm
				 */
				cm = value/10;
				if (value%cm > 5) {
					cm += 1;
				}
				while (sendDistance==0); //wait till matlab is ready to accept distance
				sendNumber(cm);
				sendDistance = 0;
				while (sendAngle==0); // wait till matlab is ready to receive angle.
				sendNumber(i);
				sendAngle = 0; // disable sendNext, 
				i=180; //any random number to get us out of for loop
			}
			if (i == 179) {
				prevI = 0;
			}
		}
	_delay_ms(500);
	}
}

/*
 * Main function, will call different phases, locate/clean
 */
int main(void)
{
	unsigned char sharp;
	unsigned int value=0;
	init_devices();
	lcd_set_4bit();
	lcd_init();
	lcd_wr_command(0x0C);// Display ON Cursor OFF
	while(1) {
		if (phase == 1) {
			locate();	
		}
		sharp = ADC_Conversion(11);	//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
		value = Sharp_GP2D12_estimation(sharp); //Stores Distance calsulated in a variable "value".
		lcd_print(2,1,value,3);
	}
	servo_1_free();
	return 0;
}
