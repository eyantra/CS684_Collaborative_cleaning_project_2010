#include<avr/io.h>
#include<util/delay.h>
#include<avr/interrupt.h>

unsigned int rightShaftCount = 0;
unsigned int leftShaftCount = 0;

ISR(INT4_vect)
{
	leftShaftCount++;
}

ISR(INT5_vect)
{
	rightShaftCount++;
}

void motion_pin_config()
{
	//init motion control ports
	DDRA = 0x0F; // 0000 1111, lower 4 bits are output
	PORTA = 0x00; // initialize
	
	//init  velocity control ports
	DDRL = 0x18; // 0001 1000, 3rd and 4th bit are set for output
	PORTL = 0x18; // initialize

	//init interrupt control ports, 4 is for left, 5 is for right
	cli(); //clear interrupts
	EICRB = EICRB | 0x0A; //for contol register B,4th and 5th interrupts 0000 1010 (falling edge)
	EIMSK = EIMSK | 0x30; //mask for 8 interuts, we use 4 and 5, so 00110000
	sei(); //enable global interrupts

	//init PORTE, PE4 and PE5 is for interrupt
	DDRE = 0xCF; // 4th and 5th are for input, 1100 1111
	PORTE = 0x00; // initialize to zero

	rightShaftCount = 0;
	leftShaftCount = 0;
}

void forward()
{
	PORTA = 0x06;
}

void backward()
{
	PORTA = 0x09;
}

void hard_left()
{
	PORTA = 0x05;
}

void hard_right()
{
	PORTA =  0x0A;
}

void soft_left()
{
	PORTA = 0x04;
}

void soft_right()
{
	PORTA = 0x02;
}


void stop()
{
	PORTA = 0x00;
}

void move_forward(int d) //d in mm
{
	unsigned int reqdLeftShaftCount=0;

	reqdLeftShaftCount = d/5;

	while( leftShaftCount < reqdLeftShaftCount) {
		forward();
	}
	stop();
}

void move_backward(int d) //d in mm
{
	unsigned int reqdLeftShaftCount=0;

	reqdLeftShaftCount = d/5;

	while( leftShaftCount < reqdLeftShaftCount) {
		backward();
	}
	stop();
}
	
void turn_soft_right(int degree)
{
	unsigned int reqdLeftShaftCount = 0;
	reqdLeftShaftCount = degree/2;

	while(leftShaftCount < reqdLeftShaftCount) {
		soft_right();
	}
	stop();
}

void turn_soft_left(int degree)
{
	unsigned int reqdRightShaftCount = 0;
	reqdRightShaftCount = degree/2;

	while(rightShaftCount < reqdRightShaftCount) {
		soft_left();
	}
	stop();
}

void turn_hard_right(int degree)
{
	unsigned int reqdRightShaftCount = 0;
	unsigned int reqdLeftShaftCount = 0;

	reqdRightShaftCount = degree/4;
	reqdLeftShaftCount = degree/4;

	while((rightShaftCount < reqdRightShaftCount) && (leftShaftCount < reqdLeftShaftCount)) {
		hard_right();
	}
	stop();
}

void turn_hard_left(int degree)
{
	unsigned int reqdRightShaftCount = 0;
	unsigned int reqdLeftShaftCount = 0;

	reqdRightShaftCount = degree/4;
	reqdLeftShaftCount = degree/4;

	while((rightShaftCount < reqdRightShaftCount) && (leftShaftCount < reqdLeftShaftCount)) {
		hard_left();
	}
	stop();
}
