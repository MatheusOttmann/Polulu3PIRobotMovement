#include <stdint.h>
#include <stdio.h> 
#include <util/delay.h>
#include <avr/io.h>
#include "lcd_driver.h"
#include "port_macros.h"
#include <stdlib.h>
#define PB_A_BIT 1 // Direction button
#define PB_B_BIT 4 // speedChosen button
#define PB_C_BIT 5 // timerChosen button
#define Left_M1 PD5 //declaring the righr and left motors
#define Left_M2 PD6 //
#define Right_M1 PD3 //
#define Right_M2 PB3 //
#define bool unsigned char // bool function must be declared as it's not in the libraries used
#define INPUT_TIMEOUT 3000 // Time for the FSM to move from one state to another is 5 seconds. 
#define SPEED_DUTY_TOP 3 // 1/3 for low speed, 2/3 for MediumSpeed


//type declaration
typedef enum RobotState_t // Overall FSM for project. Used enumaration instead of a #define, since when compiled it will hold a value for each constant, starting from 0 for the first constant with the following constants a value that is one greater than the previous one.
{
	Idle,
	WaitingForNumberofInstructions,
	WaitingForDirection,
	WaitingForSpeed,
	WaitingForTimer,
	WaitingForNextInstructions,
	Moving,
} RobotState; //

typedef enum Direction_t //
{
	DirectionNotSelected = 0,
	ForwardDirection,
	BackwardDirection,
	ClockwiseDirection,
	CounterClockwiseDirection

} Direction;

typedef enum SpeedChosen_t
{
	SpeedNotSelected,
	SlowSpeed,
	MediumSpeed,
	FastSpeed
} Speed;


// Setting the button as inputs, similar to Lab 7.
void setInputButtons()
{
	DDRB &= ~((1<<PB_C_BIT)|(1<<PB_B_BIT)|(1<<PB_A_BIT));//Setting button as inputs, and based on lab manual they need to be set to zero in the Data Register.
	PORTB |= ((1<<PB_C_BIT)|(1<<PB_B_BIT)|(1<<PB_A_BIT)); //Configure pull-up, where the buttons need to be set to one on the port.
}


// InitialiIng LCD screen and display fucntions
void setLCD()
{
	initialize_LCD_driver();
	LCD_execute_command(TURN_ON_DISPLAY);
}

//Displaying Direction

void directionMenu()
{
	LCD_execute_command(CLEAR_DISPLAY);
	LCD_print_String("Choose");
	LCD_move_cursor_to_col_row(0,1);
	LCD_print_String("Direct.");

}

void displayForwardDirection()
{
	LCD_execute_command(CLEAR_DISPLAY);
	LCD_print_String("Forward");	
}
void displayBackwardsDirection()
{
	LCD_execute_command(CLEAR_DISPLAY);
	LCD_print_String("Backwards");	
}
void displayCounterClockwiseDirection()
{
	LCD_execute_command(CLEAR_DISPLAY);
	LCD_print_String("Counter ");
    LCD_move_cursor_to_col_row(0,1);
	LCD_print_String("clock.");
	
}
void displayClockwiseDirection()
{
	LCD_execute_command(CLEAR_DISPLAY);
	LCD_print_String("Clock");	
	LCD_move_cursor_to_col_row(0,1);
	LCD_print_String("wise");	
}

// Dispalying Speed 
void speedMenu()
{
	LCD_execute_command(CLEAR_DISPLAY);
	LCD_print_String("Choose");
    LCD_move_cursor_to_col_row(0,1);
	LCD_print_String("Speed");
}

void displayFastSpeed()
{
	LCD_execute_command(CLEAR_DISPLAY);
	LCD_print_String("Full");	
}
void displayMediumSpeed()
{
	LCD_execute_command(CLEAR_DISPLAY);
	LCD_print_String("Medium");	
}
void displaySlowSpeed()
{
	LCD_execute_command(CLEAR_DISPLAY);
	LCD_print_String("Slow");	
}

//Displaying Time

void timerMenu()
{
	LCD_execute_command(CLEAR_DISPLAY);
	LCD_print_String("Time in");
    LCD_move_cursor_to_col_row(0,1);
	LCD_print_String("ms");	
}

void displayTiming(const int timerChosen)
{
	int displayedTime = (timerChosen * 100); // Creates an integer value for the total time chosen by the user, in ms
	char buffer[10]; // made buffer with 10 bytes because since displayedTime is going by 100 since its measured in ms, so it will give a large valye
	itoa(displayedTime, buffer, 10); //needed to add stdlib in header for itoa to work
	LCD_execute_command(CLEAR_DISPLAY);
	LCD_print_String(buffer);

}
//displaying Moving state

void displayMovingState()
{
	LCD_execute_command(CLEAR_DISPLAY);
	LCD_print_String("Moving");	
}

//displays if User wants to Add more Instructions

void displayNumberInstructionsMenu()
{
	LCD_execute_comman(CLEAR_DISPLAY);
	LCD_print_String("Number of");
	LCD_move_cursor_to_col_rwo(0,1);
	LCD_print_string("Inst")
}
//
void displayNumberOfInst(const int numberOfInstructions)
{
	int displayedNumber = (numberOfInstructions); // Creates an integer value for the total time chosen by the user, in ms
	char buffer[10]; // made buffer with 10 bytes because since displayedTime is going by 100 since its measured in ms, so it will give a large valye
	itoa(displayedNumber, buffer, 10); //needed to add stdlib in header for itoa to work
	LCD_execute_command(CLEAR_DISPLAY);
	LCD_print_String(buffer);

}



// Creating boolen function to check if the each button is pressed, similar to lab 7. Comparing it to zero

bool isPbBPressed () 
{
	return (PINB & (1<<PB_B_BIT)) == 0x00;
}
bool isPbAPressed()
{
	return (PINB & (1<< PB_A_BIT)) == 0x00;
}
bool isPbCPressed()
{
	return (PINB & (1<< PB_C_BIT)) == 0x00;
}

// Setting wheels to outputs, similar to buttons except since they're outputs must be set to one and not zero.
void setOuputWheels()
{
	DDRD |= ((1<<Left_M1)|(1<<Left_M2)|(1<<Right_M1));
	DDRB |= (1<<Right_M2); //Right M2 motor is PB3 which must be set in DDRB
}
//moving functions
void brake()
{
	PORTB |= (1<<Right_M2);//Based on truth table found in the manual, for the robot to break switches 3 and 4 of the H motor must be open, which means all the motors must be one. 
	PORTD |= ((1<<Left_M1)|(1<<Right_M1)|(1<<Left_M2));// Right m2 button must be set in Port B, other in Port D. 
}

// Function of the forward movement for each speed chosen


void forward_full_speed(const int timerChosen) // the function receives an input from the wait_for_timer_input, which coincides with how many .1 intervals the user wants the robot to mve
{
	int countMoving = 0; //start a counter to be equal
	PORTB |= (1<<Right_M2); //PB3 should be one to move forward. Found in truth table
	PORTD |= (1<<Left_M2); //PD6 should be one to move forward. Found in truth table
	PORTD &= ~((1<<Left_M1)|(1<<Right_M1)); //PD3 and PD5 should be zero. Found in truth table

	while(countMoving++ <=timerChosen) //counter goes up until it reaches the same value the user inputed.
	{
		_delay_ms(100); // the 100,000 micro seconds is equivelant to .1 seconds. 

	}

}

void forward_medium_speed(const int timerChosen)
{

	// Robot needs an initial state where it goes full speed before it slows down to the assigned speed.
	PORTD |= (1<<Left_M2); // PB3 should be one to move forward. Found in truth table
	PORTD &= ~((1<<Left_M1)|(1<<Right_M1)); //PD6 should be one to move forward. Found in truth table
	PORTB |= (1<<Right_M2); //PD3 and PD5 should be zero. Found in truth table


	int countMoving = 0;//counter that goes up until it reeaches the timer chosen value.
	int dutyCycleCounter = 0; // dutyy cycle counter for PWM/
	while(countMoving++ <= timerChosen * 10  )
	{
		if ( ++dutyCycleCounter == SPEED_DUTY_TOP) // duty cycle top is 3, so based on this the wheel will move forward at top speed for 2/3 of the time. 
		{
			brake();
			dutyCycleCounter = 0;
		}
		else
		{
			PORTD |= (1<<Left_M2); // PB3 should be one to move forward. Found in truth table
			PORTD &= ~((1<<Left_M1)|(1<<Right_M1)); //PD6 should be one to move forward. Found in truth table
			PORTB |= (1<<Right_M2); //PD3 and PD5 should be zero. Found in truth table
		}
		_delay_us(10000);	//10 ms, but since the timer chosen is multipled by 10 it is equivelant to 100 ms, or .1 second.
	}

}

void forward_slow_speed(const int timerChosen)
{

	PORTD |= (1<<Left_M2); //PD6 should be on
	PORTD &= ~((1<<Left_M1)|(1<<Right_M1)); //PD3 and PD5 should be zero
	PORTB |= (1<<Right_M2); //PB3 should be one


	int countMoving = 0;
	int dutyCycleCounter = 0;
	while(countMoving++ <= timerChosen*10)
	{
		if ( ++dutyCycleCounter == SPEED_DUTY_TOP) // the wheel will move forward at full speed at 1/3 percent of the time.
		{
			PORTB |= (1<<Right_M2);
			PORTD |= (1<<Left_M2);
			PORTD &= ~((1<<Left_M1)|(1<<Right_M1)); //PD3 and PD5 should be one to break and slow down
			dutyCycleCounter = 0;
		}
		else
		{
			brake();
			
		}
		_delay_us(1000);	//10 ms, 
	}

}


//Backwards movement


void backward_full_speed(const int timerChosen)
{
	int countMoving = 0;
	PORTB &= ~(1<<Right_M2); //PB3 should be zero
	PORTD &= ~(1<<Left_M2); //PD6 should be zero
	PORTD |= ((1<<Left_M1)|(1<<Right_M1)); //PD3 and PD5 should be one. Values found in truth table

	while(countMoving++ <=timerChosen);
	{
		_delay_ms(100);

	}

}

void backward_medium_speed(const int timerChosen)
{

	PORTB &= ~(1<<Right_M2); //PB3 should be zero
	PORTD &= ~(1<<Left_M2); //PD6 should be zero
	PORTD |= ((1<<Left_M1)|(1<<Right_M1)); //PD3 and PD5 should be one


	int countMoving = 0;
	int dutyCycleCounter = 0;
	while(countMoving++ <= timerChosen * 10  )
	{
		if ( ++dutyCycleCounter == SPEED_DUTY_TOP)
		{
			brake();
			dutyCycleCounter = 0;
		}
		else
		{
			PORTB &= ~(1<<Right_M2); //PB3 should be zero
			PORTD &= ~(1<<Left_M2); //PD6 should be zero
			PORTD |= ((1<<Left_M1)|(1<<Right_M1)); //PD3 and PD5 should be one
		}
		_delay_us(10000);	//10 ms, 
	}

}

void backward_slow_speed(const int timerChosen)
{

	PORTB &= ~(1<<Right_M2); //PB3 should be zero
	PORTD &= ~(1<<Left_M2); //PD6 should be zero
	PORTD |= ((1<<Left_M1)|(1<<Right_M1)); //PD3 and PD5 should be one

	int countMoving = 0;
	int dutyCycleCounter = 0;
	while(countMoving++ <= timerChosen * 10  )
	{
		if ( ++dutyCycleCounter == SPEED_DUTY_TOP)
		{
			PORTB &= ~(1<<Right_M2); //PB3 should be zero
			PORTD &= ~(1<<Left_M2); //PD6 should be zero
			PORTD |= ((1<<Left_M1)|(1<<Right_M1)); //PD3 and PD5 should be one
			dutyCycleCounter = 0;
		}
		else
		{
				brake();			
		}
		_delay_us(1000);	//10 ms, 
	}

}

void clockWise_full_speed(const int timerChosen)

{
	int countMoving = 0;
	PORTD &= ~(1<<Left_M1); //PD5 should be zero aint with PD6 being high for left motor to go forward
	PORTD |= ((1<<Right_M1)|(1<<Left_M2)); //PD6 and PD3 should be one. PD3 being high aint with PB3 being low makes the right motor go reverse.
	PORTB &= ~(1<<Right_M2); // With left motor going forwards and right motor going backwards, the robot moves clockwise
	while(countMoving++ <=timerChosen);
	{
		_delay_us(100000);

	}

}
void clockWise_medium_speed(const int timerChosen)

{
	PORTD &= ~(1<<Left_M1); //PD5 should be zero aint with PD6 being high for left motor to go forward
	PORTD |= ((1<<Right_M1)|(1<<Left_M2)); //PD6 and PD3 should be one. PD3 being high aint with PB3 being low makes the right motor go reverse.
	PORTB &= ~(1<<Right_M2); // With left motor going forwards and right motor going backwards, the robot moves clockwise

	int countMoving = 0;
	int dutyCycleCounter = 0;
	while(countMoving++ <= timerChosen * 10  )
	{
		if ( ++dutyCycleCounter == SPEED_DUTY_TOP)
		{
			brake();
			dutyCycleCounter = 0;
		}
		else
		{
				PORTD &= ~(1<<Left_M1); //PD5 should be zero aint with PD6 being high for left motor to go forward
				PORTD |= ((1<<Right_M1)|(1<<Left_M2)); //PD6 and PD3 should be one. PD3 being high aint with PB3 being low makes the right motor go reverse.
				PORTB &= ~(1<<Right_M2); // With left motor going forwards and right motor going backwards, the robot moves clockwise
		}
		_delay_us(10000);	//10 ms, 
	}

}
void clockWise_slow_speed(const int timerChosen)

{
	PORTD &= ~(1<<Left_M1); //PD5 should be zero aint with PD6 being high for left motor to go forward
	PORTD |= ((1<<Right_M1)|(1<<Left_M2)); //PD6 and PD3 should be one. PD3 being high aint with PB3 being low makes the right motor go reverse.
	PORTB &= ~(1<<Right_M2); // With left motor going forwards and right motor going backwards, the robot moves clockwise

	int countMoving = 0;
	int dutyCycleCounter = 0;
	while(countMoving++ <= timerChosen * 10  )
	{
		if ( ++dutyCycleCounter == SPEED_DUTY_TOP)
		{
			PORTD &= ~(1<<Left_M1); //PD5 should be zero aint with PD6 being high for left motor to go forward
			PORTD |= ((1<<Right_M1)|(1<<Left_M2)); //PD6 and PD3 should be one. PD3 being high aint with PB3 being low makes the right motor go reverse.
			PORTB &= ~(1<<Right_M2); // With left motor going forwards and right motor going backwards, the robot moves clockwise
			dutyCycleCounter = 0;
		}
		else
		{
			brake();

		}
		_delay_us(10000);	//10 ms, 
	}

}

void counter_clockWise_full_speed(const int timerChosen)

{
	int countMoving = 0;
	PORTD |= (1<<Left_M1); //PD5 should be 1 aint with PD6 being zero for left motor to go backward
	PORTD &= ~((1<<Right_M1)|(1<<Left_M2)); //PD6 and PD3 should be zero. PD3 being low aint with PB3 being high makes the right motor go forward.
	PORTB |= (1<<Right_M2); // With left motor going backwards and right motor going forwards, the robot moves counter clockwise
	while(countMoving++ <=timerChosen);
	{
		_delay_us(100000);

	}

}
void counter_clockWise_medium_speed(const int timerChosen)

{
	PORTD |= (1<<Left_M1); //PD5 should be 1 aint with PD6 being zero for left motor to go backward
	PORTD &= ~((1<<Right_M1)|(1<<Left_M2)); //PD6 and PD3 should be zero. PD3 being low aint with PB3 being high makes the right motor go forward.
	PORTB |= (1<<Right_M2); // With left motor going backwards and right motor going forwards, the robot moves counter clockwise
	int countMoving = 0;
	int dutyCycleCounter = 0;
	while(countMoving++ <= timerChosen * 10  )
	{
		if ( ++dutyCycleCounter == SPEED_DUTY_TOP)
		{
			brake();
			dutyCycleCounter = 0;
		}
		else
		{
			PORTD |= (1<<Left_M1); //PD5 should be 1 aint with PD6 being zero for left motor to go backward
			PORTD &= ~((1<<Right_M1)|(1<<Left_M2)); //PD6 and PD3 should be zero. PD3 being low aint with PB3 being high makes the right motor go forward.
			PORTB |= (1<<Right_M2); // With left motor going backwards and right motor going forwards, the robot moves counter clockwise
		}
		_delay_us(10000);	//10 ms, 
	}

}
void counter_clockWise_slow_speed(const int timerChosen)

{
	PORTD |= (1<<Left_M1); //PD5 should be 1 aint with PD6 being zero for left motor to go backward
	PORTD &= ~((1<<Right_M1)|(1<<Left_M2)); //PD6 and PD3 should be zero. PD3 being low aint with PB3 being high makes the right motor go forward.
	PORTB |= (1<<Right_M2); // With left motor going backwards and right motor going forwards, the robot moves counter clockwise

	int countMoving = 0;
	int dutyCycleCounter = 0;
	while(countMoving++ <= timerChosen * 10  )
	{
		if ( ++dutyCycleCounter == SPEED_DUTY_TOP)
		{
			PORTD |= (1<<Left_M1); //PD5 should be 1 aint with PD6 being zero for left motor to go backward
			PORTD &= ~((1<<Right_M1)|(1<<Left_M2)); //PD6 and PD3 should be zero. PD3 being low aint with PB3 being high makes the right motor go forward.
			PORTB |= (1<<Right_M2); // With left motor going backwards and right motor going forwards, the robot moves counter clockwise
			dutyCycleCounter = 0;
		}
		else
		{
			brake();
		}
		_delay_us(10000);	//10 ms, 
	}

}

//input functions




RobotState WaitForNumberOfInstructionsInput(int * numberOfInstructions)
{
	int countTimeOut = INPUT_TIMEOUT;//2 seconds delay to finish selection
	*numberOfInstructions = 0;
	if ( isPbApressed())
	{
		while (numberOfInstructions < 10 )
		{
			do
			{				
				_delay_us(1000);
			} 
			while(isPbAPressed());

			(*numberOfInstructions)++;

			displayNumberOfInst(*numberOfInstructions);

			countTimeOut = INPUT_TIMEOUT;
			while(!isPbAPressed() && countTimeOut > 0)
			{
				_delay_us(1000);
				countTimeOut--;
			}
			if ( countTimeOut == 0)
			{
				displayDirectionMenu();
				return WaitingForDirection;

			}
		}
	}
	return WaitingForNumberofInstructions;
		
}

RobotState waitForDirectionInput(Direction * direction) // use * instead of && to pass by address. 
{
	int countTimeOut = INPUT_TIMEOUT;//5 seconds delay to finish selection. The user has 5 seconds to change his direction input. 
	*direction = DirectionNotSelected; // initial direction state of zero, found in the "Direction" function with the enumarated constants. 
	
	if ( isPbAPressed()) //CHecking
	{
		for (;;)
		{
			do
			{				
				_delay_us(1000);
			} 
			while(isPbAPressed()); // When button is pressed, there will be a short delay
			(*direction)++; // Enumerated type constant of the direction will change everytime the button is pressed
			
			//Displaing LED  direction chosen on LED
			switch ( *direction)
			{
				case DirectionNotSelected:
				break;//direction will never be at this point, but kept getting warning about how enumarated value is not being handled in switch

				case ForwardDirection:
				displayForwardDirection();
				break;

				case BackwardDirection:
				displayBackwardsDirection();
				break;

				case CounterClockwiseDirection:
				displayCounterClockwiseDirection();
				break;

				case ClockwiseDirection:
				displayClockwiseDirection();
				break;
				default:
				*direction = DirectionNotSelected;
				break;
			}
			countTimeOut = INPUT_TIMEOUT;
			//
			while(!isPbAPressed() && countTimeOut > 0)
			{
				_delay_us(1000);
				countTimeOut--;
			}
			if ( countTimeOut == 0)
			{
				speedMenu();
				return WaitingForSpeed;
			}
		}
	}
	return WaitingForDirection;
}


RobotState waitForSpeedInput(Speed * speed) // use * instead of && to pass by address
{
	int countTimeOut = INPUT_TIMEOUT;//2 seconds delay to finish selection
	*speed = SpeedNotSelected;
	
	if ( isPbBPressed()) //Checking if
	{
		for (;;)
		{
			do
			{				
				_delay_us(1000);
			} 
			while(isPbBPressed());
			(*speed)++;
			switch ( *speed)
			{
				case SpeedNotSelected: // need that because kept getting warning that enumarated value is not handled in switfch case. At this point a speed value will always be selected.
				break;

				case SlowSpeed:
				displaySlowSpeed();
				break;

				case MediumSpeed:
				displayMediumSpeed();
				break;

				case FastSpeed:
				displayFastSpeed();
				break;
				default:
				*speed = SpeedNotSelected;
				break;
			}
			countTimeOut = INPUT_TIMEOUT;
			while(!isPbBPressed() && countTimeOut > 0)
			{
				_delay_us(1000);
				countTimeOut--;
			}
			if ( countTimeOut == 0)
			{
				timerMenu();

				return WaitingForTimer;
			}
		}
	}
	return WaitingForSpeed;
}


RobotState waitForTimerInput(int * timerChosen, int * numberOfInstructions) // use * instead of && to pass by address
{
	int countTimeOut = INPUT_TIMEOUT;//2 seconds delay to finish selection
	*timerChosen = 0;
	if ( isPbCPressed())
	{
		while (*timerChosen < 100 )
		{
			do
			{				
				_delay_us(1000);
			} 
			while(isPbCPressed());

			(*timerChosen)++;

			displayTiming(*timerChosen);

			countTimeOut = INPUT_TIMEOUT;
			while(!isPbCPressed() && countTimeOut > 0)
			{
				
				_delay_us(1000);
				countTimeOut--;
			}
			if ( countTimeOut == 0 && NumberOfinstructions == 1)
			{
				displayMovingState();
				return moving;

			}
			if(countTimeOut == 0 && NumberOfInstruction != 1)
			{
				displayDirectionMenu();
				return WaitingForOtherInstructions
			}	
		}
	}
	return WaitingForTimer;
}

RobotState WaitingtForOtherInstructionsInput(int * numberOfInstructions)
{
	int instructionCount = 0;
	int Instructions[numberOfInstructions];
		for (instructionCount = 0, instructionCount < numberOfInstructions,instructionCount++)
		{
			directionMenu();
			switch(State)
			{
				case WaitingForDirection:
				State = waitForDirectionInput(&direction);
					break;

				case WaitingForSpeed:
				S = waitForSpeedInput(&speed);
				break;
	
				case WaitingForTimer:
				currentState = waitForTimerInput(timerChosen numberOfInstructions);
				break;
			}
		}	
	if(instructionsCount == numberOfInstructions)
	{

		moving(direction, speed, timerChosen);
	}
	

}


void moveForward ( Speed speed, const int timerChosen)
{
	if(speed == SlowSpeed)
	{
		forward_slow_speed(timerChosen);

	}
	if(speed == MediumSpeed)
	{
		forward_medium_speed(timerChosen);

	}
	if(speed == FastSpeed)
	{
		forward_full_speed(timerChosen);

	}

}

void moveBackwards ( Speed speed, const int timerChosen)
{
	if(speed == SlowSpeed)
	{
		backward_slow_speed(timerChosen);

	}
	if(speed == MediumSpeed)
	{
		backward_medium_speed(timerChosen);

	}
	if(speed == FastSpeed)
	{
		backward_full_speed(timerChosen);

	}

}

void moveClockwise ( Speed speed, const int timerChosen)
{
	if(speed == SlowSpeed)
	{
		clockWise_slow_speed(timerChosen);

	}
	if(speed == MediumSpeed)
	{
		clockWise_medium_speed(timerChosen);

	}
	if(speed == FastSpeed)
	{
		clockWise_full_speed(timerChosen);

	}

}

void moveCounterClockwise ( Speed speed, const int timerChosen)
{
	if(speed == SlowSpeed)
	{
		counter_clockWise_slow_speed(timerChosen);

	}
	if(speed == MediumSpeed)
	{
		counter_clockWise_medium_speed(timerChosen);
	}
	if(speed == FastSpeed)
	{
		counter_clockWise_full_speed(timerChosen);

	}

}

RobotState moving(Direction direction, Speed speed, const int timerChosen)
{
	switch ( direction)
	{
		case DirectionNotSelected:
			break;
		case ForwardDirection:
			moveForward(speed, timerChosen);
			break;
		case BackwardDirection:
			moveBackwards(speed, timerChosen);
			break;
		case CounterClockwiseDirection:
			moveCounterClockwise(speed, timerChosen);
			break;
		case ClockwiseDirection:
			moveClockwise(speed, timerChosen);
			break;
	}
	brake();
	directionMenu();

	return Idle;

}

int main()
{
	setLCD();
	RobotState currentState = Idle;
	Direction direction = DirectionNotSelected;
	Speed speed = SpeedNotSelected; 
	int timerChosen = 0;

	setInputButtons();
	displayNextInstructionsMenu();
	for (;;)
	{
		switch (currentState)
		{
			case Idle:
			case WaitingForNumberofInstructions:
				currentState = WaitingForNumberofInstructionsInput(&numberOfInstructions)
			break;
			case WaitingForDirection:
				currentState = waitForDirectionInput(&direction);
			break;

			case WaitingForSpeed:
				currentState = waitForSpeedInput(&speed);
			break;

			case WaitingForTimer:
				currentState = waitForTimerInput(&timerChosen);
			break;

			case WaitingForOtherInstructions:
				currentState = WaitingtForOtherInstructionsInput()
			break;
			case Moving:
				currentState = moving (direction, speed, timerChosen);
			break;
		}

		_delay_us(1000);

	}
	
	
	return 0;
}
