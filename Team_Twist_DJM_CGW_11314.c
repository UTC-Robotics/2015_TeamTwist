/* 
UTC IEEE Robotics Team, 2015 Competition

Authors:
David Myers - Project Manager
Charles G. Wheeler - Subsystem Lead for Team Twist (Etch-a-Sketch and Rubik's Cube)

Stepper Motor control code for MSP430G2553
*/

#define LEFT 0
#define RIGHT 1
#define RUBIKS 3
#define TOTAL_KNOB_MOVES 49 //total times knob is selected and moved

#define ABS(a) (a)<0 ? -(a) : (a)

#include "msp430.h"

//globals
volatile int chopStepCnt = 0; 
volatile int masterSaysGo = 0;

void configureClocks(void)
{
  DCOCTL = CAL_DCO_16MHZ;                               //set DCO to be at 16 MHz???????????????????
  BCSCTL1 = 0x0F;                                       //set RSELx such that DCO = 16Mhz????????????????
  BCSCTL2 = SELM_0 +  DIVM_0 + DIVS_0;                  //set MCLK and SMCLK sourced from DCO and div by 1????????????
  BCSCTL2 &= ~SELS;                                     //so 16Mhz system clock?????????????????
  
}

void configurePorts(void)
{
  /*
  For MSP430 20pin chip:
  Pin Number = Function on stepper driver (which stepper driver)
  
  VCC = 3.3V                    GND = 0V 
  P1.0 = M0 (all)               P1.7 = Dir (all)
  P1.1 = M1 (all)               P2.5 = nSLP (EaSL)
  P1.2 = M2 (all)               p2.4 = nSLP (EaSR)
  P1.3 = Master Input           p1.6 = Master Output 
  P1.4 = Step (all)             p2.3 = nSLP (Rubiks)
  P2.0 = nRST (all)                  
  */
  
  //Set Pin Directions
  P1DIR = 0x00;                                         //reset register to all 0's/set P1.3 to input
  P2DIR = 0x00;                                         //reset register to all 0's
  P1DIR |= 0xD7;                                        //Set P1.0, 1.1, 1.2, 1.4, 1.6, 1.7 to output 
  P2DIR |= 0x39;                                        //Set P2.0, 2.3, 2.4, 2.5, to output
  
  //Set Step Type (full, 1/2,1/4,1/8...1/32) pins to high...to 1/32 mode 
  P1OUT = 0x00;                                         //make sure 0's everywhere in P1 register
  P2OUT = 0x00;                                         //make sure 0's everywhere in P2 register
  P1OUT |= 0x07;                                        //write 1's to P1.0, 1.1, 1.2
  
    
  //Set Pull-up Resistors  
  P1REN |= 0x08;                                        // Enable a pull-up/down resistor on P1.3
  P1OUT &= ~0x08;                                        // Set to be pull down
  
  //Set up interrupt pin
  P1IE  = 0x08;                                         // Enable the interrupt from P1.3 (disable all other Port 1 pin interrupts)
  P1IES &= ~0x08;                                        // Set the interrupt to trigger on a low-to-high transition
  P1IFG = 0;                                            //clear P1 interrupt flag
}

void configureTimerISR(void)
{
  TA0CTL = TASSEL_2 + MC_1;                             // Timer A0, source from SMCLK, count UP mode
  TA0CCR0 = 230;                                          // Set the compare register 0 to 640 cycles (toggle at 25kHz)????????????????
  TA0CCTL0 &= ~CCIE;                                      // Do Not Enable timer A interrup
}

void turnKnobs(int _knobSelect, int _knobSteps)
{	
	
	// Select the appropriate chopper
	if (_knobSelect == LEFT)
        {
		P2OUT |= 0x20;                                        //p2.5 ON - bring EaS Left chopper awake 
	}
	if (_knobSelect == RIGHT)
        {
		P2OUT |= 0x10;                                        //p2.4 ON - bring EaS Right chopper awake
	}
	if (_knobSelect == RUBIKS)
        {
		P2OUT |= 0x08;                                        //p2.3 ON - bring Rubik's chopper awake
	}
	
        __delay_cycles(230000);                                         
		//delay for 0.04 seconds to make sure chopper is awake. 0.04 seconds????????????????
        
	// Select direction
	if (_knobSteps > 0)
        {
		P1OUT |= 0x80;                                        //set p1.7 high--direction CCW 
	}
	if (_knobSteps < 0)
        {
		P1OUT &= ~0x80;                                        //set p1.7 low--direction CW 
	}
	
	// take absolute vaule, multiple by 2 to get high and low
	_knobSteps = 2*ABS(_knobSteps);
	chopStepCnt = 0;
	
	TA0CCTL0 |= CCIE;                                      //Enable timer A interrup
	
    while(chopStepCnt<_knobSteps){}
	
    TA0CCTL0 &= ~CCIE;                                     //disable timer interupt
    
    //Turn off All choppers for safety
    P2OUT &= ~0x20;                                        //p2.5 OFF - Lul EaS Left chopper back to sleep
    P2OUT &= ~0x10;                                        //p2.4 OFF - Lul EaS Right chopper back to sleep
    P2OUT &= ~0x08;                                        //p2.3 OFF - Lul Rubik's chopper back to sleep
    
}

void main()
{
	
	// ********************* CONFIGURE  STUFF ************************************
	WDTCTL = WDTPW + WDTHOLD;                          // Stop watchdog timer to prevent time out reset
        configureClocks();                                 //set up Clocks
	configurePorts();                                  //set up Ports
	configureTimerISR();                               //set up Timer ISR
	__enable_interrupt(); 	                           //Enable global interrupts
	// ********************* DRAW ON ETCH-A-SKETCH *********************************
	// INITALIZE INSTRUCTION LISTS

	// knobSelct gives the ordering in which  the knob are turned
	// 0 -> Left, 1-> Right
	int knobSelect[]=
       {RIGHT, LEFT, RIGHT, LEFT, RIGHT, LEFT, RIGHT, LEFT, RIGHT, LEFT, RIGHT, LEFT,   //select EaS knob--draw I
        RIGHT, LEFT, RIGHT, LEFT, RIGHT, LEFT, RIGHT, LEFT, RIGHT, LEFT, RIGHT, LEFT,   //select EaS knob--draw E
        RIGHT, LEFT, RIGHT, LEFT, RIGHT, LEFT, RIGHT, LEFT, RIGHT, LEFT, RIGHT, LEFT,   //select EaS knob--draw E
        RIGHT, LEFT, RIGHT, LEFT, RIGHT, LEFT, RIGHT, LEFT, RIGHT, LEFT, RIGHT, LEFT, LEFT};   //select EaS knob--draw E final
		
	// knobSteps gives the numbers of cycles sent to the chopper drives
	// positives values-> CCW turns
	// negative  values-> CW turns
        int knobSteps[]= 
	{-334, -323, -762, 323, -217, -762, 334, 323, 821, -264, 206, -235,          //number of steps--draw I
        -1377, -586, 334, 440, 235, -323, 235, 323, 235, -440, 323, -235,            //number of steps--draw E    
        -1377, -586, 334, 440, 235, -323, 235, 323, 235, -440, 323, -235,            //number of steps--draw E
        -1377, -586, 334, 440, 235, -323, 235, 323, 235, -440, 323, 1597, 1597};      //number of steps--draw E final  
	
        
        int j = 0;
        for(j=0; j<TOTAL_KNOB_MOVES; j++)
        {
           knobSteps[j] *= 10;
        }

        
	// WE MUST WAIT UNTIL MASTER TELLS US TO PLAY
	//     (WE LIVE BY HIS WORD) 
	masterSaysGo = 0;
	while(!masterSaysGo){}
       
	// TIME TO TURN THE ETCH-A-SKETCH KNOBS
	int i = 0;
	for(i=0; i<TOTAL_KNOB_MOVES; i++)
        {
            turnKnobs(knobSelect[i],knobSteps[i]);
	}
	
	// WE MUST TELL MASTER WE ARE DONE WITH ETCH-A-SKETCH
	//     (WE HUMBLY SUBMIT OUR DRAWING FOR HIS HONOR)
     
        P1OUT |= 0x40;                                        //Verify pin, p1.6, ON
        __delay_cycles(50000000);                              //delay 10000000 cycles for a 0.25second ON period??????????????????????????
        P1OUT &= ~0x40;                                       //Verify pin, p1.6, OFF
	
	// ********************* TURN THE RUBICKS CUBE *********************************

	// WE MUST WAIT AGAIN UNTIL MASTER TELLS US TO PLAY
	//         (HE IS SO WISE, OUR MASTER)
	
	//number of pulses needed to turn cube
	int rubiksTurnSteps = 6400;
	
	
	masterSaysGo = 0;
	while(!masterSaysGo){}
	turnKnobs(RUBIKS,rubiksTurnSteps);
	
	// WE MUST TELL MASTER WE ARE DONE WITH THE RUBICKS CUBE
	//         (IT IS GREAT PLEASURE TO SERVE HIM)
	P1OUT |= 0x40;                                        //Verify pin, p1.6, ON
        __delay_cycles(50000000);                                  //delay 1000000 cycles for a 0.25second ON period???????????????????
        P1OUT &= ~0x40;                                       //Verify pin, p1.6, OFF
}


// Port1 ISR from Master
#pragma vector=PORT1_VECTOR
  __interrupt void Port_1 (void)
 {
  __delay_cycles(100000);                               // Wait for 32ms (128000 cycles at 4MHz)???????????????????
  masterSaysGo = 1;
  P1IFG = 0;                                            // Clear interrupt flag
 }

// Timer0 ISR - Step Control
#pragma vector=TIMER0_A0_VECTOR
  __interrupt void Timer_A0 (void)
  {
    chopStepCnt++;	                                       //step the mtoors by 1
    P1OUT ^= 0x10;                                         //toggle chopper step pin, p1.4
  }
