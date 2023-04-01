// TrackExplorer.c
// Runs on TM4C123
// This is the starter file for CECS 347 Project 2 - A Track Explorer
// This project use PWM to control two DC Motors, SysTick timer 
// to control sampling rate, ADC to collect analog inputs from
// Sharp IR sensors and a potentiometer.
// Two GP2Y0A21YK0F analog IR distance sensors are used to allow
// the robot to follow a wall. A Minimum of two IR sensors are mounted
// looking forward to the left and forward to the right. 
// A optional third IR sensor looks directly forward can be used to avoid
// a head-on collision. Basically, the goal is to control power to each wheel so the
// left and right distances to the walls are equal.
// If an object is detected too close to the front of the robot,
// both wheels are immediately stopped.
/*
    ------------------------------------------wall---------
                      /
                     /
                    / 
                   /
         -----------
         |         |
         | Robot   | ---> direction of motion and third sensor
         |         |
         -----------
                   \
                    \
                     \
                      \
    ------------------------------------------wall---------
*/
// The original project is provided by Dr. Daniel Valvano, Jonathan Valvano
// September 12, 2013
// Modification is made by Dr. Min He to provide this starter project.

// PE1 connected to forward facing IR distance sensor
// PE4 connected to forward right IR distance sensor
// PE5 connected to forward left IR distance sensor

#include "ADCMultiSamples.h"
#include "PLL.h"
#include "tm4c123gh6pm.h"
#include "stdint.h"
#include "PWM.h"
#include "GPIO.h"

// basic functions defined at end of startup.s
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void WaitForInterrupt(void);  // low power mode
void FIR_filter_5(uint16_t *left, uint16_t *right, uint16_t *forward);

uint8_t sample=0;

// You use datasheet to calculate the following ADC values
// then test your sensors to adjust the values 
#define CRASH             IR15CM// if there is less than this distance ahead of the robot, it will immediately stop
#define IR15CM            2233  // ADC output for 15cm:1.8v -> (1.8/3.3)*4095=2233 
#define IR20CM            1724  // ADC output for 20cm:1.39v -> (1.39/3.3)*4095=1724
#define IR30CM            1116  // ADC output for 30cm:0.9v -> (0.9/3.3)*4095=1116
#define IR40CM            918   // ADC output for 40cm:0.74v -> (0.74/3.3)*4095=918
#define IR80CM            496   // ADC output for 80cm:0.4v -> (0.4/3.3)*4095=496
                                // with equal power to both motors (LeftH == RightH), the robot still may not drive straight
                                // due to mechanical differences in the motors, so bias the left wheel faster or slower than
                                // the constant right wheel
#define LEFTMINPCT        30    // minimum percent duty cycle of left wheel (10 to 90)
#define LEFTMAXPCT        50    // maximum percent duty cycle of left wheel (10 to 90)
#define RIGHTCONSTPCT     40    // constant percent duty cycle of right wheel (10 to 90)
uint16_t ain2;
uint16_t ain9;
uint16_t ain8;
uint16_t input;
volatile  int flag = 0;
void System_Init(void);
void LEDSW_Init(void);
void Motor_Init(void);
//void SysTick_Init(void);
void steering(uint16_t ahead_dist,uint16_t right_dist, uint16_t left_dist);
void ReadADCFIRFilter(uint16_t *ain2, uint16_t *ain9, uint16_t *ain8);
void ReadADCIIRFilter(uint16_t *ain2, uint16_t *ain9, uint16_t *ain8);
uint16_t median(uint16_t u1, uint16_t u2, uint16_t u3);
void ReadADCMedianFilter(uint16_t *ain2, uint16_t *ain9, uint16_t *ain8);// This function samples AIN2 (PE1), AIN9 (PE4), AIN8 (PE5) and
void Delay(int num);
const struct State {
uint32_t Out;
uint32_t left_DC; // left duty cycles controle
uint32_t right_DC;// right duty cycles controle
uint32_t Next[8]; // list of next states
};
typedef const struct State STyp;

#define forward 		0
#define turn_left  	1
#define turn_right 	2
#define pivot_right 3
#define pivot_left	4
#define backwards		5

STyp FSM[8] = {
	{FORWARD, START_SPEED, START_SPEED,{forward, turn_right, pivot_right, pivot_left, turn_left, forward, pivot_right, backwards}},
	{FORWARD, LOW_SPEED, START_SPEED,{forward, turn_right, pivot_right, pivot_left, turn_left, forward, pivot_right, backwards}},
	{LEFTPIVOT, START_SPEED, START_SPEED,{forward, turn_right, pivot_right, pivot_left, turn_left, forward, pivot_right, backwards}},
	{LEFTPIVOT, START_SPEED, START_SPEED,{forward, turn_right, pivot_right, pivot_left, turn_left, forward, pivot_right, backwards}},
	{FORWARD, START_SPEED, LOW_SPEED,{forward, turn_right, pivot_right, pivot_left, turn_left, forward, pivot_right, backwards}},
	{RIGHTPIVOT, START_SPEED, START_SPEED,{forward, turn_right, pivot_right, pivot_left, turn_left, forward, pivot_right, backwards}},
	{RIGHTPIVOT, START_SPEED, START_SPEED,{forward, turn_right, pivot_right, pivot_left, turn_left, forward, pivot_right, backwards}},
	{BACKWARD, START_SPEED, START_SPEED,{forward, turn_right, pivot_right, pivot_left, turn_left, forward, pivot_right, backwards}},
};

int main(void){
  uint16_t left, right, ahead;
  
  DisableInterrupts();  // disable interrupts while initializing
  EnableInterrupts();   // enable after all initialization are done
  System_Init();

	// 
  while(1){
   if(flag) {
      sample = 0;
      // choose one of the software filter methods
      ReadADCMedianFilter(&ahead, &right, &left);
      steering(ahead,right,left);
   }
  }
}

void System_Init(void) {
  PLL_Init();           // bus clock at 16 MHz
//  SysTick_Init();       // Optional: Initialize SysTick timer with interrupt for smapling control
  ADC_Init298();        // initialize ADC to sample AIN2 (PE1), AIN9 (PE4), AIN8 (PE5)
  LEDSW_Init();         // configure onboard LEDs and push buttons
  Motor_Init();         // Initialize signals for the two DC Motors
}

void LEDSW_Init(void){
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF;	// Activate F clocks
	while ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOF)==0){};
	GPIO_PORTF_LOCK_R = 0x4C4F434B;   // unlock GPIO Port F
	GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0
  // only PF0 needs to be unlocked, other bits can't be locked
  GPIO_PORTF_AMSEL_R &= ~0x1F;      // 3) disable analog function
  GPIO_PORTF_PCTL_R &= ~0x0000FFFF; // 4) GPIO clear bit PCTL 
	GPIO_PORTF_DIR_R &= ~0x1F;        //	6) PF0-PF4 input/switches		
  GPIO_PORTF_DIR_R |= 0x0E;         // 6) PF1-PF3 output/ LEDs
  GPIO_PORTF_AFSEL_R &= ~0x1F;      // 7) no alternate function     
  GPIO_PORTF_DEN_R |= 0x1F;         // 8) enable digital pins PF4-PF0
  GPIO_PORTF_PUR_R |= 0x11;     		//     enable weak pull-up on PF4 and PF0
  GPIO_PORTF_IS_R &= ~0x11;     		// PF4 and PF0 are edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x11;    		//     PF4 and PF0 are not both edges
  GPIO_PORTF_IEV_R &= ~0x11;   			//     PF4 and PF0 falling edge event
  GPIO_PORTF_ICR_R |= 0x11;      		// clear flag4
  GPIO_PORTF_IM_R |= 0x11;      		//  arm interrupt on PF4 and PF0
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF1FFFFF)|0x00A00000; // (g) priority 5
  NVIC_EN0_R |= 0x40000000;      // (h) enable interrupt 30 in NVIC 	 
}

void Motor_Init(void){
  Car_Dir_Init();
	PWM_PB76_Init();
	PWM_PB76_Duty(START_SPEED, START_SPEED);
}

//void SysTick_Init(void){
//}

void GPIOPortF_Handler(void){

	if (GPIO_PORTF_RIS_R & 0x10){
	  flag = 1;
		GPIO_PORTF_ICR_R |= 0x10;      // acknowledge flag4: 00010000
		PWM0_ENABLE_R |= 0x00000003; // enable both wheels
	}
	if (GPIO_PORTF_RIS_R & 0x01) { // switch 2 is pressed
		flag = 0;  
		PWM0_ENABLE_R &= ~0x00000003; // stop both wheels
		LED = Dark;
		GPIO_PORTF_ICR_R |= 0x01;      // acknowledge flag 0 
	}
}

void steering(uint16_t left_dist,uint16_t ahead_dist, uint16_t right_dist){
  // Suggest the following simple control as starting point:
  // 1. If any one of the senors see obstacle <15cm, stop
  // 2. If left sees obstacle within 30cm, turn right
  // 3. If right sees obstacle within 30cm, turn left
	// 4. If all sensors detect no obstacle within 35cm, stop
  // 5. If both sensors see no obstacle within 30cm, go straight  
  // Feel free to add more controlls to fine tune your robot car.
	// Example: if error=ABS(left-right) great than 100, make a turn
  // Make sure to take care of both wheel movements and LED display here.
	uint16_t ahead_bit = 0b00;
	uint16_t right_bit = 0b00;
	uint16_t left_bit = 0b00;
	
	if ( ahead_dist <= IR80CM ) { 
		LED = Blue; // Mission ended with success
	} else {
	if ( left_dist > right_dist ) {
		left_bit = 0b01;
		right_bit = 0b00;
	} else {
		left_bit = 0b00;
		right_bit = 0b01;
	}	
	if ( ahead_dist > IR15CM ) {
		ahead_bit = 0b01;
		if ((right_dist > IR15CM) && (left_dist > IR15CM)) {
			LED = Red;
			left_bit = 0b01;
			right_bit = 0b01;
		}
			input = (left_bit << 2) + (ahead_bit << 1) + (right_bit);
	} else {
		ahead_bit = 0b00;
		LED = Green;
		input = (left_bit << 2) + (ahead_bit << 1) + (right_bit);
	}
		
		PWM_PB76_Duty(FSM[input].left_DC, FSM[input].right_DC);
		WHEEL_DIR = FSM[input].Out;
		Delay(5);
	}
}
//void SysTick_Handler(void){
//  sample = 1;
//}

// Returns the results in the corresponding variables.  Some
// kind of filtering is required because the IR distance sensors
// output occasional erroneous spikes.  This is an FIR filter:
// y(n) = (x(n) + x(n-1))/2
// Assumes: ADC initialized by previously calling ADC_Init298()
void ReadADCFIRFilter(uint16_t *ain2, uint16_t *ain9, uint16_t *ain8){
  static uint16_t ain2previous=0; // after the first call, the value changed to 12
  static uint16_t ain9previous=0;
  static uint16_t ain8previous=0;
  // save some memory; these do not need to be 'static'
  //            x(n)
  uint16_t ain2newest;
  uint16_t ain9newest;
  uint16_t ain8newest;
  ADC_In298(&ain2newest, &ain9newest, &ain8newest); // sample AIN2(PE1), AIN9 (PE4), AIN8 (PE5)
  *ain2 = (ain2newest + ain2previous)/2;
  *ain9 = (ain9newest + ain9previous)/2;
  *ain8 = (ain8newest + ain8previous)/2;
  ain2previous = ain2newest; ain9previous = ain9newest; ain8previous = ain8newest;
}

// This function samples AIN2 (PE1), AIN9 (PE4), AIN8 (PE5) and
// returns the results in the corresponding variables.  Some
// kind of filtering is required because the IR distance sensors
// output occasional erroneous spikes.  This is an IIR filter:
// y(n) = (x(n) + y(n-1))/2
// Assumes: ADC initialized by previously calling ADC_Init298()
void ReadADCIIRFilter(uint16_t *ain2, uint16_t *ain9, uint16_t *ain8){
  //                   y(n-1)
  static uint16_t filter2previous=0;
  static uint16_t filter9previous=0;
  static uint16_t filter8previous=0;
  // save some memory; these do not need to be 'static'
  //            x(n)
  uint16_t ain2newest;
  uint16_t ain9newest;
  uint16_t ain8newest;
  ADC_In298(&ain2newest, &ain9newest, &ain8newest); // sample AIN2(PE1), AIN9 (PE4), AIN8 (PE5)
  *ain2 = filter2previous = (ain2newest + filter2previous)/2;
  *ain9 = filter9previous = (ain9newest + filter9previous)/2;
  *ain8 = filter8previous = (ain8newest + filter8previous)/2;
}

// Median function from EE345M Lab 7 2011; Program 5.1 from Volume 3
// helper function for ReadADCMedianFilter() but works for general use
uint16_t median(uint16_t u1, uint16_t u2, uint16_t u3){
uint16_t result;
  if(u1>u2)
    if(u2>u3)   result=u2;     // u1>u2,u2>u3       u1>u2>u3
      else
        if(u1>u3) result=u3;   // u1>u2,u3>u2,u1>u3 u1>u3>u2
        else      result=u1;   // u1>u2,u3>u2,u3>u1 u3>u1>u2
  else
    if(u3>u2)   result=u2;     // u2>u1,u3>u2       u3>u2>u1
      else
        if(u1>u3) result=u1;   // u2>u1,u2>u3,u1>u3 u2>u1>u3
        else      result=u3;   // u2>u1,u2>u3,u3>u1 u2>u3>u1
  return(result);
}

// This function samples AIN2 (PE1), AIN9 (PE4), AIN8 (PE5) and
// returns the results in the corresponding variables.  Some
// kind of filtering is required because the IR distance sensors
// output occasional erroneous spikes.  This is a median filter:
// y(n) = median(x(n), x(n-1), x(n-2))
// Assumes: ADC initialized by previously calling ADC_Init298()
void ReadADCMedianFilter(uint16_t *ain2, uint16_t *ain9, uint16_t *ain8){
  //                   x(n-2)        x(n-1)
  static uint16_t ain2oldest=0, ain2middle=0;
  static uint16_t ain9oldest=0, ain9middle=0;
  static uint16_t ain8oldest=0, ain8middle=0;
  // save some memory; these do not need to be 'static'
  //            x(n)
  uint16_t ain2newest;
  uint16_t ain9newest;
  uint16_t ain8newest;
  ADC_In298(&ain2newest, &ain9newest, &ain8newest); // sample AIN2(PE1), AIN9 (PE4), AIN8 (PE5)
  *ain2 = median(ain2newest, ain2middle, ain2oldest);
  *ain9 = median(ain9newest, ain9middle, ain9oldest);
  *ain8 = median(ain8newest, ain8middle, ain8oldest);
  ain2oldest = ain2middle; ain9oldest = ain9middle; ain8oldest = ain8middle;
  ain2middle = ain2newest; ain9middle = ain9newest; ain8middle = ain8newest;
}

void Delay(int num){
	unsigned long volatile time;
  time = 727240*num/91; 
  while(time){
		time--;
  }
}