/*  Time of Flight for 2DX4 -- Studio W8-0
                Code written to support data collection from VL53L1X using the Ultra Light Driver.
                I2C methods written based upon MSP432E4 Reference Manual Chapter 19.
                Specific implementation was based upon format specified in VL53L1X.pdf pg19-21
                Code organized according to en.STSW-IMG009\Example\Src\main.c
                
                The VL53L1X is run with default firmware settings.


            Written by Tom Doyle
            Updated by  Hafez Mousavi Garmaroudi
            Last Update: March 17, 2020
						
						Last Update: March 03, 2022
						Updated by Hafez Mousavi
						__ the dev address can now be written in its original format. 
								Note: the functions  beginTxI2C and  beginRxI2C are modified in vl53l1_platform_2dx4.c file

*/

// Name: Arji Thaiyib
// Student Number: 400 336 020
// LED: D3 (PF4) and Bus Speed: 60MHz

#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "vl53l1x_api.h"
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"




#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up

void PortM_Init(void){ // For STOP push button (uses logic LO setup)
	//Use PortM pins for output
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;				// activate clock for Port M
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};	// allow time for clock to stabilize
	GPIO_PORTM_DIR_R |= 0x00	;        								// making PM0 an input  
  GPIO_PORTM_AFSEL_R &= ~0xFF;     								// disable alt funct on PN0
  GPIO_PORTM_DEN_R |= 0xFF;        								// enable digital I/O on PN0
																									
  GPIO_PORTM_AMSEL_R &= ~0xFF;     								// disable analog functionality on PN0		
	return;
}

void PortJ_Init(void){ // For onboard push button, PJ1, to record measurements (input)
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;					// activate clock for Port J
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};	// allow time for clock to stabilize
  GPIO_PORTJ_DIR_R &= ~0x02;    										// make PJ1 in 
  GPIO_PORTJ_DEN_R |= 0x02;     										// enable digital I/O on PJ1
	
	GPIO_PORTJ_PCTL_R &= ~0x000000F0;	 								//  configure PJ1 as GPIO 
	GPIO_PORTJ_AMSEL_R &= ~0x02;											//  disable analog functionality on PJ1		
	GPIO_PORTJ_PUR_R |= 0x02;													//	enable weak pull up resistor
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortL_Init(void){ // For verifying the assigned bus speed
  //Use Port M
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R10;             // Activate clock for Port N
  while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R10) == 0){};    // Allow time for clock to stabilize
  GPIO_PORTL_DIR_R &= 0x04;                            // Make PG0 in (HiZ)
  GPIO_PORTL_DIR_R |= 0x04;                            // Make PG0 in (HiZ)

    GPIO_PORTL_AFSEL_R &= ~0x07;                         // Disable alt funct on PG0
  GPIO_PORTL_DEN_R |= 0x07;                            // Enable digital I/O on PG0
  GPIO_PORTL_AMSEL_R &= ~0x07;                         // Disable analog functionality on PN0
  return;
}


void I2C_Init(void){ // For initializing ToF sensor 
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        						// 8) configure for 100 kbps clock
        
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}
void PortH_Init(void){ // For stepper motor
SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7; //activate the clock for Port M
while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){}; //allow time for clock to stabilize
	GPIO_PORTH_DIR_R = 0b00001111; // Make PM0:PM3 inputs, reading if the button is pressed or not
	GPIO_PORTH_DEN_R = 0b00001111; // Enable PM0:PM3
return;
}

//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}

void spin_Clockwise()						//function to spin clockwise 45 degrees
{
		for(int i=0; i<64; i++){						//one full rotation through full stepping takes 2048 steps
			GPIO_PORTH_DATA_R = 0b00001001;			//we have four steps in loop and wish to go 1/8 of the way
			SysTick_Wait10ms(2);								//therefore i = (2048/4)/8 = 64
			GPIO_PORTH_DATA_R = 0b00000011;
			SysTick_Wait10ms(2);
			GPIO_PORTH_DATA_R = 0b00000110;
			SysTick_Wait10ms(2);
			GPIO_PORTH_DATA_R = 0b00001100;
			SysTick_Wait10ms(2);
	}
}

void spin_CounterClockwise()						//function to spin clockwise 45 degrees
{
		for(int i=0; i<64; i++){						//one full rotation through full stepping takes 2048 steps
			GPIO_PORTH_DATA_R = 0b00001100;		//therefore 11.25 deg is 64 step
			SysTick_Wait10ms(2);							//since we're doing 4 steps in the loop we do 64/4=16
			GPIO_PORTH_DATA_R = 0b00000110;   // ** isn't it just same as CW just diff order? **
			SysTick_Wait10ms(2);
			GPIO_PORTH_DATA_R = 0b00000011;
			SysTick_Wait10ms(2);
			GPIO_PORTH_DATA_R = 0b00001001;
			SysTick_Wait10ms(2);
		}
}


//*********************************************************************************************************
//*********************************************************************************************************
//***********					MAIN Function				*****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************
uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status=0;
int direction = 0;


int main(void) {
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;
	uint8_t id;
	uint8_t type;
	uint16_t both;

	//initialize
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	PortH_Init();
	PortJ_Init();
	PortM_Init();
	PortL_Init();
	
	
    //for(int i = 0; i < 5000000; i++){ // to verify the assigned bus speed
        //GPIO_PORTL_DATA_R ^= 0b00000100; // using PL2
        //SysTick_Wait10ms(1);
    //}
		
/* Those basic I2C read functions can be used to check your own I2C functions 
	status = VL53L1X_GetSensorId(dev, &wordData);

	sprintf(printf_buffer,"(Model_ID, Module_Type)=0x%x\r\n",wordData);
	UART_printf(printf_buffer);
*/
	// Booting ToF chip
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState); //1) boot ToF sensor
		SysTick_Wait10ms(10);
  }
	//UART_printf("ToF Chip Booted!\r\n Please Wait...\r\n");
	
	status = VL53L1X_ClearInterrupt(dev); /* 2) clear interrupt has to be called to enable next interrupt*/
	
  /* 3) This function must to be called to initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);

	
  /* Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */
//  status = VL53L1X_SetDistanceMode(dev, 2); /* 1=short, 2=long */
//  status = VL53L1X_SetTimingBudgetInMs(dev, 100); /* in ms possible values [20, 50, 100, 200, 500] */
//  status = VL53L1X_SetInterMeasurementInMs(dev, 200); /* in ms, IM must be > = TB */

  status = VL53L1X_StartRanging(dev);   // 4) This function has to be called to enable the ranging (measurement process begins)

	//status = VL53L1_RdByte(dev, 0x010F, &id); //for model ID (0xEA)
	//status = VL53L1_RdByte(dev, 0x0110, &type); //for module type (0xCC)
	//status = VL53L1_RdWord(dev, 0x010F, &both); //for both model ID and type
	//sprintf(printf_buffer,"%x, %x, %x\r\n",id, type, both);
	// Get the Distance Measures 50 times
	int counter = 0;		

	while(counter < 3){
		
		if(GPIO_PORTJ_DATA_R == 0b00000000){
			counter++;
			for(int i = 0; i < 8; i++) {
				
				if((GPIO_PORTM_DATA_R&0b00000001)==0){ //STOP BUTTON: if pressed output 0, breaks loop
					counter = 10;
          break;
        }else{
        }
				//5) wait until the ToF sensor's data is ready
				while (dataReady == 0){
					status = VL53L1X_CheckForDataReady(dev, &dataReady);
					FlashLED3(1); //Assigned LED D3 blinks
					VL53L1_WaitMs(dev, 5);
				}
				dataReady = 0;
				status = VL53L1X_GetDistance(dev, &Distance);					//The Measured Distance value (&Distance) 

				if(direction%2==0){ //to avoid tangling
					spin_Clockwise();	
				}else {
					spin_CounterClockwise(); 
				}
				

				status = VL53L1X_ClearInterrupt(dev); /*clear interrupt has to be called to enable next interrupt*/
				
				// print the resulted readings to UART
				sprintf(printf_buffer,"%u\n",Distance);
				UART_printf(printf_buffer);
				SysTick_Wait10ms(50);
						
			}
		}else{
			SysTick_Wait10ms(50);
		}
		
		direction++;

	}
	VL53L1X_StopRanging(dev);
  while(1) {}

}

