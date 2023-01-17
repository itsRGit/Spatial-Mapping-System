# **COMPENG 2DX4 Final Project:** Spacial Mapping System 

<p align="left"><i>
Please refer to the Project Report pdf for more details.
</i></p>

## **Project Overview**

---

### **The Challenge**
- Design and build an embedded spatial measurement system using a time-of-flight (ToF) sensor to acquire information about the area around you. Using a rotary mechanism to provide a 360 degree measurement of distance within a single vertical geometric plane (e.g., y-z).
- Ensure that the mapped spatial information is stored in onboard memory and later communicated to a personal computer or web application for reconstruction and graphical presentation.

### **General Description of my Design**
&nbsp;&nbsp;&nbsp;&nbsp;The motherboard for this specific design is comprised of the Simple Link MSP-EXP432E401Y which is a 32-biit ARM Cortext-M4F based microcontroller and can perform multiple required tasks. It enables the configuration of the ToF sensor and by using the I2C communication protocol the ToF sensor’s distance measurement readings can be communicated with and collected by the microcontroller. Every 45° rotation of the stepper motor the sensor collects data for a total of 8 readings with the onboard LED (specific to my student number, D3) lighting up with each iteration, as a representation of the ToF’s sensor’s status during this process. This by convention would also mean the MCU is responsible for the stepper motor, as it will rotate at a specific frequency, for a total of 3 complete 360° revolutions.

&nbsp;&nbsp;&nbsp;&nbsp;When the onboard button, PJ1, is pressed it starts the next set of readings after a complete rotation has finished and 8 readings collected (at a new set displacement, or x-value). Moreover, the MCU is able to communicate with my personal PC via UART communication protocol. Lastly, I also implemented a logic LO push button, connected to a 3.3V source, to completely stop the stepper motor and ToF sensor, effectively being a reset button.

&nbsp;&nbsp;&nbsp;&nbsp;The MOT-28BYJ48 Stepper Motor, connected to a 5V input voltage and Port H, is also used in the design configuration to rotate the ToF sensor a complete 360° revolution. Speaking of which, the VL53LIX Time-of-Flight sensor is used to measure distance using LIDAR technology. It calculates the amount of time a photon of light takes to bounce off of an object and be reflected back; and from this time measurement the distance can further be calculated. The sensor as mentioned previously will be attached the stepper motor and take measurements every 45°.

## **Built Using**
---
- <b>Python:</b> OpenGL library, Pyserial 
- <b>C:</b> Extracting Distance Measurements, I2C and UART Communication
- <b>Assembly:</b> System Clock Setting, Port Configurations
- <b>MCU:</b> SimpleLink MSP432E401Y Microcontroller
- <b>Hardware:</b> Stepper Motor, Time of Flight Sensor, Breadboard, Resistors

## **Block Diagram**
---
<p align="center">
<img width="800" alt="Block Diagram" src="https://user-images.githubusercontent.com/113951482/212814075-86b92ad7-0976-4f54-8ec0-a18579ab3380.png">
</p>

## **Setup**
---
![IMG-3488](https://user-images.githubusercontent.com/113951482/212814314-1c5d877a-dcea-4651-905c-546c7f99b70f.jpg)
<p align="center"><i>
<b>Figure 1: </b> Circuit configuration with all its components.
</i></p>

<p align="center">
<img width="500" alt="Cardboard box used as a makeshift hallway" src="https://user-images.githubusercontent.com/113951482/212815986-19e61742-f064-439f-bb00-7cf5925b075b.jpg">
</p>
<p align="center"><i>
<b>Figure 2: </b> Cardboard box used as a makeshift hallway.
</i></p>

## **Hallway Reconstruction Render**
---
<p align="center">
<img width="500" alt="Hallway Reconstruction Render" src="https://user-images.githubusercontent.com/113951482/212817259-7f25f4ca-1cd7-4e93-8fe8-e58fd874ee7c.png">
</p>
<p align="center"><i>
<b>Figure 3: </b> Open3D (Python OpenGL library) render including dimensions.
</i></p>
As seen above, the figure shows all the axis planes. The x plane (displacement) is representative of the device moving forward, while the y and z planes (spatial) are vertical slices which the sensor reads.

## **Circuit Schematic**
---
<p align="center">
<img width="500" alt="Circuit Schematic" src="https://user-images.githubusercontent.com/113951482/212819508-5d9a45f1-82ae-443b-83cf-2476e87935d4.png">
</p>
<p align="center"><i>
<b>Figure 4: </b> Circuit Schematic
</i></p>

## **Programming Logic Flow**
---
<p align="center">
<img width="500" alt="Programming Logic Flow" src="https://user-images.githubusercontent.com/113951482/212819721-137b8ef2-7ca6-40e8-922b-3e1f74cc40f6.png">
</p>
<p align="center"><i>
<b>Figure 5: </b> Programming Logic Flowchart for MCU and Python Code
</i></p>

## **Code Snippets**
---
### <b>Port Initialization for Push Button:</b>
```assembly
void PortJ_Init(void){ // For onboard push button, PJ1, to record measurements (input)
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;					// activate clock for Port J
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};	// allow time for clock to stabilize
  GPIO_PORTJ_DIR_R &= ~0x02;    										// make PJ1 in 
  GPIO_PORTJ_DEN_R |= 0x02;     										// enable digital I/O on PJ1
	
	GPIO_PORTJ_PCTL_R &= ~0x000000F0;	 								//  configure PJ1 as GPIO 
	GPIO_PORTJ_AMSEL_R &= ~0x02;											//  disable analog functionality on PJ1		
	GPIO_PORTJ_PUR_R |= 0x02;													//	enable weak pull up resistor
}
```

### <b>Initialization of I2C Protocols to be able to Communicate with the ToF Sensor:</b>
```assembly
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
```

### <b>On-Board Port Initialization (serves as an OFF button):</b>
```assembly
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
```

### <b>Stepper Motor Rotation Functions (using a half-step method):</b>
```assembly
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
```

### <b>Extracting the Distance Measurements Using I2C and UART Communication Protocols:</b>
```assembly
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
```
