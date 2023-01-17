# **COMPENG 2DX4 Final Project:** Spacial Mapping System 

<p align="center"><i>
Please refer to the Project Report pdf for more details.
</i></p>

## **Project Overview**

---

### **The Challenge**
- Design and build an embedded spatial measurement system using a time-of-flight sensor to acquire information about the area around you. Using a rotary mechanism to provide a 360 degree measurement of distance within a single vertical geometric plane (e.g., y-z).
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
<img width="800" alt="Block Diagram" src="https://user-images.githubusercontent.com/113951482/212814075-86b92ad7-0976-4f54-8ec0-a18579ab3380.png">

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

## **Hallway Reconstruction Renders**
---
![Picture2](https://user-images.githubusercontent.com/113951482/212817259-7f25f4ca-1cd7-4e93-8fe8-e58fd874ee7c.png)
<p align="center"><i>
<b>Figure 3: </b> Open3D (Python OpenGL library) render including dimensions.
</i></p>
As seen above, the figure shows all the axis planes. The x plane (displacement) is representative of the device moving forward, while the y and z planes (spatial) are vertical slices which the sensor reads.

## **Circuit Schematic**
---
![Picture3](https://user-images.githubusercontent.com/113951482/212819508-5d9a45f1-82ae-443b-83cf-2476e87935d4.png)
<p align="center"><i>
<b>Figure 4: </b> Circuit Schematic
</i></p>

## **Programming Logic Flow**
---
![Untitled Diagram drawio (15)](https://user-images.githubusercontent.com/113951482/212819721-137b8ef2-7ca6-40e8-922b-3e1f74cc40f6.png)

## **Code Snippets**
---
### <b>Port Initialization</b>

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
