/*  Names: Deborah Lark, Nate Pritchard, Alseny Sylla
    Section: 1B
    Date: 4/19/16
    File name: AutoCar
    Program Description:

	This program will use the ranger and compass to control the car's movement.
	An LCD display will allow for a desired heading input.
	A potentiometer will control the speed of the drive motor.
	The battery voltage of the car will be returned as well.
*/
#include <c8051_SDCC.h>
#include <stdio.h>
#include <stdlib.h>
#include <i2c.h>

//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
void Port_Init(void);
void PCA_Init (void);
void XBR0_Init();
void SMB_Init(void);
void ADC_Init(void);
void PCA_ISR (void) __interrupt 9;
unsigned int Read_Compass();
void Direction(void);
unsigned int ReadRanger (void);
void Drive_Motor(void);
unsigned char Motor_Speed(unsigned char n); //ADC P1.4
unsigned char Battery_Voltage(unsigned char m); //ADC P1.5
unsigned int Pick_Heading_Gain();
void pause(void);

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
//Compass
char new_heading;
unsigned int heading; // the heading returned in degrees between 0 and 3599
unsigned int pws;
signed int error;
signed int range_error;
//Ranger
unsigned int range = 0; 	// this is the value the ranger function will return
unsigned char new_range =0;  // this will track the overflows for the ranger
unsigned int PW_NEUT = 2769;
unsigned int MOTOR_PW = 0;  // pulse width of motor
//Counters
int r_count = 0;  // keeping track of time for the ranger
int m_count = 0; 	// keeping track of time for the motor
char h_count = 0;
char n_count = 0;
char display_count = 0;
//User Set Values
unsigned int desired_heading; //expressed in deg*10
unsigned int value;
char k; 				//error gain
char r = 18; //range gain
unsigned int center_pw = 2769; 		//steering centerline
//Other Stuff
unsigned int volts;
signed int keypad;
signed int keypad2;
unsigned char AD_speed;
unsigned char Speed_Percent;

__sbit __at 0xB7 SSs; //switch to turn wheels back to parallel
__sbit __at 0xB6 SSm; // switch to turn motor on and off


//-----------------------------------------------------------------------------
// Main Function
//-----------------------------------------------------------------------------
void main(void)
{
    // initialize board
    Sys_Init();
    putchar(' '); //the quotes in this line may not format correctly
    Port_Init();
    XBR0_Init();
    PCA_Init();
	SMB_Init();
	ADC_Init();
	printf("\rStart");
	MOTOR_PW = PW_NEUT;	
	while (m_count < 29){} //set the servo motor in neutral for one second so motor warms up
	PCA0CP2 = 0xFFFF - MOTOR_PW;
	
	pause();	//LCD start-up
	lcd_clear();
    lcd_print("Calibration:\nHello world!\n012_345_678:\nabc def ghij");
	pause();
	lcd_clear();

	Pick_Heading_Gain();
	desired_heading = value;

	while(1)
	{	
		if(!SSs)
		{
			printf("\r\nSteering Slide Switch is Off");
			pws = center_pw;
			PCA0CP0 = 0xFFFF - pws;
		}
		if(SSs)
		{
			if(new_heading) // enough overflows for a new heading
			{	
				Read_Compass();
				new_heading = 0;
			}
			if(new_range)
			{	
				ReadRanger();
				new_range = 0;
			}
			Direction();
		}
		if(!SSm)
		{
			printf("\r\nMotor Slide Switch is Off");
			MOTOR_PW = 2769; // neutral - so motor stops
			PCA0CP2 = 0xFFFF - MOTOR_PW;
		}
		if(SSm) 
		{
			Drive_Motor();
		}
		if(display_count >= 10) //display every 200ms
		{	
			//volts = Battery_Voltage(5);
			Speed_Percent = ((AD_speed*100)/255);
			printf("\r\n%u,	%u,	%u", heading, range, pws);
			lcd_clear();
			lcd_print("Heading = %u\n Distance = %u\nVoltage = %d\nSpeed_Percent = %d", heading, range, volts, Speed_Percent);
			display_count = 0;
		}
	}
}

//-----------------------------------------------------------------------------
// Initializations
//-----------------------------------------------------------------------------
void Port_Init()
{
    P1MDIN &= 0xCF; 	// Set P1.4, 1.5 for analog input
	P1MDOUT &= 0x05;  //set output pin for CEX0 or CEX2 in push-pull mode
	P1MDOUT |= 0xCF; //1.4, 1.5 to open drain
	P1 |= ~0xCF; 	// Send logic 1 to input pin P1.4
	P3MDOUT = 0x3F; //P3.7 & P3.6 switches to input
}

void XBR0_Init()
{
    XBR0 = 0x27;  //configure crossbar as directed in the laboratory
}

void SMB_Init(void)
{
	SMB0CR = 0x93;   // set SCL to 100 KHz (actual freq ~ 95,410 Hz)
	ENSMB = 1;  // bit 6 of SMB0CN, enable the SMBus
}

void PCA_Init(void)
{
    PCA0MD = 0x81;
	PCA0CPM0 = 0xC2; // CCM0 in 16 bit
	PCA0CPM2 = 0xC2; //CCM2 in 16 bit
	PCA0CN = 0x40; // Enable PCA counter
	EIE1 = 0x08; //Enable PCA interrupt
	EA = 1;
}

void ADC_Init(void)
{
	REF0CN = 0x03; 		// Set Vref to use internal reference voltage (2.4 V)
	ADC1CN = 0x80; 		// Enable A/D converter (ADC1)
	ADC1CF |= 0x01; 	// Set A/D converter gain to 1
	ADC0CN = 0x80; 		// Enable A/D converter (ADC0)
	ADC0CF |= 0x01; 	// Set A/D converter gain to 1
}

//-----------------------------------------------------------------------------
// PCA_ISR
//-----------------------------------------------------------------------------
void PCA_ISR (void) __interrupt 9
{
    if (CF)
	{
		CF = 0;
		PCA0 = 28614; //20ms per interrupt
	}
	PCA0CN &= 0xC0; //handle other interrupt sources
	h_count ++; //compass delay
	r_count ++; //ranger delay
	n_count ++;
	display_count ++;
	if(m_count < 30)
	m_count ++;
	if(h_count >= 3)
	{
		new_heading = 1;
		h_count = 0;
	}
	if (r_count >= 5)
	{
		new_range = 1;
		r_count = 0;
	}
}

//-----------------------------------------------------------------------------
// Read Compass
//-----------------------------------------------------------------------------	
unsigned int Read_Compass()
{ 
	unsigned char addr = 0xC0; // the address of the sensor, 0xC0 for the compass
	unsigned char Data[2]; // Data is an array with a length of 2

	i2c_read_data(addr, 2, Data, 2); // read two byte, starting at reg 2 

	heading =(((unsigned int)Data[0] << 8) | Data[1]); //combine the two values
	//printf("Data0 = %u     Data1 %u   \r\n",Data[0],Data[1]);
	//heading has units of 1/10 of a degree

	return heading; // the heading returned in degrees between 0 and 3599 
} 

//-----------------------------------------------------------------------------
// Read Ranger
//-----------------------------------------------------------------------------	
unsigned int ReadRanger()  // This is to read the data
{
	unsigned char Data[2];

	unsigned char addr=0xe0; //the address of the ranger is 0xe0
	i2c_read_data(addr,2,Data,2); //read 2 bytes starting at reg 2
	range = (((unsigned int) Data[0] << 8) | Data[1]);

	//start a new ping
	Data[0] = 0x51;  //Write 0x51 to reg 0 of the ranger:
	i2c_write_data(addr,0,Data,1);  //Write 1 byte of data to reg 0 at addr
	return range;
}

//-----------------------------------------------------------------------------
// Direction
//-----------------------------------------------------------------------------	
void Direction(void)
{
	error = (desired_heading - heading);
	if(error <= -1800)
		error += 3600;
	if(error > 1800)
		error -= 3600;

	if(range > 60)
		range_error = 0;
	if(range < 60)
	{
		range_error = (61-range)*r;
		error = 0;
	}

	pws = center_pw - ((error*(-k))/10) + range_error;
	
	if(pws < 2019)
	pws = 2019;
	if(pws > 3519)
	pws = 3519;

	PCA0CP0 = 0xFFFF - pws;

}



//-----------------------------------------------------------------------------
// Drive Motor
//-----------------------------------------------------------------------------	
void Drive_Motor()
{
	AD_speed = Motor_Speed(4);

	MOTOR_PW = (6 * AD_speed) + 2028;
	if(MOTOR_PW > 3502)
		MOTOR_PW = 3502;
	if(range < 10)
		MOTOR_PW = PW_NEUT;
	PCA0CP2 = 0xFFFF - MOTOR_PW; // this actually sets the motor to pw stuff
}

//-----------------------------------------------------------------------------
// Motor Speed ADC
//-----------------------------------------------------------------------------	
unsigned char Motor_Speed(unsigned char n)
{
	AMX1SL = n;		// Set P1.n as the analog input for motor speed
	ADC1CN = ADC1CN & ~0x20;		// clear the "conversion completed" flag
	ADC1CN = ADC1CN | 0x10;		// initiate the A to D conversoin
	
	while ((ADC1CN & 0x20) == 0x00)
	{
		//wait for conversion to complete
	}
	return ADC1;		// Returns digital value in ADC1 register
}

//-----------------------------------------------------------------------------
// Battery Voltage ADC
//-----------------------------------------------------------------------------	
unsigned char Battery_Voltage(unsigned char m)
{
	AMX1SL = m;		// Set P1.5 as the analog input for battery voltage
	ADC1CN = ADC1CN & ~0x20;		// clear the "conversion completed" flag
	ADC1CN = ADC1CN | 0x10;		// initiate the A to D conversoin
	
	while ((ADC1CN & 0x20) == 0x00)
	{
		//wait for conversion to complete
	}
	return ADC1;		// Returns digital value in ADC0 register
}

//-----------------------------------------------------------------------------
// Pick Heading & Gain
//-----------------------------------------------------------------------------	
unsigned int Pick_Heading_Gain()
{
	char digit1 = -1; 
	char digit2 = -1;
	char digit3 = -1;
	char digit4 = -1;
	keypad = -1;
	keypad2 = -1;
	keypad = read_keypad();
   	pause();	    // This pauses for 1 PCA0 counter clock cycle (20ms) 
                    // If the keypad is read too frequently (no delay), it will
                    // lock up and stop responding. Must power down to reset.	
	if(keypad == 0)
	printf("   **Wire Connection/XBR0 Error**   ");		
	
	lcd_print("Select Heading Type\n1 = Preset\n2 = User Defined");
	while(keypad == -1)
	{
		keypad = read_keypad();
		pause();
		if(keypad == 49)
		{
			while (keypad != -1)
			{
				keypad = read_keypad();
        		pause();
			}
			lcd_clear();
			lcd_print("Select Preset\n1=0deg 2=90deg\n3=180deg 4=270deg");
			while(keypad2 == -1)
			{
				keypad2 = read_keypad();
				pause();
				if(keypad2 == 49)
				{
					value = 0;
					break;
				}
				if(keypad2 == 50)
				{
					value = 900;
					break;
				}
				if(keypad2 == 51)
				{
					value = 1800;
					break;
				}
				if(keypad2 == 52)
				{
					value = 2700;
					break;
				}
			}
		break;
		}
		else if(keypad == 50)
		{
			while (keypad != -1)
			{
			keypad = read_keypad();
        	pause();
			}
			lcd_clear();
			lcd_print("Input Heading\n4 Digits\n0000-3600deg");		
			while(digit1 == -1)
			{
				keypad = read_keypad();
       			pause();
				if(keypad != -1)
				digit1 = keypad;
				while (keypad != -1)
				{
				keypad = read_keypad();
       			pause();
				}
			}
			lcd_clear();
			lcd_print("digit1 = %d", digit1);
			while(digit2 == -1)
			{
				keypad = read_keypad();
       			pause();
				if(keypad != -1)
				digit2 = keypad;
				while (keypad != -1)
				{
				keypad = read_keypad();
        		pause();
				}
			}
			lcd_print("\ndigit2 = %d", digit2);
			while(digit3 == -1)
			{
				keypad = read_keypad();
       			pause();
				if(keypad != -1)
				digit3 = keypad;
				while (keypad != -1)
				{
					keypad = read_keypad();
        			pause();
				}
			}
			lcd_print("\ndigit3 = %d", digit3);
			while(digit4 == -1)
			{
				keypad = read_keypad();
      		 	pause();
				if(keypad != -1)
				digit4 = keypad;
				while (keypad != -1)
				{
					keypad = read_keypad();
        			pause();
				}
			}
			lcd_print("\ndigit4 = %d", digit4);
		value = (((digit1 - 48) * 1000) + ((digit2 - 48) * 100) + ((digit3 - 48) * 10) + (digit4-48));
		break;
		}
	}
	lcd_clear();
	lcd_print("Set Steering Gain");
	while(keypad == -1) //set gain
	{
		keypad = read_keypad();
		pause();
		if(keypad != -1)
		{
			k = keypad - 48;
			lcd_print("\nk = %d", k);
			while (keypad != -1)
			{
				keypad = read_keypad();
        		pause();
			};
			break;
		}
	}
	lcd_clear();
	lcd_print("Desired Heading=%u\nSteering Gain=%d", value, k);
	printf("\r\nDesired Heading = %u	Steering Gain = %d", value, k);
	return value;
}

//-----------------------------------------------------------------------------
// Delay Functions
//-----------------------------------------------------------------------------	

void pause(void)
{
    n_count = 0;
    while (n_count < 6);
}