/*  Names: Deborah Lark, Nate Pritchard, Alseny Sylla
    Section: 1B
    Date: 5/10/16
    File name: Lab 5
    Program Description:

	This program will use the accelerometer to control the car's movement
	as it transverses up and down a ramp.
	An LCD display will allow for a desired heading and gains input.
	The battery voltage of the car will be returned as well.
*/
#include <c8051_SDCC.h>
#include <stdlib.h>
#include <stdio.h>
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
void Accel_Init(void);
//void updateLCD(void);

void calibrate ();
void read_accel();
void read_things ();
void Direction(void);
void Set_Gains();
void pause(void);
void steering_fb (void);
void drive_fb (void);

void Drive_Motor(void);
unsigned char Potmeter(unsigned char n); //ADC P1.4
unsigned char Battery_Voltage(unsigned char m); //ADC P1.5

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
//unsigned int center_pw = 2769; 		//steering centerline
unsigned int pws;

//Motor
//unsigned int PW_NEUT = 2769;
unsigned int MOTOR_PW = 0;  // pulse width of motor

//Accelerometer
unsigned int status_reg_a = 0;
signed int avg_gx = 0;
signed int avg_gy = 0;
signed int gx = 0; //average of x readings
signed int gy = 0; //average of y readings
char kdx = 0; //drive feedback set by user
char kdy = 0; //drive feedback set by potmeter
char ks = 0; //steering feedback gain set by user 

//Counters
char m_count = 0; 	// keeping track of time for the motor
char a_count = 0;	//acceleration count
char new_accel = 0;
char lcd_count = 0; 
char new_lcd = 0;
char n_count = 0;
char display_count = 0;
unsigned char read_index = 0;

//Other Stuff
unsigned int volts;
signed int keypad;
signed int keypad2;
signed int x0;
signed int y0;
char up = 0;
char down = 0;
unsigned char gain_set =0;

__sbit __at 0xB6 SS; // switch to turn car on and off

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
	Accel_Init();  

	printf("\n\rStart");
	MOTOR_PW = 2769; //PW_NEUT	
	while (m_count < 29) //set the servo motor in neutral for one second so motor warms up
	
	PCA0CP2 = 0xFFFF - MOTOR_PW;

	pause();	//LCD start-up
	pause();
	lcd_clear();
    lcd_print("Calibration:\nHello world!\n012_345_678:\nabc def ghij");
	pause();
	
	lcd_clear();
	printf("\r\nLCD: Set Initial Gains");
	Set_Gains();  //uses LCD Keyboard to choose initial gains	
	pause();

	m_count =0;
	while (m_count<4); //wait
	
	//Calibrate the car 
	printf("\r\n Calibrating");
	calibrate();
	printf("\r\nCalibration complete, Flip switch to start car");
	printf("\r\nPrint Values:\r\nGX, GY, Motor_PW, Steering_PW");
	while(1)
	{	
		if (!SS)	//wait while bigger switch is off
		{
			MOTOR_PW = 2769; // neutral - so motor stops
			PCA0CP2 = 0xFFFF - MOTOR_PW;
		}
		
		if (SS)
		{
			if (new_accel) //enough overflows for a new acceleration reading
			{
				read_accel ();
				gain_set = Potmeter(6);
				Direction ();
				new_accel = 0;
			}

			if(display_count >= 10) // every 200 ms
			{	
				volts = Battery_Voltage(5);
				lcd_clear();
				lcd_print("Volt=%d	\ngx= %d	gy= %d	\nkdy=%u kdx=%u ks=%u\nM:%u	S: %u", volts, gx, gy, kdy, kdx, ks, MOTOR_PW, pws);
				printf("\r\n%u, %u, %u, %u", gx, gy, MOTOR_PW, pws);
				display_count = 0;
			}
		}
	}
}

//-----------------------------------------------------------------------------
// Initializations
//-----------------------------------------------------------------------------
void Port_Init()
{
    P1MDIN &= 0x9F; 	// Set P1.4, 1.6 for analog input
	P1MDOUT |= 0x05;  //set output pin for CEX0 or CEX2 in push-pull mode
	P1MDOUT &= 0x9F; //1.4, 1.5 to open drain (set to 0)
	P1 |= ~0x9F; 	// Send logic 1 to input pin P1.4
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

void ADC_Init(void)
{
	REF0CN = 0x03; 		// Set Vref to use internal reference voltage (2.4 V)
	ADC1CN = 0x80; 		// Enable A/D converter (ADC1)
	ADC1CF |= 0x01; 	// Set A/D converter gain to 1
}

void PCA_Init(void)
{
    PCA0MD = 0x81;
		PCA0CPM0 = 0xC2; // CCM0 in 16 bit
		PCA0CPM1 = 0xC2;
		PCA0CPM2 = 0xC2; //CCM2 in 16 bit
		PCA0CPM4 = 0xC2;
	PCA0CN = 0x40; // Enable PCA counter
	EIE1 = 0x08; //Enable PCA interrupt
	EA = 1;
}

void PCA_ISR (void) __interrupt 9
{
    if (CF)
	{
		CF = 0;
		PCA0 = 28614; //20ms per interrupt
	}
	n_count ++; //used for the pause function for LCD
	display_count ++; //for showing things on the LCD
	a_count ++; //acceleration delay
	lcd_count ++;
	if(m_count < 30) m_count ++; //allows for motor to start up
	if(a_count >= 2) //every 40ms new accel read
	{
		new_accel = 1;
		a_count = 0;
	}

	PCA0CN &= 0xC0; //handle other interrupt sources
}
//-----------------------------------------------------------------------------
// Accelerometer things
//-----------------------------------------------------------------------------
void calibrate () // this is used to calibrate the coad to this specific accelomiter
{
	unsigned char read_index = 0;
	signed int xxx = 0;
	signed int yyy = 0;
	unsigned char Data[4]; //array can hold 4 numbers
	unsigned char addr= 0x30; //the address of the accelerometer

	//to cut back on noise, average readings together
	keypad = -1;
	lcd_clear();
	lcd_print("Press  any  key  to start calibration!!!");
	while (keypad == -1)
	{
		keypad = read_keypad();
		pause();
	}
	lcd_clear();
	lcd_print("Calibrating...");
	while (read_index < 64) //gather 4 data points
	{
		i2c_read_data(addr, 0x27,Data,2); //read 2 bytes starting at reg 0x27
	
		if((Data[0] & 0x03) == 0x03) //Make sure the 2 LSbits are high --> this is checking the status reg
		{
			//status_reg_a = (((unsigned int) Data[0] << 8) | Data[1]);
			i2c_read_data(addr_accel, 0x28|0x80, Data, 4); //assert MSB to read mult. Bytes
			//sums data reads togeather
			xxx += ((Data[1] << 8) >> 4); 
			yyy += ((Data[3] << 8) >> 4); 
		}
		read_index += 1;
	}
	x0 = xxx/read_index;
	y0 = yyy/read_index;
	lcd_clear();
	lcd_print("Calibration Complete\nx0 = %u\ny0 = %u",x0,y0);
}

void read_accel() 
// side to side tilt sets PWs for steering servo
// side to side, front to back tilt sets the PWmotor
{
	unsigned char read_index = 0;
	unsigned char numb_times_read = 8;

	//to cut back on noise, average readings togeather
	while (read_index < numb_times_read) //gather 4 data points
	{
		read_things();
		read_index += 1;
	}	
	avg_gx = avg_gx/numb_times_read;
	avg_gy = avg_gy/numb_times_read;
	
	gx = avg_gx - x0; //find nominal gx offset
	gy = avg_gy - y0; //find nominal gy offset
	avg_gx = 0;
	avg_gy = 0;
}

void read_things()
{
	unsigned char Data[4]; //array can hold 4 numbers
	unsigned char addr= 0x30; //the address of the accelerometer
	i2c_read_data(addr, 0x27,Data,2); //read 2 bytes starting at reg 0x27
	
	if((Data[0] & 0x03) == 0x03) //Make sure the 2 LSbits are high --> this is checking the status reg
	{
		//status_reg_a = (((unsigned int) Data[0] << 8) | Data[1]);
		i2c_read_data(addr_accel, 0x28|0x80, Data, 4); //assert MSB to read mult. Bytes
		//sums data reads togeather
		avg_gx += ((Data[1] << 8) >> 4); 
		avg_gy += ((Data[3] << 8) >> 4); 
	}
	
	else 
	{
	// only calls this if it wasn't able to take desired # of reads
	//	printf("\r\nsomething went wrong with reading data\r\n"); 
	}
}
//-----------------------------------------------------------------------------
// Direction
//-----------------------------------------------------------------------------	
void Direction(void)
{
//(ks is the steering feedback gain --> set by user
//(kdy is the y-axis drive feedback gain --> set by potmeter
//(kdx is the x-axis drive feedback gain --> set by user
	kdy =(gain_set/5);  //allows to the gain to vary from 1 to 50
	
	if (up) 	
	{
		pws = 2765 - (ks * gx); //if pointed up, steer up the hill
		MOTOR_PW = 2765 + ((int)kdy * gy) + (kdx * abs(gx));
		if (abs(gy)<10 && abs(gx)<5) MOTOR_PW = 2765;
	}
	if (down)		
	{
		pws = 2765 + (ks * gx); //if pointed down, steer down the hill
		MOTOR_PW = 2765 - ((int)kdy * gy); //gy is neg going down hill, so sub to make pos
		MOTOR_PW += kdx * abs(gx); 	//corrects for side-to-side tilt
		if (abs(gy)<10 && abs(gx)<5) MOTOR_PW = 2765;
	}

	// these are the hard stops
	if(pws < 2019) pws = 2019;
	if(pws > 3519) pws = 3519;
	if(MOTOR_PW < 2028) MOTOR_PW = 2028;
	if(MOTOR_PW > 3502) MOTOR_PW = 3502;
	
	PCA0CP0 = 0xFFFF - pws;
	PCA0CP2 = 0xFFFF - MOTOR_PW; 
}

//-----------------------------------------------------------------------------
//A/D CONVERSIONS
//-----------------------------------------------------------------------------	
unsigned char Potmeter(unsigned char n) //ADC  --> POTMETER
{
	AMX1SL = n;		// Set P1.4 as the analog input for motor speed
	ADC1CN = ADC1CN & ~0x20;		// clear the "conversion completed" flag
	ADC1CN = ADC1CN | 0x10;		// initiate the A to D conversoin
	
	while ((ADC1CN & 0x20) == 0x00);	//wait for conversion to complete
	return ADC1;		// Returns digital value in ADC1 register
}
	
unsigned char Battery_Voltage(unsigned char m)// Battery Voltage ADC
{
	AMX1SL = m;		// Set P1.5 as the analog input for battery voltage
	ADC1CN = ADC1CN & ~0x20;		// clear the "conversion completed" flag
	ADC1CN = ADC1CN | 0x10;		// initiate the A to D conversoin
	
	while ((ADC1CN & 0x20) == 0x00);	//wait for conversion to complete
	return ADC1;		// Returns digital value in ADC1 register
}

//-----------------------------------------------------------------------------
// Pick Heading & Gain
//-----------------------------------------------------------------------------	
void Set_Gains()
{
	keypad = -1;
	keypad2 = -1;
	keypad = read_keypad();
   	pause();	    // This pauses for 1 PCA0 counter clock cycle (20ms) 
                    // If the keypad is read too frequently (no delay), it will
                    // lock up and stop responding. Must power down to reset.	
	if(keypad == 0)
	printf("   **Wire Connection/XBR0 Error**   ");		
	
	lcd_print("Select KS Gain Type\n1 = Preset\n2 = User Defined");
	while(keypad == -1)
	{
		keypad = read_keypad();
		pause();
		if(keypad == 49) // user selected "Preset"
		{
			while (keypad != -1)
			{
				keypad = read_keypad();
        		pause();
			}
			lcd_clear();
			lcd_print("Select Preset\n1=2 2=4\n3=6 4=8");
			while(keypad2 == -1)
			{
				keypad2 = read_keypad();
				pause();
				if(keypad2 == 49)
				{
					ks = 2;
					break;
				}
				if(keypad2 == 50)
				{
					ks = 4;
					break;
				}
				if(keypad2 == 51)
				{
					ks = 6;
					break;
				}
				if(keypad2 == 52)
				{
					ks = 8;
					break;
				}
			}
		break;
		}
		else if(keypad == 50)// user selected "user defined"
		{
			lcd_clear();
			lcd_print("Steering feedback \nGain (ks)");;
			ks = kpd_input(0); 
			lcd_clear();
		}
	}
	keypad = -1;
	keypad2 = -1;
	
	lcd_print("Select Kdx Gain Type\n1 = Preset\n2 = User Defined");
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
			lcd_print("Select Preset\n1=2 2=4\n3=6 4=8");
			while(keypad2 == -1)
			{
				keypad2 = read_keypad();
				pause();
				if(keypad2 == 49)
				{
					kdx = 2;
					break;
				}
				if(keypad2 == 50)
				{
					kdx = 4;
					break;
				}
				if(keypad2 == 51)
				{
					kdx = 6;
					break;
				}
				if(keypad2 == 52)
				{
					kdx = 8;
					break;
				}
			}
			break;
		}
		else if(keypad == 50)
		{
			lcd_clear();
			lcd_print("X FeedbackGain (kdx)");
			kdx = kpd_input(0);
			lcd_clear();
		}
	}
	keypad = -1;
	keypad2 = -1;
	up = 0;
	down = 0;
	lcd_print("Am I going up (1)\n or down (2)?");
	while(keypad == -1)
	{
		keypad = read_keypad();
		pause();
		if(keypad == 49)	up = 1;
		
		else if(keypad == 50) 	down = 1;
		pause();
		pause();
		pause();
	}

	lcd_clear();
	printf("\r\nSteering Gain=%u	X Drive Gain=%u", ks, kdx);
	lcd_print("\nSteering Gain=%u\nX Drive Gain=%u", ks, kdx);
}
//-----------------------------------------------------------------------------
// Delay Functions
//-----------------------------------------------------------------------------	

void pause(void)
{
    n_count = 0;
    while (n_count < 6);
}