//Can Bus Simulator
//See function radtest for transmitting data


#include "c8051F040.h"	
#include "KL_telematics.h"						
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

sbit LED1 = P2^0; 						// led output pin 
sbit VR_BUTTON  	= P1^4;				// sw1 = 0 means switch pressed
sbit PH_BUTTON 		= P1^3;				// sw2 = 0 means switch pressed
sbit PH_BUTTON_END	= P1^2;
sbit VOL_UP			= P3^4;
sbit VOL_DOWN		= P3^5;
sbit IGN_RUN_0  	= P1^5;				// sw3 = 0 means switch pressed
sbit SEL_CAN_B 		= P1^7;				// CAN select pin 0 means can B, 1 means CAN c/i
sbit CAN_B_EN  		= P1^6;				// CAN B enable pin,  0 means enable, 1 means disable
					//Added for 6 position switch
sbit IGN_RUN_5		= P0^6;
sbit IGN_RUN_10 	= P0^5;				// sw4 = 0 means switch pressed
sbit IGN_START		= P1^0;
sbit IGN_OFF 		= P0^7;				// sw5 = 0 means switch pressed
sbit IGN_ACC 		= P1^1;				// sw6 = 0 means switch pressed
//-----------------------------------------------------------------------------
// Global Constants
//-----------------------------------------------------------------------------

#define SYSCLK       24500000          // System Clock
#define SAR_CLK      2500000           // Desired SAR clock speed
#define numb 		 6


struct txframe
{
	unsigned char MsgNum;
	unsigned char sz;
	unsigned long arbID;
 	unsigned char cdat[8];

}txframe;
CANFRAME xdata rxframe[32];

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

long ResultL, ResultH, Result, sw_inc;                           // ADC0 decimated value
int sw, powernet_flg, CUSW_flg;
char MsgNum;
char status, temp;
int i;
int v;
int j = 0;
int h;
int iii = 0;

//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
int ADC0_Analog_readbutton(char button);
void Powernet_shutdown(void);
void CUSW_shutdown(void);
void ADC0_Init (void);
int ADC0_Analog_sel(void);
void config_CAN_timing(void);
void config_IO(void);
void receive_data(char MsgNum);
void clear_msg_objects(void);
void CUSW_radiotest(void);
void Powernet_radiotest(void);
void codedelay(unsigned int delay);
void clear_message_object (unsigned char mesgobj,unsigned char count);
void onehundredms(void);
void shortdelay(void);
void CUSW_IGN_MSG(void);
void Powernet_IGN_MSG(void);

void main(void) 
{
  
  SEL_CAN_B = 1;
  CAN_B_EN  = 1;
  LED1      = 1;
  v = 0;
  // disable watchdog timer
  WDTCN = 0xde;
  WDTCN = 0xad;
    XBR0 = 0;								// nothing used
	XBR1 = 0;								// nothing used
	
	P4MDOUT = 0x00;
	P5MDOUT = 0x00; 
	P6MDOUT = 0x00;
	P7MDOUT = 0x00;

	
	
  //configure Port I/O
  config_IO();

     // Init ADC
   ADC0_Init ();  						
   
   // Enable the ADC                     
   ADC0CN |= 0x80;

	////////////////////////////////////////
	// switch to external oscillator
	////////////////////////////////////////
	SFRPAGE = CONFIG_PAGE;      	  		// switch to config page to config oscillator
	OSCXCN = 0x67;                			// start external oscillator; 18.0/24.0 MHz Crystal
	codedelay(255);							// delay about 1ms  
	while ((OSCXCN & 0x80) == 0);   
	CLKSEL |= 0x01;               			// switch to external oscillator


  // Clear CAN RAM
  clear_msg_objects();


  //Function call to start CAN
  
  config_CAN_timing();


  clear_message_object(1,32);
   	EIE2 = 0x20;
	EIP2 |= 0x20;							// set priority high 

   	//Function call to start CAN
	SFRPAGE = CAN0_PAGE;
	CAN0CN = 0x0A;				   			//Enables Int's, Error and IE (rx or tx)

  //Loop and wait for interrupts*/
  while (1)
    {
		// poll for current switch number
		sw = ADC0_Analog_sel();	
		//sw = 1;
		switch(sw)
		{

			case 1:

				if(powernet_flg != 0)
				{
					Powernet_shutdown();
					powernet_flg = 0;
				}
				
				CUSW_flg = 1;

				CUSW_IGN_MSG();
				if (IGN_OFF != 0)
				{
					CUSW_radiotest();
					iii = 0;
				}


			break;

			case 2:

				if(CUSW_flg != 0)
				{
					CUSW_shutdown();
					CUSW_flg = 0;
				}
				
				powernet_flg = 1;

				Powernet_IGN_MSG();
				if (IGN_OFF != 0)
				{
					Powernet_radiotest();
					iii = 0;
				}
			break;
			case 3:
			break;
			case 4:
			break;
			case 5:
			break;
			case 6:
			break;
			default:
			break;
		}

		//sw = ADC0_Analog_sel();
		
	}	    
}

byte tx_can_frame (unsigned char KWPopts, int BIT_ID) 
{   		
	int timeout = 4096;						// roughley 2.5ms

	EIE2 &= ~0x01;                 			// disable Timer3 interrupts

	SFRPAGE = CAN0_PAGE;
	CAN0STA &= ~BIT3;						// Clear status bits

	CAN0ADR = IF1CMDMSK;          			// Point to Command Mask 1
	CAN0DAT = 0x00B7;             			// Config to WRITE to CAN RAM, write data bytes, set TXrqst/NewDat, Clr IntPnd
	CAN0ADR = IF1ARB1;


	if (BIT_ID == 29)
	{
		//29BIT XTD ID
		CAN0DAT = (unsigned int)(txframe.arbID & 0x0000FFFF);
		CAN0DAT = ((unsigned int)((txframe.arbID & 0x7FFF0000) >> 16)) | 0xe000;
	}

	else
	{
		// 11BIT ID
		CAN0DAT = 0x0000;					// Arb1 Data			
		CAN0DAT = ((unsigned int)(txframe.arbID << 2)) | 0xA000;
	}

	CAN0ADR = IF1MSGC;				 		// Data Size

	CAN0DAT = 0x0880 | txframe.sz;			// w/ TxIE set						 


	CAN0ADR = IF1DATA1;           			// Point to 1st byte of Data Field	
	CAN0DAT = (unsigned int)txframe.cdat[0] | ((unsigned int)txframe.cdat[1] << 8);
	CAN0DAT = (unsigned int)txframe.cdat[2] | ((unsigned int)txframe.cdat[3] << 8);
	CAN0DAT = (unsigned int)txframe.cdat[4] | ((unsigned int)txframe.cdat[5] << 8);
	CAN0DAT = (unsigned int)txframe.cdat[6] | ((unsigned int)txframe.cdat[7] << 8);

	CAN0ADR = IF1CMDRQST;         			// Point to Command Request Reg.
	CAN0DATL = txframe.MsgNum;     			// Move new data for TX to Msg Obj "MsgNum"

}





void config_CAN_timing(void)
{
	SFRPAGE = CAN0_PAGE;
	CAN0CN |= 0x41;							// Set CCE bit to enable write access
	CAN0ADR = BITREG;						// Point to the Bit Timing Register

	CAN0DATH = 0x5A;					// 18.0mhz BRP=7 125kbps
	CAN0DATL = 0xC7;					// Tseg2=6, Tseg1=11, SJW=4
}

void config_IO(void)
{
  

char SFRPAGE_SAVE = SFRPAGE;

		SFRPAGE = CONFIG_PAGE;
		
		
		P0MDOUT  = 0x03;			//pin P0^0 & P0^1 are push pull
		P2MDIN 	|= 0x01;			//pin P2^0 is digital
		P2MDOUT  = 0x01;	 		//pin P2^0 is push pull	
		P1MDOUT  = 0x00;			//Port 1 is open drain
		P1MDIN	 = 0xff; 

		P3MDOUT  = 0xFE;
		P3MDIN	 = 0x00;

		//P3 		|= 0xFE;

		XBR2	 = 0x40;			// enable crossbar and disable weak pullups ***changed from 0x40 to 0x00
		XBR3     = 0x80;     // Configure CAN TX pin (CTX) as push-pull digital output	
		SFRPAGE = SFRPAGE_SAVE;


}


void receive_data(char MsgNum)
{
  
  SFRPAGE  = CAN0_PAGE; // IF1 already set up for RX
  CAN0ADR  = IF2CMDRQST;// Point to Command Request Reg.
  CAN0DATL = MsgNum;    // Move new data for RX from Msg Obj "MsgNum"
                        // Move new data to a
  CAN0ADR  = IF2DATA1;  // Point to 1st byte of Data Field

  
}


void ISRname(void) interrupt 19
{
  status = CAN0STA;
  if ((status&0x10) != 0)
    {                            // RxOk is set, interrupt caused by reception
      CAN0STA = (CAN0STA&0xEF)|0x07;         // Reset RxOk, set LEC to NoChange
      /* read message number from CAN INTREG */
      receive_data (0x01);             // Up to now, we have only one RX message
    }
  if ((status&0x08) != 0)
    {                            // TxOk is set, interrupt caused by transmision
      CAN0STA = (CAN0STA&0xF7)|0x07;        // Reset TxOk, set LEC to NoChange
    }
  if (((status&0x07) != 0)&&((status&0x07) != 7))
    {                           // Error interrupt, LEC changed
      /* error handling ? */
      CAN0STA = CAN0STA|0x07;              // Set LEC to NoChange
    }
}

void clear_msg_objects(void)
{
  SFRPAGE  = CAN0_PAGE;
  CAN0ADR  = IF1CMDMSK;    // Point to Command Mask Register 1
  CAN0DATL = 0xFF;         // Set direction to WRITE all IF registers to Msg Obj
  for (i=1;i<33;i++)
    {
      CAN0ADR = IF1CMDRQST; // Write blank (reset) IF registers to each msg obj
      CAN0DATL = i;
    }
}

void CUSW_shutdown(void)
{

		int ii = 0;

			while (ii < 50)
			{
				onehundredms();				  	//100ms delay loop
				txframe.MsgNum = 1;			  	//Message number(sequential starting at 1 for unique messages							
				txframe.arbID = 0x190;		  	//Message ID	CBC_13 Vehicle start packet									
				txframe.sz = 5;				  	//Message size(number of bytes 1-8)
				txframe.cdat[0] = 0x00;	      	//ignition run, start not active  04 = run ;02=ACC ;0D= NormalStart, key in Ign, 						
				txframe.cdat[1] = 0x22;		  	//key in ignition
				txframe.cdat[2] = 0x00;	      	//null					
				txframe.cdat[3] = 0x00;	  		//null  
				txframe.cdat[4] = 0x00;
				tx_can_frame(0,11);			  	//transmits can frame
				ii++;
				
			}

			for(ii=0; ii<300; ii++)
			{
				onehundredms();				  	//100ms delay loop
			}
}

void CUSW_IGN_MSG (void)
{

	if (IGN_OFF == 0)
	{
			
			while (iii < 300)
			{
				if (IGN_OFF != 0)
					break;
				onehundredms();				  	//100ms delay loop
				txframe.MsgNum = 1;			  	//Message number(sequential starting at 1 for unique messages							
				txframe.arbID = 0x190;		  	//Message ID	CBC_13 Vehicle start packet									
				txframe.sz = 5;				  	//Message size(number of bytes 1-8)
				txframe.cdat[0] = 0x00;	      	//ignition run, start not active  04 = run ;02=ACC ;0D= NormalStart, key in Ign, 						
				txframe.cdat[1] = 0x22;		  	//key in ignition
				txframe.cdat[2] = 0x00;	      	//null					
				txframe.cdat[3] = 0x00;	  		//null  
				txframe.cdat[4] = 0x00;
				tx_can_frame(0,11);			  	//transmits can frame
				iii++;
				
			}

	}
	
	if (IGN_ACC == 0)
	{
			shortdelay();				  	//100ms delay loop
			txframe.MsgNum = 1;			  	//Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x190;		  	//Message ID	CBC_13 Vehicle start packet									
			txframe.sz = 5;				  	//Message size(number of bytes 1-8)
			txframe.cdat[0] = 0x03;	      	//ignition run, start not active  04 = run ;02=ACC ;0D= NormalStart, key in Ign, 						
			txframe.cdat[1] = 0x22;		  	//key in ignition
			txframe.cdat[2] = 0x00;	      	//null					
			txframe.cdat[3] = 0x00;	  		//null  
			txframe.cdat[4] = 0x00;	  		//null  
			tx_can_frame(0,11);			  	//transmits can frame

			shortdelay();
			txframe.MsgNum = 20;			  //Speed message 	bytes 3 and 2 set speed (VEH_SPEED)  						
			txframe.arbID = 0x098;		   	// speed = 	km/h  /  0.0078125 km/h *	(50k/h / 0.0078125 km/h)=  6400 = 0x1900 						
			txframe.sz = 8;				  
			txframe.cdat[0] = 0x00;	      						
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x00;	      					
			txframe.cdat[3] = 0x00;	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0x00;
			tx_can_frame(0,11);
	}

	if (IGN_RUN_0 == 0)
	{
			shortdelay();				  	//100ms delay loop
			txframe.MsgNum = 1;			  	//Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x190;		  	//Message ID	CBC_13 Vehicle start packet									
			txframe.sz = 5;				  	//Message size(number of bytes 1-8)
			txframe.cdat[0] = 0x04;	      	//ignition run, start not active  04 = run ;02=ACC ;0D= NormalStart, key in Ign, 						
			txframe.cdat[1] = 0x22;		  	//key in ignition
			txframe.cdat[2] = 0x00;	      	//null					
			txframe.cdat[3] = 0x00;	  		//null  
			txframe.cdat[4] = 0x00;	  		//null  
			tx_can_frame(0,11);			  	//transmits can frame

			shortdelay();
			txframe.MsgNum = 20;			  //Speed message 	bytes 3 and 2 set speed (VEH_SPEED)  						
			txframe.arbID = 0x098;		   	// speed = 	km/h  /  0.0078125 km/h *	(50k/h / 0.0078125 km/h)=  6400 = 0x1900 						
			txframe.sz = 8;				  
			txframe.cdat[0] = 0x00;	      						
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x00;	      					
			txframe.cdat[3] = 0x00;	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0x00;
			tx_can_frame(0,11);
	}

	if (IGN_RUN_10 == 0)
	{
			shortdelay();				  	//100ms delay loop
			txframe.MsgNum = 1;			  	//Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x190;		  	//Message ID	CBC_13 Vehicle start packet									
			txframe.sz = 5;				  	//Message size(number of bytes 1-8)
			txframe.cdat[0] = 0x04;	      	//ignition run, start not active  04 = run ;02=ACC ;0D= NormalStart, key in Ign, 						
			txframe.cdat[1] = 0x22;		  	//key in ignition
			txframe.cdat[2] = 0x00;	      	//null					
			txframe.cdat[3] = 0x00;	  		//null  
			txframe.cdat[4] = 0x00;	  		//null  
			tx_can_frame(0,11);			  	//transmits can frame

			shortdelay();
			txframe.MsgNum = 20;			  //Speed message 	bytes 3 and 2 set speed (VEH_SPEED)  						
			txframe.arbID = 0x098;		   	// speed = 	km/h  /  0.0078125 km/h *	(50k/h / 0.0078125 km/h)=  6400 = 0x1900 						
			txframe.sz = 8;				  
			txframe.cdat[0] = 0x00;	      						
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x01;	      					
			txframe.cdat[3] = 0x00;	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0x00;
			tx_can_frame(0,11);
	}

	if (IGN_RUN_5 == 0)
	{
			shortdelay();				  	//100ms delay loop
			txframe.MsgNum = 1;			  	//Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x190;		  	//Message ID	CBC_13 Vehicle start packet									
			txframe.sz = 5;				  	//Message size(number of bytes 1-8)
			txframe.cdat[0] = 0x04;	      	//ignition run, start not active  04 = run ;02=ACC ;0D= NormalStart, key in Ign, 						
			txframe.cdat[1] = 0x22;		  	//key in ignition
			txframe.cdat[2] = 0x00;	      	//null					
			txframe.cdat[3] = 0x00;	  		//null  
			txframe.cdat[4] = 0x00;	  		//null  
			tx_can_frame(0,11);			  	//transmits can frame

			shortdelay();
			txframe.MsgNum = 20;			  //Speed message 	bytes 3 and 2 set speed (VEH_SPEED)  						
			txframe.arbID = 0x098;		   	// speed = 	km/h  /  0.0078125 km/h *	(50k/h / 0.0078125 km/h)=  6400 = 0x1900 						
			txframe.sz = 8;				  
			txframe.cdat[0] = 0x00;	      						
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x00;	      					
			txframe.cdat[3] = 0x80;	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0x00;
			tx_can_frame(0,11);
	}

	if (IGN_START == 0)
	{
			shortdelay();				  	//100ms delay loop
			txframe.MsgNum = 1;			  	//Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x190;		  	//Message ID	CBC_13 Vehicle start packet									
			txframe.sz = 5;				  	//Message size(number of bytes 1-8)
			txframe.cdat[0] = 0x05;	      	//ignition run, start not active  04 = run ;02=ACC ;0D= NormalStart, key in Ign, 						
			txframe.cdat[1] = 0x22;		  	//key in ignition
			txframe.cdat[2] = 0x00;	      	//null					
			txframe.cdat[3] = 0x00;	  		//null  
			txframe.cdat[4] = 0x00;	  		//null  
			tx_can_frame(0,11);			  	//transmits can frame

			shortdelay();
			txframe.MsgNum = 20;			  //Speed message 	bytes 3 and 2 set speed (VEH_SPEED)  						
			txframe.arbID = 0x098;		   	// speed = 	km/h  /  0.0078125 km/h *	(50k/h / 0.0078125 km/h)=  6400 = 0x1900 						
			txframe.sz = 8;				  
			txframe.cdat[0] = 0x00;	      						
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x00;	      					
			txframe.cdat[3] = 0x00;	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0x00;
			tx_can_frame(0,11);
	}
}


void CUSW_radiotest(void)

{	
		//VIN Entry -------------------------------------------------------------------------------

		char VIN[17] = "1C3CCCCB9FN1136UC";
		

					  		
			
			shortdelay();
			txframe.MsgNum = 2;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x339;		//phone pickup												
			txframe.sz = 8;				
			txframe.cdat[0] = 0x00;	      						
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x00;	      					
			txframe.cdat[3] = 0x00;	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0x00;			  
			tx_can_frame(0,11);
			
			shortdelay();
			txframe.MsgNum = 3;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x33B;		//phone pickup												
			txframe.sz = 8;				
			txframe.cdat[0] = 0x00;	      						
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x00;	      					
			txframe.cdat[3] = 0x00;	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0x00;			  
			tx_can_frame(0,11);
			
			shortdelay();
			txframe.MsgNum = 4;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x3CB;		//phone pickup												
			txframe.sz = 8;				
			txframe.cdat[0] = 0x00;	      						
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x00;	      					
			txframe.cdat[3] = 0x00;	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0x00;			  
			tx_can_frame(0,11);
			
			shortdelay();
			txframe.MsgNum = 5;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x3D1;		//phone pickup												
			txframe.sz = 4;				
			txframe.cdat[0] = 0x00;	      						
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x00;	      					
			txframe.cdat[3] = 0x00;	  			  
			tx_can_frame(0,11);			
			
			shortdelay();
			txframe.MsgNum = 6;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x3d3;		  //Message ID VIN, VIN_LO hex 31 = ascii 1										
			txframe.sz = 8;				  //8 bytes
			txframe.cdat[0] = 0x00;	      						
			txframe.cdat[1] = VIN[0];	
			txframe.cdat[2] = VIN[1];	      					
			txframe.cdat[3] = VIN[2];	  
			txframe.cdat[4] = VIN[3];	      						
			txframe.cdat[5] = VIN[4];	
			txframe.cdat[6] = VIN[5];
			txframe.cdat[7] = VIN[6];	  
			tx_can_frame(0,11);

			onehundredms();
			onehundredms();
			shortdelay();
			txframe.MsgNum = 7;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x3d3;		  //Message ID VIN, VIN_MID hex 31 = ascii 1										
			txframe.sz = 8;
			txframe.cdat[0] = 0x01;				  //8 bytes
			txframe.cdat[1] = VIN[7];	
			txframe.cdat[2] = VIN[8];	      					
			txframe.cdat[3] = VIN[9];	  
			txframe.cdat[4] = VIN[10];	      						
			txframe.cdat[5] = VIN[11];	
			txframe.cdat[6] = VIN[12];
			txframe.cdat[7] = VIN[13];	  
			tx_can_frame(0,11);

			onehundredms();
			shortdelay();
			txframe.MsgNum = 8;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x3d3;		  //Message ID VIN, VIN_HI hex 31 = ascii 1										
			txframe.sz = 8;
			txframe.cdat[0] = 0x02;				  //8 bytes
			txframe.cdat[1] = VIN[14];	
			txframe.cdat[2] = VIN[15];	      					
			txframe.cdat[3] = VIN[16];	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0x00;  
			tx_can_frame(0,11);

			shortdelay();
			txframe.MsgNum = 9;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x499;		//phone pickup												
			txframe.sz = 5;				
			txframe.cdat[0] = 0x00;	      						
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x00;	      					
			txframe.cdat[3] = 0x05;	  
			txframe.cdat[4] = 0x00;	      									  
			tx_can_frame(0,11);

			shortdelay();
			txframe.MsgNum = 10;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x4A3;		//phone pickup												
			txframe.sz = 8;				
			txframe.cdat[0] = 0x00;	      						
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x00;	      					
			txframe.cdat[3] = 0x00;	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0x00;			  
			tx_can_frame(0,11);

			shortdelay();
			txframe.MsgNum = 11;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x548;		//phone pickup												
			txframe.sz = 4;				
			txframe.cdat[0] = 0x00;	      						
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x00;	      					
			txframe.cdat[3] = 0x00;	  		  
			tx_can_frame(0,11);

			shortdelay();
			txframe.MsgNum = 12;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x5C8;		//phone pickup												
			txframe.sz = 2;				
			txframe.cdat[0] = 0x00;	      						
			txframe.cdat[1] = 0x00;		  
			tx_can_frame(0,11);

			shortdelay();
			txframe.MsgNum = 13;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x5CA;		//phone pickup												
			txframe.sz = 8;				
			txframe.cdat[0] = 0x00;	      						
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0xFF;	      					
			txframe.cdat[3] = 0xFF;	  
			txframe.cdat[4] = 0xFF;	      						
			txframe.cdat[5] = 0xFF;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0x00;			  
			tx_can_frame(0,11);
			
			shortdelay();
			txframe.MsgNum = 14;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x5CC;		//phone pickup												
			txframe.sz = 4;				
			txframe.cdat[0] = 0x00;	      						
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x00;	      					
			txframe.cdat[3] = 0x00;	  			  
			tx_can_frame(0,11);
			
			shortdelay();
			txframe.MsgNum = 15;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x5DC;		//phone pickup												
			txframe.sz = 5;				
			txframe.cdat[0] = 0x00;	      						
			txframe.cdat[1] = 0x20;	
			txframe.cdat[2] = 0x00;	      					
			txframe.cdat[3] = 0x00;	  
			txframe.cdat[4] = 0x00;	      									  
			tx_can_frame(0,11);
			
			shortdelay();
			txframe.MsgNum = 16;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x6A4;		//phone pickup												
			txframe.sz = 8;				
			txframe.cdat[0] = 0x00;	      						
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x00;	      					
			txframe.cdat[3] = 0x00;	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0x00;			  
			tx_can_frame(0,11);
			
			shortdelay();
			txframe.MsgNum = 17;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x75A;		//phone pickup												
			txframe.sz = 2;				
			txframe.cdat[0] = 0x00;	      						
			txframe.cdat[1] = 0x00;				  
			tx_can_frame(0,11);
			
			shortdelay();
			txframe.MsgNum = 18;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x762;		//phone pickup												
			txframe.sz = 3;				
			txframe.cdat[0] = 0x0F;	      						
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x00;	      								  
			tx_can_frame(0,11);
			
			shortdelay();
			txframe.MsgNum = 19;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x772;		//phone pickup												
			txframe.sz = 4;				
			txframe.cdat[0] = 0x00;	      						
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x00;	      					
			txframe.cdat[3] = 0x00;	  		  
			tx_can_frame(0,11);
			

				if (PH_BUTTON == 0 )                   

      				{ 
        					shortdelay();
							txframe.MsgNum = 21;			  //Message number(sequential starting at 1 for unique messages							
							txframe.arbID = 0x4A3;		//phone pickup												
							txframe.sz = 8;				
							txframe.cdat[0] = 0x00;	      						
							txframe.cdat[1] = 0x00;	
							txframe.cdat[2] = 0x00;	      					
							txframe.cdat[3] = 0x00;	  
							txframe.cdat[4] = 0x00;	      						
							txframe.cdat[5] = 0x02;	
							txframe.cdat[6] = 0x00;
							txframe.cdat[7] = 0x00;			  
							tx_can_frame(0,11);

						
	  			
						}			
				else if (VR_BUTTON == 0)  
      				  {  
       					 	shortdelay();
							txframe.MsgNum = 22;			  //Message number(sequential starting at 1 for unique messages							
							txframe.arbID= 0x4A3;		  									
							txframe.sz = 8;				  
							txframe.cdat[0] = 0x00;	      						
							txframe.cdat[1] = 0x00;	
							txframe.cdat[2] = 0x00;	      					
							txframe.cdat[3] = 0x20;	  
							txframe.cdat[4] = 0x00;	      						
							txframe.cdat[5] = 0x00;	
							txframe.cdat[6] = 0x00;
							txframe.cdat[7] = 0x00;		  	  
							tx_can_frame(0,11);
     				  }      
				
				else if (PH_BUTTON_END == 0)  
      				  {  
       					 	shortdelay();
							txframe.MsgNum = 23;			  //Message number(sequential starting at 1 for unique messages							
							txframe.arbID= 0x4A3;		  									
							txframe.sz = 8;				  
							txframe.cdat[0] = 0x00;	      						
							txframe.cdat[1] = 0x00;	
							txframe.cdat[2] = 0x00;	      					
							txframe.cdat[3] = 0x02;	  
							txframe.cdat[4] = 0x00;	      						
							txframe.cdat[5] = 0x00;	
							txframe.cdat[6] = 0x00;
							txframe.cdat[7] = 0x00;	
							tx_can_frame(0,11);
     				  }    
					  
				/*else if (VOL_UP == 0)  
      				  {  
       					 	shortdelay();
							txframe.MsgNum = 23;			  //Message number(sequential starting at 1 for unique messages							
							txframe.arbID= 0x4A3;		  									
							txframe.sz = 8;				  
							txframe.cdat[0] = 0x00;	      						
							txframe.cdat[1] = 0x00;	
							txframe.cdat[2] = 0x00;	      					
							txframe.cdat[3] = 0x00;	  
							txframe.cdat[4] = 0x00;	      						
							txframe.cdat[5] = 0x08;	
							txframe.cdat[6] = 0x00;
							txframe.cdat[7] = 0x00;	
							tx_can_frame(0,11);
     				  }  */          
			
				else if (VOL_DOWN  == 0)  
      				  {  
       					 	shortdelay();
							txframe.MsgNum = 23;			  //Message number(sequential starting at 1 for unique messages							
							txframe.arbID= 0x4A3;		  									
							txframe.sz = 8;				  
							txframe.cdat[0] = 0x00;	      						
							txframe.cdat[1] = 0x00;	
							txframe.cdat[2] = 0x00;	      					
							txframe.cdat[3] = 0x00;	  
							txframe.cdat[4] = 0x00;	      						
							txframe.cdat[5] = 0x20;	
							txframe.cdat[6] = 0x00;
							txframe.cdat[7] = 0x00;	
							tx_can_frame(0,11);
     				  }  
				else 
						{
							shortdelay();
							txframe.MsgNum = 24;			  //Message number(sequential starting at 1 for unique messages							
							txframe.arbID= 0x4A3;		//default state									
							txframe.sz = 8;				
							txframe.cdat[0] = 0x00;	      						
							txframe.cdat[1] = 0x00;	
							txframe.cdat[2] = 0x00;	      					
							txframe.cdat[3] = 0x00;	  
							txframe.cdat[4] = 0x00;	      						
							txframe.cdat[5] = 0x00;	
							txframe.cdat[6] = 0x00;
							txframe.cdat[7] = 0x00;		  
							tx_can_frame(0,11);
					
					}	
					

			




		
	}

void Powernet_shutdown(void)
{

		int ii = 0;

			while (ii < 50)
			{
				onehundredms();				  	//100ms delay loop
				txframe.MsgNum = 1;			  	//Message number(sequential starting at 1 for unique messages							
				txframe.arbID = 0x122;		  	//Message ID	CBC_13 Vehicle start packet									
				txframe.sz = 4;				  	//Message size(number of bytes 1-8)
				txframe.cdat[0] = 0x01;	      	//ignition run, start not active  04 = run ;02=ACC ;0D= NormalStart, key in Ign, 						
				txframe.cdat[1] = 0x02;		  	//key in ignition
				txframe.cdat[2] = 0x00;	      	//null					
				txframe.cdat[3] = 0x00;	  		//null  
				tx_can_frame(0, 11);			  	//transmits can frame
				ii++;
				
			}

			for(ii=0; ii<300; ii++)
			{
				onehundredms();				  	//100ms delay loop
			}
}

void Powernet_IGN_MSG (void)
{

	if (IGN_OFF == 0)
	{
			
			while (iii < 300)
			{
				if (IGN_OFF != 0)
					break;
				onehundredms();				  	//100ms delay loop
				txframe.MsgNum = 1;			  	//Message number(sequential starting at 1 for unique messages							
				txframe.arbID = 0x122;		  	//Message ID	CBC_13 Vehicle start packet									
				txframe.sz = 4;				  	//Message size(number of bytes 1-8)
				txframe.cdat[0] = 0x01;	      	//ignition run, start not active  04 = run ;02=ACC ;0D= NormalStart, key in Ign, 						
				txframe.cdat[1] = 0x02;		  	//key in ignition
				txframe.cdat[2] = 0x00;	      	//null					
				txframe.cdat[3] = 0x00;	  		//null  
				tx_can_frame(0, 11);			  	//transmits can frame
				iii++;
				
			}

	}
	
	if (IGN_ACC == 0)
	{
			shortdelay();				  	//100ms delay loop
			txframe.MsgNum = 1;			  	//Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x122;		  	//Message ID	CBC_13 Vehicle start packet									
			txframe.sz = 4;				  	//Message size(number of bytes 1-8)
			txframe.cdat[0] = 0x02;	      	//ignition run, start not active  04 = run ;02=ACC ;0D= NormalStart, key in Ign, 						
			txframe.cdat[1] = 0x02;		  	//key in ignition
			txframe.cdat[2] = 0x00;	      	//null					
			txframe.cdat[3] = 0x00;	  		//null  
			tx_can_frame(0, 11);			  	//transmits can frame

			shortdelay();
			txframe.MsgNum = 20;			  //Speed message 	bytes 3 and 2 set speed (VEH_SPEED)  						
			txframe.arbID = 0x322;		   	// speed = 	km/h  /  0.0078125 km/h *	(50k/h / 0.0078125 km/h)=  6400 = 0x1900 						
			txframe.sz = 8;				  
			txframe.cdat[0] = 0x00;	      						
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x00;	      					
			txframe.cdat[3] = 0x00;	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0x00;
			tx_can_frame(0, 11);
	}

	if (IGN_RUN_0 == 0)
	{
			shortdelay();				  	//100ms delay loop
			txframe.MsgNum = 1;			  	//Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x122;		  	//Message ID	CBC_13 Vehicle start packet									
			txframe.sz = 4;				  	//Message size(number of bytes 1-8)
			txframe.cdat[0] = 0x04;	      	//ignition run, start not active  04 = run ;02=ACC ;0D= NormalStart, key in Ign, 						
			txframe.cdat[1] = 0x02;		  	//key in ignition
			txframe.cdat[2] = 0x00;	      	//null					
			txframe.cdat[3] = 0x00;	  		//null  
			tx_can_frame(0, 11);			  	//transmits can frame

			shortdelay();
			txframe.MsgNum = 20;			  //Speed message 	bytes 3 and 2 set speed (VEH_SPEED)  						
			txframe.arbID = 0x322;		   	// speed = 	km/h  /  0.0078125 km/h *	(50k/h / 0.0078125 km/h)=  6400 = 0x1900 						
			txframe.sz = 8;				  
			txframe.cdat[0] = 0x00;	      						
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x00;	      					
			txframe.cdat[3] = 0x00;	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0x00;
			tx_can_frame(0, 11);
	}

	if (IGN_RUN_10 == 0)
	{
			shortdelay();				  	//100ms delay loop
			txframe.MsgNum = 1;			  	//Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x122;		  	//Message ID	CBC_13 Vehicle start packet									
			txframe.sz = 4;				  	//Message size(number of bytes 1-8)
			txframe.cdat[0] = 0x04;	      	//ignition run, start not active  04 = run ;02=ACC ;0D= NormalStart, key in Ign, 						
			txframe.cdat[1] = 0x02;		  	//key in ignition
			txframe.cdat[2] = 0x00;	      	//null					
			txframe.cdat[3] = 0x00;	  		//null  
			tx_can_frame(0, 11);			  	//transmits can frame

			shortdelay();
			txframe.MsgNum = 20;			  //Speed message 	bytes 3 and 2 set speed (VEH_SPEED)  						
			txframe.arbID = 0x322;		   	// speed = 	km/h  /  0.0078125 km/h *	(50k/h / 0.0078125 km/h)=  6400 = 0x1900 						
			txframe.sz = 8;				  
			txframe.cdat[0] = 0x00;	      						
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x08;	      					
			txframe.cdat[3] = 0x00;	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0x00;
			tx_can_frame(0, 11);
	}

	if (IGN_RUN_5 == 0)
	{
			shortdelay();				  	//100ms delay loop
			txframe.MsgNum = 1;			  	//Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x122;		  	//Message ID	CBC_13 Vehicle start packet									
			txframe.sz = 4;				  	//Message size(number of bytes 1-8)
			txframe.cdat[0] = 0x04;	      	//ignition run, start not active  04 = run ;02=ACC ;0D= NormalStart, key in Ign, 						
			txframe.cdat[1] = 0x02;		  	//key in ignition
			txframe.cdat[2] = 0x00;	      	//null					
			txframe.cdat[3] = 0x00;	  		//null  
			tx_can_frame(0, 11);			  	//transmits can frame

			shortdelay();
			txframe.MsgNum = 20;			  //Speed message 	bytes 3 and 2 set speed (VEH_SPEED)  						
			txframe.arbID = 0x322;		   	// speed = 	km/h  /  0.0078125 km/h *	(50k/h / 0.0078125 km/h)=  6400 = 0x1900 						
			txframe.sz = 8;				  
			txframe.cdat[0] = 0x00;	      						
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x04;	      					
			txframe.cdat[3] = 0x05;	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0x00;
			tx_can_frame(0, 11);
	}

	if (IGN_START == 0)
	{
			shortdelay();				  	//100ms delay loop
			txframe.MsgNum = 1;			  	//Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x122;		  	//Message ID	CBC_13 Vehicle start packet									
			txframe.sz = 4;				  	//Message size(number of bytes 1-8)
			txframe.cdat[0] = 0x05;	      	//ignition run, start not active  04 = run ;02=ACC ;0D= NormalStart, key in Ign, 						
			txframe.cdat[1] = 0x02;		  	//key in ignition
			txframe.cdat[2] = 0x00;	      	//null					
			txframe.cdat[3] = 0x00;	  		//null  
			tx_can_frame(0, 11);			  	//transmits can frame

			shortdelay();
			txframe.MsgNum = 20;			  //Speed message 	bytes 3 and 2 set speed (VEH_SPEED)  						
			txframe.arbID = 0x322;		   	// speed = 	km/h  /  0.0078125 km/h *	(50k/h / 0.0078125 km/h)=  6400 = 0x1900 						
			txframe.sz = 8;				  
			txframe.cdat[0] = 0x00;	      						
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x00;	      					
			txframe.cdat[3] = 0x00;	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0x00;
			tx_can_frame(0, 11);
	}
}

void Powernet_radiotest(void)

{	
		//VIN Entry -------------------------------------------------------------------------------

		char VIN[17] = "1C3CCCCB7FN1124UC";

			shortdelay();
			txframe.MsgNum = 2;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x3f2;		  //Message ID CBC_12  									
			txframe.sz = 8;				  //8 bytes
			txframe.cdat[0] = 0x95;	      						
			txframe.cdat[1] = 0x02;	
			txframe.cdat[2] = 0x57;	      					
			txframe.cdat[3] = 0xA0;	  
			txframe.cdat[4] = 0x09;	      						
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0x02;	      // Change to 0x02 for USB and AUX, 0x03 for USB/SD/AUX	  
			tx_can_frame(0, 11); 

			
			shortdelay();
			txframe.MsgNum = 4;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x3e0;		  //Message ID VIN, VIN_LO hex 31 = ascii 1										
			txframe.sz = 8;				  //8 bytes
			txframe.cdat[0] = 0x00;	      						
			txframe.cdat[1] = VIN[0];	
			txframe.cdat[2] = VIN[1];	      					
			txframe.cdat[3] = VIN[2];	  
			txframe.cdat[4] = VIN[3];	      						
			txframe.cdat[5] = VIN[4];	
			txframe.cdat[6] = VIN[5];
			txframe.cdat[7] = VIN[6];		  
			tx_can_frame(0, 11);

			onehundredms();
			shortdelay();
			txframe.MsgNum = 5;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x3e0;		  //Message ID VIN, VIN_MID hex 31 = ascii 1										
			txframe.sz = 8;				  //8 bytes
			txframe.cdat[0] = 0x01;	      						
			txframe.cdat[1] = VIN[7];	
			txframe.cdat[2] = VIN[8];	      				
			txframe.cdat[3] = VIN[9];	  
			txframe.cdat[4] = VIN[10];	      						
			txframe.cdat[5] = VIN[11];	
			txframe.cdat[6] = VIN[12];
			txframe.cdat[7] = VIN[13];		  
			tx_can_frame(0, 11);

			onehundredms();
			shortdelay();
			txframe.MsgNum = 6;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x3e0;		  //Message ID VIN, VIN_HI hex 31 = ascii 1										
			txframe.sz = 8;				  //8 bytes
			txframe.cdat[0] = 0x02;	      						
			txframe.cdat[1] = VIN[14];	
			txframe.cdat[2] = VIN[15];	      					
			txframe.cdat[3] = VIN[16];	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0x00;		  
			tx_can_frame(0, 11);
		
		
			shortdelay();
			txframe.MsgNum = 10;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x3E8;		  //Message ID CBC_12  									
			txframe.sz = 8;				  //8 bytes
			txframe.cdat[0] = 0x39;	      						
			txframe.cdat[1] = 0x1D;	
			txframe.cdat[2] = 0x22;	      					
			txframe.cdat[3] = 0x00;	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0x00;		  
			tx_can_frame(0, 11);
			
		
			shortdelay();
			txframe.MsgNum = 11;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x3EA;		  //Message ID CBC_12  									
			txframe.sz = 8;				  //8 bytes
			txframe.cdat[0] = 0x01;	      						
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x00;	      					
			txframe.cdat[3] = 0x00;	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0xC0;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0x00;		  
			tx_can_frame(0, 11);

			shortdelay();
			txframe.MsgNum = 15;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x22F;		  //Message ID CBC_12  									
			txframe.sz = 1;				  //8 bytes
			txframe.cdat[0] = 0x01;	      								  
			tx_can_frame(0, 11);

			if (PH_BUTTON == 0 )                   

      				{ 
        					shortdelay();
							txframe.MsgNum = 13;			  //Message number(sequential starting at 1 for unique messages							
							txframe.arbID = 0x318;		  //Message ID VR active message byte 5 x02										
							txframe.sz = 8;				  //8 bytes
							txframe.cdat[0] = 0x00;	      						
							txframe.cdat[1] = 0x00;	
							txframe.cdat[2] = 0x00;	 // lever in reverse     					
							txframe.cdat[3] = 0x00;	  
							txframe.cdat[4] = 0x00;	      						
							txframe.cdat[5] = 0x01;	
							txframe.cdat[6] = 0x00;
							txframe.cdat[7] = 0x00;		  
							tx_can_frame(0, 11);

						
	  			
						}			
				else if (VR_BUTTON == 0)  
      				  {  
       					 	shortdelay();
							txframe.MsgNum = 12;			  //Message number(sequential starting at 1 for unique messages							
							txframe.arbID= 0x318;		  //Message ID VR off									
							txframe.sz = 8;				  //8 bytes
							txframe.cdat[0] = 0x00;	      						
							txframe.cdat[1] = 0x00;	
							txframe.cdat[2] = 0x00;	  // lever in Park    					
							txframe.cdat[3] = 0x00;	  
							txframe.cdat[4] = 0x00;	      						
							txframe.cdat[5] = 0x02;	
							txframe.cdat[6] = 0x00;
							txframe.cdat[7] = 0x00;		  
							tx_can_frame(0, 11);
     				  }               
			
				else
						{
							shortdelay();
							txframe.MsgNum = 12;			  //Message number(sequential starting at 1 for unique messages							
							txframe.arbID= 0x318;		  //Message ID VR off									
							txframe.sz = 8;				  //8 bytes
							txframe.cdat[0] = 0x00;	      						
							txframe.cdat[1] = 0x00;	
							txframe.cdat[2] = 0x00;	  // lever in Park    					
							txframe.cdat[3] = 0x00;	  
							txframe.cdat[4] = 0x00;	      						
							txframe.cdat[5] = 0x00;	
							txframe.cdat[6] = 0x00;
							txframe.cdat[7] = 0x00;		  
							tx_can_frame(0, 11);
					
					}
		
}

void codedelay(unsigned int delay)	//unsigned in
{
	while (delay)
	delay--;
} 

void onehundredms(void)
{

	
				codedelay(255000);
				codedelay(255000);
				codedelay(245000);
				codedelay(100062);
}



void shortdelay(void)
{
				codedelay(200000);
}



void clear_message_object (unsigned char mesgobj,unsigned char count) 
{
   	SFRPAGE = CAN0_PAGE;
	CAN0ADR = IF1CMDMSK;          			//Point to Command Mask Register 1
	CAN0DATL = 0xFF;              			//Set direction to WRITE all IF registers to Msg Obj

	while (mesgobj < 33 && count)
	{
		CAN0ADR = IF1CMDRQST;  				//Write blank IF registers to GLOBAL msg obj
		CAN0DATL = mesgobj;
		mesgobj++;
		count--;
	}
}				  


//-----------------------------------------------------------------------------
// ADC0_Init
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// Discription  : Intializes the ADC0for pin AIN3.0, inorder to obtain curent switch position for analog switch
//
//-----------------------------------------------------------------------------
void ADC0_Init (void)
{

    SFRPAGE = ADC0_PAGE;


	// ADC0 disabled; normal tracking
    // mode; ADC0 conversions are initiated
    // on AD0BUSY; ADC0 data is
    // right-justified
    ADC0CN = 0x00;      
	
	                
	// Enable on-chip VREF
    REF0CN = 0x0F; 
	                     
	
	// Select AIN3.0 pin as ADC mux input
	// Select AIN3.0 port
    AMX0SL = 0x06;                      
    AMX0PRT = 0x01;		
				   				  

	// ADC conversion clock = 2.5MHz, Gain=1
    ADC0CF = (SYSCLK/SAR_CLK) << 3; 
	                 
}

//-----------------------------------------------------------------------------
// ADC0_Analog_sel
//-----------------------------------------------------------------------------
//
// Return Value : int - current switch number
// Parameters   : None
//
// Discription  : polls ADC0 and calculates current switch position based on the total # of switch positions
//
//-----------------------------------------------------------------------------
int ADC0_Analog_sel(void)
{

	int k;

	SFRPAGE = ADC0_PAGE;

	//calculate the voltage increment value based on max value (1023) 
	//divided by the total # of switch positions
	sw_inc = 1023/numb;									


	// start AD conversion isr train and wait for conversion complete flag (AD0BUSY)
	ADC0CN |= 0x10; 								
	while(AD0BUSY != 0);							


	//retrieve lower 8 bits and higher 2 bits from ADC0 data register 
	//then combine to get an adc value between 0-1023
	ResultL = ADC0L;								
	ResultH = ADC0H;
	Result = (ResultH << 8) | ResultL;
	

	//check where current polling switch position is at compared to 
	//the switch increments and total # of switch positions
	for(k=1; k <= numb; k++)
	{
		if(Result<(k*sw_inc))
		{
			return k;
		}
	}

	return numb;
}
