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

// IGNITION SWITCHES
sbit IGN_RUN_0  	= P1^5;				// sw3 = 0 means switch pressed
sbit IGN_RUN_5		= P0^6;
sbit IGN_RUN_10 	= P0^5;				// sw4 = 0 means switch pressed
sbit IGN_START		= P1^0;
sbit IGN_OFF 		= P0^7;				// sw5 = 0 means switch pressed
sbit IGN_ACC 		= P1^1;				// sw6 = 0 means switch pressed

// STEERING WHEEL SWITCHES
sbit VOL_UP			= P3^4;
sbit VOL_DWN		= P3^5;
sbit SEEK_UP		= P3^6;
sbit SEEK_DWN		= P3^7;
sbit MODE			= P3^2;
sbit PRESET			= P3^3;

sbit SEL_CAN_B = P1^7;					// CAN select pin 0 means can B, 1 means CAN c/i
sbit CAN_B_EN  = P1^6;					// CAN B enable pin,  0 means enable, 1 means disable



struct txframe
{
	unsigned char MsgNum;
	unsigned char sz;
	unsigned long arbID;
 	unsigned char cdat[8];

}txframe;
CANFRAME xdata rxframe[32];

byte xdata IntReg;
char MsgNum;
char status;
int i;
int v;
int j = 0;
int h;
int ii = 0;
int iii = 0;
int count1 = 0;
int flag_eng = 1;
int flag_spa = 0;
int flag_ger = 0;
int flag_fre = 0;
int flag_ita = 0;
int flag_jap = 0;
int flag_chs = 0;
int flag_cht = 0;

void config_CAN_timing(void);
void config_IO(void);
void receive_data(char MsgNum);
void clear_msg_objects(void);
void radiotest(void);
void codedelay(unsigned int delay);
void clear_message_object (unsigned char mesgobj,unsigned char count);
void onehundredms(void);
void shortdelay(void);

void init_msg_object_rx (unsigned char MsgNum, unsigned int arbID);
void check_lang(void);
void lang_cfg(void);
void SWS_CUSW(void);
void IGN_MSG(void);
//void Onetime_Setup(void);@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@




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

								// Enable Global Interrupts
	/*
	SFRPAGE = IE;					// Configure all Interrupt registers
	IE = 0x80;
	SFRPAGE = EIE1;
	EIE1 = 0x00;
	SFRPAGE = EIE2;
	EIE2 = 0x20;
	*/

	init_msg_object_rx(30, 0x09E);
	rxframe[1].MsgNum = 0;

  //Loop and wait for interrupts
  //Onetime_Setup();@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  while (1)
    {
		

		IGN_MSG();
		if (IGN_RUN_0 == 0)
		{
			radiotest();
			iii = 0;
		}

		
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
		XBR2	 = 0x40;			// enable crossbar and disable weak pullups
		XBR3     = 0x80;     // Configure CAN TX pin (CTX) as push-pull digital output		
		SFRPAGE = SFRPAGE_SAVE;


}


void receive_data(char MsgNum)
{
  
  SFRPAGE = CAN0_PAGE; //Saves 

	
	for (count1 =  MsgNum; count1 >= 0;count1--)
	{
	rxframe[1].MsgNum = 30;

	CAN0ADR = IF2CMDMSK;       						// Point to Command Mask 1
	CAN0DATL = 0x3F;           						// Config to READ CAN RAM, read data bytes, clr NewDat and IntPnd, arb ID
	CAN0ADR = IF2CMDRQST;      						// Point to Command Request Reg.
	CAN0DATL = rxframe[1].MsgNum;				// Move new data for RX from Msg Obj "MsgNum"

	//Get Arb ID
	CAN0ADR = IF2ARB2;								//Point to Arbitration 2
	rxframe[1].arbID = (CAN0DAT & 0x1FFF) >> 2;

	//Get Data Size
	CAN0ADR = IF2MSGC;								//Point to IF2 Message Control Register
	rxframe[1].sz = CAN0DATL & 0x0F;
	
	//Move new data to a buffer	
	CAN0ADR = IF2DATA1;     	   					// Point to 1st byte of Data Field	   
	rxframe[1].cdat[0] = CAN0DATL;
	CAN0ADR = IF2DATA1;     	   		   
	rxframe[1].cdat[1] = CAN0DATH;

	CAN0ADR = IF2DATA2;
	rxframe[1].cdat[2] = CAN0DATL;
	CAN0ADR = IF2DATA2;
	rxframe[1].cdat[3] = CAN0DATH;

	CAN0ADR = IF2DATB1;
	rxframe[1].cdat[4] = CAN0DATL;
	CAN0ADR = IF2DATB1;
	rxframe[IntReg].cdat[5] = CAN0DATH;
	

	check_lang();	// change configuration message depending on request from radio
	}
  
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
	EA = 0;
	EA = 0;
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

/*void Ontime_Setup(void)@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
{
	shortdelay();
	txframe.MsgNum = 4;			  //Message number(sequential starting at 1 for unique messages							
	txframe.arbID = 0x;		  //Message ID VIN, VIN_LO hex 31 = ascii 1										
	txframe.sz = 8;				  //8 bytes
	txframe.cdat[0] = 0x;	      						
	txframe.cdat[1] = 0x;	
	txframe.cdat[2] = 0x;	      					
	txframe.cdat[3] = 0x;	  
	txframe.cdat[4] = 0x;	      						
	txframe.cdat[5] = 0x;	
	txframe.cdat[6] = 0x;
	txframe.cdat[7] = 0x;	  
	tx_can_frame(0,11);
}
*/


void radiotest(void)

{	
		


		char VIN[17] = "ZACCJBDT2GP3015UC";



			
		  	//transmits can 

			
			shortdelay();
			txframe.MsgNum = 4;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x3E0;		  //Message ID VIN, VIN_LO hex 31 = ascii 1										
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
			txframe.MsgNum = 5;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x3E0;		  //Message ID VIN, VIN_MID hex 31 = ascii 1										
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
			txframe.MsgNum = 6;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x3E0;		  //Message ID VIN, VIN_HI hex 31 = ascii 1										
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
			txframe.MsgNum = 6;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x092;		  //Message ID VIN, VIN_HI hex 31 = ascii 1										
			txframe.sz = 3;
			txframe.cdat[0] = 0x00;				  //8 bytes
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x00;	      					  
			tx_can_frame(0,11);

			shortdelay();
			txframe.MsgNum = 6;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x3B4;		  //Message ID VIN, VIN_HI hex 31 = ascii 1										
			txframe.sz = 6;
			txframe.cdat[0] = 0x00;				  //8 bytes
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x00;	      					
			txframe.cdat[3] = 0x00;	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;	 
			tx_can_frame(0,11);
			
			shortdelay();
			txframe.MsgNum = 6;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x3B6;		  //Message ID VIN, VIN_HI hex 31 = ascii 1										
			txframe.sz = 6;
			txframe.cdat[0] = 0x00;				  //8 bytes
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x00;	      					
			txframe.cdat[3] = 0x00;	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;	 
			tx_can_frame(0,11);

			shortdelay();
			txframe.MsgNum = 6;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x3DA;		  //Message ID VIN, VIN_HI hex 31 = ascii 1										
			txframe.sz = 1;
			txframe.cdat[0] = 0x00;				  //8 bytes  
			tx_can_frame(0,11);
			
			shortdelay();
			txframe.MsgNum = 6;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x3DC;		  //Message ID VIN, VIN_HI hex 31 = ascii 1										
			txframe.sz = 7;
			txframe.cdat[0] = 0x00;				  //8 bytes
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x00;	      					
			txframe.cdat[3] = 0x00;	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x00;  
			tx_can_frame(0,11);

			shortdelay();
			txframe.MsgNum = 6;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x3DE;		  //Message ID VIN, VIN_HI hex 31 = ascii 1										
			txframe.sz = 8;
			txframe.cdat[0] = 0x00;				  //8 bytes
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x00;	      					
			txframe.cdat[3] = 0x00;	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0x00;  
			tx_can_frame(0,11);
			
			shortdelay();
			txframe.MsgNum = 6;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x3E2;		  //Message ID VIN, VIN_HI hex 31 = ascii 1										
			txframe.sz = 8;
			txframe.cdat[0] = 0x00;				  //8 bytes
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x04;	      					
			txframe.cdat[3] = 0x00;	  
			txframe.cdat[4] = 0x3C;	      						
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x03;
			txframe.cdat[7] = 0x00;  
			tx_can_frame(0,11);

			shortdelay();
			txframe.MsgNum = 6;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x3E4;		  //Message ID VIN, VIN_HI hex 31 = ascii 1										
			txframe.sz = 8;
			txframe.cdat[0] = 0x00;				  //8 bytes
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x00;	      					
			txframe.cdat[3] = 0x00;	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0x00;  
			tx_can_frame(0,11);

			shortdelay();
			txframe.MsgNum = 6;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x3E6;		  //Message ID VIN, VIN_HI hex 31 = ascii 1										
			txframe.sz = 8;
			txframe.cdat[0] = 0x00;				  //8 bytes
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x00;	      					
			txframe.cdat[3] = 0x00;	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0x00;  
			tx_can_frame(0,11);

			shortdelay();
			txframe.MsgNum = 6;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x468;		  //Message ID VIN, VIN_HI hex 31 = ascii 1										
			txframe.sz = 2;
			txframe.cdat[0] = 0x00;				  //8 bytes
			txframe.cdat[1] = 0x00;	  
			tx_can_frame(0,11);

			/*************************  Moved to LANG_SETUP************
			
			shortdelay();
			txframe.MsgNum = 6;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x5BC;		  //Message ID VIN, VIN_HI hex 31 = ascii 1										
			txframe.sz = 8;
			txframe.cdat[0] = 0x00;				  //8 bytes
			txframe.cdat[1] = 0x01;	
			txframe.cdat[2] = 0x60;	      					
			txframe.cdat[3] = 0x40;	  
			txframe.cdat[4] = 0x05;	      						
			txframe.cdat[5] = 0x19;	
			txframe.cdat[6] = 0x90;
			txframe.cdat[7] = 0x80;  
			tx_can_frame(0,11);
			**********************************************************/

			shortdelay();
			txframe.MsgNum = 6;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x5C0;		  //Message ID VIN, VIN_HI hex 31 = ascii 1										
			txframe.sz = 8;
			txframe.cdat[0] = 0x00;				  //8 bytes
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x00;	      					
			txframe.cdat[3] = 0x00;	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0x00;  
			tx_can_frame(0,11);

			shortdelay();
			txframe.MsgNum = 6;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x5C2;		  //Message ID VIN, VIN_HI hex 31 = ascii 1										
			txframe.sz = 3;
			txframe.cdat[0] = 0x00;				  //8 bytes
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x00;	      					  
			tx_can_frame(0,11);

			shortdelay();
			txframe.MsgNum = 6;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x5C4;		  //Message ID VIN, VIN_HI hex 31 = ascii 1										
			txframe.sz = 8;
			txframe.cdat[0] = 0x00;				  //8 bytes
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x00;	      					
			txframe.cdat[3] = 0x00;	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0x00;  
			tx_can_frame(0,11);

			shortdelay();
			txframe.MsgNum = 6;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x5CE;		  //Message ID VIN, VIN_HI hex 31 = ascii 1										
			txframe.sz = 8;
			txframe.cdat[0] = 0x65;				  //8 bytes
			txframe.cdat[1] = 0x43;	
			txframe.cdat[2] = 0x40;	      					
			txframe.cdat[3] = 0x00;	  
			txframe.cdat[4] = 0x02;	      						
			txframe.cdat[5] = 0x01;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0x00;  
			tx_can_frame(0,11);

			shortdelay();
			txframe.MsgNum = 6;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x760;		  //Message ID VIN, VIN_HI hex 31 = ascii 1										
			txframe.sz = 6;
			txframe.cdat[0] = 0x00;				  //8 bytes
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x00;	      					
			txframe.cdat[3] = 0x00;	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;  
			tx_can_frame(0,11);

			shortdelay();
			txframe.MsgNum = 6;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x762;		  //Message ID VIN, VIN_HI hex 31 = ascii 1										
			txframe.sz = 8;
			txframe.cdat[0] = 0x00;				  //8 bytes
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x00;	      					
			txframe.cdat[3] = 0x00;	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0x00;  
			tx_can_frame(0,11);

			shortdelay();
			txframe.MsgNum = 6;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x764;		  //Message ID VIN, VIN_HI hex 31 = ascii 1										
			txframe.sz = 6;
			txframe.cdat[0] = 0x00;				  //8 bytes
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x00;	      					
			txframe.cdat[3] = 0x00;	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;	  
			tx_can_frame(0,11);

			/*
			shortdelay();
			txframe.MsgNum = 6;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x7B6;		  //Message ID VIN, VIN_HI hex 31 = ascii 1										
			txframe.sz = 8;
			txframe.cdat[0] = 0x00;				  //8 bytes
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x00;	      					
			txframe.cdat[3] = 0x00;	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0x00;  
			tx_can_frame(0,11);

			shortdelay();
			txframe.MsgNum = 6;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x7B8;		  //Message ID VIN, VIN_HI hex 31 = ascii 1										
			txframe.sz = 4;
			txframe.cdat[0] = 0x00;				  //8 bytes
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x00;	      					
			txframe.cdat[3] = 0x00;	  
			tx_can_frame(0,11);
			*/

			shortdelay();
			txframe.MsgNum = 6;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x7C0;		  //Message ID VIN, VIN_HI hex 31 = ascii 1										
			txframe.sz = 3;
			txframe.cdat[0] = 0x00;				  //8 bytes
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x00;  
			tx_can_frame(0,11);
			
			SWS_CUSW();
			lang_cfg();			

					
				
		
			//==================================================  Addition Steering Wheel buttons
			
			
			
			EA = 1;
			receive_data(30);
					
					
			
		//=====================================================  Ignition off message		
	




		
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

void check_lang(void)
{		

	
	//This row of if statements checks to see if the data conatins 0A 03 04 which is sent by the lang = ENG request
	if(rxframe[1].cdat[0] == 0x0B)
	{
		flag_eng = 1;
		flag_spa = 0;
		flag_ger = 0;
		flag_fre = 0;
		flag_ita = 0;
		flag_jap = 0;
		flag_chs = 0;
		flag_cht = 0;		
	}
	/*

	//This row of if statements checks to see if the data conatins 0A 03 01 which is sent by the lang = SPA request
	if((rxframe[1].cdat[0] == 0x0A) && (rxframe[1].cdat[1] == 0x03) && (rxframe[1].cdat[2] == 0x04))
	{
		flag_eng = 0;
		flag_spa = 1;
		flag_ger = 0;
		flag_fre = 0;
		flag_ita = 0;
		flag_jap = 0;
		flag_chs = 0;
		flag_cht = 0;	
			
		
	}

	
	
	if((rxframe[1].cdat[0] == 0x0A) && (rxframe[1].cdat[1] == 0x03) && (rxframe[1].cdat[2] == 0x00))
	{
		flag_eng = 0;
		flag_spa = 0;
		flag_ger = 1;
		flag_fre = 0;
		flag_ita = 0;
		flag_jap = 0;
		flag_chs = 0;
		flag_cht = 0;		
	}
	

	if((rxframe[1].cdat[0] == 0x0A) && (rxframe[1].cdat[1] == 0x03) && (rxframe[1].cdat[2] == 0x02))
	{
		flag_eng = 0;
		flag_spa = 0;
		flag_ger = 0;
		flag_fre = 1;
		flag_ita = 0;
		flag_jap = 0;
		flag_chs = 0;
		flag_cht = 0;		
	}

	
	if((rxframe[1].cdat[0] == 0x0A) && (rxframe[1].cdat[1] == 0x03) && (rxframe[1].cdat[2] == 0x03))
	{
		flag_eng = 0;
		flag_spa = 0;
		flag_ger = 0;
		flag_fre = 0;
		flag_ita = 1;
		flag_jap = 0;
		flag_chs = 0;
		flag_cht = 0;		
	}
	
	
	
	if((rxframe[1].cdat[0] == 0x0A) && (rxframe[1].cdat[1] == 0x03) && (rxframe[1].cdat[2] == 0x05))
	{
		flag_eng = 0;
		flag_spa = 0;
		flag_ger = 0;
		flag_fre = 0;
		flag_ita = 0;
		flag_jap = 1;
		flag_chs = 0;
		flag_cht = 0;	
			
		
	}
	
	*/
	if(rxframe[1].cdat[0] == 0x0E)
	{
		flag_eng = 0;
		flag_spa = 0;
		flag_ger = 0;
		flag_fre = 0;
		flag_ita = 0;
		flag_jap = 0;
		flag_chs = 1;
		flag_cht = 0;		
	}

	/*
	if((rxframe[1].cdat[0] == 0x0A) && (rxframe[1].cdat[1] == 0x03) && (rxframe[1].cdat[2] == 0x0E))
	{
		flag_eng = 0;
		flag_spa = 0;
		flag_ger = 0;
		flag_fre = 0;
		flag_ita = 0;
		flag_jap = 0;
		flag_chs = 0;
		flag_cht = 1;		
	}




	
	*/

	

}	

void SWS_CUSW(void)	  
{
	if (PH_BUTTON == 0 )                   

      				{ 
        					shortdelay();
							txframe.MsgNum = 13;			  //Message number(sequential starting at 1 for unique messages							
							txframe.arbID = 0x2EE;		//phone pickup												
							txframe.sz = 4;				
							txframe.cdat[0] = 0x00;	      						
							txframe.cdat[1] = 0x00;	
							txframe.cdat[2] = 0x10;	      					
							txframe.cdat[3] = 0x00;	  			  
							tx_can_frame(0,11);

						
	  			
						}			
				else if (VR_BUTTON == 0)  
      				  {  
       					 	shortdelay();
							txframe.MsgNum = 12;			  //Message number(sequential starting at 1 for unique messages							
							txframe.arbID= 0x2EE;		  									
							txframe.sz = 4;				  
							txframe.cdat[0] = 0x00;	      						
							txframe.cdat[1] = 0x04;	
							txframe.cdat[2] = 0x00;	      					
							txframe.cdat[3] = 0x00;	  		  	  
							tx_can_frame(0,11);
     				  }      
				
				else if (PH_BUTTON_END == 0)  
      				  {  
       					 	shortdelay();
							txframe.MsgNum = 12;			  //Message number(sequential starting at 1 for unique messages							
							txframe.arbID= 0x2EE;		  									
							txframe.sz = 4;				  
							txframe.cdat[0] = 0x00;	      						
							txframe.cdat[1] = 0x00;	
							txframe.cdat[2] = 0x40;	      					
							txframe.cdat[3] = 0x00;	  	
							tx_can_frame(0,11);
     				  }             
					
					

						 
				 else if (VOL_UP == 0)
					{
				shortdelay();
				txframe.MsgNum = 12;			  //Message number(sequential starting at 1 for unique messages							
				txframe.arbID= 0x2EE;		  //Message ID VR off									
				txframe.sz = 4;				  //8 bytes
				txframe.cdat[0] = 0x00;	      						
				txframe.cdat[1] = 0x00;	
				txframe.cdat[2] = 0x00;	  // lever in Park    					
				txframe.cdat[3] = 0x01;			  
				tx_can_frame(0,11);
			}
			else if (VOL_DWN == 0)
			{
				shortdelay();
				txframe.MsgNum = 12;			  //Message number(sequential starting at 1 for unique messages							
				txframe.arbID= 0x2EE;		  //Message ID VR off									
				txframe.sz = 4;				  //8 bytes
				txframe.cdat[0] = 0x00;	      						
				txframe.cdat[1] = 0x00;	
				txframe.cdat[2] = 0x00;	  // lever in Park    					
				txframe.cdat[3] = 0x04;	  		  
				tx_can_frame(0,11);
			}
			else if (SEEK_UP == 0)
			{
				shortdelay();
				txframe.MsgNum = 12;			  //Message number(sequential starting at 1 for unique messages							
				txframe.arbID= 0x2EE;		  //Message ID VR off									
				txframe.sz = 4;				  //8 bytes
				txframe.cdat[0] = 0x00;	      						
				txframe.cdat[1] = 0x00;	
				txframe.cdat[2] = 0x00;	  // lever in Park    					
				txframe.cdat[3] = 0x40;			  
				tx_can_frame(0,11);
			}
			else if (SEEK_DWN == 0)
			{
				shortdelay();
				txframe.MsgNum = 12;			  //Message number(sequential starting at 1 for unique messages							
				txframe.arbID= 0x2EE;		  //Message ID VR off									
				txframe.sz = 4;				  //8 bytes
				txframe.cdat[0] = 0x00;	      						
				txframe.cdat[1] = 0x00;	
				txframe.cdat[2] = 0x01;	  // lever in Park    					
				txframe.cdat[3] = 0x00;	 		  
				tx_can_frame(0,11);
			}
			else if (MODE == 0)
			{
				shortdelay();
				txframe.MsgNum = 12;			  //Message number(sequential starting at 1 for unique messages							
				txframe.arbID= 0x2EE;		  //Message ID VR off									
				txframe.sz = 4;				  //8 bytes
				txframe.cdat[0] = 0x00;	      						
				txframe.cdat[1] = 0x01;	
				txframe.cdat[2] = 0x00;	  // lever in Park    					
				txframe.cdat[3] = 0x00;			  
				tx_can_frame(0,11);
			}
			else if (PRESET == 0)
			{
				shortdelay();
				txframe.MsgNum = 12;			  //Message number(sequential starting at 1 for unique messages							
				txframe.arbID= 0x2EE;		  //Message ID VR off									
				txframe.sz = 4;				  //8 bytes
				txframe.cdat[0] = 0x00;	      						
				txframe.cdat[1] = 0x00;	
				txframe.cdat[2] = 0x00;	  // lever in Park    					
				txframe.cdat[3] = 0x10;			  
				tx_can_frame(0,11);
			}
			else 
			{
				shortdelay();
				txframe.MsgNum = 12;			  //Message number(sequential starting at 1 for unique messages							
				txframe.arbID= 0x2EE;		  //Message ID VR off									
				txframe.sz = 4;				  //8 bytes
				txframe.cdat[0] = 0x00;	      						
				txframe.cdat[1] = 0x00;	
				txframe.cdat[2] = 0x00;	  // lever in Park    					
				txframe.cdat[3] = 0x00;			  
				tx_can_frame(0,11);
			}             
			
			
}


void lang_cfg()
{
		
		if (flag_eng == 1)			// Set language configuration based on check_lang() function ---- English
		{
			shortdelay();
			txframe.MsgNum = 6;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x5BC;		  //Message ID VIN, VIN_HI hex 31 = ascii 1										
			txframe.sz = 8;
			txframe.cdat[0] = 0x0B;				  //8 bytes
			txframe.cdat[1] = 0x01;	
			txframe.cdat[2] = 0x60;	      					
			txframe.cdat[3] = 0x40;	  
			txframe.cdat[4] = 0x05;	      						
			txframe.cdat[5] = 0x19;	
			txframe.cdat[6] = 0x90;
			txframe.cdat[7] = 0x80;  
			tx_can_frame(0,11);
		}
		
		/*
		if (flag_spa == 1)			// Set language configuration based on check_lang() function ---- Spanish
		{
			shortdelay();
			txframe.MsgNum = 6;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x5BC;		  //Message ID VIN, VIN_HI hex 31 = ascii 1										
			txframe.sz = 8;
			txframe.cdat[0] = 0x00;				  //8 bytes
			txframe.cdat[1] = 0x01;	
			txframe.cdat[2] = 0x60;	      					
			txframe.cdat[3] = 0x40;	  
			txframe.cdat[4] = 0x05;	      						
			txframe.cdat[5] = 0x19;	
			txframe.cdat[6] = 0x90;
			txframe.cdat[7] = 0x80;  
			tx_can_frame(0,11);
		}

		
		if (flag_ger == 1)			// Set language configuration based on check_lang() function ---- Spanish
		{
			shortdelay();
			txframe.MsgNum = 6;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x5BC;		  //Message ID VIN, VIN_HI hex 31 = ascii 1										
			txframe.sz = 8;
			txframe.cdat[0] = 0x00;				  //8 bytes
			txframe.cdat[1] = 0x01;	
			txframe.cdat[2] = 0x60;	      					
			txframe.cdat[3] = 0x40;	  
			txframe.cdat[4] = 0x05;	      						
			txframe.cdat[5] = 0x19;	
			txframe.cdat[6] = 0x90;
			txframe.cdat[7] = 0x80;  
			tx_can_frame(0,11);
		}
		

		if (flag_fre == 1)			// Set language configuration based on check_lang() function ---- Spanish
		{
			shortdelay();
			txframe.MsgNum = 6;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x5BC;		  //Message ID VIN, VIN_HI hex 31 = ascii 1										
			txframe.sz = 8;
			txframe.cdat[0] = 0x00;				  //8 bytes
			txframe.cdat[1] = 0x01;	
			txframe.cdat[2] = 0x60;	      					
			txframe.cdat[3] = 0x40;	  
			txframe.cdat[4] = 0x05;	      						
			txframe.cdat[5] = 0x19;	
			txframe.cdat[6] = 0x90;
			txframe.cdat[7] = 0x80;  
			tx_can_frame(0,11);
		}

		
		if (flag_ita == 1)			// Set language configuration based on check_lang() function ---- English
		{
		shortdelay();
			txframe.MsgNum = 6;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x5BC;		  //Message ID VIN, VIN_HI hex 31 = ascii 1										
			txframe.sz = 8;
			txframe.cdat[0] = 0x00;				  //8 bytes
			txframe.cdat[1] = 0x01;	
			txframe.cdat[2] = 0x60;	      					
			txframe.cdat[3] = 0x40;	  
			txframe.cdat[4] = 0x05;	      						
			txframe.cdat[5] = 0x19;	
			txframe.cdat[6] = 0x90;
			txframe.cdat[7] = 0x80;  
			tx_can_frame(0,11);
		}
		
		
		if (flag_jap == 1)			// Set language configuration based on check_lang() function ---- Spanish
		{
		shortdelay();
		txframe.MsgNum = 12;			  //Message number(sequential starting at 1 for unique messages							
		txframe.arbID= 0x49D;		  //Message ID VR off									
		txframe.sz = 8;				  //8 bytes
		txframe.cdat[0] = 0x00;	      						
		txframe.cdat[1] = 0x00;	
		txframe.cdat[2] = 0x00;	  // lever in Park    					
		txframe.cdat[3] = 0x00;	  
		txframe.cdat[4] = 0x00;	      						
		txframe.cdat[5] = 0x00;	
		txframe.cdat[6] = 0x50;
		txframe.cdat[7] = 0x00;		  
		tx_can_frame(0,11);
		}
		*/

		if (flag_chs == 1)			// Set language configuration based on check_lang() function ---- Spanish
		{
			shortdelay();
			txframe.MsgNum = 6;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x5BC;		  //Message ID VIN, VIN_HI hex 31 = ascii 1										
			txframe.sz = 8;
			txframe.cdat[0] = 0x0E;				  //8 bytes
			txframe.cdat[1] = 0x01;	
			txframe.cdat[2] = 0x60;	      					
			txframe.cdat[3] = 0x40;	  
			txframe.cdat[4] = 0x05;	      						
			txframe.cdat[5] = 0x19;	
			txframe.cdat[6] = 0x90;
			txframe.cdat[7] = 0x80;  
			tx_can_frame(0,11);
		}

		/*
		if (flag_cht == 1)			// Set language configuration based on check_lang() function ---- Spanish
		{
		shortdelay();
		txframe.MsgNum = 12;			  //Message number(sequential starting at 1 for unique messages							
		txframe.arbID= 0x49D;		  //Message ID VR off									
		txframe.sz = 8;				  //8 bytes
		txframe.cdat[0] = 0x00;	      						
		txframe.cdat[1] = 0x00;	
		txframe.cdat[2] = 0x00;	  // lever in Park    					
		txframe.cdat[3] = 0x00;	  
		txframe.cdat[4] = 0x00;	      						
		txframe.cdat[5] = 0x00;	
		txframe.cdat[6] = 0x90;
		txframe.cdat[7] = 0x00;		  
		tx_can_frame(0,11);
		}
		*/
}

void init_msg_object_rx (unsigned char MsgNum,unsigned int arbID) 
{
	SFRPAGE = CAN0_PAGE;
	CAN0ADR = IF2CMDMSK;  		        	// Point to Command Mask 1
	CAN0DAT = 0x00B8;     		        	// Set to WRITE, and alter all Msg Obj except ID MASK and data bits
	CAN0ADR = IF2ARB1;    		        	// Point to arbitration1 register
	CAN0DAT = 0x0000;
	CAN0DAT = (arbID << 2) | 0x8000;
	CAN0DAT = 0x0480;     		        	// Msg Cntrl: set RX IE, remote frame function not enabled
	CAN0ADR = IF2CMDRQST; 		        	// Point to Command Request reg.
	CAN0DATL = MsgNum;    		        	// Select Msg Obj passed into function parameter list --initiates write to Msg Obj
   											// 3-6 CAN clock cycles to move IF register contents to the Msg Obj in CAN RAM.
}

void IGN_MSG (void)
{

	if (IGN_OFF == 0)
	{
			
			while (iii < 300)
			{
				if (IGN_OFF != 0)
					break;
			  	//transmits can frame	

				onehundredms();				  	//100ms delay loop
				txframe.MsgNum = 1;			  	//Message number(sequential starting at 1 for unique messages							
				txframe.arbID = 0x46C;		  	//Message ID	CBC_13 Vehicle start packet									
				txframe.sz = 8;				  	//Message size(number of bytes 1-8)
				txframe.cdat[0] = 0x00;	      	//ignition run, start not active  04 = run ;02=ACC ;0D= NormalStart, key in Ign, 						
				txframe.cdat[1] = 0x20;		  	//key in ignition
				txframe.cdat[2] = 0x40;	      	//null					
				txframe.cdat[3] = 0x00;	  		//null  
				txframe.cdat[4] = 0x00;
				txframe.cdat[5] = 0x00;
				txframe.cdat[6] = 0x00;
				txframe.cdat[7] = 0x00;
				tx_can_frame(0,11);			  	//transmits can 
				
				shortdelay();
				txframe.MsgNum = 6;			  //Message number(sequential starting at 1 for unique messages							
				txframe.arbID = 0x356;		  //Message ID VIN, VIN_HI hex 31 = ascii 1										
				txframe.sz = 8;
				txframe.cdat[0] = 0x00;				  //8 bytes
				txframe.cdat[1] = 0x80;	
				txframe.cdat[2] = 0x1F;	      					
				txframe.cdat[3] = 0xFF;	  
				txframe.cdat[4] = 0xC0;	      						
				txframe.cdat[5] = 0x14;	
				txframe.cdat[6] = 0x00;
				txframe.cdat[7] = 0x00;  
				tx_can_frame(0,11);

				iii++;
				
			}

	}
	
	/*if (IGN_ACC == 0)
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
	*/
	if (IGN_RUN_0 == 0)
	{
			  	//transmits can frame

			onehundredms();				  	//100ms delay loop
			txframe.MsgNum = 1;			  	//Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x46C;		  	//Message ID	CBC_13 Vehicle start packet									
			txframe.sz = 8;				  	//Message size(number of bytes 1-8)
			txframe.cdat[0] = 0x00;	      	//ignition run, start not active  04 = run ;02=ACC ;0D= NormalStart, key in Ign, 						
			txframe.cdat[1] = 0x00;		  	//key in ignition
			txframe.cdat[2] = 0x40;	      	//null					
			txframe.cdat[3] = 0x00;	  		//null  
			txframe.cdat[4] = 0x00;
			txframe.cdat[5] = 0x00;
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0x00;
			tx_can_frame(0,11);
			
			shortdelay();
			txframe.MsgNum = 6;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x356;		  //Message ID VIN, VIN_HI hex 31 = ascii 1										
			txframe.sz = 8;
			txframe.cdat[0] = 0x00;				  //8 bytes
			txframe.cdat[1] = 0x80;	
			txframe.cdat[2] = 0xFF;	      					
			txframe.cdat[3] = 0xFF;	  
			txframe.cdat[4] = 0xC0;	      						
			txframe.cdat[5] = 0x14;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0x00;  
			tx_can_frame(0,11);	
	}
/*
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
	}*/
}