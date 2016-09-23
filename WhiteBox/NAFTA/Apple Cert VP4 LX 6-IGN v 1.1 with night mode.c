//Can Bus Simulator
//See function radtest for transmitting data


#include "c8051F040.h"	
#include "telematics.h"						
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

sbit LED1 = P2^0; 						// led output pin 
//sbit VR_BUTTON  	= P1^4;				// sw1 = 0 means switch pressed
////sbit NIGHT_MODE_OFF  	= P1^4;
//sbit PH_BUTTON 		= P1^3;				// sw2 = 0 means switch pressed
//sbit PH_BUTTON_END	= P1^2;				// sw2 = 0 means switch pressed
sbit SEL_CAN_B 		= P1^7;				// CAN select pin 0 means can B, 1 means CAN c/i
sbit CAN_B_EN  		= P1^6;				// CAN B enable pin,  0 means enable, 1 means disable
					//Added for 6 position switch

sbit IGN_OFF 		= P0^7;				// sw5 = 0 means switch pressed
sbit IGN_ACC 		= P1^1;				// sw6 = 0 means switch pressed
sbit IGN_RUN		= P1^0;

sbit IGN_RUN_0  	= P1^5;				// sw3 = 0 means switch pressed
sbit IGN_RUN_5		= P0^6;
sbit IGN_RUN_10 	= P0^5;				// sw4 = 0 means switch pressed

sbit GEAR_PARK  	= P1^4;				// sw1 = 0 means switch pressed
sbit GEAR_DRIVE 	= P1^3;				// sw2 = 0 means switch pressed
sbit GEAR_REV		= P1^2;	

//sbit NIGHT_MODE_OFF  = P0^3;


struct txframe
{
	unsigned char MsgNum;
	unsigned char sz;
	unsigned long arbID;
 	unsigned char cdat[8];

}txframe;
CANFRAME xdata rxframe[32];


byte xdata IntReg; ////
char MsgNum;
char status;
int i;
int v;
int j = 0;
int h;
int iii = 0;

//unsigned char Night_Mode_Flg=0x01;
int count1 = 0; ////

void config_CAN_timing(void);
void config_IO(void);
void receive_data(char MsgNum);
void clear_msg_objects(void);
void radiotest(void);
void codedelay(unsigned int delay);
void clear_message_object (unsigned char mesgobj,unsigned char count);
void onehundredms(void);
void shortdelay(void);
void IGN_MSG(void);

/*
//void init_msg_object_rx (unsigned char MsgNum, unsigned int arbID); ////
void check_lang(void); ////
void lang_cfg(void); ////
*/


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

//	init_msg_object_rx(30, 0x314); ////
//	rxframe[1].MsgNum = 0; ////

//	flag_eng = 1;	 ////
  //Loop and wait for interrupts
  while (1)
    {
		IGN_MSG();
		if (IGN_OFF != 0)
		{
			radiotest();
			iii = 0;
		}
		
	}	    
}


byte tx_can_frame (unsigned char KWPopts) 
{   		
	int timeout = 4096;						// roughley 2.5ms

	EIE2 &= ~0x01;                 			// disable Timer3 interrupts

	SFRPAGE = CAN0_PAGE;
	CAN0STA &= ~BIT3;						// Clear status bits

	CAN0ADR = IF1CMDMSK;          			// Point to Command Mask 1
	CAN0DAT = 0x00B7;             			// Config to WRITE to CAN RAM, write data bytes, set TXrqst/NewDat, Clr IntPnd
	CAN0ADR = IF1ARB1;


	
		// 11BIT ID
		CAN0DAT = 0x0000;					// Arb1 Data			
		CAN0DAT = ((unsigned int)(txframe.arbID << 2)) | 0xA000;
	

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

//	CAN0DATH = 0x5A;					// 18.0mhz BRP=7 125kbps
//	CAN0DATL = 0xC7;					// Tseg2=6, Tseg1=11, SJW=4

	CAN0DATH = 0x5A;					// 18.0mhz BRP=7 125kbps
	CAN0DATL = 0xC1;					// Tseg2=6, Tseg1=11, SJW=4
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
  
  SFRPAGE  = CAN0_PAGE; // IF1 already set up for RX
  CAN0ADR  = IF2CMDRQST;// Point to Command Request Reg.
  CAN0DATL = MsgNum;    // Move new data for RX from Msg Obj "MsgNum"
                        // Move new data to a
  CAN0ADR  = IF2DATA1;  // Point to 1st byte of Data Field 

/*	SFRPAGE = CAN0_PAGE; //Saves ////

	
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
		} ////  
*/
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
//	EA = 0;
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

void IGN_MSG (void)
{

	if (IGN_OFF == 0)
	{
			
			while (iii < 5)
			{
				if (IGN_OFF != 0)
					break;
			//	onehundredms();				  	//100ms delay loop
				shortdelay();
				txframe.MsgNum = 1;			  	//Message number(sequential starting at 1 for unique messages							
				txframe.arbID = 0x184;		  	//Message ID	RFHUB_A1; 100ms									
				txframe.sz = 4;				  	//Message size(number of bytes 1-8)
				txframe.cdat[0] = 0x22;	      	//ignition position - IGN_LK						
				txframe.cdat[1] = 0x00;		  	//key in ignition
				txframe.cdat[2] = 0x00;	      	//null					
				txframe.cdat[3] = 0x00;	  		//null  
				tx_can_frame(0);
			  	//transmits can frame

				shortdelay();
				txframe.MsgNum = 20;			  //Message - ESP_A8; Cycle Time - 20ms  						
				txframe.arbID = 0x11C;		   	// speed = 	km/h  /  0.0078125 km/h *	(50k/h / 0.0078125 km/h)=  6400 = 0x1900 						
				txframe.sz = 8;				  
				txframe.cdat[0] = 0x0F;	      						
				txframe.cdat[1] = 0xFF;	
				txframe.cdat[2] = 0x0F;	      					
				txframe.cdat[3] = 0xFF;	  
				txframe.cdat[4] = 0x00;	      	// Speed 0					
				txframe.cdat[5] = 0x00;	
				txframe.cdat[6] = 0x00;
				txframe.cdat[7] = 0xFF;
				tx_can_frame(0);
				
				shortdelay();
				txframe.MsgNum = 20;			 //Message - TRNS_STAT; Cycle Time - 20ms  						
				txframe.arbID = 0x170;		   	// 	PRNDL_DISP = P; PRND_STAT - LVR_P				
				txframe.sz = 8;				  
				txframe.cdat[0] = 0x00;	      						
				txframe.cdat[1] = 0xFF;	
				txframe.cdat[2] = 0x50;	      					
				txframe.cdat[3] = 0x00;	  
				txframe.cdat[4] = 0x00;	      						
				txframe.cdat[5] = 0x00;	
				txframe.cdat[6] = 0x00;
				txframe.cdat[7] = 0xFF;
				tx_can_frame(0);

				shortdelay();
				txframe.MsgNum = 11;		  //Message number(sequential starting at 1 for unique messages							
				txframe.arbID = 0x144;		  //TRNS_SPD; 20ms				
				txframe.sz = 8;				  //8 bytes
				txframe.cdat[0] = 0x00;	      						
				txframe.cdat[1] = 0xDD;	
				txframe.cdat[2] = 0x00;	      					
				txframe.cdat[3] = 0x03;	  
				txframe.cdat[4] = 0x20;	      						
				txframe.cdat[5] = 0xFF;	
				txframe.cdat[6] = 0xFF;	
				txframe.cdat[7] = 0x00;				
				tx_can_frame(0);
					
				iii++;
				
			}

	}
	
	else if (IGN_ACC == 0)
	{
			shortdelay();				  	//100ms delay loop
			txframe.MsgNum = 1;			  	//Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x184;		  	//Message ID	RFHUB_A1									
			txframe.sz = 4;				  	//Message size(number of bytes 1-8)
			txframe.cdat[0] = 0x2E;	      	//ignition position - IGN_OFF_ACC						
			txframe.cdat[1] = 0x00;		  	//key in ignition
			txframe.cdat[2] = 0x00;	      	//null					
			txframe.cdat[3] = 0x00;	  		//null  
			tx_can_frame(0);

			shortdelay();
			txframe.MsgNum = 20;			  //Message - ESP_A8; Cycle Time - 20ms  						
			txframe.arbID = 0x11C;		   	// speed = 	km/h  /  0.0078125 km/h *	(50k/h / 0.0078125 km/h)=  6400 = 0x1900 						
			txframe.sz = 8;				  
			txframe.cdat[0] = 0x0F;	      						
			txframe.cdat[1] = 0xFF;	
			txframe.cdat[2] = 0x0F;	      					
			txframe.cdat[3] = 0xFF;	  
			txframe.cdat[4] = 0x00;	      	// Speed 0					
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0xFF;
			tx_can_frame(0);

			shortdelay();
			txframe.MsgNum = 20;			 //Message - TRNS_STAT; Cycle Time - 20ms  						
			txframe.arbID = 0x170;		   	// 	PRNDL_DISP = P; PRND_STAT - LVR_P				
			txframe.sz = 8;				  
			txframe.cdat[0] = 0x00;	      						
			txframe.cdat[1] = 0xFF;	
			txframe.cdat[2] = 0x50;	      					
			txframe.cdat[3] = 0x00;	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0xFF;
			tx_can_frame(0);

			shortdelay();
			txframe.MsgNum = 11;		  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x144;		  //TRNS_SPD; 20ms				
			txframe.sz = 8;				  //8 bytes
			txframe.cdat[0] = 0x00;	      						
			txframe.cdat[1] = 0xDD;	
			txframe.cdat[2] = 0x00;	      					
			txframe.cdat[3] = 0x03;	  
			txframe.cdat[4] = 0x20;	      						
			txframe.cdat[5] = 0xFF;	
			txframe.cdat[6] = 0xFF;	
			txframe.cdat[7] = 0x00;				
			tx_can_frame(0);
	}

	else if (IGN_RUN == 0)
	{

		shortdelay();				  	//100ms delay loop
		txframe.MsgNum = 1;			  	//Message number(sequential starting at 1 for unique messages							
		txframe.arbID = 0x184;		  	//Message ID	RFHUB_A1									
		txframe.sz = 4;				  	//Message size(number of bytes 1-8)
		txframe.cdat[0] = 0x32;	      	//ignition position - IGN_RUN						
		txframe.cdat[1] = 0x00;		  	//key in ignition
		txframe.cdat[2] = 0x00;	      	//null					
		txframe.cdat[3] = 0x00;	  		//null  
		tx_can_frame(0);

			if	(GEAR_PARK == 0)
//			if	(1)
			{		
				shortdelay();
				txframe.MsgNum = 20;			  //Message - ESP_A8; Cycle Time - 20ms  						
				txframe.arbID = 0x11C;		   	// speed = 	km/h  /  0.0078125 km/h *	(50k/h / 0.0078125 km/h)=  6400 = 0x1900 						
				txframe.sz = 8;				  
				txframe.cdat[0] = 0x0F;	      						
				txframe.cdat[1] = 0xFF;	
				txframe.cdat[2] = 0x0F;	      					
				txframe.cdat[3] = 0xFF;	  
				txframe.cdat[4] = 0x00;	      	// Speed 0					
				txframe.cdat[5] = 0x00;	
				txframe.cdat[6] = 0x00;
				txframe.cdat[7] = 0xFF;
				tx_can_frame(0);

				shortdelay();
				txframe.MsgNum = 20;			 //Message - TRNS_STAT; Cycle Time - 20ms  						
				txframe.arbID = 0x170;		   	// 	PRNDL_DISP = P; PRND_STAT - LVR_P				
				txframe.sz = 8;				  
				txframe.cdat[0] = 0x00;	      						
				txframe.cdat[1] = 0xFF;	
				txframe.cdat[2] = 0x50;	      					
				txframe.cdat[3] = 0x00;	  
				txframe.cdat[4] = 0x00;	      						
				txframe.cdat[5] = 0x00;	
				txframe.cdat[6] = 0x00;
				txframe.cdat[7] = 0xFF;
				tx_can_frame(0);

				shortdelay();
				txframe.MsgNum = 11;		  //Message number(sequential starting at 1 for unique messages							
				txframe.arbID = 0x144;		  //TRNS_SPD; 20ms				
				txframe.sz = 8;				  //8 bytes
				txframe.cdat[0] = 0x00;	      						
				txframe.cdat[1] = 0xDD;	
				txframe.cdat[2] = 0x00;	      					
				txframe.cdat[3] = 0x03;	  
				txframe.cdat[4] = 0x20;	      						
				txframe.cdat[5] = 0xFF;	
				txframe.cdat[6] = 0xFF;	
				txframe.cdat[7] = 0x00;				
				tx_can_frame(0);
			}

			else if (GEAR_REV == 0)
//			else if (1)
			{
				shortdelay();
				txframe.MsgNum = 20;			  //Message - ESP_A8; Cycle Time - 20ms  						
				txframe.arbID = 0x11C;		   	// speed = 	km/h  /  0.0078125 km/h *	(50k/h / 0.0078125 km/h)=  6400 = 0x1900 						
				txframe.sz = 8;				  
				txframe.cdat[0] = 0x0F;	      						
				txframe.cdat[1] = 0xFF;	
				txframe.cdat[2] = 0x0F;	      					
				txframe.cdat[3] = 0xFF;	  
				txframe.cdat[4] = 0x00;	      	// Speed 0					
				txframe.cdat[5] = 0x00;	
				txframe.cdat[6] = 0x00;
				txframe.cdat[7] = 0xFF;
				tx_can_frame(0);

				shortdelay();
				txframe.MsgNum = 20;			 //Message - TRNS_STAT; Cycle Time - 20ms  						
				txframe.arbID = 0x170;		   	// 	PRNDL_DISP = R; PRND_STAT - LVR_R				
				txframe.sz = 8;				  
				txframe.cdat[0] = 0x04;	      						
				txframe.cdat[1] = 0xFF;	
				txframe.cdat[2] = 0x52;	      					
				txframe.cdat[3] = 0x00;	  
				txframe.cdat[4] = 0x00;	      						
				txframe.cdat[5] = 0x00;	
				txframe.cdat[6] = 0x00;
				txframe.cdat[7] = 0xFF;
				tx_can_frame(0);

				shortdelay();
				txframe.MsgNum = 11;		  //Message number(sequential starting at 1 for unique messages							
				txframe.arbID = 0x144;		  //TRNS_SPD; 20ms				
				txframe.sz = 8;				  //8 bytes
				txframe.cdat[0] = 0x00;	      						
				txframe.cdat[1] = 0xBB;	
				txframe.cdat[2] = 0x00;	      					
				txframe.cdat[3] = 0x03;	  
				txframe.cdat[4] = 0x20;	      						
				txframe.cdat[5] = 0xFF;	
				txframe.cdat[6] = 0xFF;	
				txframe.cdat[7] = 0x00;				
				tx_can_frame(0);
			}

			else if (GEAR_DRIVE == 0)
			{
				shortdelay();
				txframe.MsgNum = 20;			 //Message - TRNS_STAT; Cycle Time - 20ms  						
				txframe.arbID = 0x170;		   	// 	PRNDL_DISP = D; PRND_STAT - LVR_D				
				txframe.sz = 8;				  
				txframe.cdat[0] = 0x10;	      						
				txframe.cdat[1] = 0xFF;	
				txframe.cdat[2] = 0x44;	      					
				txframe.cdat[3] = 0x00;	  
				txframe.cdat[4] = 0x00;	      						
				txframe.cdat[5] = 0x00;	
				txframe.cdat[6] = 0x00;
				txframe.cdat[7] = 0xFF;
				tx_can_frame(0);

				shortdelay();
				txframe.MsgNum = 11;		  //Message number(sequential starting at 1 for unique messages							
				txframe.arbID = 0x144;		  //TRNS_SPD; 20ms				
				txframe.sz = 8;				  //8 bytes
				txframe.cdat[0] = 0x00;	      						
				txframe.cdat[1] = 0x11;	
				txframe.cdat[2] = 0x00;	      					
				txframe.cdat[3] = 0x03;	  
				txframe.cdat[4] = 0x20;	      						
				txframe.cdat[5] = 0xFF;	
				txframe.cdat[6] = 0xFF;	
				txframe.cdat[7] = 0x00;				
				tx_can_frame(0);

				if (IGN_RUN_0 == 0)
				{
					shortdelay();
					txframe.MsgNum = 20;			  //Message - ESP_A8; Cycle Time - 20ms  						
					txframe.arbID = 0x11C;		   	// speed = 	km/h  /  0.0078125 km/h *	(50k/h / 0.0078125 km/h)=  6400 = 0x1900 						
					txframe.sz = 8;				  
					txframe.cdat[0] = 0x0F;	      						
					txframe.cdat[1] = 0xFF;	
					txframe.cdat[2] = 0x0F;	      					
					txframe.cdat[3] = 0xFF;	  
					txframe.cdat[4] = 0x00;	      	// Speed 0					
					txframe.cdat[5] = 0x00;	
					txframe.cdat[6] = 0x00;
					txframe.cdat[7] = 0xFF;
					tx_can_frame(0);

				}

				else if (IGN_RUN_5 == 0)
				{

					shortdelay();
					txframe.MsgNum = 20;			  //Message - ESP_A8; Cycle Time - 20ms  						
					txframe.arbID = 0x11C;		   	// speed = 	km/h  /  0.0078125 km/h *	(50k/h / 0.0078125 km/h)=  6400 = 0x1900 						
					txframe.sz = 8;				  
					txframe.cdat[0] = 0x0F;	      						
					txframe.cdat[1] = 0xFF;	
					txframe.cdat[2] = 0x0F;	      					
					txframe.cdat[3] = 0xFF;	  
					txframe.cdat[4] = 0x04;	      	// Speed 5 mph					
					txframe.cdat[5] = 0x06;	
					txframe.cdat[6] = 0x00;
					txframe.cdat[7] = 0xFF;
					tx_can_frame(0);		

				}

				else if (IGN_RUN_10 == 0)
				{

					shortdelay();
					txframe.MsgNum = 20;			  //Message - ESP_A8; Cycle Time - 20ms  						
					txframe.arbID = 0x11C;		   	// speed = 	km/h  /  0.0078125 km/h *	(50k/h / 0.0078125 km/h)=  6400 = 0x1900 						
					txframe.sz = 8;				  
					txframe.cdat[0] = 0x0F;	      						
					txframe.cdat[1] = 0xFF;	
					txframe.cdat[2] = 0x0F;	      					
					txframe.cdat[3] = 0xFF;	  
					txframe.cdat[4] = 0x08;	      	// Speed 5 mph					
					txframe.cdat[5] = 0x0C;	
					txframe.cdat[6] = 0x00;
					txframe.cdat[7] = 0xFF;
					tx_can_frame(0);		

				}

				else
				{
					shortdelay();
					txframe.MsgNum = 20;			  //Message - ESP_A8; Cycle Time - 20ms  						
					txframe.arbID = 0x11C;		   	// speed = 	km/h  /  0.0078125 km/h *	(50k/h / 0.0078125 km/h)=  6400 = 0x1900 						
					txframe.sz = 8;				  
					txframe.cdat[0] = 0x0F;	      						
					txframe.cdat[1] = 0xFF;	
					txframe.cdat[2] = 0x0F;	      					
					txframe.cdat[3] = 0xFF;	  
					txframe.cdat[4] = 0x00;	      	// Speed 0					
					txframe.cdat[5] = 0x00;	
					txframe.cdat[6] = 0x00;
					txframe.cdat[7] = 0xFF;
					tx_can_frame(0);

				}
			}
				
		else
		{	
			shortdelay();
			txframe.MsgNum = 20;			  //Message - ESP_A8; Cycle Time - 20ms  						
			txframe.arbID = 0x11C;		   	// speed = 	km/h  /  0.0078125 km/h *	(50k/h / 0.0078125 km/h)=  6400 = 0x1900 						
			txframe.sz = 8;				  
			txframe.cdat[0] = 0x0F;	      						
			txframe.cdat[1] = 0xFF;	
			txframe.cdat[2] = 0x0F;	      					
			txframe.cdat[3] = 0xFF;	  
			txframe.cdat[4] = 0x00;	      	// Speed 0					
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0xFF;
			tx_can_frame(0);

			shortdelay();
			txframe.MsgNum = 20;			 //Message - TRNS_STAT; Cycle Time - 20ms  						
			txframe.arbID = 0x170;		   	// 	PRNDL_DISP = P; PRND_STAT - LVR_P				
			txframe.sz = 8;				  
			txframe.cdat[0] = 0x00;	      						
			txframe.cdat[1] = 0xFF;	
			txframe.cdat[2] = 0x50;	      					
			txframe.cdat[3] = 0x00;	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0xFF;
			tx_can_frame(0);

			shortdelay();
			txframe.MsgNum = 11;		  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x144;		  //TRNS_SPD; 20ms				
			txframe.sz = 8;				  //8 bytes
			txframe.cdat[0] = 0x00;	      						
			txframe.cdat[1] = 0xDD;	
			txframe.cdat[2] = 0x00;	      					
			txframe.cdat[3] = 0x03;	  
			txframe.cdat[4] = 0x20;	      						
			txframe.cdat[5] = 0xFF;	
			txframe.cdat[6] = 0xFF;	
			txframe.cdat[7] = 0x00;				
			tx_can_frame(0);
		}
		
	}
	
	else
	{
	//	onehundredms();				  	//100ms delay loop
		shortdelay();
		txframe.MsgNum = 1;			  	//Message number(sequential starting at 1 for unique messages							
		txframe.arbID = 0x184;		  	//Message ID	RFHUB_A1; 100ms									
		txframe.sz = 4;				  	//Message size(number of bytes 1-8)
		txframe.cdat[0] = 0x22;	      	//ignition position - IGN_LK						
		txframe.cdat[1] = 0x00;		  	//key in ignition
		txframe.cdat[2] = 0x00;	      	//null					
		txframe.cdat[3] = 0x00;	  		//null  
		tx_can_frame(0);
		//transmits can frame

		shortdelay();
		txframe.MsgNum = 20;			  //Message - ESP_A8; Cycle Time - 20ms  						
		txframe.arbID = 0x11C;		   	// speed = 	km/h  /  0.0078125 km/h *	(50k/h / 0.0078125 km/h)=  6400 = 0x1900 						
		txframe.sz = 8;				  
		txframe.cdat[0] = 0x0F;	      						
		txframe.cdat[1] = 0xFF;	
		txframe.cdat[2] = 0x0F;	      					
		txframe.cdat[3] = 0xFF;	  
		txframe.cdat[4] = 0x00;	      	// Speed 0					
		txframe.cdat[5] = 0x00;	
		txframe.cdat[6] = 0x00;
		txframe.cdat[7] = 0xFF;
		tx_can_frame(0);
				
		shortdelay();
		txframe.MsgNum = 20;			 //Message - TRNS_STAT; Cycle Time - 20ms  						
		txframe.arbID = 0x170;		   	// 	PRNDL_DISP = P; PRND_STAT - LVR_P				
		txframe.sz = 8;				  
		txframe.cdat[0] = 0x00;	      						
		txframe.cdat[1] = 0xFF;	
		txframe.cdat[2] = 0x50;	      					
		txframe.cdat[3] = 0x00;	  
		txframe.cdat[4] = 0x00;	      						
		txframe.cdat[5] = 0x00;	
		txframe.cdat[6] = 0x00;
		txframe.cdat[7] = 0xFF;
		tx_can_frame(0);

		shortdelay();
		txframe.MsgNum = 11;		  //Message number(sequential starting at 1 for unique messages							
		txframe.arbID = 0x144;		  //TRNS_SPD; 20ms				
		txframe.sz = 8;				  //8 bytes
		txframe.cdat[0] = 0x00;	      						
		txframe.cdat[1] = 0xDD;	
		txframe.cdat[2] = 0x00;	      					
		txframe.cdat[3] = 0x03;	  
		txframe.cdat[4] = 0x20;	      						
		txframe.cdat[5] = 0xFF;	
		txframe.cdat[6] = 0xFF;	
		txframe.cdat[7] = 0x00;				
		tx_can_frame(0);
				
	}
	
}

void radiotest(void)

{	
		//VIN Entry -------------------------------------------------------------------------------

//		char VIN[17] = "1C3ADECZ7GV1002UC";
		char VIN[17] = "2C3CCABT6GH100096";
		
			shortdelay();
			txframe.MsgNum = 4;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x3E0;		  //VIN; 100ms									
			txframe.sz = 8;				  //8 bytes
			txframe.cdat[0] = 0x00;	      						
			txframe.cdat[1] = VIN[0];	
			txframe.cdat[2] = VIN[1];	      					
			txframe.cdat[3] = VIN[2];	  
			txframe.cdat[4] = VIN[3];	      						
			txframe.cdat[5] = VIN[4];	
			txframe.cdat[6] = VIN[5];
			txframe.cdat[7] = VIN[6];		  
			tx_can_frame(0);

	//		onehundredms();
			shortdelay();
			shortdelay();
			txframe.MsgNum = 5;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x3E0;		  //VIN_MID; 100ms										
			txframe.sz = 8;				  //8 bytes
			txframe.cdat[0] = 0x01;	      						
			txframe.cdat[1] = VIN[7];	
			txframe.cdat[2] = VIN[8];	      				
			txframe.cdat[3] = VIN[9];	  
			txframe.cdat[4] = VIN[10];	      						
			txframe.cdat[5] = VIN[11];	
			txframe.cdat[6] = VIN[12];
			txframe.cdat[7] = VIN[13];		  
			tx_can_frame(0);

	//		onehundredms();
			shortdelay();
			shortdelay();
			txframe.MsgNum = 6;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x3E0;		  //VIN_HI; 100ms									
			txframe.sz = 8;				  //8 bytes
			txframe.cdat[0] = 0x02;	      						
			txframe.cdat[1] = VIN[14];	
			txframe.cdat[2] = VIN[15];	      					
			txframe.cdat[3] = VIN[16];	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0x00;		  
			tx_can_frame(0);

			
			
			shortdelay();
			txframe.MsgNum = 11;		  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x108;		  //ECM_A1 - RPM can be changed using this message; 10ms								
			txframe.sz = 8;				  //8 bytes
			txframe.cdat[0] = 0x00;	      						
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x0F;	      					
			txframe.cdat[3] = 0xA0;	  
			txframe.cdat[4] = 0x0F;	      						
			txframe.cdat[5] = 0xA0;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0xA4;		  
			tx_can_frame(0);


			shortdelay();
			txframe.MsgNum = 11;		  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x41A;		  //NM_EPS; 10ms								
			txframe.sz = 8;				  //8 bytes
			txframe.cdat[0] = 0xFE;	      						
			txframe.cdat[1] = 0xFF;	
			txframe.cdat[2] = 0xFF;	      					
			txframe.cdat[3] = 0xFF;	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0xFF;		  
			tx_can_frame(0);

			shortdelay();
			txframe.MsgNum = 11;		  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x407;		  //NM_ESC; 10ms								
			txframe.sz = 8;				  //8 bytes
			txframe.cdat[0] = 0xFE;	      						
			txframe.cdat[1] = 0xFF;	
			txframe.cdat[2] = 0xFF;	      					
			txframe.cdat[3] = 0xFF;	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0xFF;		  
			tx_can_frame(0);
			
/*			shortdelay();
			txframe.MsgNum = 11;		  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x402;		  //NM_IC; 10ms								
			txframe.sz = 8;				  //8 bytes
			txframe.cdat[0] = 0xFE;	      						
			txframe.cdat[1] = 0xFF;	
			txframe.cdat[2] = 0xFF;	      					
			txframe.cdat[3] = 0xFF;	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0xFF;		  
			tx_can_frame(0); 	*/
			
			shortdelay();
			txframe.MsgNum = 11;		  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x400;		  //NM_RF_HUB; 10ms								
			txframe.sz = 8;				  //8 bytes
			txframe.cdat[0] = 0xFE;	      						
			txframe.cdat[1] = 0xFF;	
			txframe.cdat[2] = 0xFF;	      					
			txframe.cdat[3] = 0xFF;	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0xFF;		  
			tx_can_frame(0);
			
			shortdelay();
			txframe.MsgNum = 11;		  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x409;		  //NM_SBWM; 10ms								
			txframe.sz = 8;				  //8 bytes
			txframe.cdat[0] = 0xFE;	      						
			txframe.cdat[1] = 0xFF;	
			txframe.cdat[2] = 0xFF;	      					
			txframe.cdat[3] = 0xFF;	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0xFF;		  
			tx_can_frame(0);

			shortdelay();
			txframe.MsgNum = 11;		  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x423;		  //NM_SCCM; 10ms								
			txframe.sz = 8;				  //8 bytes
			txframe.cdat[0] = 0xFE;	      						
			txframe.cdat[1] = 0xFF;	
			txframe.cdat[2] = 0xFF;	      					
			txframe.cdat[3] = 0xFF;	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0xFF;		  
			tx_can_frame(0);

			shortdelay();
			txframe.MsgNum = 11;		  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x403;		  //NM_TPM; 10ms								
			txframe.sz = 8;				  //8 bytes
			txframe.cdat[0] = 0xFE;	      						
			txframe.cdat[1] = 0xFF;	
			txframe.cdat[2] = 0x3F;	      					
			txframe.cdat[3] = 0xFF;	  
			txframe.cdat[4] = 0x01;	      						
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0xFF;		  
			tx_can_frame(0);

			shortdelay();
			txframe.MsgNum = 11;		  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x22F;		  //ECM_A5; 20ms						
			txframe.sz = 8;				  //8 bytes
			txframe.cdat[0] = 0x00;	      						
			txframe.cdat[1] = 0xFF;	
			txframe.cdat[2] = 0x54;	      					
			txframe.cdat[3] = 0x72;	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x00;	
			txframe.cdat[7] = 0xFF;				
			tx_can_frame(0);

/*			shortdelay();
			txframe.MsgNum = 11;		  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x144;		  //TRNS_SPD; 20ms				
			txframe.sz = 8;				  //8 bytes
			txframe.cdat[0] = 0x00;	      						
			txframe.cdat[1] = 0xDD;	
			txframe.cdat[2] = 0x00;	      					
			txframe.cdat[3] = 0x03;	  
			txframe.cdat[4] = 0x20;	      						
			txframe.cdat[5] = 0xFF;	
			txframe.cdat[6] = 0xFF;	
			txframe.cdat[7] = 0x00;				
			tx_can_frame(0);	*/
			
			
			shortdelay();
			txframe.MsgNum = 11;		  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x3E1;		  //ENG_CFG; 100ms
			txframe.sz = 7;				  //7 bytes
			txframe.cdat[0] = 0x40;	      						
			txframe.cdat[1] = 0x08;	
			txframe.cdat[2] = 0xE0;	      					
			txframe.cdat[3] = 0x14;	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x00;		  
			tx_can_frame(0);
			

			
			shortdelay();
			txframe.MsgNum = 11;		  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x278;		  //ECM_B2; 100ms						
			txframe.sz = 8;				  //8 bytes
			txframe.cdat[0] = 0x00;	      						
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x00;	      					
			txframe.cdat[3] = 0xFF;	  
			txframe.cdat[4] = 0x20;	      						
			txframe.cdat[5] = 0xFF;	
			txframe.cdat[6] = 0x00;	
			txframe.cdat[7] = 0x00;				
			tx_can_frame(0);
			
			shortdelay();
			txframe.MsgNum = 11;		  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x2A0;		  //ECM_INDICATORS; 100ms				
			txframe.sz = 8;				  //8 bytes
			txframe.cdat[0] = 0x04;	      						
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0xFF;	      					
			txframe.cdat[3] = 0xFF;	  
			txframe.cdat[4] = 0xFF;	      						
			txframe.cdat[5] = 0xFF;	
			txframe.cdat[6] = 0xFF;	
			txframe.cdat[7] = 0x03;				
			tx_can_frame(0);
			
			shortdelay();
			txframe.MsgNum = 13;		  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x178;		  //TRNS_STAT2; 100ms								
			txframe.sz = 3;				  //8 bytes
			txframe.cdat[0] = 0x30;	      						
			txframe.cdat[1] = 0x96;	
			txframe.cdat[2] = 0x00;     					 
			tx_can_frame(0);
			
			shortdelay();
			txframe.MsgNum = 11;		  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x1C0;		  //RFHUB_A2; 200ms			
			txframe.sz = 6;				  //7 bytes
			txframe.cdat[0] = 0x00;	      						
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x00;	      					
			txframe.cdat[3] = 0x80;	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;		  
			tx_can_frame(0);

			if (count1 >= 20)
			{
			
				shortdelay();
				txframe.MsgNum = 11;		  //Message number(sequential starting at 1 for unique messages							
				txframe.arbID = 0x304;		  //TPM_A1; 1000ms				
				txframe.sz = 8;				  //8 bytes
				txframe.cdat[0] = 0x00;	      						
				txframe.cdat[1] = 0x00;	
				txframe.cdat[2] = 0x00;	      					
				txframe.cdat[3] = 0x24;	  
				txframe.cdat[4] = 0x24;	      						
				txframe.cdat[5] = 0x24;	
				txframe.cdat[6] = 0x24;	
				txframe.cdat[7] = 0x24;				
				tx_can_frame(0);
			
				shortdelay();
				txframe.MsgNum = 11;		  //Message number(sequential starting at 1 for unique messages							
				txframe.arbID = 0x1D0;		  //ORC_A1; 1000ms				
				txframe.sz = 8;				  //8 bytes
				txframe.cdat[0] = 0x00;	      						
				txframe.cdat[1] = 0x00;	
				txframe.cdat[2] = 0x00;	      					
				txframe.cdat[3] = 0x0C;	  
				txframe.cdat[4] = 0x00;	      						
				txframe.cdat[5] = 0x00;	
				txframe.cdat[6] = 0x00;	
				txframe.cdat[7] = 0x00;				
				tx_can_frame(0);

				shortdelay();
				txframe.MsgNum = 11;		  //Message number(sequential starting at 1 for unique messages							
				txframe.arbID = 0x2E0;		  //ESP_B1; 1000ms				
				txframe.sz = 8;				  //8 bytes
				txframe.cdat[0] = 0xFF;	      						
				txframe.cdat[1] = 0xFF;	
				txframe.cdat[2] = 0xFF;	      					
				txframe.cdat[3] = 0xFF;	  
				txframe.cdat[4] = 0x00;	      						
				txframe.cdat[5] = 0x00;	
				txframe.cdat[6] = 0x00;	
				txframe.cdat[7] = 0x00;				
				tx_can_frame(0);
				
			
				count1 = 0;
			}
			
			else
			{
				count1++;
				
			}
/*				lang_cfg(); ////

				EA = 1; ////
				receive_data(30); //// 
*/
		
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
				codedelay(900);
}

/*
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
*/

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
