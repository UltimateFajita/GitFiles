 //Can Bus Simulator
//See function radtest for transmitting data


#include "c8051F040.h"	
#include "telematics.h"						
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

sbit LED1 = P2^0; 						// led output pin 
sbit VR_BUTTON  	= P1^4;				// sw1 = 0 means switch pressed
sbit PH_BUTTON 		= P1^3;				// sw2 = 0 means switch pressed
sbit IGN_RUN_0  	= P1^5;				// sw3 = 0 means switch pressed
sbit SEL_CAN_B 		= P1^7;				// CAN select pin 0 means can B, 1 means CAN c/i
sbit CAN_B_EN  		= P1^6;				// CAN B enable pin,  0 means enable, 1 means disable
					//Added for 6 position switch
sbit IGN_RUN_5		= P0^6;
sbit IGN_RUN_10 	= P0^5;				// sw4 = 0 means switch pressed
sbit IGN_START		= P1^0;
sbit IGN_OFF 		= P0^7;				// sw5 = 0 means switch pressed
sbit IGN_ACC 		= P1^1;				// sw6 = 0 means switch pressed
//sbit temp			= P3^4;


#define SYSCLK       24500000          // System Clock
#define SAR_CLK      2500000           // Desired SAR clock speed
#define numb 		 9

struct txframe
{
	unsigned char MsgNum;
	unsigned char sz;
	unsigned long arbID;
 	unsigned char cdat[8];

}txframe;
CANFRAME xdata rxframe[32];

byte xdata IntReg;

long ResultL, ResultH, Result, sw_inc;                           // ADC0 decimated value
char MsgNum;
char status;
int tmp;
int i;
int v,sw,sw_temp;
int j = 0;
int h;
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
void IGN_MSG(void);
void check_lang(void);
void lang_cfg(void);
void init_msg_object_rx (unsigned char MsgNum,unsigned int arbID);
int ADC0_Analog_sel(void);
void ADC0_Init (void);
void SW_dim_set(int swi);



/*******************************************************
** Function Name: Main()							  **
** Varibles: NONE									  **
** Output Type:										  **
**													  **
**													  **
** Description:	The main function					  **
**													  **
**													  **
**													  **
** 												 	  **
** Date Modified:								 	  **
** Author:											  **
*******************************************************/
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



	init_msg_object_rx(30, 0x314); //TODO:change recieve address
	rxframe[1].MsgNum = 0;

  //Loop and wait for interrupts
  while (1)
    {
		
		IGN_MSG();
		if (IGN_OFF != 0)
		{
			radiotest();
			// poll for current switch number
			sw = ADC0_Analog_sel();	
			SW_dim_set(sw);
			iii = 0;
		}
		

		
		
	}	    
}






/*******************************************************
** Function name: tx_can_frame()					  **
** Varibles: unsigned char KWPopts -- 			      **
**			 										  **
**													  **
** Output type: Byte								  **
**													  **
**													  **
** Description:										  **
**													  **
**													  **
**													  **
** 												 	  **
** Date Modified:								 	  **
** Author:											  **
*******************************************************/
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




/*******************************************************
** Function name: config_CAN_timing()				  **
** Varibles: NONE									  **
** Output type:										  **
**													  **
**													  **
** Description:										  **
**													  **
**													  **
**													  **
** 												 	  **
** Date Modified:								 	  **
** Author:											  **
*******************************************************/
void config_CAN_timing(void)
{
	SFRPAGE = CAN0_PAGE;
	CAN0CN |= 0x41;							// Set CCE bit to enable write access
	CAN0ADR = BITREG;						// Point to the Bit Timing Register

	CAN0DATH = 0x5A;					// 18.0mhz BRP=7 125kbps
	CAN0DATL = 0xC7;					// Tseg2=6, Tseg1=11, SJW=4
}



/*******************************************************
** Function name: config_IO()						  **
** Varibles: NONE									  **
** Output type:										  **
**													  **
**													  **
** Description:										  **
**													  **
**													  **
**													  **
** 												 	  **
** Date Modified:								 	  **
** Author:											  **
*******************************************************/
void config_IO(void)
{
  

char SFRPAGE_SAVE = SFRPAGE;

		SFRPAGE = CONFIG_PAGE;
		
		
		P0MDOUT  = 0x03;			//pin P0^0 & P0^1 are push pull
		P2MDIN 	|= 0x01;			//pin P2^0 is digital
		P2MDOUT  = 0x01;	 		//pin P2^0 is push pull	
		P1MDOUT  = 0x00;			//Port 1 is open drain
		P1MDIN	 = 0xff; 
		
		P3MDIN	|= 0x00;
		P3MDOUT  = 0x00;

			
		XBR2	 = 0x40;			// enable crossbar and disable weak pullups
		XBR3     = 0x80;     // Configure CAN TX pin (CTX) as push-pull digital output		
		SFRPAGE = SFRPAGE_SAVE;
		//temp |= 1;

}


/*******************************************************
** Function name: receive_data()					  **
** Varibles: char MsgNum --							  **
** Output type:										  **
**													  **
**													  **
** Description:										  **
**													  **
**													  **
**													  **
** 												 	  **
** Date Modified:								 	  **
** Author:											  **
*******************************************************/
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



/*******************************************************
** Function name: ISRname()							  **
** Varibles: NONE									  **
** Output type:										  **
**													  **
**													  **
** Description:										  **
**													  **
**													  **
**													  **
** 												 	  **
** Date Modified:								 	  **
** Author:											  **
*******************************************************/
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



/*******************************************************
** Function name: clear_msg_objects()				  **
** Varibles: NONE									  **
** Output type:										  **
**													  **
**													  **
** Description:										  **
**													  **
**													  **
**													  **
** 												 	  **
** Date Modified:								 	  **
** Author:											  **
*******************************************************/
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



/*******************************************************
** Function name: ING_MSG()							  **
** Varibles: NONE									  **
** Output type:										  **
**													  **
**													  **
** Description:										  **
**													  **
**													  **
**													  **
** 												 	  **
** Date Modified:								 	  **
** Author:											  **
*******************************************************/
void IGN_MSG (void)
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
				tx_can_frame(0);			  	//transmits can frame
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
			tx_can_frame(0);			  	//transmits can frame

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
			tx_can_frame(0);
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
			tx_can_frame(0);			  	//transmits can frame

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
			tx_can_frame(0);
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
			tx_can_frame(0);			  	//transmits can frame

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
			tx_can_frame(0);
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
			tx_can_frame(0);			  	//transmits can frame

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
			tx_can_frame(0);
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
			tx_can_frame(0);			  	//transmits can frame

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
			tx_can_frame(0);
	}
}



/*******************************************************
** Function name: radiotest()						  **
** Varibles: NONE									  **
** Output type:										  **
**													  **
**													  **
** Description:										  **
**													  **
**													  **
**													  **
** 												 	  **
** Date Modified:								 	  **
** Author:											  **
*******************************************************/
void radiotest(void)

{	
		//VIN Entry -------------------------------------------------------------------------------

		char VIN[17] = "1C3CCCCB7FN1130UC";

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
			tx_can_frame(0); 
			
			shortdelay();
			txframe.MsgNum = 2;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x3E3;		  //Message ID CBC_12  									
			txframe.sz = 8;				  //8 bytes
			txframe.cdat[0] = 0xFD;	      						
			txframe.cdat[1] = 0x0C;	
			txframe.cdat[2] = 0x10;	      					
			txframe.cdat[3] = 0x08;	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0x00;	      // Change to 0x02 for USB and AUX, 0x03 for USB/SD/AUX	  
			tx_can_frame(0);
			
			shortdelay();
			txframe.MsgNum = 2;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x3E4;		  //Message ID CBC_12  									
			txframe.sz = 8;				  //8 bytes
			txframe.cdat[0] = 0x89;	      						
			txframe.cdat[1] = 0xDA;	
			txframe.cdat[2] = 0x98;	      					
			txframe.cdat[3] = 0x06;	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0x00;	      // Change to 0x02 for USB and AUX, 0x03 for USB/SD/AUX	  
			tx_can_frame(0);
			
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
			tx_can_frame(0);

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
			tx_can_frame(0);

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
			tx_can_frame(0);
		
		
			shortdelay();
			txframe.MsgNum = 10;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x3E8;		  //Message ID CBC_12  									
			txframe.sz = 8;				  //8 bytes
			txframe.cdat[0] = 0x41;	      						
			txframe.cdat[1] = 0x29;	
			txframe.cdat[2] = 0xA2;	      					
			txframe.cdat[3] = 0x15;	  
			txframe.cdat[4] = 0x1E;	      						
			txframe.cdat[5] = 0x80;	
			txframe.cdat[6] = 0xA2;
			txframe.cdat[7] = 0x10;		  
			tx_can_frame(0);
			
			shortdelay();
			txframe.MsgNum = 2;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x3E9;		  //Message ID CBC_12  									
			txframe.sz = 8;				  //8 bytes
			txframe.cdat[0] = 0x01;	      						
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x06;	      					
			txframe.cdat[3] = 0x9E;	  
			txframe.cdat[4] = 0x20;	      						
			txframe.cdat[5] = 0x20;	
			txframe.cdat[6] = 0x20;
			txframe.cdat[7] = 0x20;	      // Change to 0x02 for USB and AUX, 0x03 for USB/SD/AUX	  
			tx_can_frame(0);
			
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
			tx_can_frame(0);
		
			shortdelay();
			txframe.MsgNum = 11;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x3EA;		  //Message ID CBC_12  									
			txframe.sz = 8;				  //8 bytes
			txframe.cdat[0] = 0x41;	      						
			txframe.cdat[1] = 0x0D;	
			txframe.cdat[2] = 0x22;	      					
			txframe.cdat[3] = 0x48;	  
			txframe.cdat[4] = 0x80;	      						
			txframe.cdat[5] = 0x24;	
			txframe.cdat[6] = 0x03;
			txframe.cdat[7] = 0x40;		  
			tx_can_frame(0);
			
			shortdelay();
			txframe.MsgNum = 2;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x3EB;		  //Message ID CBC_12  									
			txframe.sz = 8;				  //8 bytes
			txframe.cdat[0] = 0x40;	      						
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x00;	      					
			txframe.cdat[3] = 0x00;	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x08;
			txframe.cdat[7] = 0xC6;	      // Change to 0x02 for USB and AUX, 0x03 for USB/SD/AUX	  
			tx_can_frame(0);
			
			shortdelay();
			txframe.MsgNum = 2;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x44A;		  //Message ID CBC_12  									
			txframe.sz = 8;				  //8 bytes
			txframe.cdat[0] = 0x01;	      						
			txframe.cdat[1] = 0;	
			txframe.cdat[2] = 0;	      					
			txframe.cdat[3] = 0;	  
			txframe.cdat[4] = 0;	      						
			txframe.cdat[5] = 0;	
			txframe.cdat[6] = 0;
			txframe.cdat[7] = 0x20;	      // Change to 0x02 for USB and AUX, 0x03 for USB/SD/AUX	  
			tx_can_frame(0);
			
			shortdelay();
			txframe.MsgNum = 2;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x44C;		  //Message ID CBC_12  									
			txframe.sz = 8;				  //8 bytes
			txframe.cdat[0] = 0x01;	      						
			txframe.cdat[1] = 0;	
			txframe.cdat[2] = 0;	      					
			txframe.cdat[3] = 0;	  
			txframe.cdat[4] = 0;	      						
			txframe.cdat[5] = 0;	
			txframe.cdat[6] = 0;
			txframe.cdat[7] = 0;	      // Change to 0x02 for USB and AUX, 0x03 for USB/SD/AUX	  
			tx_can_frame(0);
			
			shortdelay();
			txframe.MsgNum = 2;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x381;		  //Message ID CBC_12  									
			txframe.sz = 8;				  //8 bytes
			txframe.cdat[0] = 0x01;	      						
			txframe.cdat[1] = 0;	
			txframe.cdat[2] = 0;	      					
			txframe.cdat[3] = 0;	  
			txframe.cdat[4] = 0;	      						
			txframe.cdat[5] = 0;	
			txframe.cdat[6] = 0;
			txframe.cdat[7] = 0;	      // Change to 0x02 for USB and AUX, 0x03 for USB/SD/AUX	  
			tx_can_frame(0);
			
			shortdelay();
			txframe.MsgNum = 2;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x3B3;		  //Message ID CBC_12  									
			txframe.sz = 8;				  //8 bytes
			txframe.cdat[0] = 0x45;	      						
			txframe.cdat[1] = 0x30;	
			txframe.cdat[2] = 0x22;	      					
			txframe.cdat[3] = 0x01;	  
			txframe.cdat[4] = 0x61;	      						
			txframe.cdat[5] = 0xA7;	
			txframe.cdat[6] = 0xC7;
			txframe.cdat[7] = 0x98;	      // Change to 0x02 for USB and AUX, 0x03 for USB/SD/AUX	  
			tx_can_frame(0);
			
			shortdelay();
			txframe.MsgNum = 2;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x3B4;		  //Message ID CBC_12  									
			txframe.sz = 8;				  //8 bytes
			txframe.cdat[0] = 0xFD;	      						
			txframe.cdat[1] = 0x80;	
			txframe.cdat[2] = 0;	      					
			txframe.cdat[3] = 0;	  
			txframe.cdat[4] = 0;	      						
			txframe.cdat[5] = 0;	
			txframe.cdat[6] = 0;
			txframe.cdat[7] = 0;	      // Change to 0x02 for USB and AUX, 0x03 for USB/SD/AUX	  
			tx_can_frame(0);
			
			shortdelay();
			txframe.MsgNum = 15;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x22F;		  //Message ID CBC_12  									
			txframe.sz = 1;				  //8 bytes
			txframe.cdat[0] = 0x01;	      								  
			tx_can_frame(0);

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
							tx_can_frame(0);

						
	  			
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
							tx_can_frame(0);
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
							tx_can_frame(0);
					
					}

		//lang_cfg();	

		EA = 1;			//enable interupt
		//receive_data(30);
		
}


/*******************************************************
** Function name: SW_dim_set()						  **
** Varibles: NONE									  **
** Output type:										  **
**													  **
**													  **
** Description:										  **
**													  **
**													  **
**													  **
** 												 	  **
** Date Modified:								 	  **
** Author:											  **
*******************************************************/
void SW_dim_set(int swi)
{
// 					shortdelay();
// 					txframe.MsgNum = 10;			  //Message number(sequential starting at 1 for unique messages							
// 					txframe.arbID = 0x2fa;		  //Message ID CBC_12  									
// 					txframe.sz = 8;				  //8 bytes
// 					txframe.cdat[0] = 0x00;	      						
// 					txframe.cdat[1] = 0x00;	
// 					txframe.cdat[2] = 0x00;	      					
// 					txframe.cdat[3] = 0x00;	  
// 					txframe.cdat[4] = 0x00;	      						
// 					txframe.cdat[5] = 0x00;	
// 					txframe.cdat[6] = (swi << 8);
// 					txframe.cdat[7] = 0x00;		  
// 					tx_can_frame(0);
		int l;
	
		switch(swi){
			case 1:
					//
					shortdelay();
					txframe.MsgNum = 10;			  //Message number(sequential starting at 1 for unique messages							
					txframe.arbID = 0x2fa;		  //Message ID CBC_12  									
					txframe.sz = 8;				  //8 bytes
					txframe.cdat[0] = 0x00;	      						
					txframe.cdat[1] = 0x01;	
					txframe.cdat[2] = 0xC8;	      					
					txframe.cdat[3] = 0x0C;	  
					txframe.cdat[4] = 0x00;	      						
					txframe.cdat[5] = 0x00;	
					txframe.cdat[6] = 0xC8;
					txframe.cdat[7] = 0x00;		  
					tx_can_frame(0);
			break;
			case 2:
					shortdelay();
					txframe.MsgNum = 10;			  //Message number(sequential starting at 1 for unique messages							
					txframe.arbID = 0x2fa;		  //Message ID CBC_12  									
					txframe.sz = 8;				  //8 bytes
					txframe.cdat[0] = 0x00;	      						
					txframe.cdat[1] = 0x01;	
					txframe.cdat[2] = 0xC8;	      					
					txframe.cdat[3] = 0x0C;	  
					txframe.cdat[4] = 0x00;	      						
					txframe.cdat[5] = 0x00;	
					txframe.cdat[6] = 0xC8;
					txframe.cdat[7] = 0x00;		  
					tx_can_frame(0);				
			break;
			case 3:
					//If(low to high) 
					if(sw_temp > sw)
					{
						//do sweep from 0x19 to 0x1C (0x19,0x1A,0x1B,0x1C)
						for(l=0;l<=3;l++)
						{
								shortdelay();
								txframe.MsgNum = 10;			  //Message number(sequential starting at 1 for unique messages							
								txframe.arbID = 0x2fa;		  //Message ID CBC_12  									
								txframe.sz = 8;				  //8 bytes
								txframe.cdat[0] = 0x00;	      						
								txframe.cdat[1] = 0x00;	
								txframe.cdat[2] = (0x68 + (l*5));	      					
								txframe.cdat[3] = 0x0C;	  
								txframe.cdat[4] = 0x00;	      						
								txframe.cdat[5] = 0x00;	
								txframe.cdat[6] = (0x68 + (l*5));
								txframe.cdat[7] = 0x00;		  
								tx_can_frame(0);
								onehundredms();

						}
					}


					shortdelay();
					txframe.MsgNum = 10;			  //Message number(sequential starting at 1 for unique messages							
					txframe.arbID = 0x2fa;		  //Message ID CBC_12  									
					txframe.sz = 8;				  //8 bytes
					txframe.cdat[0] = 0x00;	      						
					txframe.cdat[1] = 0x00;	
					txframe.cdat[2] = 0x78;	      					
					txframe.cdat[3] = 0x0C;	  
					txframe.cdat[4] = 0x00;	      						
					txframe.cdat[5] = 0x00;	
					txframe.cdat[6] = 0x78;
					txframe.cdat[7] = 0x00;		  
					tx_can_frame(0);				
			break;
			case 4:

					//If(low to high) 
					if(sw_temp > sw)
					{
						//do sweep from 0x14 to 0x17 (0x14,0x15,0x16,0x17)
						for(l=0;l<=3;l++)
						{
								shortdelay();
								txframe.MsgNum = 10;			  //Message number(sequential starting at 1 for unique messages							
								txframe.arbID = 0x2fa;		  //Message ID CBC_12  									
								txframe.sz = 8;				  //8 bytes
								txframe.cdat[0] = 0x00;	      						
								txframe.cdat[1] = 0x00;	
								txframe.cdat[2] = (0x54 + (l*4));	      					
								txframe.cdat[3] = 0x0C;	  
								txframe.cdat[4] = 0x00;	      						
								txframe.cdat[5] = 0x00;	
								txframe.cdat[6] = (0x54 + (l*4));
								txframe.cdat[7] = 0x00;		  
								tx_can_frame(0);
								onehundredms();

						}
					}

					//if(high to low)
					else if(sw_temp < sw)
					{
						//do sweep from 0x1B to 0x19 (0x1B, 0x1A, 0x19)
						for(l=0;l<=2;l++)
						{
								shortdelay();
								txframe.MsgNum = 10;			  //Message number(sequential starting at 1 for unique messages							
								txframe.arbID = 0x2fa;		  //Message ID CBC_12  									
								txframe.sz = 8;				  //8 bytes
								txframe.cdat[0] = 0x00;	      						
								txframe.cdat[1] = 0x00;	
								txframe.cdat[2] = (0x73 - (l*5));	      					
								txframe.cdat[3] = 0x0C;	  
								txframe.cdat[4] = 0x00;	      						
								txframe.cdat[5] = 0x00;	
								txframe.cdat[6] = (0x73 - (l*5));
								txframe.cdat[7] = 0x00;		  
								tx_can_frame(0);
								onehundredms();

						}
					}

					shortdelay();
					txframe.MsgNum = 10;			  //Message number(sequential starting at 1 for unique messages							
					txframe.arbID = 0x2fa;		  //Message ID CBC_12  									
					txframe.sz = 8;				  //8 bytes
					txframe.cdat[0] = 0x00;	      						
					txframe.cdat[1] = 0x00;	
					txframe.cdat[2] = 0x63;	      					
					txframe.cdat[3] = 0x0C;	  
					txframe.cdat[4] = 0x00;	      						
					txframe.cdat[5] = 0x00;	
					txframe.cdat[6] = 0x63;
					txframe.cdat[7] = 0x00;		  
					tx_can_frame(0);				
			break;
			case 5:

					//If(low to high) 
					if(sw_temp > sw)
					{
						//do sweep from 0x0F to 0x12 (0x0F,0x10,0x11,0x12)

						for(l=0;l<=3;l++)
						{
								shortdelay();
								txframe.MsgNum = 10;			  //Message number(sequential starting at 1 for unique messages							
								txframe.arbID = 0x2fa;		  //Message ID CBC_12  									
								txframe.sz = 8;				  //8 bytes
								txframe.cdat[0] = 0x00;	      						
								txframe.cdat[1] = 0x00;	
								txframe.cdat[2] = (0x40 + (l*5));	      					
								txframe.cdat[3] = 0x0C;	  
								txframe.cdat[4] = 0x00;	      						
								txframe.cdat[5] = 0x00;	
								txframe.cdat[6] = (0x40 + (l*5));
								txframe.cdat[7] = 0x00;		  
								tx_can_frame(0);
								onehundredms();

						}
					}

					//if(high to low)
					else if(sw_temp < sw)
					{
						//do sweep from 0x17 to 0x14 (0x17, 0x16, 0x15, 0x14)
						for(l=0;l<=3;l++)
						{
								shortdelay();
								txframe.MsgNum = 10;			  //Message number(sequential starting at 1 for unique messages							
								txframe.arbID = 0x2fa;		  //Message ID CBC_12  									
								txframe.sz = 8;				  //8 bytes
								txframe.cdat[0] = 0x00;	      						
								txframe.cdat[1] = 0x00;	
								txframe.cdat[2] = (0x5F - (l*4));	      					
								txframe.cdat[3] = 0x0C;	  
								txframe.cdat[4] = 0x00;	      						
								txframe.cdat[5] = 0x00;	
								txframe.cdat[6] = (0x5F - (l*4));
								txframe.cdat[7] = 0x00;		  
								tx_can_frame(0);
								onehundredms();
						}
					}

					shortdelay();
					txframe.MsgNum = 10;			  //Message number(sequential starting at 1 for unique messages							
					txframe.arbID = 0x2fa;		  //Message ID CBC_12  									
					txframe.sz = 8;				  //8 bytes
					txframe.cdat[0] = 0x00;	      						
					txframe.cdat[1] = 0x00;	
					txframe.cdat[2] = 0x50;	      					
					txframe.cdat[3] = 0x0C;	  
					txframe.cdat[4] = 0x00;	      						
					txframe.cdat[5] = 0x00;	
					txframe.cdat[6] = 0x50;
					txframe.cdat[7] = 0x00;		  
					tx_can_frame(0);				
			break;
			case 6:
					//If(low to high) 
					if(sw_temp > sw)
					{
						//do sweep from 0x0B to 0x0D (0x0B,0x0C,0x0D)
						for(l=0;l<=2;l++)
						{
								shortdelay();
								txframe.MsgNum = 10;			  //Message number(sequential starting at 1 for unique messages							
								txframe.arbID = 0x2fa;		  //Message ID CBC_12  									
								txframe.sz = 8;				  //8 bytes
								txframe.cdat[0] = 0x00;	      						
								txframe.cdat[1] = 0x00;	
								txframe.cdat[2] = (0x2E + (l*6));	      					
								txframe.cdat[3] = 0x0C;	  
								txframe.cdat[4] = 0x00;	      						
								txframe.cdat[5] = 0x00;	
								txframe.cdat[6] = (0x2E + (l*6));
								txframe.cdat[7] = 0x00;		  
								tx_can_frame(0);
								onehundredms();
						}
					}

					//if(high to low)
					else if(sw_temp < sw)
					{
						//do sweep from 0x12 to 0x0F (0x12, 0x11, 0x10, 0x0F)
						for(l=0;l<=3;l++)
						{
								shortdelay();
								txframe.MsgNum = 10;			  //Message number(sequential starting at 1 for unique messages							
								txframe.arbID = 0x2fa;		  //Message ID CBC_12  									
								txframe.sz = 8;				  //8 bytes
								txframe.cdat[0] = 0x00;	      						
								txframe.cdat[1] = 0x00;	
								txframe.cdat[2] = (0x4B - (l*5));	      					
								txframe.cdat[3] = 0x0C;	  
								txframe.cdat[4] = 0x00;	      						
								txframe.cdat[5] = 0x00;	
								txframe.cdat[6] = (0x4B - (l*5));
								txframe.cdat[7] = 0x00;		  
								tx_can_frame(0);
								onehundredms();
						}
					}

					shortdelay();
					txframe.MsgNum = 10;			  //Message number(sequential starting at 1 for unique messages							
					txframe.arbID = 0x2fa;		  //Message ID CBC_12  									
					txframe.sz = 8;				  //8 bytes
					txframe.cdat[0] = 0x00;	      						
					txframe.cdat[1] = 0x00;	
					txframe.cdat[2] = 0x3B;	      					
					txframe.cdat[3] = 0x0C;	  
					txframe.cdat[4] = 0x00;	      						
					txframe.cdat[5] = 0x00;	
					txframe.cdat[6] = 0x3B;
					txframe.cdat[7] = 0x00;		  
					tx_can_frame(0);				
			break;
			case 7:
					//If(low to high) 
					if(sw_temp > sw)
					{
						//do sweep from 0x06 to 0x09 (0x06,0x07,0x08, 0x09)
						for(l=0;l<=3;l++)
						{
								shortdelay();
								txframe.MsgNum = 10;			  //Message number(sequential starting at 1 for unique messages							
								txframe.arbID = 0x2fa;		  //Message ID CBC_12  									
								txframe.sz = 8;				  //8 bytes
								txframe.cdat[0] = 0x00;	      						
								txframe.cdat[1] = 0x00;	
								txframe.cdat[2] = (0x18 + l*5);	      					
								txframe.cdat[3] = 0x0C;	  
								txframe.cdat[4] = 0x00;	      						
								txframe.cdat[5] = 0x00;	
								txframe.cdat[6] = (0x18 + l*5);
								txframe.cdat[7] = 0x00;		  
								tx_can_frame(0);
								onehundredms();

						}
					}
					//if(high to low)
					else if(sw_temp < sw)
					{
						//do sweep from 0x0D to 0x0B (0x0D, 0x0C, 0x0B)
						for(l=0;l<=2;l++)
						{
								shortdelay();
								txframe.MsgNum = 10;			  //Message number(sequential starting at 1 for unique messages							
								txframe.arbID = 0x2fa;		  //Message ID CBC_12  									
								txframe.sz = 8;				  //8 bytes
								txframe.cdat[0] = 0x00;	      						
								txframe.cdat[1] = 0x00;	
								txframe.cdat[2] = (0x35 - (l*6));	      					
								txframe.cdat[3] = 0x0C;	  
								txframe.cdat[4] = 0x00;	      						
								txframe.cdat[5] = 0x00;	
								txframe.cdat[6] = (0x35 - (l*6));
								txframe.cdat[7] = 0x00;		  
								tx_can_frame(0);
								onehundredms();

						}
					}

					shortdelay();
					txframe.MsgNum = 10;			  //Message number(sequential starting at 1 for unique messages							
					txframe.arbID = 0x2fa;		  //Message ID CBC_12  									
					txframe.sz = 8;				  //8 bytes
					txframe.cdat[0] = 0x00;	      						
					txframe.cdat[1] = 0x00;	
					txframe.cdat[2] = 0x28;	      					
					txframe.cdat[3] = 0x0C;	  
					txframe.cdat[4] = 0x00;	      						
					txframe.cdat[5] = 0x00;	
					txframe.cdat[6] = 0x28;
					txframe.cdat[7] = 0x00;		  
					tx_can_frame(0);				
			break;
			case 8:
					
					//if(high to low)
					if(sw_temp < sw)
					{
						//do sweep from 0x09 to 0x06 (0x09, 0x08, 0x07, 0x06)
						for(l=0;l<=3;l++)
						{
								shortdelay();
								txframe.MsgNum = 10;			  //Message number(sequential starting at 1 for unique messages							
								txframe.arbID = 0x2fa;		  //Message ID CBC_12  									
								txframe.sz = 8;				  //8 bytes
								txframe.cdat[0] = 0x00;	      						
								txframe.cdat[1] = 0x00;	
								txframe.cdat[2] = (0x23 - (l*5));	      					
								txframe.cdat[3] = 0x0C;	  
								txframe.cdat[4] = 0x00;	      						
								txframe.cdat[5] = 0x00;	
								txframe.cdat[6] = (0x23 - (l*5));
								txframe.cdat[7] = 0x00;		  
								tx_can_frame(0);
								onehundredms();
						}
					}

					shortdelay();
					txframe.MsgNum = 10;			  //Message number(sequential starting at 1 for unique messages							
					txframe.arbID = 0x2fa;		  //Message ID CBC_12  									
					txframe.sz = 8;				  //8 bytes
					txframe.cdat[0] = 0x00;	      						
					txframe.cdat[1] = 0x00;	
					txframe.cdat[2] = 0x13;	      					
					txframe.cdat[3] = 0x0C;	  
					txframe.cdat[4] = 0x00;	      						
					txframe.cdat[5] = 0x00;	
					txframe.cdat[6] = 0x13;
					txframe.cdat[7] = 0x00;		  
					tx_can_frame(0);				
			break;
			default:
					shortdelay();
					txframe.MsgNum = 10;			  //Message number(sequential starting at 1 for unique messages							
					txframe.arbID = 0x2fa;		  //Message ID CBC_12  									
					txframe.sz = 8;				  //8 bytes
					txframe.cdat[0] = 0x00;	      						
					txframe.cdat[1] = 0x04;	
					txframe.cdat[2] = 0x00;	      					
					txframe.cdat[3] = 0x0C;	  
					txframe.cdat[4] = 0x00;	      						
					txframe.cdat[5] = 0x00;	
					txframe.cdat[6] = 0x00;
					txframe.cdat[7] = 0x00;		  
					tx_can_frame(0);				
			}

			sw_temp = sw;
			
			

}


/*******************************************************
** Function name: codedelay()						  **
** Varibles: NONE									  **
** Output type:										  **
**													  **
**													  **
** Description:										  **
**													  **
**													  **
**													  **
** 												 	  **
** Date Modified:								 	  **
** Author:											  **
*******************************************************/
void codedelay(unsigned int delay)	//unsigned in
{
	while (delay)
	delay--;
} 



/*******************************************************
** Function name: onehundredms()					  **
** Varibles: NONE									  **
** Output type:										  **
**													  **
**													  **
** Description:										  **
**													  **
**													  **
**													  **
** 												 	  **
** Date Modified:								 	  **
** Author:											  **
*******************************************************/
void onehundredms(void)
{

	
				codedelay(255000);
				codedelay(255000);
				codedelay(245000);
				codedelay(100062);
}



/*******************************************************
** Function name: shortdelay()						  **
** Varibles: NONE									  **
** Output type:										  **
**													  **
**													  **
** Description:										  **
**													  **
**													  **
**													  **
** 												 	  **
** Date Modified:								 	  **
** Author:											  **
*******************************************************/
void shortdelay(void)
{
				codedelay(200000);
}



/*******************************************************
** Function name: clear_message_object ()			  **
** Varibles: NONE									  **
** Output type:										  **
**													  **
**													  **
** Description:										  **
**													  **
**													  **
**													  **
** 												 	  **
** Date Modified:								 	  **
** Author:											  **
*******************************************************/
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


/*******************************************************
** Function name: init_msg_object_rx ()				  **
** Varibles: unsigned char MsgNum --				  **
**			 unsigned int arbID	--					  **
**													  **
** Output type:										  **
**													  **
**													  **
** Description:										  **
**													  **
**													  **
**													  **
** 												 	  **
** Date Modified:								 	  **
** Author:											  **
*******************************************************/
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


/*******************************************************
** Function name: check_lang()						  **
** Varibles: NONE									  **
** Output type:										  **
**													  **
**													  **
** Description:										  **
**													  **
**													  **
**													  **
** 												 	  **
** Date Modified:								 	  **
** Author:											  **
*******************************************************/
void check_lang(void)
{		

	
	//This row of if statements checks to see if the data conatins 0A 03 04 which is sent by the lang = ENG request
	if((rxframe[1].cdat[0] == 0x0A) && (rxframe[1].cdat[1] == 0x03) && (rxframe[1].cdat[2] == 0x01))
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

	//This row of if statements checks to see if the data conatins 0A 03 01 which is sent by the lang = SPA request
	if((rxframe[1].cdat[0] == 0x0A) && (rxframe[1].cdat[1] == 0x03) && (rxframe[1].cdat[2] == 0x09))
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
	if((rxframe[1].cdat[0] == 0x0C)) //&& (rxframe[1].cdat[1] == 0x03) && (rxframe[1].cdat[2] == 0x00))
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
	

	if((rxframe[1].cdat[0] == 0x0E)) //&& (rxframe[1].cdat[1] == 0x03) && (rxframe[1].cdat[2] == 0x02))
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
	

	if((rxframe[1].cdat[0] == 0x0A) && (rxframe[1].cdat[1] == 0x03) && (rxframe[1].cdat[2] == 0x09))
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


/*******************************************************
** Function name: lang_cfg()						  **
** Varibles: NONE									  **
** Output type:										  **
**													  **
**													  **
** Description:										  **
**													  **
**													  **
**													  **
** 												 	  **
** Date Modified:								 	  **
** Author:											  **
*******************************************************/
void lang_cfg(void)
{
		
		if (flag_eng == 1)			// Set language configuration based on check_lang() function ---- English
		{
		shortdelay();
		txframe.MsgNum = 12;			  //Message number(sequential starting at 1 for unique messages							
		txframe.arbID= 0x332;		  //Message ID VR off									
		txframe.sz = 8;				  //8 bytes
		txframe.cdat[0] = 0x00;	      						
		txframe.cdat[1] = 0x00;	
		txframe.cdat[2] = 0x00;	     					
		txframe.cdat[3] = 0x00;	  
		txframe.cdat[4] = 0x00;	      						
		txframe.cdat[5] = 0x08;	
		txframe.cdat[6] = 0x00;
		txframe.cdat[7] = 0x00;		  
		tx_can_frame(0);
		}
		
		/*
		if (flag_spa == 1)			// Set language configuration based on check_lang() function ---- Spanish
		{
		shortdelay();
		txframe.MsgNum = 12;			  //Message number(sequential starting at 1 for unique messages							
		txframe.arbID= 0x332;		  //Message ID VR off									
		txframe.sz = 8;				  //8 bytes
		txframe.cdat[0] = 0x00;	      						
		txframe.cdat[1] = 0x00;	
		txframe.cdat[2] = 0x00;	  // lever in Park    					
		txframe.cdat[3] = 0x00;	  
		txframe.cdat[4] = 0x00;	      						
		txframe.cdat[5] = 0x00;	
		txframe.cdat[6] = 0x00;
		txframe.cdat[7] = 0x00;		  
		tx_can_frame(0);
		}

		
		if (flag_ger == 1)			// Set language configuration based on check_lang() function ---- Spanish
		{
		shortdelay();
		txframe.MsgNum = 12;			  //Message number(sequential starting at 1 for unique messages							
		txframe.arbID= 0x332;		  //Message ID VR off									
		txframe.sz = 8;				  //8 bytes
		txframe.cdat[0] = 0x00;	      						
		txframe.cdat[1] = 0x00;	
		txframe.cdat[2] = 0x00;	  // lever in Park    					
		txframe.cdat[3] = 0x00;	  
		txframe.cdat[4] = 0x00;	      						
		txframe.cdat[5] = 0x00;	
		txframe.cdat[6] = 0x00;
		txframe.cdat[7] = 0x00;		  
		tx_can_frame(0);
		}
		

		if (flag_fre == 1)			// Set language configuration based on check_lang() function ---- French
		{
		shortdelay();
		txframe.MsgNum = 12;			  //Message number(sequential starting at 1 for unique messages							
		txframe.arbID= 0x332;		  //Message ID VR off									
		txframe.sz = 8;				  //8 bytes
		txframe.cdat[0] = 0x00;	      						
		txframe.cdat[1] = 0x00;	
		txframe.cdat[2] = 0x00;	  // lever in Park    					
		txframe.cdat[3] = 0x00;	  
		txframe.cdat[4] = 0x00;	      						
		txframe.cdat[5] = 0x00;	
		txframe.cdat[6] = 0x00;
		txframe.cdat[7] = 0x00;		  
		tx_can_frame(0);
		}

		
		if (flag_ita == 1)			// Set language configuration based on check_lang() function ---- English
		{
		shortdelay();
		txframe.MsgNum = 12;			  //Message number(sequential starting at 1 for unique messages							
		txframe.arbID= 0x332;		  //Message ID VR off									
		txframe.sz = 8;				  //8 bytes
		txframe.cdat[0] = 0x00;	      						
		txframe.cdat[1] = 0x00;	
		txframe.cdat[2] = 0x00;	     					
		txframe.cdat[3] = 0x00;	  
		txframe.cdat[4] = 0x00;	      						
		txframe.cdat[5] = 0x00;	
		txframe.cdat[6] = 0x60;
		txframe.cdat[7] = 0x00;		  
		tx_can_frame(0);
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

		if (flag_chs == 1)			// Set language configuration based on check_lang() function ---- Chinese
		{
		shortdelay();
		txframe.MsgNum = 12;			  //Message number(sequential starting at 1 for unique messages							
		txframe.arbID= 0x332;		  //Message ID VR off									
		txframe.sz = 8;				  //8 bytes
		txframe.cdat[0] = 0x00;	      						
		txframe.cdat[1] = 0x00;	
		txframe.cdat[2] = 0x00;	  // lever in Park    					
		txframe.cdat[3] = 0x00;	  
		txframe.cdat[4] = 0x00;	      						
		txframe.cdat[5] = 0x48;	
		txframe.cdat[6] = 0x00;
		txframe.cdat[7] = 0x00;		  
		tx_can_frame(0);
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
    AMX0PRT = 0x10;		
				   				  

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
	
	//Result = 200*(Result/1023)
	

	//check where current polling switch position is at compared to 
	//the switch increments and total # of switch positions
	for(k=1; k <= numb; k++)
	{
		if(Result<((k*sw_inc)+(sw_inc/4)))
		{
			return k;
		}
	}

	return numb;
}