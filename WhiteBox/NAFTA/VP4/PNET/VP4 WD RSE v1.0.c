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
sbit PHONE_END		= P1^2;
sbit INGITION_SW  	= P1^5;				// sw3 = 0 means switch pressed
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

char MsgNum;
char status;
int i;
int v;
int j = 0;
int h;



void config_CAN_timing(void);
void config_IO(void);
void receive_data(char MsgNum);
void clear_msg_objects(void);
void radiotest(void);
void codedelay(unsigned int delay);
void clear_message_object (unsigned char mesgobj,unsigned char count);
void onehundredms(void);
void shortdelay(void);


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

  //Loop and wait for interrupts
  while (1)
    {
		radiotest();
		
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

	CAN0DATH = 0x5A;						// 18.0mhz BRP=7 125kbps
	CAN0DATL = 0xC7;						// Tseg2=6, Tseg1=11, SJW=4
}

void config_IO(void)
{
  

char SFRPAGE_SAVE = SFRPAGE;

		SFRPAGE = CONFIG_PAGE;
		
		
		P0MDOUT  = 0x03;					//pin P0^0 & P0^1 are push pull
		P2MDIN 	|= 0x01;					//pin P2^0 is digital
		P2MDOUT  = 0x01;	 				//pin P2^0 is push pull	
		P1MDOUT  = 0x00;					//Port 1 is open drain
		P1MDIN	 = 0xff; 	
		XBR2	 = 0x40;					// enable crossbar and disable weak pullups
		XBR3     = 0x80;     				// Configure CAN TX pin (CTX) as push-pull digital output		
		SFRPAGE = SFRPAGE_SAVE;


}


void receive_data(char MsgNum)
{
  
  SFRPAGE  = CAN0_PAGE; 					// IF1 already set up for RX
  CAN0ADR  = IF2CMDRQST;					// Point to Command Request Reg.
  CAN0DATL = MsgNum;    					// Move new data for RX from Msg Obj "MsgNum"
                        					// Move new data to a
  CAN0ADR  = IF2DATA1;  					// Point to 1st byte of Data Field

  
}


void ISRname(void) interrupt 19
{
  status = CAN0STA;
  if ((status&0x10) != 0)
    {                            			// RxOk is set, interrupt caused by reception
      CAN0STA = (CAN0STA&0xEF)|0x07;        // Reset RxOk, set LEC to NoChange
      /* read message number from CAN INTREG */
      receive_data (0x01);             		// Up to now, we have only one RX message
    }
  if ((status&0x08) != 0)
    {                            			// TxOk is set, interrupt caused by transmision
      CAN0STA = (CAN0STA&0xF7)|0x07;        // Reset TxOk, set LEC to NoChange
    }
  if (((status&0x07) != 0)&&((status&0x07) != 7))
    {                           			// Error interrupt, LEC changed
      /* error handling ? */
      CAN0STA = CAN0STA|0x07;              // Set LEC to NoChange
    }
}

void clear_msg_objects(void)
{
  SFRPAGE  = CAN0_PAGE;
  CAN0ADR  = IF1CMDMSK;    					// Point to Command Mask Register 1
  CAN0DATL = 0xFF;         					// Set direction to WRITE all IF registers to Msg Obj
  for (i=1;i<33;i++)
    {
      CAN0ADR = IF1CMDRQST; 				// Write blank (reset) IF registers to each msg obj
      CAN0DATL = i;
    }
}

void radiotest(void)

{	
		
		/////////////////////////////
		//IGNITION SWITCH
		////////////////////////////

		if(INGITION_SW == 0)
			
			{		  
				h=0;
					  		
			onehundredms();
			shortdelay();				  	//100ms delay loop							
			txframe.arbID = 0x122;		  	//Message ID:									
			txframe.sz = 4;				  	//Message size(number of bytes 1-8)
			txframe.cdat[0] = 0x0C;	      	//CmdIgnStat 4=IGN_RUN and StTyp 1=NOR_ST						
			txframe.cdat[1] = 0x02;		  	//key in ignition
			txframe.cdat[2] = 0x00;	      	//null					
			txframe.cdat[3] = 0x00;	  		//null  
			tx_can_frame(0);			  	//transmits can frame
			
			
			////////////////////////
			//
			///////////////////////
			
			shortdelay();
			txframe.MsgNum = 2;			  	//Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x3f2;		  	//Message ID:CBC::EcuCfg3 									
			txframe.sz = 8;				  	//Message size(number of bytes 1-8)
			txframe.cdat[0] = 0x95;	      	//EC_ECUCfg3_Stat 1=PROGRAMMED and EC_AudTel1A						
			txframe.cdat[1] = 0x02;			//EC_AudTel1B
			txframe.cdat[2] = 0x57;	      	//EC_AudTel1B				
			txframe.cdat[3] = 0xA0;	  		//EC_AudTel1B
			txframe.cdat[4] = 0x09;	      	//EC_AudTel2					
			txframe.cdat[5] = 0x00;			//EC_AudTel2
			txframe.cdat[6] = 0x00;			//EC_AudTel2
			txframe.cdat[7] = 0x00;			//EC_AudTel2		  
			tx_can_frame(0);


			shortdelay();
			txframe.MsgNum = 4;			  	//Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x3e0;		  	//Message ID: CBC::VIN, VIN_LO hex 31 = ascii 1										
			txframe.sz = 8;				  	//Message size(number of bytes 1-8)
			txframe.cdat[0] = 0x00;	      	//VIN_LO=00						
			txframe.cdat[1] = 0x31;			//1st digit of VIN
			txframe.cdat[2] = 0x43;	      	//2nd digit of VIN				
			txframe.cdat[3] = 0x36;	  		//3rd digit of VIN
			txframe.cdat[4] = 0x52;	      	//4th digit of VIN						
			txframe.cdat[5] = 0x52;			//5th digit of VIN
			txframe.cdat[6] = 0x37;			//6th digit of VIN
			txframe.cdat[7] = 0x55;		  	//7th digit of VIN
			tx_can_frame(0);

			onehundredms();
			onehundredms();
			shortdelay();
			txframe.MsgNum = 5;			  	//Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x3e0;		  	//Message ID: CBC::VIN, VIN_MID hex 31 = ascii 1										
			txframe.sz = 8;				  	//Message size(number of bytes 1-8)
			txframe.cdat[0] = 0x01;	      	//VIN_MID=01						
			txframe.cdat[1] = 0x54;			//8th digit of VIN
			txframe.cdat[2] = 0x35;	   		//9th digit of VIN   				
			txframe.cdat[3] = 0x44;	  		//10th digit of VIN
			txframe.cdat[4] = 0x41;	      	//11th digit of VIN						
			txframe.cdat[5] = 0x30;			//12th digit of VIN
			txframe.cdat[6] = 0x30;			//13th digit of VIN
			txframe.cdat[7] = 0x39;		  	//14th digit of VIN
			tx_can_frame(0);


			shortdelay();
			txframe.MsgNum = 6;			  	//Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x3e0;		  	//Message ID: CBC::VIN, VIN_HI hex 31 = ascii 1										
			txframe.sz = 8;				  	//Message size(number of bytes 1-8)
			txframe.cdat[0] = 0x02;	      	//VIN_HI=02					
			txframe.cdat[1] = 0x33;			//15th digit of VIN
			txframe.cdat[2] = 0x55;	      	//16th digit of VIN				
			txframe.cdat[3] = 0x43;	  		//17th digit of VIN
			txframe.cdat[4] = 0x00;	      	//null					
			txframe.cdat[5] = 0x00;			//null
			txframe.cdat[6] = 0x00;			//null
			txframe.cdat[7] = 0x00;		 	//null 
			tx_can_frame(0);


			shortdelay();
			txframe.MsgNum = 10;			//Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x3E8;		  	//Message ID: CBC::VehCfg1									
			txframe.sz = 8;				  	//Message size(number of bytes 1-8)
			txframe.cdat[0] = 0x39;	      	//VC_VehCfg_Stat =1 (2014)					
			txframe.cdat[1] = 0x23;			//VC_VEH_LINE = WK
			txframe.cdat[2] = 0x22;	      	//VC_COUNTRY (2=USA) and VC_LHD_RHD (2=LHD)	and VC_RemStPrsnt		
			txframe.cdat[3] = 0x00;	  		//VC_XWD, VC_ATC_HVAC and VC_BODY_STYLE
			txframe.cdat[4] = 0x00;	      	//VC_MAX_VEH_SPD					
			txframe.cdat[5] = 0x00;			//VC_ShftrTyp, VC_PTS_DispTyp and VC_PP_Prsnt (Power Performance Pages present) 80=present
			txframe.cdat[6] = 0x00;			//null
			txframe.cdat[7] = 0x00;		  	//null
			tx_can_frame(0);


			shortdelay();
			txframe.MsgNum = 11;			//Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x3EA;		  	//Message ID: CBC::VehCfg3									
			txframe.sz = 8;				  	//Message size(number of bytes 1-8)
			txframe.cdat[0] = 0x01;	      	//VC_VehCfg3_Stat,VC_POLICE,VC_PWR_LFT_GT,VC_PWR_RT_SLDR,VC_PWR_LT_SLDR,VC_RKE_PRSNT,VC_SunShade_Present					
			txframe.cdat[1] = 0x00;			//VC_TcaseTyp,VC_WHL_BASE
			txframe.cdat[2] = 0x00;	      	//VC_FUEL_CAP				
			txframe.cdat[3] = 0x00;			//VC_WIN_EXPR_FEAT,VC_ADJ_PDL_PRSNT,VC_AUTO_HL,VC_DrAlrPrsnt,VC_DualFuelSens,VC_SRT_PRSNT,VC_DSBL_CLK_DISP
			txframe.cdat[4] = 0x80;	      	//VC_BrkTyp,VC_BrkTypMod,VC_SUSPENSION,VC_TEMP_DISP,			
			txframe.cdat[5] = 0x30;			//VC_HVAC_Config,VC_VEH_BRAND
			txframe.cdat[6] = 0x00;			//VC_CMP_DISP,VC_IntEC_MIRR,VC_ESCL_PRSNT,VC_HID_HL,VC_SecLkPrsnt,VC_Liftgate_Trunk,VC_CMS
			txframe.cdat[7] = 0x00;		  	//VC_FCW,VC_Base_TPM,VC_Prem_TPM
			tx_can_frame(0);
			

			shortdelay();						
			txframe.arbID = 0x3C9;		  	//Message ID: CBC::AMB									
			txframe.sz = 4;				  	//Message size(number of bytes 1-8)
			txframe.cdat[0] = 0x70;	      	//					
			txframe.cdat[1] = 0x00;			//
			txframe.cdat[2] = 0x00;	      	//				
			txframe.cdat[3] = 0x40;	  		//
			tx_can_frame(0);
			

			shortdelay();
			txframe.MsgNum = 15;			  //Message number(sequential starting at 1 for unique messages							
			txframe.arbID = 0x22F;		  //Message ID 									
			txframe.sz = 1;				  //Message size(number of bytes 1-8)
			txframe.cdat[0] = 0x01;	      //Disable Loss of COM Error								  
			tx_can_frame(0);



			shortdelay();				
			txframe.arbID = 0x3E3;		  	//Message ID:									
			txframe.sz = 8;				  	//Message size(number of bytes 1-8)
			txframe.cdat[0] = 0x01;	      						
			txframe.cdat[1] = 0x00;	
			txframe.cdat[2] = 0x00;	      					
			txframe.cdat[3] = 0x32;	  
			txframe.cdat[4] = 0x00;	      						
			txframe.cdat[5] = 0x00;	
			txframe.cdat[6] = 0x00;
			txframe.cdat[7] = 0x00;		  
			tx_can_frame(0);

			
			onehundredms();
			shortdelay();
			txframe.MsgNum = 20;			//Speed message: bytes 3 and 2 set speed (VEH_SPEED)  speed=km/h  / 0.0078125 km/h*(50k/h/0.0078125 km/h)=6400=0x1900 						
			txframe.arbID = 0x322;		  	//Message ID: TGW::GW_C1 							
			txframe.sz = 8;				  	//Message size(number of bytes 1-8) 
			txframe.cdat[0] = 0x00;	      	//EngRPM					
			txframe.cdat[1] = 0x00;			//VEH_SPEED
			txframe.cdat[2] = 0x00;	      	//null				
			txframe.cdat[3] = 0x14;	  		//null
			txframe.cdat[4] = 0x00;	      	//null					
			txframe.cdat[5] = 0x00;			//null
			txframe.cdat[6] = 0x00;			//null
			txframe.cdat[7] = 0x00;			//null
			tx_can_frame(0);
						

				if (PH_BUTTON == 0 )                   

      				{ 
        					shortdelay();
							txframe.MsgNum = 13;			//Message number(sequential starting at 1 for unique messages							
							txframe.arbID = 0x22d;		  	//Message ID: CBC::StW_Actn_Rq (VR active message byte 5 x02)										
							txframe.sz = 8;				  	//Message size(number of bytes 1-8)
							txframe.cdat[0] = 0x00;	      	//TurnIndLvr_Stat,HiBmLvr_Stat,WprWashSw_Psd,HrnSw_Psd,StW_Cond_Flt					
							txframe.cdat[1] = 0x00;			//WprWash_R_Sw_Posn_V3,StW_Lvr_Stat
							txframe.cdat[2] = 0x00;	 		//WprSw6Posn,     					
							txframe.cdat[3] = 0x10;	  		//AUD_MODE_ADV,VOL,SEEK,PRESET_CFG
							txframe.cdat[4] = 0x00;	      	//MENU_RQ,DN_ARW_STEP_RQ,Up_Arw_Rq,RT_ARW_RST_RQ					
							txframe.cdat[5] = 0x00;			//CELL_VR,StW_TempSens_Flt
							txframe.cdat[6] = 0x00;			//StW_Temp
							txframe.cdat[7] = 0x00;		  	//null
							tx_can_frame(0);

						
	  			
						}			
				else if (VR_BUTTON == 0)  
      				  {  
       					 	shortdelay();
							txframe.MsgNum = 14;			//Message number(sequential starting at 1 for unique messages							
							txframe.arbID= 0x318;		  	//Message ID VR off									
							txframe.sz = 8;				  	//Message size(number of bytes 1-8)
							txframe.cdat[0] = 0x00;	      						
							txframe.cdat[1] = 0x00;	
							txframe.cdat[2] = 0x00;	      					
							txframe.cdat[3] = 0x00;	  
							txframe.cdat[4] = 0x00;	      						
							txframe.cdat[5] = 0x02;	
							txframe.cdat[6] = 0x00;
							txframe.cdat[7] = 0x00;		  
							tx_can_frame(0);
     				  }               

				else if (PHONE_END == 0)  
      				  {  
       					 	shortdelay();
							txframe.MsgNum = 15;			//Message number(sequential starting at 1 for unique messages							
							txframe.arbID= 0x22D;		  	//Message ID VR off									
							txframe.sz = 8;				  	//Message size(number of bytes 1-8)
							txframe.cdat[0] = 0x00;	      						
							txframe.cdat[1] = 0x00;	
							txframe.cdat[2] = 0x00;	      					
							txframe.cdat[3] = 0x40;	  
							txframe.cdat[4] = 0x00;	      						
							txframe.cdat[5] = 0x00;	
							txframe.cdat[6] = 0x00;
							txframe.cdat[7] = 0x00;		  
							tx_can_frame(0);
     				  }               
			
				else
						{
							shortdelay();
							txframe.MsgNum = 12;			  //Message number(sequential starting at 1 for unique messages							
							txframe.arbID= 0x22D;		  //Message ID VR off									
							txframe.sz = 8;				  //Message size(number of bytes 1-8)
							txframe.cdat[0] = 0x00;	      						
							txframe.cdat[1] = 0x00;	
							txframe.cdat[2] = 0x00;	  // lever in Park    					
							txframe.cdat[3] = 0x00;	  
							txframe.cdat[4] = 0x00;	      						
							txframe.cdat[5] = 0x00;	
							txframe.cdat[6] = 0x00;
							txframe.cdat[7] = 0x00;		  
							tx_can_frame(0);

							shortdelay();
							txframe.MsgNum = 12;			  //Message number(sequential starting at 1 for unique messages							
							txframe.arbID= 0x318;		  //Message ID VR off									
							txframe.sz = 8;				  //Message size(number of bytes 1-8)
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
					
			}
			
		else
			{		
					

					if(h <= 5)
						{
					shortdelay();				  	//100ms delay loop
					txframe.MsgNum = 3;			  	//Message number(sequential starting at 1 for unique messages							
					txframe.arbID = 0x122;		  	//Message ID	CBC_13 Vehicle start packet									
					txframe.sz = 4;				  	//Message size(number of bytes 1-8)
					txframe.cdat[0] = 0x01;	      	//ignition run, start not active						
					txframe.cdat[1] = 0x02;		  	//key in ignition
					txframe.cdat[2] = 0x00;	      	//null					
					txframe.cdat[3] = 0x00;	  		//null  
					tx_can_frame(0);			  	//transmits can frame
					h++;
						}
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
