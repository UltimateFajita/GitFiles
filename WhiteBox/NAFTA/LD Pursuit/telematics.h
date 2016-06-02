#define BETA				'V'
#define BOOTLD				'1'
#define CUBEMAJOR			'3'
#define CUBEMINOR			'b'	
#define BLEV 				'0'

//#define	DIAG_KWP			0
#define DIAG_UDS			1

/////////////////////
// Mode definitions
/////////////////////

//#define UDSDEBUG
//#define CANWAKEY
//#define RADIO
//#define TGW_SIM
//#define AIRSUSPENSION

//#define XTDTEST

//#define TRAILERTOW

#define CELL_VR

/////////////////
// Hardware defs
/////////////////

//#define ECANHW
//#define PCANHW
//#define ICANHW

//#define ETCUSBHW
	
// If VDD hardware exists
#define VDD_MONITOR	

#ifdef ECANHW
	#define SYSCLK		24000000	
#endif
#ifdef PCANHW
	#define SYSCLK 		18000000            // approximate SYSCLK frequency in Hz
#else
	#define SYSCLK 		18000000            // approximate SYSCLK frequency in Hz
#endif	

#ifndef BYTEDEFINED							// If byte not defined,
typedef unsigned char byte;					//	define byte data type!
#define BYTEDEFINED
#endif

#ifdef TRAILERTOW

	#define USBPORT		

	#define CANBAUD		125
	#define	DIAG_KWP	0
// #define DIAG_UDS	1

	sbit GREEN = P1 ^ 0;
	sbit RED = P1 ^ 1;


// RX P0.0 int0
// TX P0.1 int1
// Reset P0.2


	sbit int0 = P0 ^ 0;
	sbit int1 = P0 ^ 1;
	sbit reset = P0 ^ 2;

	sbit rly1 = P4 ^ 0;
	sbit rly2 = P4 ^ 1;
	sbit rly3 = P4 ^ 2;
	sbit rly4 = P4 ^ 3;

#endif

#ifdef XTDTEST

	// Code running on pCAN hardware

	sbit int0 = P1 ^ 3;
	sbit int1 = P1 ^ 4;

	sbit GREEN = P1 ^ 6;
	sbit RED = P1 ^ 7;

	#define CANBAUD		500
//	#define USBPORT		
	#define DIAG_UDS	1

	// USB Frame Structure
	#define MAXPAYLOAD	256
	typedef struct tUSBFRAME
	{
		unsigned int sync;
		unsigned int sz;
		unsigned int seq;
		unsigned int fnc;
		unsigned char udat[MAXPAYLOAD];
		unsigned int crc;
		unsigned int cksm;
	} USBFRAME;

	//////////////////////
	// CCP protocol defs
	//////////////////////

	typedef struct tRASTER2
	{
		unsigned long RxID;					// Raster RxID
		byte mrate;							// Raster maximum Rate 
		byte sizd;							// Raster size limit
		byte daqlistnum,eventnum;
	} RASTER2;

	typedef struct tECU_IDS					// ECU CAN ID structure (1 per ECU used)
	{										// locate at &RateGrp + RateGrpCnt*sizeof(RATEGRP)
		unsigned long KWPTx;
		unsigned long KWPRx;
		unsigned int CCPstationID;

		unsigned long CCPTx;
		unsigned long CCPRx;
		RASTER2 Rast[4];
		int ModFlags;
		int ECUstat;
		byte count[4];
		byte frstPID[4];
	} ECU_IDS;

	// status bits
	#define CCPPROTO		1
	#define KWPPROTO		2
	#define SLOWKWP			4
	#define NOCCP			0x10		
	#define NOPROTO			0x20		

	#define RASTREQ			0x100			// bits 8-11 Shifted for each raster
	#define RASTCMPT		0x1000			// bits 12-15 shifted for each raster

	#define CANBUFFSIZE		32
	#define CANTXBUFFSIZE	64

	// CCP module type defs
	#define ccpGPEC		1
	#define ccpNGC3e	2
	#define ccpNGC3t	3
	#define ccpGMHYB	4
	#define ccp8GMC		5

#endif

#ifdef RADIO

	#ifdef ICANHW
		// Cycler Code running on iCAN hardware
		
		sbit int0 = P1 ^ 1;
		sbit int1 = P1 ^ 2;

		sbit GREEN = P1 ^ 7;
		sbit RED = P1 ^ 6;

		#define CANBAUD	125	

	#else

		sbit UPSWCH = P0 ^ 0;
		sbit DWNSWCH = P0 ^ 1;
	
		sbit UPRELAY = P1 ^ 1;
		sbit DWNRELAY = P1 ^ 2;
	
		sbit GREEN = P1 ^ 7;
		sbit RED = P1 ^ 6;

		#define CANBAUD 125
		#define CYCLETIME	10

	#endif

#endif

#ifdef TGW_SIM
	sbit UPSWCH = P0 ^ 0;
	sbit DWNSWCH = P0 ^ 1;
	
	sbit UPRELAY = P1 ^ 1;
	sbit DWNRELAY = P1 ^ 2;
	
	sbit JUMBO = P1 ^ 5;
	sbit GREEN = P1 ^ 7;
	sbit RED = P1 ^ 6;

	#define CANBAUD 125
	#define CYCLETIME	10
#endif

#ifdef CELL_VR
	sbit UPSWCH = P0 ^ 0;
	sbit DWNSWCH = P0 ^ 1;
	
	sbit UPRELAY = P1 ^ 1;
	sbit DWNRELAY = P1 ^ 2;
	
	sbit JUMBO = P1 ^ 5;
	sbit GREEN = P1 ^ 7;
	sbit RED = P1 ^ 6;

	#define CANBAUD 125
	#define CYCLETIME 10
	// Switch type - use NO_SWITCH for first type of pushbutton
	//#define NO_SWITCH
#endif

#ifdef AIRSUSPENSION
	#ifdef ECANHW
		// Cycler Code running on eCAN hardware
		// Lantronix eCAN 2.4

		sbit NSSB = P1 ^ 1;
		sbit NSSC = P1 ^ 2;

		sbit RED = P1 ^ 3;
		sbit GREEN = P1 ^ 4;

		sbit RTS = P1 ^ 5;
		sbit CTS = P1 ^ 6;
		sbit SDET = P1 ^ 7;

		sbit NSSA = P0 ^ 3;

		#define CANBAUD		500
		#define ETHERNET		
		#define DIAG_UDS	1

		// SYSCLK = 24000000 - UART1 - Lantronix
		#define			L115k			0x98	// SYSCLK
		#define			L230k			0xcc	// SYSCLK
		#define			L460k			0xe6	// SYSCLK
		#define			L921k			0xf0	// SYSCLK					
	#endif
	#ifdef PCANHW
		// Code running on pCAN hardware

//		sbit int0 = P1 ^ 0;
//		sbit int1 = P1 ^ 1;

		sbit int0 = P1 ^ 3;
		sbit int1 = P1 ^ 4;

		sbit GREEN = P1 ^ 6;
		sbit RED = P1 ^ 7;

//		sbit NSSA = P0 ^ 3;
	
//		#define NSSB F0
//		#define NSSC F0

		#define CANBAUD		500
		#define USBPORT		
		#define DIAG_UDS	1

	#endif

#endif


#ifdef USBPORT

	// USB Frame Structure
	#define MAXPAYLOAD	256
	typedef struct tUSBFRAME
	{
		unsigned int sync;
		unsigned int sz;
		unsigned int seq;
		unsigned int fnc;
		unsigned char udat[MAXPAYLOAD];
		unsigned int crc;
		unsigned int cksm;
	} USBFRAME;

	//////////////////////
	// VIB protocol defs
	//////////////////////
	#define ERROR 				128

	#define REQSWVERSION		0x2000
	#define REQALPAGE			0x2003
	#define GETCLOCK			0x2070
	#define SETCLOCK			0x2071

	#define IQUBERESET			0x2005
	#define WIPEDATAFLASH		0x2007

	#define STARTBRDTRAP		0x5200
	#define TRAPLISTBT			0x5204
	#define SINGCANMESG			0x5205
	#define SIMMSGLIST			0x5206
	#define SIMMODE				0x5207
	#define MIRROR				0x5208

	#define SENDAUTOLD			0x8034
	#define AUTOLDPACKET		0x8036
	#define ENDAUTOLDXFER		0x8037
	#define SETAUTOLDPAGE		0x8005

	#define REQSTREAMMODE		0x8008
	#define STREAMPACKET		0x800A

	#define REQDCAPMODE			0x8024
	#define DCAPPACKET			0x802A
		
	#define DCAPTRIGGER			0x8010
	#define STOPDATA			0x807E

	#define MODCONFIGFLASH		0xC000
	#define MODREQFLASH			0xC001
	#define MODFLASHPACKET		0xC008
	#define MODFLASHCHECKSUM	0xC009
	#define MODFLASHSTATUS		0xC00F

	#define PING_MODS			0xD010
	#define MODRESET			0xD011
	#define C_FAULTS			0xD014
	#define C_ALL_FAULTS		0xD015
	#define R_FAULTS			0xD018
	#define R_VIN				0xD01A
	#define W_VIN				0xD01B
	#define R_EDIT				0xD021
	#define W_EDIT				0xD03B
	#define DIAL_PID			0xD027

	#define VIN_ODO				0xD030
	#define	VOLTS_IO			0xD031

	#define ECU_PN				0xD040
	#define ECU_SN				0xD041			
	#define ECU_HV				0xD042
	#define ECU_SV				0xD043

	#define	RLY_STATE			0xD050

	/////////////////////////////////////////////////////
	// RunValues for running functions from usb commands
	/////////////////////////////////////////////////////
	enum RunValues
	{
		GETVERSION = 1,
		SIMTOGGLE,
		READFAULTS,
		CLEARFAULTS,

		READODO,
		READVOLTS,

		READECUPARTNUM,
		READECUSN,
		READECUHV,
		READECUSV,
		SETRELAY,

		STANDRESP,
		GCLOCK,
		SCLOCK
	};

#endif

////////////
// CAN Defs
////////////

// Words
sfr16 CAN0DAT = 0xD8;
sfr16 RCAP3 = 0xCA;                 			// Timer3 reload value
sfr16 TMR3 = 0xCC;                 				// Timer3 counter

sfr16 RCAP2 = 0xCA;                 			// Timer2 reload value
sfr16 TMR2 = 0xCC;                 				// Timer2 counter

/////////////////////////////////////////////////////////////////////////////////////
//CAN Protocol Register Index for CAN0ADR, from TABLE 18.1 of the C8051F040 datasheet
/////////////////////////////////////////////////////////////////////////////////////
#define CANCTRL            0x00                 //Control Register
#define CANSTAT            0x01                 //Status register
#define ERRCNT             0x02                 //Error Counter Register
#define BITREG             0x03                 //Bit Timing Register
#define INTREG             0x04                 //Interrupt Low Byte Register
#define CANTEST            0x05                 //Test register
#define BRPEXT             0x06                 //BRP Extension         Register
/////////////////////////////////////////////////////////////////////////////////
//IF1 Interface Registers
/////////////////////////////////////////////////////////////////////////////////
#define IF1CMDRQST         0x08                 //IF1 Command Rest      Register
#define IF1CMDMSK          0x09                 //IF1 Command Mask      Register
#define IF1MSK1            0x0A                 //IF1 Mask1             Register
#define IF1MSK2            0x0B                 //IF1 Mask2             Register
#define IF1ARB1            0x0C                 //IF1 Arbitration 1     Register
#define IF1ARB2            0x0D                 //IF1 Arbitration 2     Register
#define IF1MSGC            0x0E                 //IF1 Message Control   Register
#define IF1DATA1           0x0F                 //IF1 Data A1           Register
#define IF1DATA2           0x10                 //IF1 Data A2           Register
#define IF1DATB1           0x11                 //IF1 Data B1           Register
#define IF1DATB2           0x12                 //IF1 Data B2           Register
/////////////////////////////////////////////////////////////////////////////////
//IF2 Interface Registers
/////////////////////////////////////////////////////////////////////////////////
#define IF2CMDRQST         0x20                 //IF2 Command Rest      Register
#define IF2CMDMSK          0x21                 //IF2 Command Mask      Register
#define IF2MSK1            0x22                 //IF2 Mask1             Register
#define IF2MSK2            0x23                 //IF2 Mask2             Register
#define IF2ARB1            0x24                 //IF2 Arbitration 1     Register
#define IF2ARB2            0x25                 //IF2 Arbitration 2     Register
#define IF2MSGC            0x26                 //IF2 Message Control   Register
#define IF2DATA1           0x27                 //IF2 Data A1           Register
#define IF2DATA2           0x28                 //IF2 Data A2           Register
#define IF2DATB1           0x29                 //IF2 Data B1           Register
#define IF2DATB2           0x2A                 //IF2 Data B2           Register
/////////////////////////////////////////////////////////////////////////////////
//Message Handler Registers
/////////////////////////////////////////////////////////////////////////////////
#define TRANSREQ1          0x40                 //Transmission Rest1 Register
#define TRANSREQ2          0x41                 //Transmission Rest2 Register

#define NEWDAT1            0x48                 //New Data 1            Register
#define NEWDAT2            0x49                 //New Data 2            Register

#define INTPEND1           0x50                 //Interrupt Pending 1   Register
#define INTPEND2           0x51                 //Interrupt Pending 2   Register
                    
#define MSGVAL1            0x58                 //Message Valid 1       Register
#define MSGVAL2            0x59                 //Message Valid 2       Register

/////////////////////////////////////////////////////////////////////////////////
// CAN software register assignements
/////////////////////////////////////////////////////////////////////////////////

// general user message registers 0x0 - 0x17

// Diag Message registers x18-x1F
#define GLOBAL_RX 	0x1F				
#define DIAG_TX		0x1E
#define DIAG_RX		0x1D

// Tester Present messages from x1A-x1C
#define FCM_GWAY_TX 0x1A					// Tester Present messages from x1A-x1C
#define GLOBAL_TX1 	0x1B
#define GLOBAL_TX2 	0x1C

/////////////////////////////////////////////////////////////////////////////////
// BIT0-f value defines 
/////////////////////////////////////////////////////////////////////////////////
#define BIT0	1
#define BIT1	2
#define BIT2	4
#define BIT3	8
#define BIT4	0x10
#define BIT5	0x20
#define BIT6	0x40
#define BIT7	0x80
#define BIT8	0x100
#define BIT9	0x200
#define BITa	0x400
#define BITb	0x800
#define BITc	0x1000
#define BITd	0x2000
#define BITe	0x4000
#define BITf	0x8000

#define BIT31	0x80000000

/////////////////////////////////////////////////////////////////////////////////
// CAN Frame structure type defs
/////////////////////////////////////////////////////////////////////////////////

//CAN Frame Structure
typedef struct tCANFRAME
{
	unsigned char MsgNum;
	unsigned char sz;
	unsigned long arbID;
	unsigned char cdat[8];
} CANFRAME;

typedef struct tXMITFRAME
{
	unsigned char MsgNum;
	unsigned char sz;
	unsigned long arbID;
	unsigned char cdat[8];
} XMITFRAME;

typedef struct tKWPMESG
{
	unsigned char ecu;
	unsigned long TxID,RxID;
	unsigned int LenOut;
	unsigned int LenIn;
	unsigned char xdata *mesg;
	unsigned int delay;
	unsigned char pause;
} KWPMESG;

// KWP stuff
int CAN_KWP_command(unsigned char);
void PassSecurity(unsigned char);
void GateWay(void);
void mSecDelay(unsigned int);

#ifdef BUZZER
void BzBuzzer(unsigned char beeps);
void PzBuzzer(unsigned int rate, unsigned int delay);
#endif

#ifdef XTDTEST
void init_msg_object_rx (unsigned char MsgNum,unsigned long arbID); 
byte tx_can_frame (unsigned char KWPopts); 

//void SetupCCPdaqs(void);
//byte CCP_CalPage(byte ecu);
byte CCP_Edit(byte ecu);
byte CCPrequest(byte ecu,char *message,byte flags);
byte CCP_Security(byte ecu);

void xtst(void);
#endif

#ifdef TRAILERTOW
void init_msg_object_rx (unsigned char MsgNum,unsigned long arbID); 
byte tx_can_frame (unsigned char KWPopts); 

void trailtow(void);
#endif

#ifdef UDSDEBUG
void udstest(void);
#endif

#ifdef RADIO
void radtest(void);
#endif

#ifdef TGW_SIM
void tgwsim(void);
#endif

#ifdef CELL_VR
byte tx_can_frame (unsigned char KWPopts); 
void cbcsim(void);
#endif

#ifdef AIRSUSPENSION
void airsusp(void);
#endif
