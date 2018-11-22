//==============================================================================
// GLOBALS - The main global definitions for the program 
//==============================================================================
#ifndef _Tarantula_GLOBALS_H_
#define _Tarantula_GLOBALS_H_
#include "Tarantula_Cfg.h"

//=============================================================================
//[CONSTANTS]
//=============================================================================
#define BUTTON_DOWN 0
#define BUTTON_UP 	1

#define	c1DEC		10
#define	c2DEC		100
#define	c4DEC		10000
#define	c6DEC		1000000

#define	cRR			 0   // Right Rear leg
#define	cRM			 1   // Right Middle leg
#define cRMF     2   // Right Middle Front leg
#define	cRF			 3   // Right Front leg
#define cRP      4   // Right Pedi
#define	cLR			 5   // Left Rear leg
#define	cLM			 6   // Left Middle leg
#define cLMF     7   // Left Middle Front leg
#define	cLF			 8   // Left Front leg
#define cLP      9   // Left Pedi

//[REMOTE]                 
#define cTravelDeadZone  1    //The deadzone for the analog input from the remote 

extern void GaitSelect(void);
extern short SmoothControl (short CtrlMoveInp, short CtrlMoveOut, byte CtrlDivider);

//-----------------------------------------------------------------------------
// Define Global variables
//-----------------------------------------------------------------------------
extern boolean TarantulaOn;			//Switch to turn on TARANTULA
extern boolean Prev_TarantulaOn;//Previous loop state 

//Body position
extern long		 BodyPosX; 	        //Global Input for the position of the body
extern long		 BodyPosY; 
extern long		 BodyPosZ; 

//Body Inverse Kinematics
extern long		 BodyRotX1; 	    	//Global Input pitch of the body
extern long		 BodyRotY1;	        //Global Input rotation of the body
extern long		 BodyRotZ1; 		    //Global Input roll of the body

//[gait]
extern byte		 GaitType;			    //Gait type
extern short   NomGaitSpeed;	    //Nominal speed of the gait
extern short	 LegLiftHeight;		  //Current Travel height
extern long		 TravelLengthX;	  	//Current Travel length X
extern long		 TravelLengthZ; 	  //Current Travel length Z
extern long		 TravelRotationY;   //Current Travel Rotation Y

//[Pedi control]
extern long PediRotation;

//[Single Leg Control]
extern byte		 SelectedLeg;
extern short	 SLLegX;
extern short	 SLLegY;
extern short	 SLLegZ;
extern boolean fSLHold;	    	   	//Single leg control mode

//--------------------------------------------------------------------
//[GP PLAYER]
extern boolean GPStart;						//Start the GP Player
extern byte	   GPSeq;							//Number of the sequence
extern boolean GPEnable;					//Enables the GP player when the SSC version ends with "GP<cr>"

//[Balance]
extern boolean BalanceMode;

//[TIMING]
extern byte InputTimeDelay;	     //Delay that depends on the input to get the "sneaking" effect
extern word SpeedControl;	       //Adjustible Delay
extern word SSCTime;		         //Time for servo updates

// maybe should do this different
extern void	ServoDriverStart(void);
extern void	ServoDriverCommit(void);

extern void MSound(uint8_t _pin, byte cNotes, ...);
extern int DBGPrintf(const char *format, ...);
extern int SSCPrintf(const char *format, ...);
extern int SSCRead (byte* pb, int cb, word wTimeout, word wEOL);

// The defined controller must provide the following
extern void InitController(void);
extern void	ControlInput(void);
extern void	AllowControllerInterrupts(boolean fAllow);

// debug handler...
extern boolean g_fDBGHandleError;
#endif

//--------------------------------------------------------------------
//[TABLES]
//ArcCosinus Table
//Table build in to 3 part to get higher accuracy near cos = 1. 
//The biggest error is near cos = 1 and has a biggest value of 3*0.012098rad = 0.521 deg.
//-    Cos 0 to 0.9 is done by steps of 0.0079 rad. [1/127]
//-    Cos 0.9 to 0.99 is done by steps of 0.0008 rad [0.1/127]
//-    Cos 0.99 to 1 is done by step of 0.0002 rad [0.01/64]
//Since the tables are overlapping the full range of 127+127+64 is not necessary. Total bytes: 277

static const byte GetACos[] = {    
                    255,254,252,251,250,249,247,246,245,243,242,241,240,238,237,236,234,233,232,231,229,228,227,225, 
                    224,223,221,220,219,217,216,215,214,212,211,210,208,207,206,204,203,201,200,199,197,196,195,193, 
                    192,190,189,188,186,185,183,182,181,179,178,176,175,173,172,170,169,167,166,164,163,161,160,158, 
                    157,155,154,152,150,149,147,146,144,142,141,139,137,135,134,132,130,128,127,125,123,121,119,117, 
                    115,113,111,109,107,105,103,101,98,96,94,92,89,87,84,81,79,76,73,73,73,72,72,72,71,71,71,70,70, 
                    70,70,69,69,69,68,68,68,67,67,67,66,66,66,65,65,65,64,64,64,63,63,63,62,62,62,61,61,61,60,60,59,
                    59,59,58,58,58,57,57,57,56,56,55,55,55,54,54,53,53,53,52,52,51,51,51,50,50,49,49,48,48,47,47,47,
                    46,46,45,45,44,44,43,43,42,42,41,41,40,40,39,39,38,37,37,36,36,35,34,34,33,33,32,31,31,30,29,28,
                    28,27,26,25,24,23,23,23,23,22,22,22,22,21,21,21,21,20,20,20,19,19,19,19,18,18,18,17,17,17,17,16,
                    16,16,15,15,15,14,14,13,13,13,12,12,11,11,10,10,9,9,8,7,6,6,5,3,0 };
                    
//Sin table 90 deg, precision 0.5 deg [180 values]
static const word GetSin[] = {
                    0,    87,   174,  261,  348,  436,  523,  610,  697,  784,  871,  958,  1045, 1132, 1218, 1305, 1391, 1478, 1564, 
                    1650, 1736, 1822, 1908, 1993, 2079, 2164, 2249, 2334, 2419, 2503, 2588, 2672, 2756, 2840, 2923, 3007, 
                    3090, 3173, 3255, 3338, 3420, 3502, 3583, 3665, 3746, 3826, 3907, 3987, 4067, 4146, 4226, 4305, 4383, 
                    4461, 4539, 4617, 4694, 4771, 4848, 4924, 4999, 5075, 5150, 5224, 5299, 5372, 5446, 5519, 5591, 5664, 
                    5735, 5807, 5877, 5948, 6018, 6087, 6156, 6225, 6293, 6360, 6427, 6494, 6560, 6626, 6691, 6755, 6819, 
                    6883, 6946, 7009, 7071, 7132, 7193, 7253, 7313, 7372, 7431, 7489, 7547, 7604, 7660, 7716, 7771, 7826, 
                    7880, 7933, 7986, 8038, 8090, 8141, 8191, 8241, 8290, 8338, 8386, 8433, 8480, 8526, 8571, 8616, 8660, 
                    8703, 8746, 8788, 8829, 8870, 8910, 8949, 8987, 9025, 9063, 9099, 9135, 9170, 9205, 9238, 9271, 9304, 
                    9335, 9366, 9396, 9426, 9455, 9483, 9510, 9537, 9563, 9588, 9612, 9636, 9659, 9681, 9702, 9723, 9743, 
                    9762, 9781, 9799, 9816, 9832, 9848, 9862, 9876, 9890, 9902, 9914, 9925, 9935, 9945, 9953, 9961, 9969, 
                    9975, 9981, 9986, 9990, 9993, 9996, 9998, 9999, 10000 };
                                   

// Walking comp tables for 48step:
//    GaitLegNr(cLF) = 28
//    GaitLegNr(cRR) = 40
//    GaitLegNr(cRF) = 3
//    GaitLegNr(cLR) = 16
//                                              (RF)                                               (LR)                                 
//                                   0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 
static const byte X_Axis_Shift[] = {88, 89, 90, 91, 95,100,110,115,120,128,135,145,150,155,158,158,158,159,160,161,162,163,163,162,161,160,
                                   159,158,158,158,155,150,145,135,128,120,115,110,100, 95, 91, 90, 89, 88, 87, 86, 85, 86, 87 };
//                                  26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48
//                                         (LF)                                            (RR)
//                               
//                                              (RF)                                           (LR)
//                                   0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 
static const byte Z_Axis_Shift[] = {128,128,128,128,128,128,128,129,130,131,133,135,137,139,140,142,140,139,137,135,133,131,130,128,128,128,
                                    128,128,128,128,128,128,129,130,131,133,135,137,139,140,142,140,139,137,135,133,131,130,128 }; 
//                                  26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48                       
//;                                        (LF)                                            (RR)                           
//;----------------------------------------------------
//// Walking comp tables for 24step: TARANTULA (Example)
////    GaitLegNr(cLF) = 13
////    GaitLegNr(cRR) = 19
////    GaitLegNr(cRF) = 1
////    GaitLegNr(cLR) = 7
////
////                                      (RF)                    (LR)                   (LF)                    (RR)
////                                  ;0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24,
//static const byte X_Axis_Shift[] = {89, 90,100,110,120,130,140,150,151,152,153,152,151,150,140,130,120,110,100, 90, 89, 88, 87, 88, 89,
////                                      (RF)                    (LR)                   (LF)                    (RR)
////                                   ;0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24,
//static const byte Z_Axis_Shift[] = {140,138,140,142,144,146,148,150,148,146,144,142,140,138,140,142,144,146,148,150,148,146,144,142,140                      

// SSC Pin numbers
static const byte cCoxaPin[]     = {cRRCoxaPin,  cRMCoxaPin,  cRMFCoxaPin,  cRFCoxaPin,  cLRCoxaPin,  cLMCoxaPin,  cLMFCoxaPin,  cLFCoxaPin,  RPedCoxaPin,  LPedCoxaPin};
static const byte cFemurPin[]    = {cRRFemurPin, cRMFemurPin, cRMFFemurPin, cRFFemurPin, cLRFemurPin, cLMFemurPin, cLMFFemurPin, cLFFemurPin, RPedFemurPin, LPedFemurPin};
static const byte cTibiaPin[]    = {cRRTibiaPin, cRMTibiaPin, cRMFTibiaPin, cRFTibiaPin, cLRTibiaPin, cLMTibiaPin, cLMFFemurPin, cLFTibiaPin, RPedFemurPin, LPedTibiaPin};

// Min / Max values
static const short cCoxaMin1[]   = {cRRCoxaMin1,  cRMCoxaMin1,  cRMFCoxaMin1,  cRFCoxaMin1,  cLRCoxaMin1,  cLMCoxaMin1,  cLMFCoxaMin1,  cLFCoxaMin1,  RPedCoxaMin,  LPedCoxaMin};
static const short cCoxaMax1[]   = {cRRCoxaMax1,  cRMCoxaMax1,  cRMFCoxaMax1,  cRFCoxaMax1,  cLRCoxaMax1,  cLMCoxaMax1,  cLMFCoxaMax1,  cLFCoxaMax1,  RPedCoxaMax,  LPedCoxaMax};
static const short cFemurMin1[]  = {cRRFemurMin1, cRMFemurMin1, cRMFFemurMin1, cRFFemurMin1, cLRFemurMin1, cLMFemurMin1, cLMFFemurMin1, cLFFemurMin1, RPedFemurMin, LPedFemurMin};
static const short cFemurMax1[]  = {cRRFemurMax1, cRMFemurMax1, cRMFFemurMax1, cRFFemurMax1, cLRFemurMax1, cLMFemurMax1, cLMFFemurMax1, cLFFemurMax1, RPedFemurMax, LPedFemurMax};
static const short cTibiaMin1[]  = {cRRTibiaMin1, cRMTibiaMin1, cRMFTibiaMin1, cRFTibiaMin1, cLRTibiaMin1, cLMTibiaMin1, cLMFTibiaMin1, cLFTibiaMin1, RPedTibiaMin, LPedTibiaMin};
static const short cTibiaMax1[]  = {cRRTibiaMax1, cRMTibiaMax1, cRMFTibiaMax1, cRFTibiaMax1, cLRTibiaMax1, cLMTibiaMax1, cLMFTibiaMax1, cLFTibiaMax1, RPedTibiaMax, LPedTibiaMax};

// Body Offsets [distance between the center of the body and the center of the coxa]
static const short cOffsetX[]    = {cRROffsetX, cRMOffsetX, cRMFOffsetX, cRFOffsetX, cLROffsetX, cLMOffsetX, cLMFOffsetX, cLFOffsetX, RPedOffsetX, LPedOffsetX};
static const short cOffsetZ[]    = {cRROffsetZ, cRMOffsetZ, cRMFOffsetZ, cRFOffsetZ, cLROffsetZ, cLMOffsetZ, cLMFOffsetZ, cLFOffsetZ, RPedOffsetZ, LPedOffsetZ};

// Default leg angle
static const short cCoxaAngle1[] = {cRRCoxaAngle1, cRMCoxaAngle1, cRMFCoxaAngle1, cRFCoxaAngle1, cLRCoxaAngle1, cLMCoxaAngle1, cLMFCoxaAngle1, cLFCoxaAngle1, RPedAngle1, LPedAngle1};

// Start positions for the leg
static const short cInitPosX[]   = {cRRInitPosX, cRMInitPosX, cRMFInitPosX, cRFInitPosX, cLRInitPosX, cLMInitPosX, cLMFInitPosX, cLFInitPosX, RPedInitPosX, LPedInitPosX};
static const short cInitPosY[]   = {cRRInitPosY, cRMInitPosY, cRMFInitPosY, cRFInitPosY, cLRInitPosY, cLMInitPosY, cLMFInitPosY, cLFInitPosY, RPedInitPosY, LPedInitPosY};
static const short cInitPosZ[]   = {cRRInitPosZ, cRMInitPosZ, cRMFInitPosZ, cRFInitPosZ, cLRInitPosZ, cLMInitPosZ, cLMFInitPosZ, cLFInitPosZ, RPedInitPosZ, LPedInitPosZ};

//--------------------------------------------------------------------
