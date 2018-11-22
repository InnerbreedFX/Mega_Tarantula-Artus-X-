/******************************************************************
*  Configuration file 
******************************************************************/
//--------------------------------------------------------------------
#define DBGSerial Serial
#define SSCSerial Serial1

#define USEPS2

//[SERIAL CONNECTIONS]
#define cSSC_OUT         19 // Output pin for (SSC32 RX) on (MEGA 1TX - pin19)
#define cSSC_IN          18 // Input pin for (SSC32 TX) on (MEGA 1RX - pin18)
#define cSSC_BAUD      9600 // SSC32 BAUD rate
#define	cSSC_BINARYMODE	  0 // Define if your SSC-32 card supports binary mode.
//--------------------------------------------------------------------
//[Arduino Mega Pin Numbers for PS2]
#define PS2_DAT           5   
#define PS2_CMD           6  
#define PS2_SEL           7  
#define PS2_CLK           8   

//--------------------------------------------------------------------
//[SSC PIN NUMBERS]
#define cRRCoxaPin         0 // Rear Right leg Hip Horizontal
#define cRRFemurPin        1 // Rear Right leg Hip Vertical
#define cRRTibiaPin        2 // Rear Right leg Knee

#define cRMCoxaPin         3 // Middle Right leg Hip Horizontal
#define cRMFemurPin        4 // Middle Right leg Hip Vertical
#define cRMTibiaPin        5 // Middle Right leg Knee

#define cRMFCoxaPin        6 // Middle Right leg Hip Horizontal
#define cRMFFemurPin       7 // Middle Right leg Hip Vertical
#define cRMFTibiaPin       8 // Middle Right leg Knee

#define cRFCoxaPin         9 // Front Right leg Hip Horizontal
#define cRFFemurPin       10 // Front Right leg Hip Vertical
#define cRFTibiaPin       11 // Front Right leg Knee

#define RPedCoxaPin       12 // Secondary Appendages
#define RPedFemurPin      13 // Secondary Appendages
#define RPedTibiaPin      14 // Secondary Appendages

//#define                   15

#define cLRCoxaPin        16 // Rear Left leg Hip Horizontal
#define cLRFemurPin       17 // Rear Left leg Hip Vertical
#define cLRTibiaPin       18 // Rear Left leg Knee

#define cLMCoxaPin        19 // Middle Left leg Hip Horizontal
#define cLMFemurPin       20 // Middle Left leg Hip Vertical
#define cLMTibiaPin       21 // Middle Left leg Knee

#define cLMFCoxaPin       22 // Middle Left leg Hip Horizontal
#define cLMFFemurPin      23 // Middle Left leg Hip Vertical
#define cLMFTibiaPin      24 // Middle Left leg Knee

#define cLFCoxaPin        25 // Front Left leg Hip Horizontal
#define cLFFemurPin       26 // Front Left leg Hip Vertical
#define cLFTibiaPin       27 // Front Left leg Knee

#define LPedCoxaPin       28 // Secondary Appendages
#define LPedFemurPin      29 // Secondary Appendages
#define LPedTibiaPin      30 // Secondary Appendages

//#define                   31
//--------------------------------------------------------------------
//[MIN/MAX ANGLES]   
// Mechanical limits 
#define cRRCoxaMin1     -300 
#define cRRCoxaMax1      300
#define cRRFemurMin1    -300
#define cRRFemurMax1     300
#define cRRTibiaMin1    -900
#define cRRTibiaMax1     900

#define cRMCoxaMin1     -300 
#define cRMCoxaMax1      300
#define cRMFemurMin1    -300
#define cRMFemurMax1     300
#define cRMTibiaMin1    -900
#define cRMTibiaMax1     900

#define cRMFCoxaMin1    -300 
#define cRMFCoxaMax1     300
#define cRMFFemurMin1   -300
#define cRMFFemurMax1    300
#define cRMFTibiaMin1   -900
#define cRMFTibiaMax1    900

#define cRFCoxaMin1     -300 
#define cRFCoxaMax1      300
#define cRFFemurMin1    -300
#define cRFFemurMax1     300
#define cRFTibiaMin1    -900
#define cRFTibiaMax1     900
//-------------
#define cLRCoxaMin1     -300 
#define cLRCoxaMax1      300
#define cLRFemurMin1    -300
#define cLRFemurMax1     300
#define cLRTibiaMin1    -900
#define cLRTibiaMax1     900

#define cLMCoxaMin1     -300 
#define cLMCoxaMax1      300
#define cLMFemurMin1    -900
#define cLMFemurMax1     300
#define cLMTibiaMin1    -900
#define cLMTibiaMax1     900

#define cLMFCoxaMin1    -300 
#define cLMFCoxaMax1     300
#define cLMFFemurMin1   -300
#define cLMFFemurMax1    300
#define cLMFTibiaMin1   -900
#define cLMFTibiaMax1    900

#define cLFCoxaMin1     -300 
#define cLFCoxaMax1      300
#define cLFFemurMin1    -300
#define cLFFemurMax1     300
#define cLFTibiaMin1    -900
#define cLFTibiaMax1     900

#define RPedCoxaMin      900
#define RPedCoxaMax      900
#define RPedFemurMin     900
#define RPedFemurMax     900
#define RPedTibiaMin     900
#define RPedTibiaMax     900

#define LPedCoxaMin      300
#define LPedCoxaMax      300
#define LPedFemurMin     300
#define LPedFemurMax     300
#define LPedTibiaMin     300
#define LPedTibiaMax     300
//--------------------------------------------------------------------
//[BODY DIMENSIONS] - Cartesian Coordinate System when standing
// Pedipalps (Secondary Appendages)
#define PedCoxaLength     30 // Direct measurement between coxa and femur (hip)
#define PedFemurLength    74 // Direct measurement between femur and tibia (knee)
#define PedTibiaLength   110 // Direct measurement between tibia and tar (foot)
// Legs
#define cCoxaLength       52 // Direct measurement between coxa and femur (hip)
#define cFemurLength     143 // Direct measurement between femur and tibia (knee)
#define cTibiaLength     252 // Direct measurement between tibia and tar (foot)
//-------------
#define cRRCoxaAngle1   -450 // Default Coxa setup angle relative to the body X axis
#define cRMCoxaAngle1    200 // Default Coxa setup angle relative to the body X axis
#define cRMFCoxaAngle1     0 // Default Coxa setup angle relative to the body X axis
#define cRFCoxaAngle1    450 // Default Coxa setup angle relative to the body X axis
#define cLRCoxaAngle1   -450 // Default Coxa setup angle relative to the body X axis
#define cLMCoxaAngle1    200 // Default Coxa setup angle relative to the body X axis
#define cLMFCoxaAngle1     0 // Default Coxa setup angle relative to the body X axis
#define cLFCoxaAngle1    450 // Default Coxa setup angle relative to the body X axis

#define RPedAngle1       900 // Default Coxa setup angle relative to the body X axis 
#define LPedAngle1       900 // Default Coxa setup angle relative to the body X axis 

//-------------
#define cRROffsetX       -77 // Distance X from center of the body to the Right Rear coxa
#define cRROffsetZ       100 // Distance Z from center of the body to the Right Rear coxa
#define cRMOffsetX      -138 // Distance X from center of the body to the Right Middle coxa
#define cRMOffsetZ         2 // Distance Z from center of the body to the Right Middle coxa
#define cRMFOffsetX     -156 // Distance X from center of the body to the Right Middle coxa
#define cRMFOffsetZ     -130 // Distance Z from center of the body to the Right Middle coxa
#define cRFOffsetX      -118 // Distance X from center of the body to the Right Front coxa
#define cRFOffsetZ      -238 // Distance Z from center of the body to the Right Front coxa
#define RPedOffsetX      -54 // Distance X from center of the body to the Right Front Pedi coxa
#define RPedOffsetZ     -310 // Distance Z from center of the body to the Right Front Pedi coxa

#define cLROffsetX        77 // Distance X from center of the body to the Left Rear coxa
#define cLROffsetZ       100 // Distance Z from center of the body to the Left Rear coxa
#define cLMOffsetX       138 // Distance X from center of the body to the Left Middle coxa
#define cLMOffsetZ         2 // Distance Z from center of the body to the Left Middle coxa
#define cLMFOffsetX      156 // Distance X from center of the body to the Left Middle coxa
#define cLMFOffsetZ     -130 // Distance Z from center of the body to the Left Middle coxa
#define cLFOffsetX       118 // Distance X from center of the body to the Left Front coxa
#define cLFOffsetZ      -238 // Distance Z from center of the body to the Left Front coxa
#define LPedOffsetX       54 // Distance X from center of the body to the Left Front Pedi coxa
#define LPedOffsetZ     -310 // Distance Z from center of the body to the Left Front Pedi coxa
//--------------------------------------------------------------------
//[START POSITIONS FEET] - Cartesian Coordinate System when folded
#define PediInitX          0 // Distance X from Pedi coxa to the tar
#define PediInitZ         54 // Distance Z from Pedi coxa to the tar

#define FrontTarInitX    140 // Distance X from coxa to the tar 
#define FrontTarInitZ    140 // Distance Z from coxa to the tar

#define FrontMidTarInitX 200 // Distance X from coxa to the tar 
#define FrontMidTarInitZ   0 // Distance Z from coxa to the tar 

#define RearMidTarInitX  180 // Distance X from coxa to the tar   
#define RearMidTarInitZ   85 // Distance Z from coxa to the tar 

#define RearTarInitX     100 // Distance X from coxa to the tar 
#define RearTarInitZ     175 // Distance Z from coxa to the tar 

#define HightInitY	      10 // Distance Y from tar to floor 

//-------------
#define cRRInitPosX  -RearTarInitX         
#define cRRInitPosY   HightInitY
#define cRRInitPosZ   RearTarInitZ

#define cRMInitPosX  -RearMidTarInitX      
#define cRMInitPosY   HightInitY
#define cRMInitPosZ   RearMidTarInitZ

#define cRMFInitPosX -FrontMidTarInitX    
#define cRMFInitPosY  HightInitY
#define cRMFInitPosZ -FrontMidTarInitZ

#define cRFInitPosX  -FrontTarInitX    
#define cRFInitPosY   HightInitY
#define cRFInitPosZ  -FrontTarInitZ

#define RPedInitPosX -PediInitX           
#define RPedInitPosY  HightInitY
#define RPedInitPosZ -PediInitZ

#define cLRInitPosX   RearTarInitX       
#define cLRInitPosY   HightInitY
#define cLRInitPosZ   RearTarInitZ

#define cLMInitPosX   RearMidTarInitX      
#define cLMInitPosY   HightInitY
#define cLMInitPosZ   RearMidTarInitZ

#define cLMFInitPosX  FrontMidTarInitX    
#define cLMFInitPosY  HightInitY
#define cLMFInitPosZ -FrontMidTarInitZ
        
#define cLFInitPosX   FrontTarInitX     
#define cLFInitPosY   HightInitY
#define cLFInitPosZ  -FrontTarInitZ         

#define LPedInitPosX  PediInitX       
#define LPedInitPosY  HightInitY
#define LPedInitPosZ -PediInitZ

//--------------------------------------------------------------------
