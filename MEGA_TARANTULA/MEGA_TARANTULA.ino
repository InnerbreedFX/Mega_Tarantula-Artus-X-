//  A.R.T.U.S - X [TARANTULA]
//  Articulated Robotic Tarantula Universal Simulation - 10

// FOR ARDUINO MEGA running IDE1.8.7 with SSC32_U
//==========================================================================
// Programmers: Jeroen Janssen (aka Xan) & Kåre Halvorsen (aka Zenta)
// Developer:   Jonny Poole (aka Innerbreed)

// Project file order:
// 1. MEGA_TARANTULA.ino (this one)
// 2. Tarantula_Cfg.h
// 3. Tarantula_Globals.h
// 4. PS2X_lib.cpp 
// 5. PS2X_lib.h   
// 6. PS2_controller.cpp   

//==========================================================================
// Engineer / Developers notes:
// ==============================
// PS2 Receiver Connections:
//    PS2 | MEGA
//    DAT - PIN 05
//    CMD - PIN 06
//    ATT - PIN 07
//    CLK - PIN 08
//    VCC - 5V
//    GND - GND
//----------------------
// SSC > MEGA Connections:
//    SSC32 | MEGA
//    RX    - PIN 19
//    TX    - PIN 18
//    GND   - GND
//----------------------
// SCREEN > MEGA Connection:
//    SCREEN | MEGA
//    SDA    - PIN 20 
//    SCK    - PIN 21
//    VCC    - 3.3v
//    GND    - GND
//----------------------
// Additional > SSC / MEGA Connections:
//    NAME            SSC   |   MEGA
//--------------------------|-----------
//    EYE LEDs              -   PIN 
//    AbdomenPitch          -   PIN 02                                 
//    AbdomenTaw            -   PIN 03                 
//    PediTilt              -   PIN 09 
//    RRCoxaPin      PIN 00 -  
//    RRFemurPin     PIN 01 - 
//    RRTibiaPin     PIN 02 - 
//    RMCoxaPin      PIN 03 - 
//    RMFemurPin     PIN 04 -  
//    RMTibiaPin     PIN 05 - 
//    RMFCoxaPin     PIN 06 - 
//    RMFFemurPin    PIN 07 - 
//    RMFTibiaPin    PIN 08 - 
//    RFCoxaPin      PIN 09 - 
//    RFFemurPin     PIN 10 - 
//    RFTibiaPin     PIN 11 - 
//    RPedCoxaPin    PIN 12 -  
//    RPedFemurPin   PIN 13 - 
//    RPedTibiaPin   PIN 14 - 
//       FREE        PIN 15 - 
//    LRCoxaPin      PIN 16 - 
//    LRFemurPin     PIN 17 - 
//    LRTibiaPin     PIN 18 - 
//    LMCoxaPin      PIN 19 - 
//    LMFemurPin     PIN 20 - 
//    LMTibiaPin     PIN 21 - 
//    LMFCoxaPin     PIN 22 - 
//    LMFFemurPin    PIN 23 - 
//    LMFTibiaPin    PIN 24 - 
//    LFCoxaPin      PIN 25 - 
//    LFFemurPin     PIN 26 - 
//    LFTibiaPin     PIN 27 - 
//    LPedCoxaPin    PIN 28 - 
//    LPedFemurPin   PIN 29 - 
//    LPedTibiaPin   PIN 30 - 
//       FREE        PIN 31 - 
//==========================================================================
//PS2 CONTROLS:
//----------------------
//[Common Controls]
//- Start            - Turn on/off 
//- Select           - Switch gaits/Switch legs (Mode dependant)
//- L1               - Toggle Shift mode
//- L2               - Toggle Rotate mode
//- L3               - Hold for additional Rotate mode
//- R1               - Toggle Double gait travel speed
//- R2               - Toggle Double gait step length
//- R3               - Toggle walk modes 1-2
//- Cross            - Disabled
//- Circle           - Toggle Single leg mode
//  Square           - Toggle Balance mode
//- Triangle         - Move body to 35 mm from the ground (Walk pos) and back to the ground
//- D-Pad            - Up Body up 10 mm
//- D-Pad            - Down Body down 10 mm
//- D-Pad            - Left decrease speed with 50mS
//- D-Pad            - Right increase speed with 50mS
//- Left Stick       - (Walk mode 1) Walk/Strafe               (Walk mode 2) Disable
//- Right Stick      - (Walk mode 1) Rotate body/Rotate Pedi's (Walk mode 2) Walk/Rotate body
//----------------------
//[Walk Controls]      R3
//Walk method 1:
//- Left Stick       - Walk/Strafe
//- Right Stick      - Rotate body/Rotate Pedi's

//Walk method 2:
//- Left Stick       - Disabled
//- Right Stick      - Walk/Rotate

//- Select             Switch gaits
//- R1               - Toggle Double gait travel speed
//- R2               - Toggle Double gait step length
//- R3               - Toggle walk mode
//----------------------
//[Shift Controls]     L1
//- Left Stick       - Shift body X/Z
//- Right Stick      - Shift body Y and rotate body Y
//----------------------
//[Rotate Controls]    L2
//- Left Stick       - Rotate body X/Z
//- Right Stick      - Rotate body Y
//----------------------
//[Rotate Controls]    L3 Hold
//- Left Stick       - Rotate body X/Z
//- Right Stick      - Rotate body Y
//- Adjust CPR_Z     - Adjust Z value for centerpoint of rotation (BodyRotOffsetZ)

//----------------------
//[Single leg Control] Circle
//- select           - Switch legs
//- Left Stick       - Move Leg X/Z (relative)
//- Right Stick      - Move Leg Y (absolute)
//- R2Hold/release   - leg position hold position
//----------------------
//[GP Player Controls] Cross NOT USED!
//- Select           - Disabled
//- R2               - Disabled
//
//==========================================================================

//(BODY INVERSE KINEMATICS) 
// BodyRotX          - Global Input pitch of the body 
// BodyRotY          - Global Input rotation of the body 
// BodyRotZ          - Global Input roll of the body 
// SinB              - Sin buffer for BodyRotX
// CosB              - Cos buffer for BodyRotX
// SinG              - Sin buffer for BodyRotZ
// CosG              - Cos buffer for BodyRotZ
// RotationY         - Input Rotation for the gait 
// PosX              - Input position of the feet X 
// PosZ              - Input position of the feet Z 
// BodyIKPosX        - Output Position X of feet with Rotation 
// BodyIKPosY        - Output Position Y of feet with Rotation 
// BodyIKPosZ        - Output Position Z of feet with Rotation

//[LEG INVERSE KINEMATICS] Calculates the angles of the coxa, femur and tibia for the given position of the feet
// IKFeetPosX        - Input position of the Feet X
// IKFeetPosY        - Input position of the Feet Y
// IKFeetPosZ        - Input Position of the Feet Z
// IKSolution        - Output true if the solution is possible
// IKSolutionWarning - Output true if the solution is NEARLY possible
// IKSolutionError   - Output true if the solution is NOT possible
// FemurAngle1       - Output Angle of Femur in degrees
// TibiaAngle1       - Output Angle of Tibia in degrees
// CoxaAngle1        - Output Angle of Coxa in degrees
//==========================================================================
/* 
Cartesian Coordinate System
   X = Left   and -X = Right
   Y = Down   and -Y = Up
   Z = Back   and -Z = Forward

        ISO View            SIDE View           TOP View    
           -Y (up)             -Y (up)                                
            |  Z (back)         |                   Z (back)          
            | /                 |                   |     
            |/                  |                   |        
 (R) -X ----0---- X (L)  -Z ----0---- Z  (R) -X ----0---- X (L)  
           /|           (front) |  (back)           |      
          / |                   |                   |       
(front) -Z  |                   |           (front) -Z                 
           Y (down)            Y (down)                                 

*/

//==========================================================================
// Header Files
//==========================================================================
#include <pins_arduino.h>
#include "Tarantula_globals.h"
#include <SPI.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2

#define LOGO16_GLCD_HEIGHT 16 
#define LOGO16_GLCD_WIDTH  16 
static const unsigned char PROGMEM logo16_glcd_bmp[] =
{ B00000000, B11011100,
  B00000001, B11011100,
  B00000001, B11001100,
  B00000011, B11100100,
  B11110011, B11100000,
  B11111110, B11111000,
  B01111110, B11111111,
  B00110011, B10011111,
  B00011111, B11111100,
  B00001101, B01110000,
  B00011011, B10100000,
  B00111111, B11100000,
  B00111111, B11110000,
  B01111100, B11110000,
  B01110000, B01110000,
  B00000000, B00110000 };

#if (SSD1306_LCDHEIGHT != 32)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

#define BalanceDivFactor 10  // It is possible to use other values (which make the correct mathematical balance). 
                             // Using a lower value like 3 the balance mode gets much more visible and “alive”. 
                             // But can also give some unpredicted result under extreme poses (especial under single leg control). 
                             
//#define DEBUG_PINCALCS
//#define DEBUG_BODYCALCS
//#define DEBUG_GAITS

//[ANGLES]
short     CoxaAngle1[10];    // Actual Angle of the horizontal hip, decimals = 1
short     FemurAngle1[10];   // Actual Angle of the vertical hip, decimals = 1
short     TibiaAngle1[10];   // Actual Angle of the knee, decimals = 1
//--------------------------------------------------------------------
//[POSITIONS SINGLE LEG CONTROL]
boolean   fSLHold ;         // Single leg control mode

short     LegPosX[10];      // Actual X Posion of the Leg
short     LegPosY[10];      // Actual Y Posion of the Leg
short     LegPosZ[10];      // Actual Z Posion of the Leg
//--------------------------------------------------------------------
//[INPUTS]

//--------------------------------------------------------------------
//[GP PLAYER]
boolean    GPStart;         // Start the GP Player
byte       GPSeq;           // Number of the sequence
boolean    GPEnable;        // Enables the GP player when the SSC version ends with "GP<cr>"
//--------------------------------------------------------------------
//[OUTPUTS]
boolean    LedA;            // Red
boolean    LedB;            // Green
boolean    LedC;            // Orange
//--------------------------------------------------------------------
//[VARIABLES]
byte      Index;            // Index universally used
byte      LegIndex;         // Index used for leg Index Number

//GetSinCos / ArcCos
short     AngleDeg1;        // Input Angle in degrees, decimals = 1
short     sin4;             // Output Sinus of the given Angle, decimals = 4
short     cos4;             // Output Cosinus of the given Angle, decimals = 4
short     AngleRad4;        // Output Angle in radials, decimals = 4

//GetAtan2
short     AtanX;            // Input X
short     AtanY;            // Input Y
short     Atan4;            // ArcTan2 output
short     XYhyp2;           // Output presenting Hypotenuse of X and Y

//Body position
long      BodyPosX;         // Global Input for the position of the body
long      BodyPosY; 
long      BodyPosZ; 

//Body Inverse Kinematics
long      BodyRotX1;        // Global Input pitch of the body
long      BodyRotY1;        // Global Input rotation of the body
long      BodyRotZ1;        // Global Input roll of the body
short     PosX;             // Input position of the feet X
short     PosZ;             // Input position of the feet Z
short     PosY;             // Input position of the feet Y
long      TotalX;           // Total X distance between the center of the body and the feet
long      TotalZ;           // Total Z distance between the center of the body and the feet
long      BodyIKPosX;       // Output Position X of feet with Rotation
long      BodyIKPosY;       // Output Position Y of feet with Rotation
long      BodyIKPosZ;       // Output Position Z of feet with Rotation
short     BodyRotOffsetY;   // Input Y offset value to adjust centerpoint of rotation
short     BodyRotOffsetZ;   // Input Z offset value to adjust centerpoint of rotation

//Leg Inverse Kinematics
long      IKFeetPosX;       // Input position of the Feet X
long      IKFeetPosY;       // Input position of the Feet Y
long      IKFeetPosZ;       // Input Position of the Feet Z
boolean   IKSolution;       // Output true if the solution is possible
boolean   IKSolutionWarning;// Output true if the solution is NEARLY possible
boolean   IKSolutionError;  // Output true if the solution is NOT possible
//--------------------------------------------------------------------
//[TIMING]
unsigned long lTimerStart;  // Start time of the calculation cycles
unsigned long lTimerEnd;    // End time of the calculation cycles
byte      CycleTime;        // Total Cycle time

word      SSCTime;          // Time for servo updates
word      PrevSSCTime;      // Previous time for the servo updates

byte      InputTimeDelay;   // Delay that depends on the input to get the "sneaking" effect
word      SpeedControl;     // Adjustible Delay
//--------------------------------------------------------------------
//[GLOABAL]
boolean   TarantulaOn;        // Switch to turn on TARANTULA
boolean   Prev_TarantulaOn;   // Previous loop state 
//--------------------------------------------------------------------
//[Balance]
boolean   BalanceMode;
long      TotalTransX;
long      TotalTransZ;
long      TotalTransY;
long      TotalYBal1;
long      TotalXBal1;
long      TotalZBal1;

//[Pedi control]
long      PediRotation; 

//[Single Leg Control]
byte      SelectedLeg;
byte      Prev_SelectedLeg;
short     SLLegX;
short     SLLegY;
short     SLLegZ;
boolean   AllDown;

//[gait]
byte      GaitType;       // Gait type
short     NomGaitSpeed;   // Nominal speed of the gait

short     LegLiftHeight;  // Current Travel height
long      TravelLengthX;  // Current Travel length X
long      TravelLengthZ;  // Current Travel length Z
long      TravelRotationY;// Current Travel Rotation Y

short     TLDivFactor;    // Number of steps that a leg is on the floor while walking
short     NrLiftedPos;    // Number of positions that a single leg is lifted [1-5]
boolean   HalfLiftHeigth; // If TRUE the outer positions of the ligted legs will be half height    

boolean   GaitInMotion;   // Temp to check if the gait is in motion
byte      StepsInGait;    // Number of steps in gait
boolean   LastLeg;        // TRUE when the current leg is the last leg of the sequence
byte      GaitStep;       // Actual Gait step

byte      GaitLegNr[10];  // Init position of the leg
byte      GaitLegNrIn;    // Input Number of the leg

long      GaitPosX[10];   // Array containing Relative X position corresponding to the Gait
long      GaitPosY[10];   // Array containing Relative Y position corresponding to the Gait
long      GaitPosZ[10];   // Array containing Relative Z position corresponding to the Gait
long      GaitRotY[10];   // Array containing Relative Y rotation corresponding to the Gait

boolean fWalking;         // True if the robot is walking
boolean fContinueWalking; // should we continue to walk?

//==========================================================================
// Function prototypes
//==========================================================================
extern void    GaitSelect(void);
extern boolean FIsSSCGPEnabled(void);
extern void    SingleLegControl(void);
extern void    GaitSeq(void);
extern void    ServoDriverStart(void);
extern void    ServoDriverCommit(void);
extern void    FreeServos(void);
extern void    BalanceBody(void);
extern void    CheckAngles();

extern void BalCalcOneLeg (short PosX, short PosZ, short PosY, byte BalLegNr);
extern void BodyIK        (short PosX, short PosZ, short PosY, short RotationY, byte BodyIKLeg) ;
extern void LegIK         (short IKFeetPosX, short IKFeetPosY, short IKFeetPosZ, byte LegIKLegNr);
extern void Gait          (byte GaitCurrentLegNr);
extern short GetATan2     (short AtanX, short AtanY);

//-----------------------------------------------------------------------
Servo PediTilt;
Servo AbdomenYaw;
Servo AbdomenPitch; 

int ValPediTilt =0; 
#define soundpin A0 

//Foot Sensors:
const int RR_TA   =22;               // Right Rear leg
const int RM_TA   =24;               // Right Middle leg
const int RMF_TA  =26;               // Right Middle Front leg
const int RF_TA   =28;               // Right Front leg
const int RP_TA   =30;               // Right Pedi
const int LR_TA   =32;               // Left Rear leg
const int LM_TA   =34;               // Left Middle leg
const int LMF_TA  =36;               // Left Middle Front leg
const int LF_TA   =38;               // Left Front leg
const int LP_TA   =40;               // Left Pedi

const int SensorCount = 10;
int TarsToGround[SensorCount]   = {RR_TA, RM_TA, RMF_TA, RF_TA, RP_TA, LR_TA, LM_TA, LMF_TA, LF_TA, LP_TA};

int TA_Val;

//-----------------------------------------------------------------------
// SETUP: the main arduino setup function.
//-----------------------------------------------------------------------
//=======================================================================
void setup(){

    pinMode(RR_TA,  INPUT);
    pinMode(RM_TA,  INPUT);
    pinMode(RMF_TA, INPUT);
    pinMode(RF_TA,  INPUT);
    pinMode(RP_TA,  INPUT);
    pinMode(LR_TA,  INPUT);
    pinMode(LM_TA,  INPUT);
    pinMode(LMF_TA, INPUT);
    pinMode(LF_TA,  INPUT);
    pinMode(LP_TA,  INPUT);
        
       while (!Serial)
    {
    }
         pinMode(soundpin, OUTPUT);   // Speaker  
         tone(soundpin, 532, 200);  
Serial.begin (115200);
  Serial.print (" Connecting Disply ...");
  byte count = 0;
 
  Wire.begin();
  for (byte i = 1; i < 120; i++)
  {
    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0)
      {

      count++;
      delay (1); 
      } 
  } 

  if (count =0)
  {Serial.println(" UnSuccessful");}
  else
  {Serial.println(" DONE");}
  
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
  display.display();
  display.clearDisplay();
  
  // draw mulitple circles
  testdrawcircle();
  display.display();
  delay(2000);
  display.clearDisplay();
  
    DBGSerial.begin(115200);
    delay(1000); 
    
    SSCSerial.begin(38400);
   //Checks SSC version number if it ends with "GP"
   //enable the GP player if it does
    
    delay(10);
    GPEnable = FIsSSCGPEnabled();
       
        //Initialize Controller
    Serial.print(" Connecting PS2 ...");
    InitController();
    delay(1000);
        
    //Tars Init Positions
    for (LegIndex= 0; LegIndex <= 9; LegIndex++ )
    {
        LegPosX[LegIndex] = cInitPosX[LegIndex];    //Set start positions for each leg
        LegPosY[LegIndex] = cInitPosY[LegIndex];
        LegPosZ[LegIndex] = cInitPosZ[LegIndex];  
    }
    // Single leg control. Make sure no leg is selected
    SelectedLeg = 255; // No Leg selected
    Prev_SelectedLeg = 255;
 
    // Body Positions
    BodyPosX = 0;
    BodyPosY = 0;
    BodyPosZ = 0;
 
    // Body Rotations
    BodyRotX1 = 0;
    BodyRotY1 = 0;
    BodyRotZ1 = 0;
    BodyRotOffsetY = 0;        //Input Y offset value to adjust centerpoint of rotation
    BodyRotOffsetZ = 0;
 
    // Gait
    GaitType = 0;
    BalanceMode = 0;
    LegLiftHeight = 50;
    GaitStep = 1;
    GaitSelect();
    delay(15);
    Serial.println("Starting up Robot");
    delay(150);
    //SSC
    SSCTime = 150;
    TarantulaOn = 0;

    delay(15);
    
     PediTilt.attach(9);
     AbdomenYaw.attach(3);
     AbdomenPitch.attach(2); 
   ControlInput();    // Read from PS2 
//     simulateStart(); // Initializes servos
}

//===================================================================
// Debug by LensDigital: Emulate Start button press
//void simulateStart() {
//  static unsigned    g_BodyYOffset; 
//  static char        g_BodyYSift;
//  DBGPrintf("Simulate Start button Pressed\r\n");
//            if (TarantulaOn) {
//                //Turn off
//                BodyPosX = 0;
//                BodyPosY = 0;
//                BodyPosZ = 0;
//                BodyRotX1 = 0;
//                BodyRotY1 = 0;
//                BodyRotZ1 = 0;
//                TravelLengthX = 0;
//                TravelLengthZ = 0;
//                TravelRotationY = 0;
//                PediRotation = 0;
//                g_BodyYOffset = 0;
//                g_BodyYSift = 0;
//                SelectedLeg = 255;
//                TarantulaOn = 0;
//            } else {
//                //Turn on
//                TarantulaOn = 1;
//            }
//            //Calculate BodyPosY
//        BodyPosY = max(g_BodyYOffset + g_BodyYSift,  0);
//}

//===================================================================
// Loop: the main Arduino Loop function
//===================================================================
void loop(void)
{        
  //Start time
  lTimerStart = millis();

  //Read input
  ControlInput();        // Read from PS2
     Move_Abdomen();     // Ativate Abdomen control
     Move_Pedi();        // Activate Pedi control
 //    TerrainAdaption();
#ifdef DEBUG_GAITS
  // If we are in this mode, lets normalize the inputs as to make it easier to compare later...
  DebugLimitJoysticks(&TravelLengthX);
  DebugLimitJoysticks(&TravelLengthZ);
  DebugLimitJoysticks(&TravelRotationY);
  DebugLimitJoysticks(&BodyPosX);
  DebugLimitJoysticks(&BodyPosZ);
  DebugLimitJoysticks(&BodyRotY1);
  DebugLimitJoysticks(&BodyRotX1);
  DebugLimitJoysticks(&BodyRotZ1);

  // Try to also normalize BodyposY
  BodyPosY &= 0xFFF8;  // Pointers store the location of a value, rather than the value 6
#endif DEBUG_GAITS

  // Some debug stuff...
#ifdef DEBUG_GAITS
  if (TarantulaOn && (TravelLengthX || TravelLengthZ))
  {
    DBGPrintf("Gait: %d %d BP:(%d, %d, %d) TL: (%d, %d), TRY: %d\n\r",
              GaitType, GaitStep,
              (short)BodyPosX, (short)BodyPosY, (short)BodyPosZ,
              (short)TravelLengthX, (short)TravelLengthZ, (short)TravelRotationY);
  }
#endif

  SingleLegControl (); //Single leg control
  GaitSeq();           //Gait
  
  //Balance calculations
  TotalTransX = 0;     //reset values used for calculation of balance
  TotalTransZ = 0;
  TotalTransY = 0;
  TotalXBal1 = 0;
  TotalYBal1 = 0;
  TotalZBal1 = 0;
  if (BalanceMode) {
    for (LegIndex = 0; LegIndex <= 4; LegIndex++) {    // balance calculations for all Right legs
#ifdef DEBUG_GAITS
      if (TarantulaOn && (TravelLengthX || TravelLengthZ))   {
        DBGPrintf("GP? %d %d %d-", (short)GaitPosX[LegIndex], (short)GaitPosY[LegIndex], (short)GaitPosZ[LegIndex]);
      }
#endif
      BalCalcOneLeg (-LegPosX[LegIndex] + GaitPosX[LegIndex],
                      LegPosZ[LegIndex] + GaitPosZ[LegIndex],
                     (LegPosY[LegIndex] - cInitPosY[LegIndex]) + GaitPosY[LegIndex], LegIndex);
    }
    for (LegIndex = 5; LegIndex <= 9; LegIndex++) {    // balance calculations for all Right legs
#ifdef DEBUG_GAITS
      if (TarantulaOn && (TravelLengthX || TravelLengthZ))   {
        DBGPrintf("GP? %d %d %d-", (short)GaitPosX[LegIndex], (short)GaitPosY[LegIndex], (short)GaitPosZ[LegIndex]);
      }
#endif
      BalCalcOneLeg (LegPosX[LegIndex] + GaitPosX[LegIndex],
                     LegPosZ[LegIndex] + GaitPosZ[LegIndex],
                    (LegPosY[LegIndex] - cInitPosY[LegIndex]) + GaitPosY[LegIndex], LegIndex);
    }
    BalanceBody();
  }

  //Reset IKsolution indicators
  IKSolution = 0 ;
  IKSolutionWarning = 0;
  IKSolutionError = 0 ;

  //Do IK for all Right legs
  for (LegIndex = 0; LegIndex <= 4; LegIndex++) {
    BodyIK(-LegPosX[LegIndex] + BodyPosX + GaitPosX[LegIndex] - TotalTransX,
            LegPosZ[LegIndex] + BodyPosZ + GaitPosZ[LegIndex] - TotalTransZ,
            LegPosY[LegIndex] + BodyPosY + GaitPosY[LegIndex] - TotalTransY,
            GaitRotY[LegIndex], LegIndex);

    LegIK  (LegPosX[LegIndex] - BodyPosX + BodyIKPosX - (GaitPosX[LegIndex] - TotalTransX),
            LegPosY[LegIndex] + BodyPosY - BodyIKPosY + GaitPosY[LegIndex] - TotalTransY,
            LegPosZ[LegIndex] + BodyPosZ - BodyIKPosZ + GaitPosZ[LegIndex] - TotalTransZ, LegIndex);
  }

  //Do IK for all Left legs
  for (LegIndex = 5; LegIndex <= 9; LegIndex++) {
    BodyIK (LegPosX[LegIndex] - BodyPosX + GaitPosX[LegIndex] - TotalTransX,
            LegPosZ[LegIndex] + BodyPosZ + GaitPosZ[LegIndex] - TotalTransZ,
            LegPosY[LegIndex] + BodyPosY + GaitPosY[LegIndex] - TotalTransY,
            GaitRotY[LegIndex], LegIndex);
    LegIK  (LegPosX[LegIndex] + BodyPosX - BodyIKPosX + GaitPosX[LegIndex] - TotalTransX,
            LegPosY[LegIndex] + BodyPosY - BodyIKPosY + GaitPosY[LegIndex] - TotalTransY,
            LegPosZ[LegIndex] + BodyPosZ - BodyIKPosZ + GaitPosZ[LegIndex] - TotalTransZ, LegIndex);
  }

  //Check mechanical limits
  CheckAngles();

  //Write IK errors to leds
  LedC = IKSolutionWarning;
  LedA = IKSolutionError;

  //Drive Servos
  if (TarantulaOn) {

    //Set SSC time
    if ((abs(PediRotation) > cTravelDeadZone) || (abs(TravelLengthX) > cTravelDeadZone) || (abs(TravelLengthZ) > cTravelDeadZone) ||
        (abs(TravelRotationY * 2) > cTravelDeadZone)) {
      SSCTime = NomGaitSpeed + (InputTimeDelay * 2) + SpeedControl;

      //Add aditional delay when Balance mode is on
      if (BalanceMode)
        SSCTime = SSCTime + 100;
    } else //Movement speed excl. Walking
      SSCTime = 200 + SpeedControl;

    // note we broke up the servo driver into start/commit that way we can output all of the servo information
    // before we wait and only have the termination information to output after the wait.  That way we hopefully
    // be more accurate with our timings...
    ServoDriverStart();

    // Sync MEGA with SSC while walking to ensure the prev is completed before sending the next one

    fContinueWalking = false;

    // Finding any incident of GaitPos/Rot <>0:
    for (LegIndex = 0; LegIndex <= 9; LegIndex++) {
      if ( (GaitPosX[LegIndex] > 2) || (GaitPosX[LegIndex] < -2)
           || (GaitPosY[LegIndex] > 2) || (GaitPosY[LegIndex] < -2)
           || (GaitPosZ[LegIndex] > 2) || (GaitPosZ[LegIndex] < -2)
           || (GaitRotY[LegIndex] > 2) || (GaitRotY[LegIndex] < -2) )    {
        fContinueWalking = true;
        break;
      }
    }
    if (fWalking || fContinueWalking) {
      word  wDelayTime;
      fWalking = fContinueWalking;

      //Get endtime and calculate wait time
      lTimerEnd = millis();
      if (lTimerEnd > lTimerStart)
        CycleTime = lTimerEnd - lTimerStart;
      else
        CycleTime = 0xffffffffL - lTimerEnd + lTimerStart + 1;

      // if it is less, use the last cycle time...
      //Wait for previous commands to be completed while walking
      wDelayTime = (min(max ((PrevSSCTime - CycleTime), 1), NomGaitSpeed));
      //            DBGPrintf("Delay: %d %d %d %d\n\r", (word)NomGaitSpeed, (word)CycleTime, (word)PrevSSCTime, (word)wDelayTime);
      digitalWrite(3, HIGH);
      delay (wDelayTime);
      digitalWrite(3, LOW);
    }

    ServoDriverCommit();
    digitalWrite(2, LOW);

  } else {
    //Turn the bot off
    if (Prev_TarantulaOn || (AllDown = 0)) {
      SSCTime = 600;
      ServoDriverStart();
      ServoDriverCommit();
      delay(600);
    } else {
      FreeServos();
    }
  }

  //Store previous TarantulaOn State
  if (TarantulaOn)
    Prev_TarantulaOn = 1;
  else
    Prev_TarantulaOn = 0;
}

//===================================================================
//===================================================================
#ifdef DEBUG_GAITS 
void DebugLimitJoysticks(long int *pDLJVal)
{
    // If we are in this mode, lets normalize the inputs as to make it easier to compare later...
     if (*pDLJVal < -64)
         *pDLJVal = -127;
else if( *pDLJVal > 64)
         *pDLJVal = 127;
else
         *pDLJVal = 0;
}
#endif

//--------------------------------------------------------------------
//[ReadButtons] Reading input buttons from the MEGA
//--------------------------------------------------------------------
void ReadButtons(void)
{
}

//--------------------------------------------------------------------
//[GP PLAYER]
//--------------------------------------------------------------------
boolean FIsSSCGPEnabled(void)
{
    char abVer[40];        // give a nice large buffer.
    byte cbRead;
    SSCSerial.print("ver\r");
    
    cbRead = SSCRead((byte*)abVer, sizeof(abVer), 10000, 13);

    DBGPrintf("Check GP Enable - cb %d\r", cbRead);
    if (cbRead > 0) {
        byte iT;
        for (iT = 0; iT < cbRead; iT++)
            DBGPrintf("%2x ", abVer[iT]);
        DBGSerial.write((byte*)abVer, cbRead);
    }
        
    if ((cbRead > 3) && (abVer[cbRead-3]=='G') && (abVer[cbRead-2]=='P') && (abVer[cbRead-1]==13))
        return true;

    return false;
}

//--------------------------------------------------------------------
//[SINGLE LEG CONTROL]
void SingleLegControl(void)
{  
  //Check if all legs are down
    AllDown = (LegPosY[cRP]==cInitPosY[cRP]) && (LegPosY[cRF]==cInitPosY[cRF]) && (LegPosY[cRM]==cInitPosY[cRM]) && (LegPosY[cRMF]==cInitPosY[cRMF]) && (LegPosY[cRR]==cInitPosY[cRR]) & 
              (LegPosY[cLP]==cInitPosY[cLP]) && (LegPosY[cLR]==cInitPosY[cLR]) && (LegPosY[cLM]==cInitPosY[cLM]) && (LegPosY[cLMF]==cInitPosY[cLMF]) && (LegPosY[cLF]==cInitPosY[cLF]);

    if (SelectedLeg<=9) {
    if (SelectedLeg!=Prev_SelectedLeg) {
    if (AllDown) {                      // Lift leg a bit when it got selected
        LegPosY[SelectedLeg] = cInitPosY[SelectedLeg]-20; tone(soundpin, 132, 200);    
                                             
                Prev_SelectedLeg = SelectedLeg; // Store current status              
 } else {                              
                LegPosX[Prev_SelectedLeg] = cInitPosX[Prev_SelectedLeg]; // Return prev leg back to the init position
                LegPosY[Prev_SelectedLeg] = cInitPosY[Prev_SelectedLeg];
                LegPosZ[Prev_SelectedLeg] = cInitPosZ[Prev_SelectedLeg];
            }
 } else if (!fSLHold) { 
                LegPosY[SelectedLeg] = LegPosY[SelectedLeg]+SLLegY;
                LegPosX[SelectedLeg] = cInitPosX[SelectedLeg]+SLLegX;
                LegPosZ[SelectedLeg] = cInitPosZ[SelectedLeg]+SLLegZ;     
        }} else {                            
     if (!AllDown) {  //All legs to init position
            for(LegIndex = 0; LegIndex <= 9;LegIndex++) {
                LegPosX[LegIndex] = cInitPosX[LegIndex];
                LegPosY[LegIndex] = cInitPosY[LegIndex];
                LegPosZ[LegIndex] = cInitPosZ[LegIndex];
            }
        } 
     if (Prev_SelectedLeg!=255)
         Prev_SelectedLeg = 255;
    }
}

//--------------------------------------------------------------------
void GaitSelect(void)    
{   tone(soundpin, 1032, 200); 
  display.setTextSize(1); 
  display.setCursor(0,0);
  display.setTextColor(WHITE); 
  display.println ("Stepping pattern of a"); 
  display.setTextSize(2); 
//----------------
// Tarantula 16 steps
    if (GaitType == 0) {    display.println (" TARANTULA");   
//            Left                  Right      
        GaitLegNr[cLP] = 1;    GaitLegNr[cRP] = 8;
        GaitLegNr[cLF] = 8;    GaitLegNr[cRF] = 4;  
        GaitLegNr[cLMF]= 5;    GaitLegNr[cRMF]= 1; 
        GaitLegNr[cLM] = 3;    GaitLegNr[cRM] = 7;
        GaitLegNr[cLR] = 6;    GaitLegNr[cRR] = 2; 
        
             NrLiftedPos    = 3;
             HalfLiftHeigth = 0;
             TLDivFactor    = 6;      
             StepsInGait    = 16;    
             NomGaitSpeed   = 75;            
    }
//----------------
// Scorpion 16 steps
    if (GaitType == 1) {    display.println (" SCORPION");  tone(soundpin, 432, 200);    
//            Left                  Right      
        GaitLegNr[cLP] = 0;    GaitLegNr[cRP] = 0; 
        GaitLegNr[cLF] = 4;    GaitLegNr[cRF] = 8;  
        GaitLegNr[cLMF]= 7;    GaitLegNr[cRMF]= 3; 
        GaitLegNr[cLM] = 2;    GaitLegNr[cRM] = 6;
        GaitLegNr[cLR] = 5;    GaitLegNr[cRR] = 1; 

             NrLiftedPos    = 3;
             HalfLiftHeigth = 0;
             TLDivFactor    = 4;      
             StepsInGait    = 16;    
             NomGaitSpeed   = 90;       
    }
//----------------
// Ripple Gait 8 steps   
    if (GaitType == 2) {      display.println (" TICK");  tone(soundpin, 532, 200);
//            Left                  Right
        GaitLegNr[cLP] = 0;   GaitLegNr[cRP] = 0;
        GaitLegNr[cLF] = 6;   GaitLegNr[cRF] = 2;
        GaitLegNr[cLMF]= 4;   GaitLegNr[cRMF]= 8;
        GaitLegNr[cLM] = 3;   GaitLegNr[cRM] = 7;
        GaitLegNr[cLR] = 1;   GaitLegNr[cRR] = 5;

             NrLiftedPos    = 1;  
             HalfLiftHeigth = 0; 
             TLDivFactor    = 5; 
             StepsInGait    = 8;  
             NomGaitSpeed   = 100;
    }
//----------------

// Wave 18 steps
    if (GaitType == 3) {     display.println (" LOBSTER");  tone(soundpin, 632, 200);    
//            Left                  Right      
        GaitLegNr[cLP] = 0;   GaitLegNr[cRP] = 0;
        GaitLegNr[cLF] = 3;   GaitLegNr[cRF] = 1;        
        GaitLegNr[cLMF]= 6;   GaitLegNr[cRMF]= 5;     
        GaitLegNr[cLM] = 1;   GaitLegNr[cRM] = 3;
        GaitLegNr[cLR] = 5;   GaitLegNr[cRR] = 6; 

             NrLiftedPos    = 2;
             HalfLiftHeigth = 0;
             TLDivFactor    = 16;      
             StepsInGait    = 18;        
             NomGaitSpeed   = 85;
    }
//----------------
// Test Gait
    if (GaitType == 4) {     display.println ("TRIPPLER");  tone(soundpin, 732, 200);    
//            Left                  Right      
        GaitLegNr[cLP] = 4;   GaitLegNr[cRP] = 4;
        GaitLegNr[cLF] = 3;   GaitLegNr[cRF] = 1;        
        GaitLegNr[cLMF]= 2;   GaitLegNr[cRMF]= 2;     
        GaitLegNr[cLM] = 1;   GaitLegNr[cRM] = 1;
        GaitLegNr[cLR] = 2;   GaitLegNr[cRR] = 3; 

             NrLiftedPos    = 2;
             HalfLiftHeigth = 1;
             TLDivFactor    = 3;      
             StepsInGait    = 12;        
             NomGaitSpeed   = 85;
    }
//----------------

    if (GaitType == 5) {     display.println ("TARANTULA2");  tone(soundpin, 832, 200);    
int LPPedistep =random (4, 8); 
int RPPedistep =random (20, 24);
//            Left                           Right      
        GaitLegNr[cLP] = LPPedistep;    GaitLegNr[cRP] = RPPedistep;
        GaitLegNr[cLF] = 28;            GaitLegNr[cRF] = 12;  
        GaitLegNr[cLMF]= 16;            GaitLegNr[cRMF]= 0; 
        GaitLegNr[cLM] = 8;             GaitLegNr[cRM] = 24;
        GaitLegNr[cLR] = 20;            GaitLegNr[cRR] = 4; 

             NrLiftedPos    = 3;           
             HalfLiftHeigth = 0;
             TLDivFactor    = 6;      
             StepsInGait    = 32;    
             NomGaitSpeed   = 75; 
////                                 RMF,             RR, LP,          LM,            RF,            LMF,             LR, RP,         RM,             LF,                                
////                                   0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31
//static const byte X_Axis_Shift[] ={128,128,128,128,128,128,129,129,130,131,133,135,137,139,140,142,140,137,135,133,130,129,129,128,128,128,128,128,128,128,128,128};

    }


    
  display.display();
  delay(500);
  display.clearDisplay();
}
//--------------------------------------------------------------------
//[GAIT Sequence]
void GaitSeq(void)
{
    //Calculate Gait sequence
    LastLeg = 0;
    for (LegIndex = 0; LegIndex <= 9; LegIndex++) { // for all legs
        if (LegIndex == 9) // last leg
            LastLeg = 1;
    
        Gait(LegIndex);
    }    // next leg
}

//--------------------------------------------------------------------
//[GAIT]
void Gait (byte GaitCurrentLegNr)
{
  // Check if the Gait is in motion
  GaitInMotion = ((abs(PediRotation) > cTravelDeadZone) || (abs(TravelLengthX) > cTravelDeadZone) || (abs(TravelLengthZ) > cTravelDeadZone) || (abs(TravelRotationY) > cTravelDeadZone));

// Handles the body shifting (UNCOMMENT TO USE)
//   BodyPosX = X_Axis_Shift[GaitStep-128];   
//   BodyPosZ = Z_Axis_Shift[GaitStep-128];

  //Clear values under the cTravelDeadZone
  if (GaitInMotion == 0) {
      TravelLengthX = 0;
      TravelLengthZ = 0;
    TravelRotationY = 0;
       PediRotation = 0;
  }
  //Leg middle up position
  //Gait in motion                                                          Gait NOT in motion, return to home position
  if ( (GaitInMotion && (NrLiftedPos == 1 || NrLiftedPos == 3) && GaitStep == GaitLegNr[GaitCurrentLegNr])
       || ( !GaitInMotion && GaitStep == GaitLegNr[GaitCurrentLegNr] &&
            ((abs(GaitPosX[GaitCurrentLegNr]) > 2) || (abs(GaitPosZ[GaitCurrentLegNr]) > 2)
             || (abs(GaitRotY[GaitCurrentLegNr]) > 2)) ))  {        //Up
    GaitPosX[GaitCurrentLegNr] = 0;
    GaitPosY[GaitCurrentLegNr] = -LegLiftHeight;
    GaitPosZ[GaitCurrentLegNr] = 0;
    GaitRotY[GaitCurrentLegNr] = 0;
  }  else {
    //Optional Half heigth Rear
    if ( ((NrLiftedPos == 2 && GaitStep == GaitLegNr[GaitCurrentLegNr]) //+2
          || (NrLiftedPos == 3 && (GaitStep == (GaitLegNr[GaitCurrentLegNr] - 1) || GaitStep == (GaitLegNr[GaitCurrentLegNr] + (StepsInGait - 1)) )))
         && GaitInMotion) {
      GaitPosX[GaitCurrentLegNr] = -TravelLengthX / 2;
      GaitPosY[GaitCurrentLegNr] = -LegLiftHeight / (HalfLiftHeigth + 1);
      GaitPosZ[GaitCurrentLegNr] = -TravelLengthZ / 2;
      GaitRotY[GaitCurrentLegNr] = -TravelRotationY / 2;

    } else {
      //Optional half heigth front
      if (  (NrLiftedPos >= 2) && (
              (GaitStep == (GaitLegNr[GaitCurrentLegNr] + 1) || (GaitStep == (GaitLegNr[GaitCurrentLegNr] - (StepsInGait - 1))) )) &&
            GaitInMotion)  {
        GaitPosX[GaitCurrentLegNr] = TravelLengthX / 2;
        GaitPosY[GaitCurrentLegNr] = -LegLiftHeight / (HalfLiftHeigth + 1);
        GaitPosZ[GaitCurrentLegNr] = TravelLengthZ / 2;
        GaitRotY[GaitCurrentLegNr] = TravelRotationY / 2;
      } else {
        //Leg front down position
        if ((GaitStep == (GaitLegNr[GaitCurrentLegNr] + NrLiftedPos)
             || GaitStep == (GaitLegNr[GaitCurrentLegNr] - (StepsInGait - NrLiftedPos))) && (GaitPosY[GaitCurrentLegNr] < 0) ) {
          GaitPosX[GaitCurrentLegNr] = TravelLengthX / 2;
          GaitPosZ[GaitCurrentLegNr] = TravelLengthZ / 2;
          GaitRotY[GaitCurrentLegNr] = TravelRotationY / 2;
          GaitPosY[GaitCurrentLegNr] = 0;   
        }
        //Move body forward
        else {
          GaitPosX[GaitCurrentLegNr] -= ((long)TravelLengthX / (long)TLDivFactor);
          GaitPosY[GaitCurrentLegNr] = 0;
          GaitPosZ[GaitCurrentLegNr] -=  ((long)TravelLengthZ / (long)TLDivFactor);
          GaitRotY[GaitCurrentLegNr] -= ((long)TravelRotationY / (long)TLDivFactor);
        }
      }
    }
  }
  //Advance to the next step
  if (LastLeg)  {  //The last leg in this step
    GaitStep = GaitStep + 1;
    if (GaitStep > StepsInGait)
      GaitStep = 1;
  }
}

//--------------------------------------------------------------------
//[BalCalcOneLeg]
void BalCalcOneLeg (short PosX, short PosZ, short PosY, byte BalLegNr)
{
    short CPR_X;    // Final X value for centerpoint of rotation
    short CPR_Y;    // Final Y value for centerpoint of rotation
    short CPR_Z;    // Final Z value for centerpoint of rotation

    //Calculating totals from center of the body to the feet
    CPR_Z = cOffsetZ[BalLegNr]+PosZ;
    CPR_X = cOffsetX[BalLegNr]+PosX;
    CPR_Y = 150 + PosY;                // using the value 150 to lower the centerpoint of rotation 'BodyPosY +

    TotalTransY += (long)PosY;
    TotalTransZ += (long)CPR_Z;
    TotalTransX += (long)CPR_X;
    
    GetATan2(CPR_X, CPR_Z);
    TotalYBal1 += ((long)Atan4*1800) / 31415;
    
    GetATan2 (CPR_X, CPR_Y);
    TotalZBal1 += (((long)Atan4*1800) / 31415) -900; //Rotate balance circle 90 deg
    
    GetATan2 (CPR_Z, CPR_Y);
    TotalXBal1 += (((long)Atan4*1800) / 31415) - 900; //Rotate balance circle 90 deg

#ifdef DEBUG_BODYCALCS
    if ((TravelLengthX != 0) || (TravelLengthZ !=0) )
        DBGPrintf("BCOL: %d %d %d %u :: %ld %ld %ld :: %ld %ld %ld\n\r", PosX, PosZ, PosY, BalLegNr,
                TotalTransY, TotalTransZ, TotalTransX, TotalYBal1, TotalZBal1, TotalXBal1);        
#endif 
}  

//--------------------------------------------------------------------
//[BalanceBody]
void BalanceBody(void)
{  
  //Balance translation
    TotalTransZ = TotalTransZ/BalanceDivFactor;
    TotalTransX = TotalTransX/BalanceDivFactor;
    TotalTransY = TotalTransY/BalanceDivFactor;

    if (TotalYBal1 > 0)        // Rotate balance circle by +/- 180 deg
        TotalYBal1 -=  1800;
    else
        TotalYBal1 += 1800;
    
    if (TotalZBal1 < -1800)    // Compensate for extreme balance positions that causes owerflow
        TotalZBal1 += 3600;
    
    if (TotalXBal1 < -1800)    // Compensate for extreme balance positions that causes owerflow
        TotalXBal1 += 3600;
    
    //Balance rotation
    TotalYBal1 = -TotalYBal1/BalanceDivFactor;
    TotalXBal1 = -TotalXBal1/BalanceDivFactor;
    TotalZBal1 =  TotalZBal1/BalanceDivFactor;

#ifdef DEBUG_BODYCALCS
    if ((TravelLengthX != 0) || (TravelLengthZ !=0) )
        DBGPrintf("BBody: %ld %ld %ld :: %ld %ld %ld\n\r", TotalTransY, TotalTransZ,TotalTransX, 
            TotalYBal1, TotalZBal1,TotalXBal1);
#endif
}

//--------------------------------------------------------------------
//[GETSINCOS] Get the sinus and cosinus from the angle +/- multiple circles
//AngleDeg1   - Input Angle in degrees
//sin4        - Output Sinus of AngleDeg
//cos4        - Output Cosinus of AngleDeg
void GetSinCos(short AngleDeg1)
{
    short        ABSAngleDeg1;    //Absolute value of the Angle in Degrees, decimals = 1
    //Get the absolute value of AngleDeg
    if (AngleDeg1 < 0)
        ABSAngleDeg1 = AngleDeg1 *-1;
    else
          ABSAngleDeg1 = AngleDeg1;
    //Shift rotation to a full circle of 360 deg -> AngleDeg // 360
    if (AngleDeg1 < 0)    //Negative values
        AngleDeg1 = 3600-(ABSAngleDeg1-(3600*(ABSAngleDeg1/3600)));
    else                                         //Positive values
        AngleDeg1 = ABSAngleDeg1-(3600*(ABSAngleDeg1/3600));
    if (AngleDeg1>=0 && AngleDeg1<=900)          // 0 to 90 deg
    { sin4 = GetSin[AngleDeg1/5];                // 5 is the presision (0.5) of the table
      cos4 = GetSin[(900-(AngleDeg1))/5]; } 
              
    else if (AngleDeg1>900 && AngleDeg1<=1800)   // 90 to 180 deg
    { sin4 = GetSin[(900-(AngleDeg1-900))/5];    // 5 is the presision (0.5) of the table    
      cos4 = -GetSin[(AngleDeg1-900)/5]; }   
       
    else if (AngleDeg1>1800 && AngleDeg1<=2700)  // 180 to 270 deg
    { sin4 = -GetSin[(AngleDeg1-1800)/5];        // 5 is the presision (0.5) of the table
      cos4 = -GetSin[(2700-AngleDeg1)/5]; }    
      
    else if(AngleDeg1>2700 && AngleDeg1<=3600)   // 270 to 360 deg
    { sin4 = -GetSin[(3600-AngleDeg1)/5];        // 5 is the presision (0.5) of the table    
      cos4 = GetSin[(AngleDeg1-2700)/5];  }
}    

//--------------------------------------------------------------------
//(GETARCCOS) Get the sinus and cosinus from the angle +/- multiple circles
//cos4        - Input Cosinus
//AngleRad4   - Output Angle in AngleRad4
long GetArcCos(short cos4)
{
    boolean NegativeValue/*:1*/;    //If the the value is Negative
    //Check for negative value
    if (cos4<0)
    { cos4 = -cos4;
      NegativeValue = 1; }
  else
      NegativeValue = 0;
    
    //Limit cos4 to his maximal value
    cos4 = min(cos4,c4DEC);
    if ((cos4>=0) && (cos4<9000))
    { AngleRad4         = GetACos[cos4/79];                        //79=table resolution (1/127);
      AngleRad4         = ((long)AngleRad4*616)/c1DEC;  }          //616=acos resolution (pi/2/255) ;
    
    else if ((cos4>=9000) && (cos4<9900))
    { AngleRad4         = GetACos[(cos4-9000)/8+114];         // 8=table resolution (0.1/127), 114 start address 2nd bytetable range 
      AngleRad4         = (long)((long)AngleRad4*616)/c1DEC; }            //616=acos resolution (pi/2/255) 
    
    else if ((cos4>=9900) && (cos4<=10000))
    { AngleRad4         = GetACos[(cos4-9900)/2+227];         //2=table resolution (0.01/64), 227 start address 3rd bytetable range 
      AngleRad4         = (long)((long)AngleRad4*616)/c1DEC;  }           //616=acos resolution (pi/2/255) 

    //Add negative sign
    if (NegativeValue)
      AngleRad4         = 31416 - AngleRad4;
    return AngleRad4;
}    
// The primary algorithm used below to calculate the integer square root of a number 
unsigned long isqrt32 (unsigned long n) //
{
        unsigned long root;
        unsigned long remainder;
        unsigned long  place;
        root = 0;
        remainder = n;
        place = 0x40000000; // OR place = 0x4000; OR place = 0x40; - respectively

        while (place > remainder)
        place = place >> 2;
        while (place)
        { if (remainder >= root + place){
              remainder  = remainder - root - place;
              root       = root + (place << 1);}
              root       = root >> 1;
              place      = place >> 2;
        }
        return root;
}

//--------------------------------------------------------------------
//(GETATAN2) Simplyfied ArcTan2 function based on fixed point ArcCos
//ArcTanX         - Input X
//ArcTanY         - Input Y
//ArcTan4         - Output ARCTAN2(X/Y)
//XYhyp2          - Output presenting Hypotenuse of X and Y
short GetATan2 (short AtanX, short AtanY)
{
    XYhyp2 = isqrt32(((long)AtanX*AtanX*c4DEC) + ((long)AtanY*AtanY*c4DEC));
    GetArcCos (((long)AtanX*(long)c6DEC) /(long) XYhyp2);
    
    if (AtanY < 0)                // removed overhead... Atan4 = AngleRad4 * (AtanY/abs(AtanY));  
        Atan4 = -AngleRad4;
    else
        Atan4 = AngleRad4;
#if 0 //fdef DEBUG_PINCALCS    
    if (TarantulaOn)
    {
        DBGPrintf(" GAT(%d, %d)", XYhyp2, Atan4); 
    }
#endif    
    return Atan4;
}    
    
//--------------------------------------------------------------------
//(BODY INVERSE KINEMATICS) 
//BodyRotX         - Global Input pitch of the body 
//BodyRotY         - Global Input rotation of the body 
//BodyRotZ         - Global Input roll of the body 
//RotationY        - Input Rotation for the gait 
//PosX             - Input position of the feet X 
//PosZ             - Input position of the feet Z 
//SinB             - Sin buffer for BodyRotX
//CosB             - Cos buffer for BodyRotX
//SinG             - Sin buffer for BodyRotZ
//CosG             - Cos buffer for BodyRotZ
//BodyIKPosX       - Output Position X of feet with Rotation 
//BodyIKPosY       - Output Position Y of feet with Rotation 
//BodyIKPosZ       - Output Position Z of feet with Rotation

void BodyIK (short PosX, short PosZ, short PosY, short RotationY, byte BodyIKLeg) 
{
    short SinA4;  // Sin buffer for BodyRotX calculations
    short CosA4;  // Cos buffer for BodyRotX calculations
    short SinB4;  // Sin buffer for BodyRotX calculations
    short CosB4;  // Cos buffer for BodyRotX calculations
    short SinG4;  // Sin buffer for BodyRotZ calculations
    short CosG4;  // Cos buffer for BodyRotZ calculations
    short CPR_X;  // Final X value for centerpoint of rotation
    short CPR_Y;  // Final Y value for centerpoint of rotation
    short CPR_Z;  // Final Z value for centerpoint of rotation

    //Calculating totals from center of the body to the feet 
    CPR_X = cOffsetX[BodyIKLeg]+PosX;
    CPR_Y = PosY + BodyRotOffsetY;         //Define centerpoint for rotation along the Y-axis
    CPR_Z = cOffsetZ[BodyIKLeg] + PosZ + BodyRotOffsetZ;
Serial.println(BodyRotOffsetZ);

    //Successive global rotation matrix: Trigonometry
    //Math shorts for rotation: Alfa [A] = Xrotate, Beta [B] = Zrotate, Gamma [G] = Yrotate 
    //Sinus Alfa = SinA, cosinus Alfa = cosA. and so on... 
    
    //First calculate Trigonometry for each rotation: 
    GetSinCos (BodyRotX1+TotalXBal1);
    SinG4 = sin4;
    CosG4 = cos4;
    
    GetSinCos (BodyRotZ1+TotalZBal1); 
    SinB4 = sin4;
    CosB4 = cos4;
    
    GetSinCos (BodyRotY1+(RotationY*c1DEC)+TotalYBal1) ;
    SinA4 = sin4;
    CosA4 = cos4;
    
    //Calcualtion of rotation matrix: 
      BodyIKPosX = ((long)CPR_X*c2DEC - ((long)CPR_X*c2DEC*CosA4/c4DEC*CosB4/c4DEC - (long)CPR_Z*c2DEC*CosB4/c4DEC*SinA4/c4DEC 
              + (long)CPR_Y*c2DEC*SinB4/c4DEC ))/c2DEC;
      BodyIKPosZ = ((long)CPR_Z*c2DEC - ( (long)CPR_X*c2DEC*CosG4/c4DEC*SinA4/c4DEC + (long)CPR_X*c2DEC*CosA4/c4DEC*SinB4/c4DEC*SinG4/c4DEC 
              + (long)CPR_Z*c2DEC*CosA4/c4DEC*CosG4/c4DEC - (long)CPR_Z*c2DEC*SinA4/c4DEC*SinB4/c4DEC*SinG4/c4DEC 
              - (long)CPR_Y*c2DEC*CosB4/c4DEC*SinG4/c4DEC ))/c2DEC;
      BodyIKPosY = ((long)CPR_Y  *c2DEC - ( (long)CPR_X*c2DEC*SinA4/c4DEC*SinG4/c4DEC - (long)CPR_X*c2DEC*CosA4/c4DEC*CosG4/c4DEC*SinB4/c4DEC 
              + (long)CPR_Z*c2DEC*CosA4/c4DEC*SinG4/c4DEC + (long)CPR_Z*c2DEC*CosG4/c4DEC*SinA4/c4DEC*SinB4/c4DEC 
              + (long)CPR_Y*c2DEC*CosB4/c4DEC*CosG4/c4DEC ))/c2DEC;

#ifdef DEBUG_PINCALCS
    if (TarantulaOn && (TravelLengthX || TravelLengthZ))
    {
        DBGPrintf("  CPR XYZ: %d %d %d G4: %d %d ",CPR_X, CPR_Y, CPR_Z, SinG4, CosG4);
        DBGPrintf("B4: %d %d A4: %d %d", SinB4, CosB4, SinA4, CosA4);
        DBGPrintf(" BodyIK(%d %d %d) =", BodyIKPosX, BodyIKPosY, BodyIKPosZ);
    }
#endif    
}  

//--------------------------------------------------------------------
//[LEG INVERSE KINEMATICS] Calculates the angles of the coxa, femur and tibia for the given position of the feet
//IKFeetPosX            - Input position of the Feet X
//IKFeetPosY            - Input position of the Feet Y
//IKFeetPosZ            - Input Position of the Feet Z
//IKSolution            - Output true if the solution is possible
//IKSolutionWarning     - Output true if the solution is NEARLY possible
//IKSolutionError       - Output true if the solution is NOT possible
//FemurAngle1           - Output Angle of Femur in degrees
//TibiaAngle1           - Output Angle of Tibia in degrees
//CoxaAngle1            - Output Angle of Coxa in degrees

void LegIK (short IKFeetPosX, short IKFeetPosY, short IKFeetPosZ, byte LegIKLegNr)
{
    unsigned long    IKSW2;            //Length between Coxa and Tibia, decimals = 2
    unsigned long    IKA14;            //Angle of the line Coxa and Tibia with respect to the ground in radians, decimals = 4
    unsigned long    IKA24;            //Angle of the line Coxa and Tibia with respect to the femur in radians, decimals = 4
    short            IKFeetPosXZ;      //Diagonal direction from Input X and Z
    long             Temp1;            
    long             Temp2;            
    long             T3;
    
    //Calculate IKCoxaAngle and IKFeetPosXZ
    GetATan2 (IKFeetPosX, IKFeetPosZ);
    CoxaAngle1[LegIKLegNr] = (((long)Atan4*180) / 3141) + cCoxaAngle1[LegIKLegNr];
    
    //Length between the Coxa and tars [foot]
    IKFeetPosXZ = XYhyp2/c2DEC;
    
    //Using GetAtan2 for solving IKA1 and IKSW
    //IKA14 - Angle between Coxa and Tibia line and the ground in radians
    IKA14 = GetATan2 (IKFeetPosY, IKFeetPosXZ-cCoxaLength);
    
    //IKSW2 - Length between femur axis and tars
    IKSW2 = XYhyp2;
    
    //IKA2 - Angle of the line Coxa and Tibia with respect to the femur in radians
    Temp1 = ((((long)cFemurLength*cFemurLength) - ((long)cTibiaLength*cTibiaLength))*c4DEC + ((long)IKSW2*IKSW2));
    Temp2 = (long)(2*cFemurLength)*c2DEC * (unsigned long)IKSW2;
    T3 = Temp1 / (Temp2/c4DEC);
    IKA24 = GetArcCos (T3 );
#if 0 //def DEBUG_PINCALCS
    if (TarantulaOn && (TravelLengthX < - 30))
    {
        DBGPrintf("T1: %ld T2: %ld: Temp3 %ld AR: %d ", Temp1, Temp2, T3, AngleRad4);
    }
#endif    
    //IKFemurAngle
    FemurAngle1[LegIKLegNr] = -(long)(IKA14 + IKA24) * 180 / 3141 + 900;

    //IKTibiaAngle
    Temp1 = ((((long)cFemurLength*cFemurLength) + ((long)cTibiaLength*cTibiaLength))*c4DEC - ((long)IKSW2*IKSW2));
    Temp2 = (2*cFemurLength*cTibiaLength);
    GetArcCos (Temp1 / Temp2);
    TibiaAngle1[LegIKLegNr] = -(900-(long)AngleRad4*180/3141);
#if 0 //def DEBUG_PINCALCS
    if (TarantulaOn && (TravelLengthX || TravelLengthZ)) {
        DBGPrintf("T1: %lu T2: %lu: AR: %d ", Temp1, Temp2, AngleRad4);
    }
#endif

    //Set the Solution quality    
    if(IKSW2 < (cFemurLength+cTibiaLength-30)*c2DEC)
        IKSolution = 1;
    else
    {
        if(IKSW2 < (cFemurLength+cTibiaLength)*c2DEC) 
            IKSolutionWarning = 1;
        else
            IKSolutionError = 1    ;
    }
#ifdef DEBUG_PINCALCS
    if (TarantulaOn && (TravelLengthX || TravelLengthZ))
    {
      //DBGPrintf("POS (%d,%d, %d) IKA1 %d, IKSW: %d, IKA2 %d =>", IKFeetPosX, IKFeetPosY, IKFeetPosZ, IKA14, IKSW2,IKA24);
        DBGPrintf(" CA: %d FA: %d, TA:  %d  SWE:  %x%x%x\n\r", 
                CoxaAngle1[LegIKLegNr],FemurAngle1[LegIKLegNr], TibiaAngle1[LegIKLegNr], IKSolution, IKSolutionWarning, IKSolutionError  );
    }
#endif    
}

//--------------------------------------------------------------------
//[CHECK ANGLES] Checks the mechanical limits of the legs 0-9
void CheckAngles(void)
{

  for (LegIndex = 0; LegIndex <= 9; LegIndex++)
  {
    CoxaAngle1[LegIndex]  = min(max(CoxaAngle1[LegIndex], cCoxaMin1[LegIndex]), cCoxaMax1[LegIndex]);
    FemurAngle1[LegIndex] = min(max(FemurAngle1[LegIndex], cFemurMin1[LegIndex]), cFemurMax1[LegIndex]);
    TibiaAngle1[LegIndex] = min(max(TibiaAngle1[LegIndex], cTibiaMin1[LegIndex]), cTibiaMax1[LegIndex]);
  }
}

//--------------------------------------------------------------------
//[SERVO DRIVER Start] Updates the positions of the servos 
void ServoDriverStart()
{
#ifdef cSSC_BINARYMODE
  byte    abOut[30];
  byte    *pbOut;
  word    wCoxaSSCV;         // Coxa value in SSC units
  word    wFemurSSCV;        // Femur value in SSC units
  word    wTibiaSSCV;        // Tibia value in SSC units
#endif

  //Update Right Legs
  AllowControllerInterrupts(false);    // If on xbee on hserial tell hserial to not processess...
  //    disable(TIMERAINT);

#ifdef DEBUG_GAITS
  if (TarantulaOn && (TravelLengthX || TravelLengthZ))
    DBGPrintf("%d %d %d:", (short)GaitType, (short)GaitStep, (short)BalanceMode);
#endif

  for (LegIndex = 0; LegIndex <= 4; LegIndex++) {

#ifdef DEBUG_GAITS
    if (TarantulaOn && (TravelLengthX || TravelLengthZ)) {
      DBGPrintf("%02d=(%d)%04d ", (short)cCoxaPin[LegIndex], CoxaAngle1[LegIndex], wCoxaSSCV);
      DBGPrintf("%02d=(%d)%04d ", (short)cFemurPin[LegIndex], FemurAngle1[LegIndex], wFemurSSCV);
      DBGPrintf("%02d=(%d)%04d ", (short)cTibiaPin[LegIndex], TibiaAngle1[LegIndex], wTibiaSSCV);
    }
#endif

    SSCPrintf("#%dP%d", cCoxaPin[LegIndex],  ((long)(-CoxaAngle1[LegIndex]  + 900)) * 1000 / 1059 + 650);
    SSCPrintf("#%dP%d", cFemurPin[LegIndex], ((long)(-FemurAngle1[LegIndex] + 900)) * 1000 / 1059 + 650);
    SSCPrintf("#%dP%d", cTibiaPin[LegIndex], ((long)(-TibiaAngle1[LegIndex] + 900)) * 1000 / 1059 + 650);
  }

  //Update Left Legs
  for (LegIndex = 5; LegIndex <= 9; LegIndex++)
  {

#ifdef DEBUG_GAITS
    if (TarantulaOn && (TravelLengthX || TravelLengthZ)) {
      DBGPrintf("%02d=(%d)%04d ", (short)cCoxaPin[LegIndex], CoxaAngle1[LegIndex], wCoxaSSCV);
      DBGPrintf("%02d=(%d)%04d ", (short)cFemurPin[LegIndex], FemurAngle1[LegIndex], wFemurSSCV);
      DBGPrintf("%02d=(%d)%04d ", (short)cTibiaPin[LegIndex], TibiaAngle1[LegIndex], wTibiaSSCV);
    }
#endif

    SSCPrintf("#%dP%d", cCoxaPin[LegIndex],  ((long)(CoxaAngle1[LegIndex]  + 900)) * 1000 / 1059 + 650);
    SSCPrintf("#%dP%d", cFemurPin[LegIndex], ((long)(FemurAngle1[LegIndex] + 900)) * 1000 / 1059 + 650);
    SSCPrintf("#%dP%d", cTibiaPin[LegIndex], ((long)(TibiaAngle1[LegIndex] + 900)) * 1000 / 1059 + 650);
  }

  //    enable(TIMERAINT);
  AllowControllerInterrupts(true);    // Ok for hserial again...
}

//--------------------------------------------------------------------
//[SERVO DRIVER Start] Updates the positions of the servos - This outputs
//         as much of the command as we can without committing it.  This
//         once the previous update was completed to quickly 
//         get the next command to start
void ServoDriverCommit()
{
#ifdef cSSC_BINARYMODE
    byte    abOut[3];
#endif
    
    AllowControllerInterrupts(false);    // If on xbee on hserial tell hserial to not process...

#ifdef cSSC_BINARYMODE
    abOut[0] = 0xA1;
    abOut[1] = SSCTime >> 8;
    abOut[2] = SSCTime & 0xff;
#else
      //Send <CR>
    SSCPrintf("T%d\n", SSCTime);
#endif
    Serial1.println();
#ifdef DEBUG_GAITS
    if (TarantulaOn && (TravelLengthX || TravelLengthZ)) {
	DBGPrintf("T:%d\n\r", SSCTime);
    }
#endif	

    AllowControllerInterrupts(true);    
    PrevSSCTime = SSCTime;
}
//--------------------------------------------------------------------
// [SMOOTHCONTROL] - This function makes the body rotation and translation much smoother while walking
short SmoothControl (short CtrlMoveInp, short CtrlMoveOut, byte CtrlDivider)
{
    if (fWalking){
        if (CtrlMoveOut < (CtrlMoveInp - 4))
              return CtrlMoveOut + abs((CtrlMoveOut - CtrlMoveInp)/CtrlDivider);
        else if (CtrlMoveOut > (CtrlMoveInp + 4))
              return CtrlMoveOut - abs((CtrlMoveOut - CtrlMoveInp)/CtrlDivider);
              }
    return CtrlMoveInp;
}

//--------------------------------------------------------------------
//[FREE SERVOS] Frees all the servos
void FreeServos(void)
{
    AllowControllerInterrupts(false);    // If on xbee on hserial tell hserial to not process...
//  disable(TIMERAINT);
    for (LegIndex = 0; LegIndex < 32; LegIndex+=4) {
        SSCPrintf("#%dP0#%dP0#%dP0#%dP0", LegIndex, LegIndex+1, LegIndex+2, LegIndex+3);
    }
    SSCSerial.print("T200");
    Serial1.println();
//  enable(TIMERAINT);
    AllowControllerInterrupts(true);    
}
//--------------------------------------------------------------------
// [BUGBUG] Place holder
int DBGPrintf(const char *format, ...)
{
    char szTemp[80];
    int ich;
    va_list ap;
    va_start(ap, format);
    ich = vsprintf(szTemp, format, ap);
    DBGSerial.write((byte*)szTemp, ich);
    va_end(ap);
}
//--------------------------------------------------------------------
// [BUGBUG] Place holder
int SSCPrintf(const char *format, ...)
{
    char szTemp[80];
    int ich;
    va_list ap;
    va_start(ap, format);
    ich = vsprintf(szTemp, format, ap);
    SSCSerial.write((byte*)szTemp, ich);
    va_end(ap);
}
//--------------------------------------------------------------------
// Quick and dirty helper function to read so many bytes in from the SSC with a timeout and an end of character marker...
int SSCRead (byte* pb, int cb, word wTimeout, word wEOL)
{
    int ich;
    byte* pbIn = pb;
    unsigned long ulTimeLastChar = micros();
    while (cb) {
        while ((ich = SSCSerial.read()) == -1) {                            // check for timeout
            if ((word)(micros()-ulTimeLastChar) > wTimeout)
                return (int)(pb-pbIn);
        }
        *pb++ = (byte)ich;
        cb--;
        if ((word)ich == wEOL)
               break;                 // we matched so get out of here.
        ulTimeLastChar = micros();    // Update to say we received something
    }
    return (int)(pb-pbIn);
}
//--------------------------------------------------------------------
void Move_Abdomen(){
     
    int YawVAL              = map (TravelRotationY, -128,127, 0,180 );  // Global Input rotation             
    int ShiftAbdomenYaw     = map (BodyPosY,        -128,127, 0,180 );  // Global Input rotation   
    int RotateAbdomenPitch  = map (BodyRotX1,       -128,127, 0,180 );  // Global Input pitch 
    int RotateAbdomenYaw    = map (BodyRotY1,       -128,127, 0,180 );  // Global Input rotation 

// Unused Global inputs
    int RollVAL             = map (TravelLengthZ,   -128,127, 0,180 );  // Global Input pitch 
//    char PitchVAL           = map (TravelLengthX,   -128,127, 0,180 );  // Global Input roll    
//    char RotateAbdomenRoll  = map (BodyRotZ1,       -128,127, 0,180 );  // Global Input roll    

//      While Walking do.. 
        AbdomenYaw   .write(YawVAL);            // Global Input Yaw of the Abdomen

//      While Translating do..
        AbdomenPitch .write(ShiftAbdomenYaw);   // Global Input pitch of the Abdomen
        AbdomenYaw   .write(RotateAbdomenYaw);  // Global Input Yaw of the Abdomen

//      While Rotating do..
        AbdomenPitch .write(RotateAbdomenPitch);// Global Input pitch of the Abdomen
        AbdomenYaw   .write(RotateAbdomenYaw);  // Global Input Yaw of the Abdomen

   }
//--------------------------------------------------------------------
void Move_Pedi(){ //Secondary Appendages

              tone(A0, PediRotation/10, 50);  
                  
    long ValPediTilt = map (PediRotation, -128,127, 65,115 );              
        
//      While Walking do.. 
        PediTilt   .write(ValPediTilt);    
 //           Serial.println(ValPediTilt); 
  //           Serial.println(BodyPosY);   
   }
//=====================================================================
void testdrawcircle(void) {
  for (int16_t i=0; i<display.height(); i+=4) {
    display.drawCircle(display.width()/2, display.height()/2, i, WHITE);

    display.setTextSize(2); 
    display.setCursor(0,12);
    display.setTextColor(WHITE); 
    display.println (" BOOT|UP");    
    display.display();
    delay(150);
  }
}
//=====================================================================
void TerrainAdaption() {
  // Run though sensors  
 for (int TarsToGround = 0; TarsToGround < SensorCount; TarsToGround++) {

    
          digitalRead(TarsToGround);
 }             
 
  // if the leg is at InitPosY but contact hasnt been made by foot then keep lowering the leg 
    if (LegPosY[LegIndex] == cInitPosY[LegIndex] && TarsToGround[SensorCount] == LOW){
        cInitPosY+1;  
 
 }}

    
