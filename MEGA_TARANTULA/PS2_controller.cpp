/******************************************************************
*  Super amazing PS2 controller v2.0
******************************************************************/
//====================================================================
// [Include files]
#include <Arduino.h> 
#include "Tarantula_Globals.h"
#ifdef USEPS2
#include "PS2X_lib.h"

//[CONSTANTS]
#define WALKMODE          0
#define TRANSLATEMODE     1
#define ROTATEMODE        2
#define SINGLELEGMODE     3
#define GPPLAYERMODE      4

#define cTravelDeadZone   1 //The deadzone for the analog input from the remote

//=============================================================================
// Global - Local to this file only...
//=============================================================================
// BUGBUG: Move to PS2 support
PS2X ps2x; // create PS2 Controller Class

extern short     BodyRotOffsetZ;   // Input Z offset value to adjust centerpoint of rotation
static unsigned  g_BodyYOffset; 
static char      g_BodyYSift;
static byte      ControlMode;
static bool      DoubleHeightOn;
static bool      DoubleTravelOn;
static bool      WalkMethod;

extern int DBGPrintf(const char *format, ...);

//==============================================================================
// This is The function that is called by the Main program to initialize the PS2
//==============================================================================

// If both PS2 and XBee are defined then we will become secondary to the xbee
#ifdef USEXBEE
void InitPS2Controller(void)
#else
void InitController(void)
#endif
{
    int error;
    
    delay(300);

  //error = ps2x.config_gamepad(     57,      55,      56,      54);  // Setup gamepad (clock, command, attention, data) pins
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT);  // Setup gamepad (clock, command, attention, data) pins
 
    if(error == 0){
       DBGSerial.println(" DONE");
    }
   
    else if(error == 1)
       DBGSerial.println(" Unsuccessful");
   
    else if(error == 2)
       DBGSerial.println("Controller found but Error!");
    
    g_BodyYOffset = 0;
    g_BodyYSift = 0;
}

//==============================================================================
// This function is called by the main code to tell us when it is about to
// do a lot of bit-bang outputs and it would like us to minimize any interrupts
// that we do while it is active...
//==============================================================================
#ifndef USEXBEE// XBEE will provide this if defined...
void AllowControllerInterrupts(bool fAllow)
{
}
#endif

//==============================================================================
// This is The main code to input function to read inputs from the PS2 and then
// process any commands.
//==============================================================================
#ifdef USEXBEE
void PS2ControlInput(void)
#else
void ControlInput(void)
#endif
{
    // Then try to receive a packet of information from the PS2.
    ps2x.read_gamepad();          //read controller and set large motor to spin at 'vibrate' speed

    // Wish the library had a valid way to verify that the read_gamepad succeeded... Will hack for now
    if ((ps2x.Analog(1) & 0xf0) == 0x70) {
        // In an analog mode so should be OK...

        if (ps2x.ButtonPressed(PSB_START)) {
            static unsigned    g_BodyYOffset; 
            static char        g_BodyYSift;
            DBGPrintf("START button Pressed\r");
            if (TarantulaOn) {
                //Turn off
              pinMode(A0, OUTPUT);   // Speaker 
              tone(A0, 532, 200);  
              delay(100);
              tone(A0, 632, 200);
                          
                BodyPosX = 0;
                BodyPosY = 0;
                BodyPosZ = 0;
                BodyRotX1 = 0;
                BodyRotY1 = 0;
                BodyRotZ1 = 0;
                TravelLengthX = 0;
                TravelLengthZ = 0;
                TravelRotationY = 0;
                PediRotation = 0;
                BodyRotOffsetZ = 0;
                g_BodyYOffset = 0;
                PediRotation = 0;
                g_BodyYSift = 0;
                SelectedLeg = 255;
                TarantulaOn = 0;
            } else {
                //Turn on
              tone(A0, 332, 200);  
              delay(100);
              tone(A0, 132, 200);    
                TarantulaOn = 1;
            }
            //Calculate BodyPosY
        BodyPosY = max(g_BodyYOffset + g_BodyYSift,  0);
        }
//------------------------------        
        if (TarantulaOn) {
            // [SWITCH MODES]
             //Translate mode
            if (ps2x.ButtonPressed(PSB_L1)) {// L1 Button Test  
              tone(A0, 532, 200);  
              delay(100);
              tone(A0, 632, 200);  
              Serial.println("L1 TRANSLATE MODE ");  
                if (ControlMode != TRANSLATEMODE )
                    ControlMode = TRANSLATEMODE;
                else {
                    if (SelectedLeg==255) 
                        ControlMode = WALKMODE;
                    else
                        ControlMode = SINGLELEGMODE;
                }
            }
//------------------------------     
            //Rotate mode
            if (ps2x.ButtonPressed(PSB_L2)) {    // L2 Button Test
              tone(A0, 532, 200);  
              delay(100);
              tone(A0, 632, 200);
              Serial.println("L2 ROTATE MODE"); 
                if (ControlMode != ROTATEMODE)
                    ControlMode = ROTATEMODE;
                else {
                    if (SelectedLeg == 255) 
                        ControlMode = WALKMODE;
                    else
                        ControlMode = SINGLELEGMODE;
                }
            }
//------------------------------ 
            //Rotate mode additional control  
                if (ps2x.ButtonPressed(PSB_L3)) { Serial.println("L3 ROTATE MODE");  
                if (ControlMode != ROTATEMODE)
                    ControlMode = ROTATEMODE;
                }
                if (ps2x.ButtonReleased(PSB_L3)) { Serial.println("L3 RETURNED TO WALK MODE");  
                    if (SelectedLeg == 255) 
                        ControlMode = WALKMODE;
                    else
                        ControlMode = SINGLELEGMODE;
                }
                       
//------------------------------       
            // Single leg mode
            if (ps2x.ButtonPressed(PSB_CIRCLE)) { // O - Circle Button Test
              tone(A0, 532, 200);  
              delay(100);
              tone(A0, 632, 200); 
                            Serial.println("Circle SINGLE LEG"); 
                if (abs(TravelLengthX)<cTravelDeadZone && abs(TravelLengthZ)<cTravelDeadZone 
                        && abs(TravelRotationY*2)<cTravelDeadZone )   {
                    if (ControlMode != SINGLELEGMODE) {
                        ControlMode = SINGLELEGMODE;
                            if (SelectedLeg == 255)  // Select leg if none is selected
                                SelectedLeg=cRP;     // Startleg
                    } else {
                        ControlMode = WALKMODE;
                        SelectedLeg=255;
                    }
                }
            }      
//------------------------------   
//            // Was GP Player Mode X
            if (ps2x.ButtonPressed(PSB_CROSS)) { // X - Cross Button Test
              tone(A0, 532, 200);  
              delay(100);
              tone(A0, 632, 200); 
                            Serial.println("Cross DOES NOTHING"); 
//                if (ControlMode != GPPLAYERMODE) {
//                    ControlMode = GPPLAYERMODE;
//                    GPSeq=0;
//                } else
//                    ControlMode = WALKMODE;
            }
//------------------------------   
            //[Common functions]
            //Switch Balance mode on/off             
            if (ps2x.ButtonPressed(PSB_SQUARE)) { // Square Button Test
              tone(A0, 532, 200);  
              delay(100);
              tone(A0, 632, 200);
                            Serial.println("Square BALANCE / WALK "); 
                BalanceMode = !BalanceMode;
            }
//------------------------------   
            //Stand up, sit down  
            if (ps2x.ButtonPressed(PSB_TRIANGLE)) { // Triangle - Button Test
              tone(A0, 532, 200);  
              delay(100);
              tone(A0, 632, 200);
                            Serial.println("Triangle STAND / SIT");
                if (g_BodyYOffset>0) 
                    g_BodyYOffset = 0; 
                else 
                    g_BodyYOffset = 35;
            }
//------------------------------   
            if (ps2x.ButtonPressed(PSB_PAD_UP))// D-Up - Button Test
                          Serial.println("Up BODY UP"); 
                g_BodyYOffset = g_BodyYOffset+10;
          
//------------------------------   
            if (ps2x.ButtonPressed(PSB_PAD_DOWN))// D-Down - Button Test
                          Serial.println("Down BODY DOWN");
                g_BodyYOffset = g_BodyYOffset-10;
//------------------------------   
                
            if (ps2x.ButtonPressed(PSB_PAD_RIGHT)) { // D-Right - Button Test
                            Serial.println("Right SPEED DOWN");
                if (SpeedControl>0) {
                    SpeedControl = SpeedControl - 50;
                }
            }
//------------------------------               
            if (ps2x.ButtonPressed(PSB_PAD_LEFT)) { // D-Left - Button Test
                            Serial.println("Left SPEED UP");
                if (SpeedControl<2000 ) {
                    SpeedControl = SpeedControl + 50;
               }
          }
//------------------------------             
            //[Walk functions]
            if (ControlMode == WALKMODE) {
                //Switch gates 
                if (ps2x.ButtonPressed(PSB_SELECT)            // Select Button Test
                        && abs(PediRotation)<cTravelDeadZone
                        && abs(TravelLengthX)<cTravelDeadZone //No movement
                        && abs(TravelLengthZ)<cTravelDeadZone 
                        && abs(TravelRotationY*2)<cTravelDeadZone  ) {
                          Serial.println("Select SELECT GAIT");
                    if (GaitType<5) {
                       
                        GaitType = GaitType+1;
                    } else {
                        GaitType = 0;
                    }
                    GaitSelect();
                }
//------------------------------     
                //Double leg lift height
                if (ps2x.ButtonPressed(PSB_R1)) { Serial.println("R1 DOUBLE LEG HIGHT on/off"); 
                    DoubleHeightOn = !DoubleHeightOn;
                    if (DoubleHeightOn)
                        LegLiftHeight = 80;
                    else
                        LegLiftHeight = 50;
                }
//------------------------------     
                //Double Travel Length
                if (ps2x.ButtonPressed(PSB_R2)) { Serial.println("R2 DOUBLE TRAVEL LENGTH on/off"); 
                   DoubleTravelOn = !DoubleTravelOn;
                }
//------------------------------     
                // Switch between Walk method 1 && Walk method 2
                if (ps2x.ButtonPressed(PSB_R3)) { Serial.println("R3 WALK MODES 1/2"); 
                    WalkMethod = !WalkMethod;
                }
//------------------------------     
                //Walking
                if (WalkMethod)  //(Walk Methode) 
                    TravelLengthZ = (ps2x.Analog(PSS_RY) -128);   //Right Stick Up/Down  

                else {
                    TravelLengthX = -(ps2x.Analog(PSS_LX) - 128);//Left Stick Left/Right 
                    TravelLengthZ = (ps2x.Analog(PSS_LY) - 128); //Left Stick Up/Down  
                    PediRotation  = (ps2x.Analog(PSS_RY) - 128); //Right Stick Up/Down 

                }

                if (!DoubleTravelOn) {  //(Double travel length)
                    TravelLengthX = TravelLengthX/2;
                    TravelLengthZ = TravelLengthZ/2;
                }

                TravelRotationY = -(ps2x.Analog(PSS_RX) - 128)/4; //Right Stick Left/Right 
                PediRotation    = (ps2x.Analog(PSS_RY) - 128); //Left Stick Up/Down 
            }

            //[Translate functions]
            //g_BodyYSift = 0
            if (ControlMode == TRANSLATEMODE) {   
                BodyPosX = (ps2x.Analog(PSS_LX) - 128)/2;
                BodyPosZ = -(ps2x.Analog(PSS_LY) - 128)/3;
                BodyRotY1 = (ps2x.Analog(PSS_RX) - 128)*2;
                g_BodyYSift = (-(ps2x.Analog(PSS_RY) - 128)/2);
            }

            //[Rotate functions]
            if (ControlMode == ROTATEMODE) {
                BodyRotX1 = (ps2x.Analog(PSS_LY) - 128);
                BodyRotY1 = (ps2x.Analog(PSS_RX) - 128)*2;
                BodyRotZ1 = (ps2x.Analog(PSS_LX) - 128);
                g_BodyYSift = (-(ps2x.Analog(PSS_RY) - 128)/2);
                BodyRotOffsetZ = (ps2x.Analog(PSS_RY) -128);   //Right Stick Up/Down                
            }
//------------------------------   
            //[Single leg functions]
            if (ControlMode == SINGLELEGMODE) {
                //Switch leg for single leg control
                if (ps2x.ButtonPressed(PSB_SELECT)) { Serial.println("Select SWITCH LEGS");  
                    if (SelectedLeg<5)
                        SelectedLeg = SelectedLeg+1;
                    else
                        SelectedLeg=0;
                }

                SLLegX= (ps2x.Analog(PSS_LX) - 128)/2; //Left Stick Right/Left
                SLLegY= (ps2x.Analog(PSS_RY) - 128)/5; //Right Stick Up/Down
                SLLegZ = (ps2x.Analog(PSS_LY) - 128)/2; //Left Stick Up/Down
//------------------------------   

                // Hold single leg in place
                if (ps2x.ButtonPressed(PSB_R2)) { Serial.println("R2 HOLD LEG on/off"); 
                    fSLHold = !fSLHold;
                }
            }
 //------------------------------    
//            //[GPPlayer functions]
            if (ControlMode == GPPLAYERMODE) {

                //Switch between sequences
                if (ps2x.ButtonPressed(PSB_SELECT)) { Serial.println("Select SEQUENCE #");
                    if (GPStart==0 ) {
                        if (GPSeq < 5) {  //Max sequence
                           GPSeq = GPSeq+1;
                        } else {
                            GPSeq=0;
                        }
                    }
                }
                //Start Sequence
                if (ps2x.ButtonPressed(PSB_R2)) Serial.println("R2 PLAY SEQUENCE");
                    GPStart=1;
            }
//------------------------------   
            //Calculate walking time delay
            InputTimeDelay = 128 - max(max(abs(ps2x.Analog(PSS_LX) - 128), abs(ps2x.Analog(PSS_LY) - 128)), abs(ps2x.Analog(PSS_RX) - 128));
        }
  
        //Calculate BodyPosY
        BodyPosY = max(g_BodyYOffset + g_BodyYSift,  0);
    }
}

#endif //USEPS2
