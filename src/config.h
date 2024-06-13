//                    OSD Menu main attributes:                     //
//                                                                  //
//PAGE 1//////////////////////////////////////////////////////////////
//                                                                  //
//  BATx CRT - bat. 1,2 crytical voltage                            //
//  RSSI - RSSI calibration                                         //
//  SCR.SW - selection of aux channel for switching between screens (normal,radar) - (if you want to disable it, select AUX0)
//  RC SENS - sensitivity of RC commands                            //
//                                                                  //
//PAGE 2//////////////////////////////////////////////////////////////
//                                                                  //
//  BATx ADJ - tuning inaccuracies in voltage measurement           //
//  CUR.TYPE - current sensor sellection                            //
//    1 - for ACS758 100A BiDirectional                             //
//    2 - for ACS758 50A UniDirectional                             //
//    3 - for Flytron Ultralight 50A Current Sensor                 //
//    4 - for Flytron Ultralight 25A Current Sensor                 //
//    5 - for ACS758 200A BiDirectional
//    6 - for ACS756 50A BiDirectional  
//  ROL SEN - roll sensitivity of A. horizont (default 10)          //  
//  PIT SEN - pitch sensitivity of A. horizont (default 10)         //
//                                                                  //
//////////////////////////////////////////////////////////////////////


//CONFIG PART //////////////////////////////////////////////////////////////////////////////////////  

//----------------- Video signal type PAL/NTSC
// #define VIDEO_TYPE  PAL
//-----------------//


//----------------- Selectable BigNumber FONT style 
// #define HITECH_FONT      //if defined - standard PRE7.8 Font  
//-----------------//      //else - Font created by hatuul


//----------------- Bat1/Bat2 voltage levels
#define BAT1_LEVEL_2      115      //11.5V   BAT1 - decimal value!
#define BAT1_LEVEL_1      102      //10.2V
//bat1 level 0 is crytical voltage stored in eeprom

#define BAT2_LEVEL_2      7.4      //7.4V    BAT2 - float value!
#define BAT2_LEVEL_1      6.9
//bat2 level 0 is crytical voltage stored in eeprom
//-----------------//


//----------------- RSSI signal levels
#define RSSI_5 90          //90% signal
#define RSSI_4 80
#define RSSI_3 70
#define RSSI_2 60
#define RSSI_1 50          //50% signal
//-----------------//


//----------------- ROLL,PITCH Angle visualisation (Artifical-Horizont)
#define ROLL_TILT_REVERSED                     
#define PITCH_TILT_REVERSED         
//-----------------//


//----------------- Enable values on screen (if you want to show this value)
// #define CURRENT_SENSOR          //to enable current sensor connected to port N1 (MobiDrone OSD)       
// #define RSSISIGNAL              //to enable signal on screen 
// #define MODE                    //to enable mode panel L-level, B-baro, M-mag, G-gps, RTH
// #define BAT1                    //to enable Battery 1 panel  (multiwii voltage sensor)  
// #define BAT2                    //to enable Battery 2 panel  (BAT input - mobiDrone OSD)
// #define GPS                     //to enable GPS panel (home arrow, distance to home)  
// #define LAT_LON                 //to enable Latitude & Longitude
// #define COMPASS                 //to enable Compass panel
// #define FLYTIME                 //to enable fly time
#define STARTTIME               //to enable fly time
// #define ALTITUDE                   //to enable process altitude data from multiwii
// #define VERTICAL_LINES          //to enable vertical lines in A.horizont window
//-----------------//


//----------------- Size of radar in meters
#define MAX_RADAR_DISTANCE 150  
//-----------------//

//----------------- Throttle value to out from result screen
#define OUT_THROTTLE 1400
//-----------------//

//END OF CONFIG PART //////////////////////////////////////////////////////////////////////////////  
