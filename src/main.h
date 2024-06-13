
#include <Arduino.h>

static void printFlightPanel();
uint32_t read32(uint8_t id);
uint16_t read16(uint8_t id);
uint8_t read8(uint8_t id); 
static void buffering2BigNumbers(uint16_t barPointer, uint8_t id1, uint8_t id2);
static void buffering3BigNumbers(uint16_t barPointer, uint8_t id1, uint8_t id2, uint8_t id3);
static void writePixel(uint8_t x,uint8_t y,uint8_t color);
void readEEPROM(uint8_t length);
static void PrintHeaderToBuffer();
static void PrintMenuPageToBuffer(uint8_t pageNumber);
static void sendDataRequest(uint8_t request, uint8_t dataId, uint8_t fSize);
static void PrintLayoutToBuffer();                                       
static void printMainPanelToBuffer(uint8_t centerLine, uint8_t centerPoint,uint8_t rollAngle);
static void fastUpdate(); // duration 20mSec * 7                            
static void slowUpdate(); // duration 20ms * 127
void setupPinMode();                                   

#define PAL 0
#define NTSC 1

#define VIDEO_TYPE  PAL

//delays
#define delay1Pixel __asm__("nop\n\t""nop\n\t");   //~ 120nS
#define delay15 __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");   // nop - 62,5nS
#define delay13 __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"); 
#define delay10 __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");   // nop - 62,5nS 
#define delay9 __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");   // nop - 62,5nS 
#define delay8 __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");   // nop - 62,5nS
#define delay7 __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");   // nop - 62,5nS
#define delay6 __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");   // nop - 62,5nS 
#define delay5 __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");   // nop - 62,5nS 
#define delay4 __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");   // nop - 62,5nS 
#define delay3 __asm__("nop\n\t""nop\n\t""nop\n\t");   // nop - 62,5nS 
#define delay2 __asm__("nop\n\t""nop\n\t");   // nop - 62,5nS 
#define delay1 __asm__("nop\n\t");   // nop - 62,5nS 

////////////////////////Internal settings
// EEPROM data pointer
#define BAT1_LEVEL 1         // 1 byte    calibrate bat1 alarm
#define BAT2_LEVEL 2         // 1 byte    calibrate bat2 alarm
#define RSSI_CALIB 3         // 2 bytes   calibrate max rssi value
#define AUX_SW 5             // 1 byte    calibration temperature sensor (in future)
#define RC_SENS 6            // 1 byte    rc sensitivity 1 - 5
#define BAT1_CORRECTION 7    // 1 byte    bat1 voltage correction
#define BAT2_CORRECTION 8    // 1 byte    bat2 voltage correction
#define CURR_SENS_TYPE 9     // 1 byre    current sensor sellection 0 - no sensor
#define ROLL_SENSITIVITY 10  // 1 byre    adjusting roll sensitivity of artifical horizont
#define PITCH_SENSITIVITY 11 // 1 byre    adjusting pitch sensitivity of artifical horizont

#define EEPROM_DATA_LENGTH 12

////////////////////////////Config screen

// Aliases
// RC channels
#define ROLL 0
#define PITCH 1
#define YAW 2
#define THROTTLE 3
#define AUX1 4
#define AUX2 5
#define AUX3 6 
#define AUX4 7

// PID data
#define ROLL 0
#define PITCH 1
#define YAW 2
#define ALT 3
#define POS 4
#define POSR 5
#define NAVR 6
#define LEVEL 7
#define MAG 8
#define VEL 9
#define RCRATE 10
#define RCEXPO 11
#define RPRATE 12
#define YAWRATE 13
#define DYNTHRPID 14
#define THRMID 15
#define THREXPO 16

#define PIDITEMS 10
#define PID_DATA_BYTES 3 * PIDITEMS

#define RCTUNING_ITEMS 7
#define RCTUNING_ITEMS_BYTES RCTUNING_ITEMS * 3

#define GRAPHIC_SIZE 96
#define GRAPHIC_SIZE_12 GRAPHIC_SIZE/2
#define GRAPHIC_SIZE_12M1 GRAPHIC_SIZE_12 - 1
#define GRAPHIC_SIZE_18 GRAPHIC_SIZE/8
#define GRAPHIC_POS_START 45

#define RADAR_RADIUS  44
#define RADAR_RADIUS_12 RADAR_RADIUS/2
#define RADAR_RADIUSM2 RADAR_RADIUS - 2
#define RADAR_RADIUSM10 RADAR_RADIUS - 10
#define RADAR_DIAMETER RADAR_RADIUS*2 

#define GRAPHIC_POS_END GRAPHIC_POS_START + GRAPHIC_SIZE

#define RADAR_CENTER_X GRAPHIC_SIZE/2
#define RADAR_CENTER_XP4 RADAR_CENTER_X + 4
#define RADAR_CENTER_XP8 RADAR_CENTER_X + 8
#define RADAR_CENTER_XP12 RADAR_CENTER_X + 12
#define RADAR_CENTER_XP16 RADAR_CENTER_X + 16
#define RADAR_CENTER_XM3 RADAR_CENTER_X - 3

#define RADAR_CENTER_Y GRAPHIC_SIZE/2
#define RADAR_CENTER_YM3 RADAR_CENTER_Y - 3
#define RCYPRRM4 RADAR_CENTER_Y + (RADAR_RADIUS - 4)
#define RCYPRR12 RADAR_CENTER_Y + RADAR_RADIUS_12

#define MAX_RADAR_DISTANCE_HIGH MAX_RADAR_DISTANCE/100
#define MAX_RADAR_DISTANCE_MID  ((MAX_RADAR_DISTANCE/10) - ((MAX_RADAR_DISTANCE/100) * 10))
#define MAX_RADAR_DISTANCE_LOW  (MAX_RADAR_DISTANCE - ((MAX_RADAR_DISTANCE/10) * 10))

#define MAX_RADAR_DISTANCE_HALF_SIZE MAX_RADAR_DISTANCE/2

#define MAX_RADAR_DISTANCE_HALF_SIZE_HIGH MAX_RADAR_DISTANCE_HALF_SIZE/100
#define MAX_RADAR_DISTANCE_HALF_SIZE_MID  ((MAX_RADAR_DISTANCE_HALF_SIZE/10) - ((MAX_RADAR_DISTANCE_HALF_SIZE/100) * 10))
#define MAX_RADAR_DISTANCE_HALF_SIZE_LOW  (MAX_RADAR_DISTANCE_HALF_SIZE - ((MAX_RADAR_DISTANCE_HALF_SIZE/10) * 10))

#define RADAR_DISTANCE_FACTOR RADAR_RADIUS/MAX_RADAR_DISTANCE

//menu//////////////////////////////////////////////////////////////
#define MENU_START_LINE 80                          //line of start printing menu items
#define ONE_LINE_IN_MENU 20                         // 16 chars per line
#define ONE_LINE_IN_MENU_SIZE ONE_LINE_IN_MENU * 7  // n-chars size 7x8         
#define MAX_MENU_ITEMS 7                            //lines              
#define MAX_MENU_PAGES 10                           //pages
#define LEFT_MARGIN_IN_MENU 7
#define LAST_CONFIG_PAGE 8

//page index
#define INTERNAL_SETTINGS1 1
#define INTERNAL_SETTINGS2 2
#define PID1     3     
#define PID2     4     
#define PID3     5
#define MODE1    6
#define MODE2    7
#define SENSOR2  8
#define SENSOR1  9
#define INFO     10

//Layout components
#define LEFT_MARGIN_LAYOUT 6

#define RSSIBAR_PADDING 17
#define GPS_NUMSAT_BAR_MARGIN 15

#define RSSIBAR_Y_POS  40
#define TIMEBATBAR_Y_POS  RSSIBAR_Y_POS + 9      //49
#define CURBAR_Y_POS  TIMEBATBAR_Y_POS + 18      //66

#define TOPBAR_CENTER_PADDING 4
#define CENTER_PADDING 6
#define BAT2_PADDING 19

#define COMPASSBAR_Y_POS  83
#define POINT_MARGIN 23
#define COMPASS_PADDING 1
#define COMPASSBAR_MARGIN 16
#define HEADINGBAR_MARGIN 20
#define ARROW_MARGIN  20

//Mode config 
#define BOXACC       0
#define BOXBARO      1
#define BOXMAG       2
#define BOXCAMSTAB   3
#define BOXCAMTRIG   4
#define BOXARM       5
#define BOXGPSHOME   6
#define BOXGPSHOLD   7
#define BOXPASSTHRU  8
#define BOXHEADFREE  9
#define BOXBEEPERON  10
#define BOXLEDMAX    11 // we want maximum illumination
#define BOXLLIGHTS   12 // enable landing lights at any altitude
#define BOXHEADADJ   13 // acquire heading for HEADFREE mode

//Serial routine ////////////////////////////////////////////////////////////////////////////////////////////////////////
#define MSP_IDENT                100   //out message         multitype + multiwii version + protocol version + capability variable
#define MSP_STATUS               101   //out message         cycletime & errors_count & sensor present & box activation
#define MSP_RAW_IMU              102   //out message         9 DOF
#define MSP_SERVO                103   //out message         8 servos
#define MSP_MOTOR                104   //out message         8 motors
#define MSP_RC                   105   //out message         8 rc chan
#define MSP_RAW_GPS              106   //out message         fix, numsat, lat, lon, alt, speed
#define MSP_COMP_GPS             107   //out message         distance home, direction home
#define MSP_ATTITUDE             108   //out message         2 angles 1 heading
#define MSP_ALTITUDE             109   //out message         1 altitude
#define MSP_BAT                  110   //out message         vbat, powermetersum
#define MSP_RC_TUNING            111   //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112   //out message         up to 16 P I D (8 are used)
#define MSP_BOX                  113   //out message         up to 16 checkbox (11 are used)
#define MSP_MISC                 114   //out message         powermeter trig + 8 free for future use
#define MSP_MOTOR_PINS           115   //out message         which pins are in use for motors & servos, for GUI 
#define MSP_BOXNAMES             116   //out message         the aux switch names
#define MSP_PIDNAMES             117   //out message         the PID names
#define MSP_WP                   118   //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold

#define MSP_SET_RAW_RC           200   //in message          8 rc chan
#define MSP_SET_RAW_GPS          201   //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202   //in message          up to 16 P I D (8 are used)
#define MSP_SET_BOX              203   //in message          up to 16 checkbox (11 are used)
#define MSP_SET_RC_TUNING        204   //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205   //in message          no param
#define MSP_MAG_CALIBRATION      206   //in message          no param
#define MSP_SET_MISC             207   //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208   //in message          no param
#define MSP_WP_SET               209   //in message          sets a given WP (WP#,lat, lon, alt, flags)

#define MSP_EEPROM_WRITE         250   //in message          no param
#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4


//start time title
#define TIMEBAR_TITLE_SIZE 3
#define TIMEBAR_TITLE_START 0  
#define TIMEBAR_TITLE_END TIMEBAR_TITLE_START + (TIMEBAR_TITLE_SIZE * 7)

//start time value
#define TIMEBAR_SIZE 3
#define TIMEBAR_START TIMEBAR_TITLE_END  
#define TIMEBAR_END TIMEBAR_START + (TIMEBAR_SIZE * 7)

//fly time title
#define FLYTIMEBAR_TITLE_SIZE 3
#define FLYTIMEBAR_TITLE_START TIMEBAR_END  
#define FLYTIMEBAR_TITLE_END FLYTIMEBAR_TITLE_START + (FLYTIMEBAR_TITLE_SIZE * 7)

//fly time value
#define FLYTIMEBAR_SIZE 3
#define FLYTIMEBAR_START FLYTIMEBAR_TITLE_END  
#define FLYTIMEBAR_END FLYTIMEBAR_START + (FLYTIMEBAR_SIZE * 7)

//functions bar
#define FUNCTIONSBAR_SIZE 9
#define FUNCTIONSBAR_START  FLYTIMEBAR_END
#define FUNCTIONSBAR_END  FUNCTIONSBAR_START + (FUNCTIONSBAR_SIZE * 7)

//rssi bar
#define RSSIBAR_SIZE 8
#define RSSIBAR_START  FUNCTIONSBAR_END
#define RSSIBAR_END  RSSIBAR_START + (RSSIBAR_SIZE * 7)

//current icon
#define CURRENTBAR_SIZE 3
#define CURRENTBAR_START  RSSIBAR_END
#define CURRENTBAR_END  CURRENTBAR_START + (CURRENTBAR_SIZE * 7)

//capacity (mAh)
#define CAPACITYBAR_SIZE 3
#define CAPACITYBAR_START  CURRENTBAR_END
#define CAPACITYBAR_END  CAPACITYBAR_START + (CAPACITYBAR_SIZE * 7)

//bat2 image
#define BAT2BAR_SIZE 3
#define BAT2BAR_START  CAPACITYBAR_END
#define BAT2BAR_END  BAT2BAR_START + (BAT2BAR_SIZE * 7)

//latitude bar
#define LATBAR_SIZE 9
#define LATBAR_START  BAT2BAR_END
#define LATBAR_END  LATBAR_START + (LATBAR_SIZE * 7)

//longitude bar
#define LONBAR_SIZE 9
#define LONBAR_START  LATBAR_END
#define LONBAR_END  LONBAR_START + (LONBAR_SIZE * 7)

//gps satelite bar
#define NUMSATBAR_SIZE 3
#define NUMSATBAR_START  LONBAR_END
#define NUMSATBAR_END  NUMSATBAR_START + (NUMSATBAR_SIZE * 7)

//gps distance bar
#define GPSDISTANCEBAR_SIZE 6
#define GPSDISTANCEBAR_START  NUMSATBAR_END
#define GPSDISTANCEBAR_END  GPSDISTANCEBAR_START + (GPSDISTANCEBAR_SIZE * 7)

//heading bar
#define HEADINGBAR_SIZE 4
#define HEADINGBAR_START  GPSDISTANCEBAR_END
#define HEADINGBAR_END  HEADINGBAR_START + (HEADINGBAR_SIZE * 7)

//compass bar
#define COMPASSBAR_SIZE 11
#define COMPASSBAR_START  HEADINGBAR_END
#define COMPASSBAR_END  COMPASSBAR_START + (COMPASSBAR_SIZE * 7)

//bat2 voltage value
#define BAT2VOLTAGEBAR_SIZE 3
#define BAT2VOLTAGEBAR_START  COMPASSBAR_END
#define BAT2VOLTAGEBAR_END  BAT2VOLTAGEBAR_START + (BAT2VOLTAGEBAR_SIZE * 7)

//current value
#define CURRENTVALUEBAR_SIZE 5
#define CURRENTVALUEBAR_START  BAT2VOLTAGEBAR_END
#define CURRENTVALUEBAR_END  CURRENTVALUEBAR_START + (CURRENTVALUEBAR_SIZE * 7)

//capacity value
#define CAPACITYVALUEBAR_SIZE 5
#define CAPACITYVALUEBAR_START  CURRENTVALUEBAR_END
#define CAPACITYVALUEBAR_END  CAPACITYVALUEBAR_START + (CAPACITYVALUEBAR_SIZE * 7)

//altitude title bar
#define ALTITUDE_TITLE_SIZE 3
#define ALTITUDE_TITLE_START  CAPACITYVALUEBAR_END
#define ALTITUDE_TITLE_END  ALTITUDE_TITLE_START + (ALTITUDE_TITLE_SIZE * 7)

//altitude units bar
#define ALTITUDE_UNITS_SIZE 3
#define ALTITUDE_UNITS_START  ALTITUDE_TITLE_END
#define ALTITUDE_UNITS_END  ALTITUDE_UNITS_START + (ALTITUDE_UNITS_SIZE * 7)

//altitude title bar
#define SPEED_TITLE_SIZE 3
#define SPEED_TITLE_START  ALTITUDE_UNITS_END
#define SPEED_TITLE_END  SPEED_TITLE_START + (SPEED_TITLE_SIZE * 7)

//altitude units bar
#define SPEED_UNITS_SIZE 3
#define SPEED_UNITS_START  SPEED_TITLE_END
#define SPEED_UNITS_END  SPEED_UNITS_START + (SPEED_UNITS_SIZE * 7)

//gps home direction 
#define ARROWBAR_SIZE 1
#define ARROWBAR_START  SPEED_UNITS_END
#define ARROWBAR_END ARROWBAR_START + ARROWBAR_SIZE*32

//bat1 big numbers bar
#define BAT1_BIGNUMBAR_SIZE 2
#define BAT1_BIGNUMBAR_START  ARROWBAR_END
#define BAT1_BIGNUMBAR_END  BAT1_BIGNUMBAR_START + (BAT1_BIGNUMBAR_SIZE * 32)

//bat2 big numbers bar
#define BAT2_BIGNUMBAR_SIZE 2
#define BAT2_BIGNUMBAR_START  BAT1_BIGNUMBAR_END
#define BAT2_BIGNUMBAR_END  BAT2_BIGNUMBAR_START + (BAT2_BIGNUMBAR_SIZE * 32)

//flytime big numbers bar
#define FLYTIME_BIGNUMBAR_SIZE 2
#define FLYTIME_BIGNUMBAR_START  BAT2_BIGNUMBAR_END
#define FLYTIME_BIGNUMBAR_END  FLYTIME_BIGNUMBAR_START + (FLYTIME_BIGNUMBAR_SIZE * 32)

//starttime big numbers bar
#define STARTTIME_BIGNUMBAR_SIZE 3
#define STARTTIME_BIGNUMBAR_START  FLYTIME_BIGNUMBAR_END
#define STARTTIME_BIGNUMBAR_END  STARTTIME_BIGNUMBAR_START + (STARTTIME_BIGNUMBAR_SIZE * 32)

//altitude big numbers bar
#define ALTITUDE_BIGNUMBAR_SIZE 3
#define ALTITUDE_BIGNUMBAR_START  STARTTIME_BIGNUMBAR_END
#define ALTITUDE_BIGNUMBAR_END  ALTITUDE_BIGNUMBAR_START + (ALTITUDE_BIGNUMBAR_SIZE * 32)

//speed big numbers bar
#define SPEED_BIGNUMBAR_SIZE 2
#define SPEED_BIGNUMBAR_START  ALTITUDE_BIGNUMBAR_END
#define SPEED_BIGNUMBAR_END  SPEED_BIGNUMBAR_START + (SPEED_BIGNUMBAR_SIZE * 32)

//bat1 value in extended buffer (variable shared with compass screen)
#define BAT1BAR_SIZE 3
#define BAT1BAR_START  VIDEO_BUFFER_SIZE
#define BAT1BAR_END  BAT1BAR_START + (BAT1BAR_SIZE * 7)

//bat1 voltage value
#define BAT1VOLTAGEBAR_SIZE 5
#define BAT1VOLTAGEBAR_START  BAT1BAR_END
#define BAT1VOLTAGEBAR_DECIMAL_START  BAT1VOLTAGEBAR_START + 2*7
#define BAT1VOLTAGEBAR_END  BAT1VOLTAGEBAR_START + (BAT1VOLTAGEBAR_SIZE * 7)

#define CHARSIZE 7  //size of char 7 bytes


//Screen & Horizont panel settings
#define FLIGHT_PANEL_START_LINE 94

#if (VIDEO_TYPE == NTSC)
  #define TOP_MARGIN 120
#else
  #define TOP_MARGIN 40  
#endif

#define MAIN_PANEL_CENTER_POINT 6
#define MAIN_PANEL_RADIUS       1

//char to font
#define C(x)  x-54

const unsigned char header[] PROGMEM =                     
{ 
  #if (VIDEO_TYPE == NTSC)
    C('J'),C('O'),C('H'),C('N'), 10,C('G'),C('O'),C('L'),C('F'),10, C('O'),C('S'),C('D'),10,C('V'),0,38,1,10,C('N'),C('T'),C('S'),C('C'),
  #else
    C('J'),C('O'),C('H'),C('N'), 10,C('G'),C('O'),C('L'),C('F'),10, C('O'),C('S'),C('D'),10,C('V'),0,38,1,10,C('P'),C('A'),C('L'),
  #endif
};

#define HEADER sizeof(header)
#define HEADER_SIZE HEADER * 7


const unsigned char layout[] PROGMEM =
{ 
  //start time title
  C('O'),C('N'),C('N'),10,  
  //start time value
  36,0,0,0,
  //fly time title
  C('F'),C('L'),C('Y'),
  //fly time value
  36,0,0,
  //functions bar
  10,10,10,10,10,10,10,10,10,
  //rssi bar
  47,52,53,54,55,56,0,0,
  //current bar
  65,10,10,
  //capacity bar
  51,C('A'),62,
  //bat2 bar
  57,57,59,
  //latitude bar
  0,0,0,38,0,0,0,0,0,
  //longitude bar
  0,0,0,38,0,0,0,0,0,
  //numsat
  48,10,10,
  //gps distance 
  10,10,10,10,0,51,
  //heading
  0,0,0,66,
  //compass bar
  63,63,63,63,63,63,63,63,63,63,63,
  //bat2voltage
  38,0,C('V'),
  //current value
  10,0,38,0,C('A'), 
  //capacity value
  0,10,10,10,10, 
  //altitude title 
  C('A'),C('L'),C('T'),   
  //altitude units 
  C('M'),10,10,  
  //speed title 
  C('S'),C('P'),C('D'),   
  //speed units 
  C('K'),C('M'),C('H'),    
  //arrow bar
  10,10,10,10,10,
};

const unsigned char menuPages[] PROGMEM = 
{ 
  /////////////////////////////INTERNAL SETTINGS 1////////////////////////////////////////
  'I'-54,'N'-54,'P'-54,'U'-54,'T'-54,1,39,2,10,10,10,10,10,10,10,INTERNAL_SETTINGS1/10,INTERNAL_SETTINGS1%10,39,MAX_MENU_PAGES/10,MAX_MENU_PAGES%10,
  
  #ifdef BAT1
    10,'B'-54,'A'-54,'T'-54,1,10,'C'-54,'R'-54,'T'-54,36,10,10,10,10,10,10,10,38,10,'V'-54,
  #else
    10,'B'-54,'A'-54,'T'-54,1,10,'C'-54,'R'-54,'T'-54,36,10,10,10,10,10,'N'-54,'O'-54,'N'-54,'E'-54,10,
  #endif
  
  #ifdef BAT2
    10,'B'-54,'A'-54,'T'-54,2,10,'C'-54,'R'-54,'T'-54,36,10,10,10,10,10,10,10,38,10,'V'-54,
  #else
    10,'B'-54,'A'-54,'T'-54,2,10,'C'-54,'R'-54,'T'-54,36,10,10,10,10,10,'N'-54,'O'-54,'N'-54,'E'-54,10,
  #endif
  
  10,'R'-54,'S'-54,'S'-54,'I'-54,36,10,10,10,10,10,10,10,10,10,'C'-54,'A'-54,'L'-54,10,10,
  10,'S'-54,'C'-54,'R'-54,38,'S'-54,'W'-54,36,10,10,10,10,10,10,10,'A'-54,'U'-54,'X'-54,10,10,  //pos 18
  10,'R'-54,'C'-54,'S'-54,'E'-54,'N'-54,36,10,10,10,10,10,10,10,10,10,10,10,10,10,
  10,'S'-54,'A'-54,'V'-54,'E'-54,10,'C'-54,'O'-54,'N'-54,'F'-54,10,10,10,10,10,'E'-54,'X'-54,'I'-54,'T'-54,10,  

  /////////////////////////////INTERNAL SETTINGS 2////////////////////////////////////////
  'I'-54,'N'-54,'P'-54,'U'-54,'T'-54,2,39,2,10,10,10,10,10,10,10,INTERNAL_SETTINGS2/10,INTERNAL_SETTINGS2%10,39,MAX_MENU_PAGES/10,MAX_MENU_PAGES%10,
  
  #ifdef BAT1
    10,'B'-54,'A'-54,'T'-54,1,10,'A'-54,'D'-54,'J'-54,36,10,10,10,10,10,10,10,38,10,'V'-54,       //pos 15,16,18
  #else
    10,'B'-54,'A'-54,'T'-54,1,10,'A'-54,'D'-54,'J'-54,36,10,10,10,10,10,'N'-54,'O'-54,'N'-54,'E'-54,10,
  #endif
  
  #ifdef BAT2
    10,'B'-54,'A'-54,'T'-54,2,10,'A'-54,'D'-54,'J'-54,36,10,10,10,10,10,10,10,38,10,'V'-54,
  #else
    10,'B'-54,'A'-54,'T'-54,2,10,'A'-54,'D'-54,'J'-54,36,10,10,10,10,10,'N'-54,'O'-54,'N'-54,'E'-54,10,
  #endif
  
  #ifdef CURRENT_SENSOR
    10,'C'-54,'U'-54,'R'-54,38,'T'-54,'Y'-54,'P'-54,'E'-54,36,10,10,10,10,10,10,10,10,10,10,      //pos 15
  #else
    10,'C'-54,'U'-54,'R'-54,38,'T'-54,'Y'-54,'P'-54,'E'-54,36,10,10,10,10,10,'N'-54,'O'-54,'N'-54,'E'-54,10,      
  #endif
  
  10,'R'-54,'O'-54,'L'-54,10,'S'-54,'E'-54,'N'-54,36,10,10,10,10,10,10,10,10,10,10,10,          //pos 15,16
  10,'P'-54,'I'-54,'T'-54,10,'S'-54,'E'-54,'N'-54,36,10,10,10,10,10,10,10,10,10,10,10,          //pos 15,16
  10,'S'-54,'A'-54,'V'-54,'E'-54,10,'C'-54,'O'-54,'N'-54,'F'-54,10,10,10,10,10,'E'-54,'X'-54,'I'-54,'T'-54,10,  
  
  /////////////////////////////////////////////////////////////////////
  'P'-54,'I'-54,'D'-54,1,39,3,10,10,10,10,10,10,10,10,10,PID1/10,PID1%10,39,MAX_MENU_PAGES/10,MAX_MENU_PAGES%10,
  10,'R'-54,'O'-54,'L'-54,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
  10,'P'-54,'I'-54,'T'-54,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
  10,'Y'-54,'A'-54,'W'-54,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
  10,'A'-54,'L'-54,'T'-54,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
  10,'P'-54,'O'-54,'S'-54,10,10,10,10,10,10,10,10,10,10,10,10,10,'N'-54,'C'-54,10,
  10,'S'-54,'A'-54,'V'-54,'E'-54,10,'P'-54,'I'-54,'D'-54,10,10,10,10,10,10,'E'-54,'X'-54,'I'-54,'T'-54,10,  

  /////////////////////////////////////////////////////////////////////
  'P'-54,'I'-54,'D'-54,2,39,3,10,10,10,10,10,10,10,10,10,PID2/10,PID2%10,39,MAX_MENU_PAGES/10,MAX_MENU_PAGES%10,
  10,'P'-54,'S'-54,'R'-54,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
  10,'N'-54,'V'-54,'R'-54,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
  10,'L'-54,'V'-54,'L'-54,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
  10,'M'-54,'A'-54,'G'-54,10,10,10,10,10,10,10,'N'-54,'C'-54,10,10,10,10,'N'-54,'C'-54,10,
  10,'V'-54,'E'-54,'L'-54,10,10,'N'-54,'C'-54,10,10,10,'N'-54,'C'-54,10,10,10,10,'N'-54,'C'-54,10,
  10,'S'-54,'A'-54,'V'-54,'E'-54,10,'P'-54,'I'-54,'D'-54,10,10,10,10,10,10,'E'-54,'X'-54,'I'-54,'T'-54,10,  

  /////////////////////////////////////////////////////////////////////
  'P'-54,'I'-54,'D'-54,3,39,3,10,10,10,10,10,10,10,10,10,PID3/10,PID3%10,39,MAX_MENU_PAGES/10,MAX_MENU_PAGES%10,
  10,'R'-54,'T'-54,'E'-54,10,10,10,10,10,10,10,'N'-54,'C'-54,10,10,10,10,'N'-54,'C'-54,10,
  10,'E'-54,'X'-54,'P'-54,10,10,10,10,10,10,10,'N'-54,'C'-54,10,10,10,10,'N'-54,'C'-54,10,
  10,'R'-54,'P'-54,'R'-54,10,10,10,10,10,10,10,'N'-54,'C'-54,10,10,10,10,'N'-54,'C'-54,10,         //roll pitch rate
  10,'Y'-54,'R'-54,'T'-54,10,10,10,10,10,10,10,'N'-54,'C'-54,10,10,10,10,'N'-54,'C'-54,10,         //yaw rate
  10,'D'-54,'T'-54,'R'-54,10,10,10,10,10,10,10,'N'-54,'C'-54,10,10,10,10,'N'-54,'C'-54,10,
  10,'S'-54,'A'-54,'V'-54,'E'-54,10,'P'-54,'I'-54,'D'-54,10,10,10,10,10,10,'E'-54,'X'-54,'I'-54,'T'-54,10,  

  /////////////////////////////////////////////////////////////////////
  'M'-54,'O'-54,'D'-54,'E'-54,1,39,2,10,'A'-54,1,2,3,4,10,10,MODE1/10,MODE1%10,39,MAX_MENU_PAGES/10,MAX_MENU_PAGES%10,
  10,'L'-54,'V'-54,'L'-54,10,68,10,69,10,68,10,69,10,68,10,69,10,68,10,69,
  10,'B'-54,'A'-54,'R'-54,10,68,10,69,10,68,10,69,10,68,10,69,10,68,10,69,
  10,'M'-54,'A'-54,'G'-54,10,68,10,69,10,68,10,69,10,68,10,69,10,68,10,69,
  10,'C'-54,'M'-54,'S'-54,10,68,10,69,10,68,10,69,10,68,10,69,10,68,10,69,
  10,'C'-54,'M'-54,'T'-54,10,68,10,69,10,68,10,69,10,68,10,69,10,68,10,69,
  10,'C'-54,'H'-54,'A'-54,'N'-54,'G'-54,'E'-54,10,'M'-54,'O'-54,'D'-54,'E'-54,10,10,10,'E'-54,'X'-54,'I'-54,'T'-54,10,  

  /////////////////////////////////////////////////////////////////////
  'M'-54,'O'-54,'D'-54,'E'-54,2,39,2,10,'A'-54,1,2,3,4,10,10,MODE2/10,MODE2%10,39,MAX_MENU_PAGES/10,MAX_MENU_PAGES%10,
  10,'A'-54,'R'-54,'M'-54,10,68,10,69,10,68,10,69,10,68,10,69,10,68,10,69,
  10,'G'-54,'H'-54,'E'-54,10,68,10,69,10,68,10,69,10,68,10,69,10,68,10,69,
  10,'G'-54,'H'-54,'D'-54,10,68,10,69,10,68,10,69,10,68,10,69,10,68,10,69,
  10,'P'-54,'A'-54,'S'-54,10,68,10,69,10,68,10,69,10,68,10,69,10,68,10,69,
  10,'H'-54,'D'-54,'E'-54,10,68,10,69,10,68,10,69,10,68,10,69,10,68,10,69,
  10,'C'-54,'H'-54,'A'-54,'N'-54,'G'-54,'E'-54,10,'M'-54,'O'-54,'D'-54,'E'-54,10,10,10,'E'-54,'X'-54,'I'-54,'T'-54,10,  

  /////////////////////////////////////////////////////////////////////
  'S'-54,'E'-54,'N'-54,'S'-54,'O'-54,'R'-54,'S'-54,1,39,2,10,10,10,10,10,SENSOR2/10,SENSOR2%10,39,MAX_MENU_PAGES/10,MAX_MENU_PAGES%10,
  10,'A'-54,'C'-54,'C'-54,10,10,10,10,10,10,10,10,10,10,10,'C'-54,'A'-54,'L'-54,10,10,
  10,'M'-54,'A'-54,'G'-54,10,10,10,10,10,10,10,10,10,10,10,'C'-54,'A'-54,'L'-54,10,10,
  10,'G'-54,'P'-54,'S'-54,10,'H'-54,'O'-54,'M'-54,'E'-54,10,10,10,10,10,10,'S'-54,'E'-54,'T'-54,10,10,
  10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
  10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
  10,'S'-54,'A'-54,'V'-54,'E'-54,10,'A'-54,'L'-54,'L'-54,10,10,10,10,10,10,'E'-54,'X'-54,'I'-54,'T'-54,10,

  /////////////////////////////////////////////////////////////////////
  'S'-54,'E'-54,'N'-54,'S'-54,'O'-54,'R'-54,'S'-54,2,39,2,10,10,10,10,10,SENSOR1/10,SENSOR1%10,39,MAX_MENU_PAGES/10,MAX_MENU_PAGES%10,
  'A'-54,'C'-54,'C'-54,39,'N'-54,'K'-54,36,10,68,10,10,10,69,10,10,68,10,10,10,69,
  'B'-54,'A'-54,'R'-54,'O'-54,10,10,36,10,68,10,10,10,69,10,10,68,10,10,10,69,
  'M'-54,'A'-54,'G'-54,10,10,10,36,10,68,10,10,10,69,10,10,68,10,10,10,69,
  'G'-54,'P'-54,'S'-54,10,10,10,36,10,68,10,10,10,69,10,10,68,10,10,10,69,
  'I'-54,2,'C'-54,10,'E'-54,'R'-54,'R'-54,36,10,10,10,10,10,10,10,10,10,10,10,10,
  10,'S'-54,'A'-54,'V'-54,'E'-54,10,'A'-54,'L'-54,'L'-54,10,10,10,10,10,10,'E'-54,'X'-54,'I'-54,'T'-54,10,

  /////////////////////////////////////////////////////////////////////
  'I'-54,'N'-54,'F'-54,'O'-54,10,10,10,10,10,10,10,10,10,10,10,INFO/10,INFO%10,39,MAX_MENU_PAGES/10,MAX_MENU_PAGES%10,
  'V'-54,'E'-54,'R'-54,'S'-54,10,10,36,10,10,10,10,10,10,10,10,10,10,10,10,10,
  'R'-54,'O'-54,'L'-54,'L'-54,10,10,36,10,10,10,10,10,10,10,10,10,10,10,10,10,
  'P'-54,'I'-54,'T'-54,'C'-54,'H'-54,10,36,10,10,10,10,10,10,10,10,10,10,10,10,10,
  'Y'-54,'A'-54,'W'-54,10,10,10,36,10,10,10,10,10,10,10,10,10,10,10,10,10,
  'T'-54,'H'-54,'R'-54,'O'-54,'T'-54,10,36,10,10,10,10,10,10,10,10,10,10,10,10,10,
  10,'S'-54,'A'-54,'V'-54,'E'-54,10,'A'-54,'L'-54,'L'-54,10,10,10,10,10,10,'E'-54,'X'-54,'I'-54,'T'-54,10,

  /////////////////////////////////////////////////////////////////////
  ///Result page
  /////////////////////////////////////////////////////////////////////
  'R'-54,'E'-54,'S'-54,'U'-54,'L'-54,'T'-54,10,10,10,10,10,10,10,10,10,10,'P'-54,'A'-54,'G'-54,'E'-54,
  'F'-54,'L'-54,38,'T'-54,'I'-54,'M'-54,'E'-54,10,36,10,10,10,36,10,10,10,10,10,10,10,
  'H'-54,'I'-54,38,'A'-54,'L'-54,'T'-54,10,10,36,10,10,10,10,10,10,10,'M'-54,10,10,10,
  'H'-54,'I'-54,38,'D'-54,'I'-54,'S'-54,'T'-54,10,36,10,10,10,10,10,10,10,'M'-54,10,10,10,
  'H'-54,'I'-54,38,'S'-54,'P'-54,'E'-54,'E'-54,'D'-54,36,10,10,10,10,10,10,10,'K'-54,'M'-54,39,'H'-54,
  'H'-54,'I'-54,38,'C'-54,'U'-54,'R'-54,'R'-54,10,36,10,10,10,38,10,10,10,'A'-54,10,10,10,
  'M'-54,'O'-54,'V'-54,'E'-54,10,'T'-54,'H'-54,'R'-54,'O'-54,'T'-54,'T'-54,'L'-54,'E'-54,10,10,10,10,10,10,10,
};

// 16 x 16 bitmap.
const unsigned char bigNumbers[] PROGMEM = 
{
  #ifdef HITECH_FONT
    //0
    0b00000000,0b00000000,
    0b00000000,0b00000000,  
    0b00011111,0b11111100,
    0b00011111,0b11111100,
    0b00011110,0b00111100,
    0b00011110,0b00111100,
    0b00011110,0b00111100,
    0b01111110,0b00111100,
    0b01111110,0b00111100,
    0b01111110,0b00111100,
    0b01111110,0b00111100,
    0b01111110,0b00111100,
    0b01111110,0b00111100,
    0b01111111,0b11111100,
    0b01111111,0b11111100,
    0b00000000,0b00000000,

    //1
    0b00000000,0b00000000,
    0b00000000,0b00000000,  
    0b00000011,0b11000000,
    0b00000011,0b11000000,
    0b00000011,0b11000000,
    0b00001111,0b11000000,
    0b00001111,0b11000000,
    0b00001111,0b11000000,
    0b00001111,0b11000000,
    0b00001111,0b11000000,
    0b00001111,0b11000000,
    0b00001111,0b11000000,
    0b00001111,0b11000000,
    0b00001111,0b11000000,
    0b00001111,0b11000000,
    0b00000000,0b00000000,

    //2
    0b00000000,0b00000000,
    0b00000000,0b00000000,  
    0b11111111,0b11111000,
    0b11111111,0b11111000,
    0b11110000,0b01111000,
    0b11110000,0b01111000,
    0b00000000,0b01111000,
    0b11111111,0b11111000,
    0b11111111,0b11111000,
    0b11111100,0b00000000,
    0b11111100,0b00000000,
    0b11111100,0b01111000,
    0b11111100,0b01111000,
    0b11111111,0b11111000,
    0b11111111,0b11111000,
    0b00000000,0b00000000,

    //3
    0b00000000,0b00000000,
    0b00000000,0b00000000,
    0b01111111,0b11110000,
    0b01111111,0b11110000,
    0b00000000,0b11110000,
    0b00000000,0b11110000,
    0b00000000,0b11110000,
    0b00000111,0b11111100,
    0b00000111,0b11111100,
    0b00000000,0b11111100,
    0b00000000,0b11111100,
    0b00000000,0b11111100,
    0b00000000,0b11111100,
    0b01111111,0b11111100,
    0b01111111,0b11111100,
    0b00000000,0b00000000,

    //4
    0b00000000,0b00000000,
    0b00000000,0b00000000,
    0b01111000,0b11110000,
    0b01111000,0b11110000,
    0b01111000,0b11110000,
    0b01111000,0b11110000,
    0b01111000,0b11110000,
    0b01111111,0b11111100,
    0b01111111,0b11111100,
    0b00000000,0b11111100,
    0b00000000,0b11111100,
    0b00000000,0b11111100,
    0b00000000,0b11111100,
    0b00000000,0b11111100,
    0b00000000,0b11111100,
    0b00000000,0b00000000,

    //5
    0b00000000,0b00000000,
    0b00000000,0b00000000,
    0b11111111,0b11111000,
    0b11111111,0b11111000,
    0b11110000,0b01111000,
    0b11110000,0b01111000,
    0b11110000,0b00000000,
    0b11111111,0b11111000,
    0b11111111,0b11111000,
    0b00000001,0b11111000,
    0b00000001,0b11111000,
    0b11110001,0b11111000,
    0b11110001,0b11111000,
    0b11111111,0b11111000,
    0b11111111,0b11111000,
    0b00000000,0b00000000,

    //6
    0b00000000,0b00000000,
    0b00000000,0b00000000,
    0b00111111,0b11111000,
    0b00111111,0b11111000,
    0b00111100,0b01111000,
    0b00111100,0b01111000,
    0b00111100,0b00000000,
    0b11111111,0b11111000,
    0b11111111,0b11111000,
    0b11111100,0b01111000,
    0b11111100,0b01111000,
    0b11111100,0b01111000,
    0b11111100,0b01111000,
    0b11111111,0b11111000,
    0b11111111,0b11111000,
    0b00000000,0b00000000,

    //7
    0b00000000,0b00000000,
    0b00000000,0b00000000,
    0b01111111,0b11110000,
    0b01111111,0b11110000,
    0b00000000,0b11110000,
    0b00000000,0b11110000,
    0b00000000,0b11110000,
    0b00000000,0b11111100,
    0b00000000,0b11111100,
    0b00000000,0b11111100,
    0b00000000,0b11111100,
    0b00000000,0b11111100,
    0b00000000,0b11111100,
    0b00000000,0b11111100,
    0b00000000,0b11111100,
    0b00000000,0b00000000,

    // 8 start @256
    0b00000000,0b00000000,
    0b00000000,0b00000000,  
    0b00111111,0b11110000,
    0b00111111,0b11110000,
    0b00111100,0b11110000,
    0b00111100,0b11110000,
    0b00111100,0b11110000,
    0b01111111,0b11111100,
    0b01111111,0b11111100,
    0b01111100,0b00111100,
    0b01111100,0b00111100,
    0b01111100,0b00111100,
    0b01111100,0b00111100,
    0b01111111,0b11111100,
    0b01111111,0b11111100,
    0b00000000,0b00000000,

    // 9 start @288
    0b00000000,0b00000000,
    0b00000000,0b00000000,
    0b01111111,0b11110000,
    0b01111111,0b11110000,
    0b01111000,0b11110000,
    0b01111000,0b11110000,
    0b01111000,0b11110000,
    0b01111111,0b11111100,
    0b01111111,0b11111100,
    0b00000000,0b11111100,
    0b00000000,0b11111100,
    0b01111000,0b11111100,
    0b01111000,0b11111100,
    0b01111111,0b11111100,
    0b01111111,0b11111100,
    0b00000000,0b00000000,

  #else
    //0
    0b00000000,0b00000000,
    0b00000000,0b00000000, 
    0b00011111,0b11111000,
    0b00111111,0b11111100,
    0b00111100,0b00111100,
    0b00111100,0b00111100,
    0b00111100,0b00111100,
    0b00111100,0b00111100,
    0b00111100,0b00111100,
    0b00111100,0b00111100,
    0b00111100,0b00111100,
    0b00111100,0b00111100,
    0b00111100,0b00111100,
    0b00111111,0b11111100,
    0b00001111,0b11111000,
    0b00000000,0b00000000,

    //1
    0b00000000,0b00000000,
    0b00000000,0b00000000, 
    0b00001111,0b10000000,
    0b00011111,0b10000000,
    0b00111111,0b10000000,
    0b01111111,0b10000000,
    0b00000111,0b10000000,
    0b00000111,0b10000000,
    0b00000111,0b10000000,
    0b00000111,0b10000000,
    0b00000111,0b10000000,
    0b00000111,0b10000000,
    0b00000111,0b10000000,
    0b00000111,0b10000000,
    0b00011111,0b11100000,
    0b00000000,0b00000000,

    //2
    0b00000000,0b00000000,
    0b00000000,0b00000000, 
    0b11111111,0b11110000,
    0b11111111,0b11111000,
    0b00000000,0b01111000,
    0b00000000,0b01111000,
    0b00000000,0b01111000,
    0b01111111,0b11111000,
    0b11111111,0b11110000,
    0b11110000,0b00000000,
    0b11110000,0b00000000,
    0b11110000,0b00000000,
    0b11110000,0b00000000,
    0b11111111,0b11111000,
    0b01111111,0b11110000,
    0b00000000,0b00000000,

    //3
    0b00000000,0b00000000,
    0b00000000,0b00000000,
    0b01111111,0b11100000,
    0b01111111,0b11110000,
    0b00000000,0b11110000,
    0b00000000,0b11110000,
    0b00000000,0b11110000,
    0b00000111,0b11110000,
    0b00000111,0b11110000,
    0b00000000,0b11110000,
    0b00000000,0b11110000,
    0b00000000,0b11110000,
    0b00000000,0b11110000,
    0b01111111,0b11110000,
    0b01111111,0b11100000,
    0b00000000,0b00000000,

    //4
    0b00000000,0b00000000,
    0b00000000,0b00000000,
    0b00111000,0b11100000,
    0b01111000,0b11110000,
    0b01111000,0b11110000,
    0b01111000,0b11110000,
    0b01111000,0b11110000,
    0b01111111,0b11110000,
    0b00111111,0b11110000,
    0b00000000,0b11110000,
    0b00000000,0b11110000,
    0b00000000,0b11110000,
    0b00000000,0b11110000,
    0b00000000,0b11110000,
    0b00000000,0b01100000,
    0b00000000,0b00000000,

    //5
    0b00000000,0b00000000,
    0b00000000,0b00000000,
    0b01111111,0b11111000,
    0b11111111,0b11111000,
    0b11110000,0b00000000,
    0b11110000,0b00000000,
    0b11110000,0b00000000,
    0b11111111,0b11110000,
    0b01111111,0b11111000,
    0b00000000,0b01111000,
    0b00000000,0b01111000,
    0b00000000,0b01111000,
    0b11000000,0b01111000,
    0b11111111,0b11111000,
    0b01111111,0b11110000,
    0b00000000,0b00000000,

    //6
    0b00000000,0b00000000,
    0b00000000,0b00000000,
    0b00011111,0b11110000,
    0b00111111,0b11111000,
    0b00111100,0b00000000,
    0b00111100,0b00000000,
    0b00111100,0b00000000,
    0b00111111,0b11110000,
    0b00111111,0b11111000,
    0b00111100,0b01111000,
    0b00111100,0b01111000,
    0b00111100,0b01111000,
    0b00111100,0b01111000,
    0b00111111,0b11111000,
    0b00011111,0b11110000,
    0b00000000,0b00000000,

    //7
    0b00000000,0b00000000,
    0b00000000,0b00000000,
    0b01111111,0b11110000,
    0b01111111,0b11110000,
    0b00000000,0b11110000,
    0b00000000,0b11110000,
    0b00000000,0b11110000,
    0b00000001,0b11100000,
    0b00000011,0b11000000,
    0b00000111,0b10000000,
    0b00001111,0b00000000,
    0b00001111,0b00000000,
    0b00001111,0b00000000,
    0b00001111,0b00000000,
    0b00001111,0b00000000,
    0b00000000,0b00000000,

    // 8 start @256
    0b00000000,0b00000000,
    0b00000000,0b00000000, 
    0b00111111,0b11100000,
    0b01111111,0b11110000,
    0b01110000,0b01110000,
    0b01110000,0b01110000,
    0b01110000,0b01110000,
    0b01111111,0b11110000,
    0b01111111,0b11110000,
    0b01110000,0b01110000,
    0b01110000,0b01110000,
    0b01110000,0b01110000,
    0b01110000,0b01110000,
    0b01111111,0b11110000,
    0b00111111,0b11100000,
    0b00000000,0b00000000,

    // 9 start @288
    0b00000000,0b00000000,
    0b00000000,0b00000000,
    0b00111111,0b11100000,
    0b01111111,0b11110000,
    0b01111000,0b11110000,
    0b01111000,0b11110000,
    0b01111000,0b11110000,
    0b01111111,0b11110000,
    0b00111111,0b11110000,
    0b00000000,0b11110000,
    0b00000000,0b11110000,
    0b00000000,0b11110000,
    0b00000000,0b11110000,
    0b01111111,0b11110000,
    0b00111111,0b11100000,
    0b00000000,0b00000000,  
  #endif
};

//HOME ARROWS
const unsigned char homeArrows[] PROGMEM = 
{
  //0 start @0
  0b00000000,0b00000000,
  0b00000001,0b10000000,
  0b00000011,0b11000000,  
  0b00000111,0b11100000,  
  0b00001111,0b11110000,  
  0b00011111,0b11111000,  
  0b00111111,0b11111100,  
  0b00111111,0b11111100,  
  0b00000011,0b11000000,  
  0b00000011,0b11000000,  
  0b00000011,0b11000000,  
  0b00000011,0b11000000,  
  0b00000011,0b11000000,  
  0b00000011,0b11000000,  
  0b00000011,0b11000000, 
  0b00000001,0b10000000,  

  //1 start @32
  0b00000000,0b00000000,  
  0b00000000,0b00000000,  
  0b00000111,0b11111110,  
  0b00000111,0b11111110,  
  0b00000011,0b11111110, 
  0b00000001,0b11111110,
  0b00000001,0b11111110,  
  0b00000011,0b11111110,  
  0b00000111,0b11111110,
  0b00001111,0b10001110,  
  0b00011111,0b00000110,  
  0b00111110,0b00000000,  
  0b01111100,0b00000000,
  0b01111000,0b00000000,
  0b00000000,0b00000000,
  0b00000000,0b00000000,  

  //2 start @64
  0b00000000,0b00000000,
  0b00000000,0b00000000,
  0b00000000,0b01100000,
  0b00000000,0b01110000,
  0b00000000,0b01111000,
  0b00000000,0b11111100,
  0b01111111,0b11111110,
  0b11111111,0b11111110,
  0b11111111,0b11111110,
  0b01111111,0b11111110,
  0b00000000,0b01111100,
  0b00000000,0b01111000,
  0b00000000,0b01110000,
  0b00000000,0b01100000,
  0b00000000,0b00000000,
  0b00000000,0b00000000,

  //3 start @96
  0b00000000,0b00000000,
  0b00000000,0b00000000, 
  0b00000000,0b00000000, 
  0b01110000,0b00000000,  
  0b11111000,0b00000000,
  0b01111100,0b00000000,  
  0b00111110,0b00000110,  
  0b00011111,0b00001110, 
  0b00001111,0b10011110,
  0b00000111,0b11111110,  
  0b00000011,0b11111110,   
  0b00000001,0b11111110,    
  0b00000001,0b11111110,  
  0b00000011,0b11111110,   
  0b00000111,0b11111110, 
  0b00000111,0b11111110,

  //4 start @128
  0b00000000,0b00000000,
  0b00000001,0b10000000,
  0b00000011,0b11000000, 
  0b00000011,0b11000000, 
  0b00000011,0b11000000,  
  0b00000011,0b11000000,  
  0b00000011,0b11000000, 
  0b00000011,0b11000000,
  0b00000011,0b11000000,  
  0b00111111,0b11111100,  
  0b00111111,0b11111100,
  0b00011111,0b11111000,  
  0b00001111,0b11110000,  
  0b00000111,0b11100000,  
  0b00000011,0b11000000,  
  0b00000001,0b10000000,

  //5 start @160
  0b00000000,0b00000000,
  0b00000000,0b00000000,
  0b00000000,0b00111100,
  0b00000000,0b01111100,
  0b00000000,0b11111000,
  0b11000001,0b11110000,
  0b11100011,0b11100000,
  0b11111111,0b11000000,
  0b11111111,0b10000000,
  0b11111111,0b10000000,
  0b11111111,0b11100000,
  0b11111111,0b11110000,
  0b11111111,0b11110000,
  0b00000000,0b00000000,
  0b00000000,0b00000000,
  0b00000000,0b00000000,

  //6 start @192
  0b00000000,0b00000000,
  0b00000000,0b00000000,
  0b00000110,0b00000000,
  0b00001110,0b00000000,
  0b00011110,0b00000000,
  0b00111110,0b00000000,
  0b01111111,0b11111100,
  0b01111111,0b11111110,
  0b01111111,0b11111110,
  0b01111111,0b11111100,
  0b00111110,0b00000000,
  0b00011110,0b00000000,
  0b00001110,0b00000000,
  0b00000110,0b00000000,
  0b00000000,0b00000000,
  0b00000000,0b00000000,

  //7 start @224
  0b00000000,0b00000000,
  0b00000000,0b00000000,
  0b01111111,0b11100000,
  0b01111111,0b11100000,
  0b01111111,0b11000000,
  0b01111111,0b10000000,
  0b01111111,0b10000000,
  0b01111111,0b11000000,
  0b01111111,0b11100000,
  0b01110001,0b11110000,
  0b01100000,0b11111100,
  0b00000000,0b01111110,
  0b00000000,0b00111110,
  0b00000000,0b00000000,
  0b00000000,0b00000000,
  0b00000000,0b00000000,
  0b00000000,0b00000000
};

volatile static uint8_t lettersInRam [] = 
{
  // 0 - pos. 0 start @0 
  0b01111110,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01111110,

  // 1 - pos. 1 start @8
  0b00010000,
  0b00110000,
  0b01110000,
  0b00010000,
  0b00010000,
  0b00010000,
  0b01111110,

  // 2 - pos. 2 start @16
  0b01111110,
  0b00000010,
  0b00000010,
  0b01111110,
  0b01000000,
  0b01000000,
  0b01111110,

  // 3 - pos. 3 start @24
  0b01111110,
  0b00000010,
  0b00000010,
  0b00111110,
  0b00000010,
  0b00000010,
  0b01111110,

  // 4 - pos. 4 start @32
  0b01000010,
  0b01000010,
  0b01000010,
  0b01111110,
  0b00000010,
  0b00000010,
  0b00000010,

  // 5 - pos. 5 start @40
  0b01111110,
  0b01000000,
  0b01000000,
  0b01111110,
  0b00000010,
  0b00000010,
  0b01111110,

  // 6 - pos. 6 start @48
  0b01111110,
  0b01000000,
  0b01000000,
  0b01111110,
  0b01000010,
  0b01000010,
  0b01111110,

  // 7 - pos. 7 start @56
  0b01111110,
  0b01000010,
  0b00000100,
  0b00001000,
  0b00010000,
  0b00100000,
  0b00100000,

  // 8 - pos. 8 start @64
  0b01111110,
  0b01000010,
  0b01000010,
  0b01111110,
  0b01000010,
  0b01000010,
  0b01111110,

  // 9 - pos. 9 start @72
  0b01111110,
  0b01000010,
  0b01000010,
  0b01111110,
  0b00000010,
  0b00000010,
  0b00000010,

  // Blank - pos. 10 start @80   
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,

  // A - pos. 11 start @88
  0b01111110,
  0b01000010,
  0b01000010,
  0b01111110,
  0b01000010,  
  0b01000010,
  0b01000010,

  // B - pos. 12 start @96
  0b01111110,
  0b01000010,
  0b01000010,
  0b01111110,
  0b01000010,
  0b01000010,
  0b01111110,

  // C - pos. 13 start @104
  0b01111110,
  0b01000000,
  0b01000000,
  0b01000000,
  0b01000000,
  0b01000000,
  0b01111110,

  // D - pos. 14 start @112
  0b01111000,
  0b01000100,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01000100,
  0b01111000,

  // E - pos. 15 start @120
  0b01111110,
  0b01000000,
  0b01000000,
  0b01111110,
  0b01000000,
  0b01000000,
  0b01111110,

  // F - pos. 16 start @128
  0b01111110,
  0b01000000,
  0b01000000,
  0b01111110,
  0b01000000,
  0b01000000,
  0b01000000,

  // G - pos. 17 start @136
  0b01111110,
  0b01000000,
  0b01000000,
  0b01001110,
  0b01000010,
  0b01000010,
  0b01111110,

  // H - pos. 18 start @144
  0b01000010,
  0b01000010,
  0b01000010,
  0b01111110,
  0b01000010,
  0b01000010,
  0b01000010,

  // I - pos. 19 start @152
  0b01111110,
  0b00010000,
  0b00010000,
  0b00010000,
  0b00010000,
  0b00010000,
  0b01111110,

  // J - pos. 20 start @160
  0b00111110,
  0b00000010,
  0b00000010,
  0b00000010,
  0b01000010,
  0b01100110,
  0b00111100,
  
  //K - pos. 21 start @168
  0b01000110,
  0b01001000,
  0b01010000,
  0b01100000,
  0b01010000,
  0b01001000,
  0b01000110,

  // L - pos. 22 start @176
  0b01000000,
  0b01000000,
  0b01000000,
  0b01000000,
  0b01000000,
  0b01000000,
  0b01111110,

  // M - pos. 23 start @184
  0b01000010,
  0b01100110,
  0b01011010,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01000010,

  // N - pos. 24 start @192
  0b01000010,
  0b01100010,  
  0b01010010,
  0b01001010,
  0b01001010,
  0b01000110,
  0b01000010,
  
  // O - pos. 25 start @200
  0b01111110,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01111110,

  // P - pos. 26 start @208
  0b01111110,
  0b01000010,
  0b01000010,
  0b01111110,
  0b01000000,
  0b01000000,
  0b01000000,

  // Q - pos. 27 start @216
  0b01000010,
  0b01100110,
  0b01011010,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01000010,

  // R - pos. 28 start @224
  0b01111110,
  0b01000010,
  0b01000010,
  0b01111110,
  0b01100000,
  0b01011000,
  0b01000110,

  // S - pos. 29 start @232
  0b01111110,
  0b01000000,
  0b01000000,
  0b01111110,
  0b00000010,
  0b00000010,
  0b01111110,

  // T - pos. 30 start @240
  0b01111110,
  0b00010000,
  0b00010000,
  0b00010000,
  0b00010000,
  0b00010000,
  0b00010000,

  // U - pos. 31 start @248
  0b01000010,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01111110,

  // V - pos. 32 start @256
  0b01000010,
  0b01000110,
  0b01000100,
  0b00100100,
  0b00101100,
  0b00011000,
  0b00011000,
}; 

const unsigned char letters[] PROGMEM = 
{                          // 8 x 8 bitmap.
  
  // 0 - pos. 0 start @0 
  0b01111110,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01111110,

  // 1 - pos. 1 start @8
  0b00010000,
  0b00110000,
  0b01110000,
  0b00010000,
  0b00010000,
  0b00010000,
  0b01111110,

  // 2 - pos. 2 start @16
  0b01111110,
  0b00000010,
  0b00000010,
  0b01111110,
  0b01000000,
  0b01000000,
  0b01111110,

  // 3 - pos. 3 start @24
  0b01111110,
  0b00000010,
  0b00000010,
  0b00111110,
  0b00000010,
  0b00000010,
  0b01111110,

  // 4 - pos. 4 start @32
  0b01000010,
  0b01000010,
  0b01000010,
  0b01111110,
  0b00000010,
  0b00000010,
  0b00000010,

  // 5 - pos. 5 start @40
  0b01111110,
  0b01000000,
  0b01000000,
  0b01111110,
  0b00000010,
  0b00000010,
  0b01111110,

  // 6 - pos. 6 start @48
  0b01111110,
  0b01000000,
  0b01000000,
  0b01111110,
  0b01000010,
  0b01000010,
  0b01111110,

  // 7 - pos. 7 start @56
  0b01111110,
  0b01000010,
  0b00000100,
  0b00001000,
  0b00010000,
  0b00100000,
  0b00100000,

  // 8 - pos. 8 start @64
  0b01111110,
  0b01000010,
  0b01000010,
  0b01111110,
  0b01000010,
  0b01000010,
  0b01111110,

  // 9 - pos. 9 start @72
  0b01111110,
  0b01000010,
  0b01000010,
  0b01111110,
  0b00000010,
  0b00000010,
  0b00000010,

  // Blank - pos. 10 start @80   
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,

  // A - pos. 11 start @88
  0b01111110,
  0b01000010,
  0b01000010,
  0b01111110,
  0b01000010,  
  0b01000010,
  0b01000010,

  // B - pos. 12 start @96
  0b01111110,
  0b01000010,
  0b01000010,
  0b01111110,
  0b01000010,
  0b01000010,
  0b01111110,

  // C - pos. 13 start @104
  0b01111110,
  0b01000000,
  0b01000000,
  0b01000000,
  0b01000000,
  0b01000000,
  0b01111110,

  // D - pos. 14 start @112
  0b01111000,
  0b01000100,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01000100,
  0b01111000,

  // E - pos. 15 start @120
  0b01111110,
  0b01000000,
  0b01000000,
  0b01111110,
  0b01000000,
  0b01000000,
  0b01111110,

  // F - pos. 16 start @128
  0b01111110,
  0b01000000,
  0b01000000,
  0b01111110,
  0b01000000,
  0b01000000,
  0b01000000,

  // G - pos. 17 start @136
  0b01111110,
  0b01000000,
  0b01000000,
  0b01001110,
  0b01000010,
  0b01000010,
  0b01111110,

  // H - pos. 18 start @144
  0b01000010,
  0b01000010,
  0b01000010,
  0b01111110,
  0b01000010,
  0b01000010,
  0b01000010,

  // I - pos. 19 start @152
  0b01111110,
  0b00010000,
  0b00010000,
  0b00010000,
  0b00010000,
  0b00010000,
  0b01111110,

  // J - pos. 20 start @160
  0b00111110,
  0b00000010,
  0b00000010,
  0b00000010,
  0b01000010,
  0b01100110,
  0b00111100,
  
  //K - pos. 21 start @168
  0b01000110,
  0b01001000,
  0b01010000,
  0b01100000,
  0b01010000,
  0b01001000,
  0b01000110,

  // L - pos. 22 start @176
  0b01000000,
  0b01000000,
  0b01000000,
  0b01000000,
  0b01000000,
  0b01000000,
  0b01111110,

  // M - pos. 23 start @184
  0b01000010,
  0b01100110,
  0b01011010,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01000010,

  // N - pos. 24 start @192
  0b01000010,
  0b01100010,  
  0b01010010,
  0b01001010,
  0b01001010,
  0b01000110,
  0b01000010,
  
  // O - pos. 25 start @200
  0b01111110,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01111110,

  // P - pos. 26 start @208
  0b01111110,
  0b01000010,
  0b01000010,
  0b01111110,
  0b01000000,
  0b01000000,
  0b01000000,

  // Q - pos. 27 start @216
  0b01000010,
  0b01100110,
  0b01011010,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01000010,

  // R - pos. 28 start @224
  0b01111110,
  0b01000010,
  0b01000010,
  0b01111110,
  0b01100000,
  0b01011000,
  0b01000110,

  // S - pos. 29 start @232
  0b01111110,
  0b01000000,
  0b01000000,
  0b01111110,
  0b00000010,
  0b00000010,
  0b01111110,

  // T - pos. 30 start @240
  0b01111110,
  0b00010000,
  0b00010000,
  0b00010000,
  0b00010000,
  0b00010000,
  0b00010000,

  // U - pos. 31 start @248
  0b01000010,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01111110,

  // V - pos. 32 start @256
  0b01000010,
  0b01000110,
  0b01000100,
  0b00100100,
  0b00101100,
  0b00011000,
  0b00011000,
  
  // W - pos. 33 start @264
  0b10000010,
  0b10010010,
  0b10010010,
  0b10010010,
  0b10010010,
  0b10010010,
  0b11111110,

  // X - pos. 34 start @272
  0b01000010, 
  0b00100010, 
  0b00010100, 
  0b00001000, 
  0b00010100, 
  0b00100010, 
  0b01000010,

  // Y - pos. 35 start @280
  0b01000010, 
  0b00100010, 
  0b00010100, 
  0b00001000, 
  0b00001000, 
  0b00001000, 
  0b00001000,
  
  // : - pos. 36 start @288  
  0b00000000,
  0b00011100,
  0b00011100,
  0b00000000,
  0b00011100,
  0b00011100,
  0b00000000,

  // -  - pos. 37 start @296
  0b00000000,
  0b00000000,
  0b01111110,
  0b01111110,
  0b00000000,
  0b00000000,
  0b00000000, 

  // .  - pos. 38 start @304
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00111000,
  0b00111000,  

  // /  - pos. 39 start @312
  0b00000010,
  0b00000010,
  0b00000100,
  0b00001000,
  0b00010000,
  0b00100000,
  0b01000000,

  // %  - pos. 40 start @320
  0b01000010,
  0b01000010,
  0b00000100,
  0b00001000,
  0b00010000,
  0b00100010,
  0b01000010,

  // <- - pos. 41 start @328
  0b00100000,
  0b01100000,
  0b11111111,
  0b01100000,
  0b00100000,
  0b00000000,
  0b00000000,

  // -> - pos. 42 start @336
  0b00001000,
  0b00001100,
  0b11111110,
  0b00001100,
  0b00001000,
  0b00000000,
  0b00000000,

  // ARROW UP pos. 43 start @344
  0b00110000, 
  0b01111000,
  0b11111100,
  0b00110000,
  0b00110000,
  0b00110000,
  0b00110000,
   
  // ARROW DOWN pos. 44 start @352
  0b00001000,
  0b00001000,
  0b00001000,
  0b00001000,
  0b00111110,
  0b00011100,
  0b00001000,

  //CLOCK - pos. 45 start @360
  0b00000000,
  0b01111100,
  0b10010010,
  0b10011110,
  0b10000010,
  0b01111100,
  0b00000000,

  //BAT - pos. 46 start @368
  0b00111100,
  0b01111110,
  0b01111110,
  0b01000010,
  0b01111110,
  0b01000010,
  0b01111110,

  //SIGNAL - pos. 47 start @376
  0b00000000,
  0b10010010,
  0b01010100,
  0b00111000,
  0b00010000,
  0b00010000,
  0b00010000,

  //SATELITE - pos. 48 start @384
  0b00000010,
  0b01111010,
  0b11110100,
  0b11101110,
  0b01111100,
  0b01111000,
  0b11111110,

  //point - pos. 49 start @392
  0b00000000,
  0b00000000,
  0b00000000,
  0b00111000,
  0b01111100,
  0b00111000,
  0b00000000,

  // c - pos. 50 start @400
  0b00000000,
  0b00000000,
  0b00000000,
  0b00111110,
  0b01100000,
  0b01100000,
  0b00111110,

  // m - pos. 51 start @408
  0b00000000,
  0b00000000,
  0b00000000,
  0b11111110,
  0b10010010,
  0b10010010,
  0b10010010,

  // RSSI1 - pos. 52   
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b01111110,

  // RSSI2 - pos. 53   
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b01111110,
  0b01111110,
  0b01111110,
  
  // RSSI3 - pos. 54   
  0b00000000,
  0b00000000,
  0b01111110,
  0b01111110,
  0b01111110,
  0b01111110,
  0b01111110,
  
  // RSSI4 - pos. 55   
  0b00000000,
  0b01111110,
  0b01111110,
  0b01111110,
  0b01111110,
  0b01111110,
  0b01111110,
  
  // RSSI5 - pos. 56   
  0b01111110,
  0b01111110,
  0b01111110,
  0b01111110,
  0b01111110,
  0b01111110,
  0b01111110,
  
  //BATLEFT-FULL - pos. 57
  0b11111111,
  0b11111110,
  0b11111110,
  0b11111110,
  0b11111110,
  0b11111110,
  0b11111111,
  
  //BATCENTER-LOW - pos. 58  
  0b11111111,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b11111111,
  
  //BATRIGHT-FULL - pos. 59
  0b11111000,
  0b11111000,
  0b11111110,
  0b11111110,
  0b11111110,
  0b11111000,
  0b11111000,
   
  //BATRIGHT-LOW - pos. 60
  0b11111000,
  0b00001000,
  0b00001110,
  0b00001110,
  0b00001110,
  0b00001000,
  0b11111000,
  
  //BATLEFT-LOW - pos. 61
  0b11111111,
  0b10000000,
  0b10000000,
  0b10000000,
  0b10000000,
  0b10000000,
  0b11111111,
   
  // h  - pos. 62 start @496   
  0b00000000,
  0b10000000,
  0b10000000,
  0b11111100,
  0b10000010,
  0b10000010,
  0b10000010,

  // Compass - pos. 63 start @504   
  0b00000000,
  0b00000000,
  0b00000000,
  0b11111110,
  0b11111110,
  0b00000000,
  0b00000000,

  // Point - pos. 64 start @512    
  0b00011000,
  0b00111100,
  0b01111110,
  0b00000000,
  0b00000000, 
  0b00000000,
  0b00000000,
  
  //CURRENT - pos. 65 start @520
  0b00001000,
  0b00110000,
  0b01110000,
  0b11111110,
  0b00111100,
  0b00011000,
  0b00100000,
  
  //DEG - pos. 66 start @528
  0b00111000,
  0b00101000,
  0b00111000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,

  //HOME - pos. 67 start @536
  0b00010000,
  0b00111000,
  0b01111100,
  0b11111110,
  0b11000110,  
  0b11000110,
  0b11111110,
  
  // [ - pos. 68 start @544   
  0b11111110, 
  0b11000000,
  0b11000000,
  0b11000000,
  0b11000000,
  0b11000000,
  0b11111110,

  // ] - pos. 69 start @552   
  0b11111110, 
  0b00000110,
  0b00000110,
  0b00000110,
  0b00000110,
  0b00000110,
  0b11111110,

  // mode0 - pos. 70   
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,

  // mode1 - pos. 71 start @568   
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b01111110,

  // mode2 - pos. 72 start @576   
  0b00000000,
  0b00000000,
  0b00000000,
  0b01111110,
  0b00000000,
  0b00000000,
  0b00000000,

  // mode3 - pos. 73 start @584   
  0b00000000,
  0b00000000,
  0b00000000,
  0b01111110,
  0b00000000,
  0b00000000,
  0b01111110,

  // mode4 - pos. 74 start @592   
  0b01111110,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  
  // mode5 - pos. 75 start @600   
  0b01111110,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b01111110,
  
  // mode6 - pos. 76 start @608   
  0b01111110,
  0b00000000,
  0b00000000,
  0b01111110,
  0b00000000,
  0b00000000,
  0b00000000,
  
  // mode7 - pos. 77 start @616   
  0b01111110,
  0b00000000,
  0b00000000,
  0b01111110,
  0b00000000,
  0b00000000,
  0b01111110,  
};

const unsigned char smallNumbers[] PROGMEM = 
{                          // 4 x 6 bitmap.
  //0
  0b11100000,
  0b10100000,
  0b10100000,
  0b10100000,
  0b11100000,
  0b00000000,
  //1
  0b01000000,
  0b11000000,
  0b01000000,
  0b01000000,
  0b11100000,
  0b00000000,
  //2
  0b11100000,
  0b00100000,
  0b11100000,
  0b10000000,
  0b11100000,
  0b00000000,
  //3
  0b11100000,
  0b00100000,
  0b11100000,
  0b00100000,
  0b11100000,
  0b00000000,
  //4
  0b10100000,
  0b10100000,
  0b11100000,
  0b00100000,
  0b00100000,
  0b00000000,
  //5
  0b11100000,
  0b10000000,
  0b11100000,
  0b00100000,
  0b11000000,
  0b00000000,
  //6
  0b11000000,
  0b10000000,
  0b11100000,
  0b10100000,
  0b11100000,
  0b00000000,
  //7
  0b11100000,
  0b00100000,
  0b01000000,
  0b10000000,
  0b10000000,
  0b00000000,
  //8
  0b11100000,
  0b10100000,
  0b11100000,
  0b10100000,
  0b11100000,
  0b00000000,
  //9
  0b11100000,
  0b10100000,
  0b11100000,
  0b00100000,
  0b01100000,
  0b00000000,
};
