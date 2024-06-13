#include "main.h"
#include "config.h"
#define __PROG_TYPES_COMPAT__

#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <avr/delay.h>
#include <EEPROM.h>

#define VERSION 23

#define NUMBER_OF_CURRENT_SENSORS 6

// Hardware definitions//////
#define STANDARD_RC     //
#define MOBIDRONEOSD_V2 //
////////////////////////////

// Default values - if eeprom is empty these data will be loaded
uint8_t minRSSI = 0;
uint16_t maxRSSI = 280;

#define NULL_CORRECTION 125

#define DEFAULT_BAT1_CRITICAL_VOLTAGE 100 // 10.0V
#define DEFAULT_BAT2_CRITICAL_VOLTAGE 65  // 6.5V
#define DEFAULT_RC_SENS 1
#define DEFAULT_AUX_SWITCH 3
#define DEFAULT_BAT1_CORRECTION NULL_CORRECTION // 125 for 0.0V
#define DEFAULT_BAT2_CORRECTION NULL_CORRECTION // 125 for 0.0V
#define DEFAULT_CURRENT_TYPE 1
#define DEFAULT_ROLL_SENSITIVITY 10 // 10 - no correction
#define DEFAULT_PITCH_SENSITIVITY 10

#define clearVideoOut SPDR = 0b00000000;
#define dimOn DDRB |= 0b00000010;
#define dimOff DDRB &= 0b11111101;

///////////////////////////////////////////////////

static uint8_t videoResDetected = 0;
static uint16_t CAMERA_V_RESOLUTION12 = 0;
// static uint16_t CAMERA_V_RESOLUTION12_OLD;
volatile uint16_t FLIGHT_PANEL_END_LINE;
volatile uint8_t MAIN_PANEL_CENTER_LINE;
volatile uint8_t CLMR;
volatile uint8_t CLPR;
volatile uint8_t CLMR2;

// analog read values
#ifdef MOBIDRONEOSD_V2
// static float compCorrection = 92.16;
#endif

// static int16_t bat1_voltage = 0;
// static float sensVcc = 0;
static float sensRSSI = 0;
// static float currentAverage = 0.0;
// static float consumedmAh = 0.0;

// static uint8_t currIndex = 0;

static uint32_t calcTimes = 0;
static uint32_t calcFlyTimes = 0;
static uint32_t mSeconds = 0;
static uint16_t seconds = 0;
static uint8_t MinTime_dump = 0;
static uint8_t SecTime_dump = 0;

volatile uint32_t bat1Timer = 0;
volatile uint32_t bat2Timer = 0;
volatile uint32_t gpsFixTimer = 0;
volatile uint32_t autoModeTimer = 0;
volatile uint32_t a20mSecTimer = 0;

uint16_t viewTime = 0;
/////
volatile uint8_t setDim = 0;
volatile uint8_t curveBuffer[14];

volatile uint16_t temp1 = 0;
volatile uint8_t temp2 = 0;
int8_t pointYpos = 0;
uint8_t pointXpos = 0;
uint8_t angleType_count = 0;
volatile uint8_t math1 = 0;
volatile uint8_t math2 = 0;
uint8_t startPos_temp = 0; // startPos of angle Meter
uint8_t endPos_temp = 12;  // endPos of angle Meter
// angles
int16_t readedRollAngle = 0;
int16_t readedRollAngle_radar;
int16_t readedRollAngle_temp = 0;
int16_t readedPitchAngle = 0;
int16_t readedPitchAngle_radar;
int8_t pitchAngleRelative = 0;
int8_t pitchAngleRelative_temp = MAIN_PANEL_CENTER_LINE;
// compass
int16_t readedHeading = 0;
// gps
int16_t homeDirection = 0;
int16_t relativeDirection = 0;
uint16_t homeDistance = 0;
// speed
volatile uint8_t speed_pointer = 0;
// altitude
volatile uint8_t cmAlt_pointer = 0;
int16_t storedAlt = 0;
uint8_t firstAlt = 0;
// Statistics
int16_t highAltitude = 0;
uint16_t highSpeed = 0;
uint16_t highDistance = 0;
uint16_t highAmperage = 0;

// Motor status - armed, disarmed
uint8_t armed = 0;
uint8_t armed_old = 0;

volatile uint8_t cryticalPart = 0;
volatile uint8_t buffered = 0; // data are writed to video buffer
volatile uint8_t layoutWrited = 0;

////
uint8_t ftoken = 0;
uint8_t stoken = 0;
volatile uint16_t Hsync = 0;
volatile uint8_t lineId = 0;
volatile uint16_t varOSD = 0;
volatile uint8_t letterCounter = 0;
uint8_t letterCounter1 = 0;
uint8_t lineCounter = 0;
volatile uint8_t dataCount = 0;
uint8_t segmentCounter = 0;
uint8_t mainPanelLineGenerated = 0;

volatile uint8_t bat1Blink = 0;
volatile uint8_t bat2Blink = 0;
volatile uint8_t gpsFixBlink = 0;
volatile uint8_t autoModeBlink = 0;

// screen identifier
volatile uint8_t screenNumber = 0;
volatile uint8_t screen = 0;

uint8_t screenSwitchEnable = 1;

#define NUM_OF_AUX_SCREEN 3
uint8_t screenMap[] = {1, 2, 5};
uint8_t screenPointer = 0;

// readed eeprom data
uint8_t eepromBuffer[EEPROM_DATA_LENGTH];

// readed PID data from Multiwii
static uint8_t pidBuffer[PID_DATA_BYTES + RCTUNING_ITEMS_BYTES];
// static uint8_t pidData_format[] = {3, 3, 3, 3, 2,  // PID1 screen
//                                    3, 3, 3, 1, 0,  // PID2 screen
//                                    1, 1, 1, 1, 1}; // PID3 screen

#define BOXITEMS 14

static uint16_t activate[BOXITEMS];

// static uint16_t aux_map[] = {0x0007, 0x0038, 0x01C0, 0x0E00};
// static uint8_t aux_shift[] = {0, 3, 6, 9};

// static uint8_t sensPresent = 0;
// static uint16_t sensActivated = 0;
// static uint16_t i2c_errors = 0;

uint8_t up = 0;
uint8_t down = 0;
uint8_t left = 0;
uint8_t right = 0;
uint8_t yleft = 0;
uint8_t yright = 0;

uint8_t x_pos = 0;
uint8_t y_pos = 1;
uint8_t y_pos_old = 0;
uint8_t configDataReaded = 0;

//////////////////////////// Serial settings
#define CLOCK 16000000UL
#define BAUD 115200
#define BAUD_SETTINGS ((CLOCK / 4 / BAUD - 1) / 2)
////////////////////////////////////////////

#define CONTENT_END 30 * 7 // 30 chars 7 bytes

#define VIDEO_BUFFER_SIZE 1200 // video buffer bytes
#define EXTENDED_BUFFER 56     // for radar screen

volatile uint8_t VideoBuffer[VIDEO_BUFFER_SIZE + EXTENDED_BUFFER];

#define LAYOUT 105 // 140

// layout
#define ONE_LINE_SIZE 30 * 7 // standard 30 chars  8x8 per line

// radar screen

volatile int8_t posArray[7];
volatile uint16_t g_temp, g_temp1;

uint8_t pageNumber = 1; // id of menu page
uint8_t pageNumber_old = 0;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static uint8_t mwcRequestHeader[] = {'$', 'M', '<'};
#define HAF_RESPONSE_SIZE 6 // header + footer response size

static uint8_t requestId = 0;
static uint8_t dataRequest[] = {MSP_ATTITUDE, MSP_RAW_GPS, MSP_ATTITUDE, MSP_RC, MSP_ATTITUDE, MSP_COMP_GPS, MSP_ATTITUDE, MSP_RC, MSP_ATTITUDE,
                                MSP_ALTITUDE, MSP_ATTITUDE, MSP_RC, MSP_ATTITUDE, MSP_BAT, MSP_ATTITUDE, MSP_RC, MSP_ATTITUDE, MSP_STATUS, MSP_ATTITUDE, MSP_RC, // standard data
                                MSP_IDENT, MSP_RC_TUNING, MSP_RC, MSP_STATUS, MSP_PID, MSP_RC, MSP_BOX, MSP_STATUS, MSP_BAT};                                    // config data

#define DATA_REQUEST_SIZE 29

#define STANDARD_DATA_REQUEST_SIZE 20
#define CONFIG_DATA_REQUEST_START 19
#define CONFIG_DATA_MODULO 10

#define RS232_TX_BUFFER_SIZE 38
volatile uint8_t RS232_TX_Buffer[RS232_TX_BUFFER_SIZE];

volatile uint8_t synchronization = 0;
volatile uint8_t readEnable;

// multiwii data variables
// static int16_t rcData[8];
// static int16_t throttle_old;
// static uint8_t gpsFix = 0;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static uint8_t BARSIZE = 0;

//________________________________________________________________________________________________
ISR(INT0_vect) // interrupt routine Hsync
{
  if (screenNumber == 1)
  { // main Screen
    DDRB &= 0xf7;
    switch (lineId)
    {
    case 1: // modes, numsat, rssi
      DDRB |= 0x08;
      // on time
      delay1;
      _delay_loop_1(11);
      // dimOn;
      letterCounter++;
      BARSIZE = 7;
      break;
    case 2: // start & fly time, bat1 & bat2
      DDRB |= 0x08;
      delay1;
      _delay_loop_1(13);

#ifdef STARTTIME
      ////////////////////////////////////////////START TIME///////////////////////////////////////////////////////////
      // start time (minutes) - big numbers
      for (uint16_t osdChar = STARTTIME_BIGNUMBAR_START + letterCounter; osdChar < STARTTIME_BIGNUMBAR_END; osdChar += 16)
      {
        SPDR = VideoBuffer[osdChar];
        dimOn;
        delay3;
      }
      dimOff;
      if (letterCounter < 7)
      {
        delay3;
        // start time (seconds) - small
        for (uint16_t osdChar = TIMEBAR_START + letterCounter; osdChar < TIMEBAR_END; osdChar += 7)
        {
          SPDR = VideoBuffer[osdChar];
          dimOn;
          _delay_loop_1(2);
        }
        dimOff;
      }
      else if (letterCounter > 8)
      {
        // start time title - small
        for (uint16_t osdChar = TIMEBAR_TITLE_START + (letterCounter - 9); osdChar < TIMEBAR_TITLE_END; osdChar += 7)
        {
          SPDR = VideoBuffer[osdChar];
          dimOn;
          _delay_loop_1(2);
        }
        dimOff;
      }
      else
      {
        _delay_loop_1(25);
      }
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#else
      delayMicroseconds(11);
      //_delay_loop_1(1);
#endif

#ifdef FLYTIME
      ////////////////////////////////////////////FLY TIME///////////////////////////////////////////////////////////
      // fly time (minutes) - big numbers
      for (uint16_t osdChar = FLYTIME_BIGNUMBAR_START + letterCounter; osdChar < FLYTIME_BIGNUMBAR_END; osdChar += 16)
      {
        SPDR = VideoBuffer[osdChar];
        dimOn;
        delay3;
      }
      dimOff;
      if (letterCounter < 7)
      {
        delay3;
        // fly time (seconds) - small
        for (uint16_t osdChar = FLYTIMEBAR_START + letterCounter; osdChar < FLYTIMEBAR_END; osdChar += 7)
        {
          SPDR = VideoBuffer[osdChar];
          dimOn;
          _delay_loop_1(2);
        }
        dimOff;
      }
      else if (letterCounter > 8)
      {
        // fly time title - small
        for (uint16_t osdChar = FLYTIMEBAR_TITLE_START + (letterCounter - 9); osdChar < FLYTIMEBAR_TITLE_END; osdChar += 7)
        {
          SPDR = VideoBuffer[osdChar];
          dimOn;
          _delay_loop_1(2);
        }
        dimOff;
      }
      else
      {
        _delay_loop_1(25);
      }
#else
      delayMicroseconds(11);
      //_delay_loop_1(1);
#endif

      delayMicroseconds(11);
      //_delay_loop_1(1);

//       ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      letterCounter++;

      BARSIZE = 16; // 16 lines size
      break;
    /*
    case 3:

    letterCounter ++;
    BARSIZE = 7;
    break;
    */
    case 4: // batery 1 current
      DDRB |= 0x08;

      letterCounter++;
      BARSIZE = 7;
      break;

    case 5: // batery 1 capacity consumed
      DDRB |= 0x08;

      delayMicroseconds(14);
      _delay_loop_1(2);

      letterCounter++;
      BARSIZE = 7;
      break;

    case 6: // print point

      letterCounter++;
      BARSIZE = 3;
      break;

    case 7: // heading

      letterCounter++;
      BARSIZE = 7;
      break;

    case 9: // print horizont
      lineCounter++;
      if (lineCounter > 2)
      { // lines 3,6,9,12 ...
        printFlightPanel();
        if (lineCounter == 4)
        {
          segmentCounter++;
          lineCounter = 0;
          mainPanelLineGenerated = 0;
        }
      }
      else
      { // 1-2 ,4-5 ,7-8 ....
        // 2 lines to compute next line for acc visualisation
        if (cryticalPart == 0)
        { // acc data was computed
          if (mainPanelLineGenerated == 0)
          {
            // clearVideoOut;
            printMainPanelToBuffer(MAIN_PANEL_CENTER_LINE, MAIN_PANEL_CENTER_POINT, readedRollAngle_temp);
            mainPanelLineGenerated = 1;

            if ((segmentCounter + cmAlt_pointer + 1) % 5 == 0)
              math1 = 1;
            else
              math1 = 0;

            if ((segmentCounter + speed_pointer + 1) % 5 == 0)
              math2 = 1;
            else
              math2 = 0;
          }
        }
      }
      if (Hsync == FLIGHT_PANEL_END_LINE)
      {
        letterCounter = 0;
        lineId = 0;
      }
      break;

    case 10: // altitude + speed + arrow + distance
      DDRB |= 0x08;
      _delay_loop_1(53);
    Serial.println(letterCounter);

#ifdef ALTITUDE
      ////////////////////////////////////////////ALTITUDE ///////////////////////////////////////////////////////////
      // integer part - big numbers
      for (uint16_t osdChar = ALTITUDE_BIGNUMBAR_START + letterCounter; osdChar < ALTITUDE_BIGNUMBAR_END; osdChar += 16)
      {
        SPDR = VideoBuffer[osdChar];
        dimOn;
        delay3;
      }
      dimOff;
      if (letterCounter < 7)
      {
        delay3;
        // altitude title
        for (uint16_t osdChar = ALTITUDE_TITLE_START + letterCounter; osdChar < ALTITUDE_TITLE_END; osdChar += 7)
        {
          SPDR = VideoBuffer[osdChar];
          dimOn;
          _delay_loop_1(2);
        }
        dimOff;
      }
      else if (letterCounter > 8)
      {
        // altitude units
        for (uint16_t osdChar = ALTITUDE_UNITS_START + (letterCounter - 9); osdChar < ALTITUDE_UNITS_END; osdChar += 7)
        {
          SPDR = VideoBuffer[osdChar];
          dimOn;
          _delay_loop_1(2);
        }
        dimOff;
      }
      else
      {
        _delay_loop_1(26);
        delay1;
      }
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#endif

      letterCounter++;
      BARSIZE = 16;
      break;
    /*case 11:
    case 12:*/
    case 13: // processing
      fastUpdate();
      slowUpdate();
      lineId = 0;
      if (synchronization == 1)
      {
        synchronization = 2;
      }
      else
      {
        uint8_t rqTmp = dataRequest[requestId % STANDARD_DATA_REQUEST_SIZE];
        dataCount = 0;
        sendDataRequest(rqTmp, 0, 0);
      }

      EIMSK |= (0 << INT0); // disable H-sync interrupt
      break;

    default:
      break;
    }
  }
  else if (screenNumber == 0)
  { // print Header
    DDRB &= 0xf7;
    // header
    if ((Hsync >= TOP_MARGIN) && (Hsync <= TOP_MARGIN + 6) && a20mSecTimer >= 60)
    {
      DDRB |= 0x08;
      delayMicroseconds(13);
      // dimOn;
      for (int osdChar = varOSD; osdChar < (int)HEADER_SIZE; osdChar += 7)
      {
        SPDR = VideoBuffer[osdChar];
        // clearVideoOut;
        _delay_loop_1(2);
      }
      // dimOff;
      varOSD++;
    }

    if (Hsync == TOP_MARGIN + 30)
    {
      viewTime++;
      if (viewTime >= 150) // время показа заставки
      {                    // 10 sec timer
        viewTime = 0;
        screen = 1; // main screen
      }
    }
  }

  Hsync++;

  if (screenNumber != 1)
    return;
  else if (letterCounter >= BARSIZE)
  {
    letterCounter = 0;
    lineId = 0;
  }
  else if (lineId > 0)
    return;

  switch (Hsync)
  {
    // page lines
  case RSSIBAR_Y_POS:
    lineId = 1;
    break;
  case TIMEBATBAR_Y_POS:
    lineId = 2;
    break;
  /*case BATBAR_Y_POS :  //to 6
      lineId = 3; break;*/
  case CURBAR_Y_POS: // to 15
    lineId = 4;
    break;
  case (CURBAR_Y_POS + 9): // to 24
    lineId = 5;
    break;
  case COMPASSBAR_Y_POS: // to 2
    lineId = 6;
    break;
  case (COMPASSBAR_Y_POS + 4): // to 10    //line 87 to 93
    lineId = 7;
    break;
  case FLIGHT_PANEL_START_LINE:
    lineId = 9;
    break;
  default:
    if (Hsync == CAMERA_V_RESOLUTION12 - 51) // 242 to 248 (standard pal)
      lineId = 10;
    else if (Hsync == CAMERA_V_RESOLUTION12 - 44) // 249 to 250
      lineId = 11;
    else if (Hsync == CAMERA_V_RESOLUTION12 - 42) // 251 to 257
      lineId = 12;
    else if (Hsync == CAMERA_V_RESOLUTION12 - 35) // 258 to end of frame
      lineId = 13;
    break;
  }
}

//________________________________________________________________________________________________
ISR(INT1_vect, ISR_BLOCK)
{
  if (videoResDetected == 0)
  {
    // CAMERA_V_RESOLUTION12_OLD = CAMERA_V_RESOLUTION12;
    CAMERA_V_RESOLUTION12 = Hsync;
    // if(CAMERA_V_RESOLUTION12 >= 160 && (CAMERA_V_RESOLUTION12 == CAMERA_V_RESOLUTION12_OLD)){ //to protect line detection before uncomplete or incorrect video frame
    if (screen != 0)
    {
      videoResDetected = 1;

      FLIGHT_PANEL_END_LINE = CAMERA_V_RESOLUTION12 - 59;
      uint8_t FLIGHT_PANEL_NUMBER_OF_LINES = FLIGHT_PANEL_END_LINE - FLIGHT_PANEL_START_LINE;
      uint8_t NUMBER_OF_VERTICAL_LINES = FLIGHT_PANEL_NUMBER_OF_LINES - 3;
      NUMBER_OF_VERTICAL_LINES /= 4;
      MAIN_PANEL_CENTER_LINE = NUMBER_OF_VERTICAL_LINES / 2;
      CLMR = MAIN_PANEL_CENTER_LINE - MAIN_PANEL_RADIUS;
      CLPR = MAIN_PANEL_CENTER_LINE + MAIN_PANEL_RADIUS;
      CLMR2 = MAIN_PANEL_CENTER_LINE - 3;
    }
  }

  Hsync = 0;
  varOSD = 0;
  lineId = 0;
  letterCounter = 0;
  letterCounter1 = 0;
  lineCounter = 0;
  mainPanelLineGenerated = 0;
  segmentCounter = 0;
  pointYpos = 0;
  angleType_count = 0;
  pointXpos = startPos_temp;

  switch (screen) // switch screen
  {
  case 0: // intro title
    screenNumber = 0;
    PrintHeaderToBuffer();
    layoutWrited = 0;
    readEnable = 0;
    break;
  case 1: // main screen
    screenNumber = 1;
    if (layoutWrited == 0)
    {
      PrintLayoutToBuffer();
      pageNumber_old = 0;
      layoutWrited = 1;
    }
    readEnable = 1;
    break;
  case 2: // radar screen
    screenNumber = 2;
    layoutWrited = 0;
    readEnable = 1;
    break;
  case 3: // saved title
    screenNumber = 3;
    layoutWrited = 0;
    readEnable = 0;
    break;
  case 4: // canceled title
    screenNumber = 4;
    layoutWrited = 0;
    readEnable = 0;
    break;
  case 5: // blank screen
    screenNumber = 5;
    readEnable = 1;
    layoutWrited = 0;
    break;
  case 6: // internal settings
    screenNumber = 6;
    if (pageNumber != pageNumber_old)
    {
      PrintMenuPageToBuffer(pageNumber);
      pageNumber_old = pageNumber;
      if (pageNumber == MAX_MENU_PAGES + 1)
      {
        y_pos = y_pos_old;
      }
      else
      {
        y_pos = 1;
        y_pos_old = 0;
      }
    }
    readEnable = 1;
    break;
  }

  a20mSecTimer++;
  EIMSK |= (1 << INT0);
}

//________________________________________________________________________________________________
void setup()
{
  // Init Serial communication.
  // Serial.begin(BAUD);
  // set baudrate

  // UBRR0H = (unsigned char)(BAUD_SETTINGS >> 8);
  // UBRR0L = (unsigned char)(BAUD_SETTINGS);

  // /* enable 2x speed mode U2X0 = 1*/
  // UCSR0A = 0b0000010;
  // /* interrupts disabled, rx and tx enabled*/
  // UCSR0B = (1 << RXEN0) | (1 << TXEN0);
  // /* Asynchronous mode, no parity, 1-stop bit, 8-bit data (8N1) */
  // UCSR0C = (3 << UCSZ00);

  setupPinMode();
  readEEPROM(EEPROM_DATA_LENGTH);

  for (uint16_t x = VIDEO_BUFFER_SIZE; x < VIDEO_BUFFER_SIZE + EXTENDED_BUFFER; x++)
    VideoBuffer[x] = 0x00;

  PORTB = 0x0c; // SS+MOSI, SS(slave select)
  DDRB |= 0x04; // SS is active low, but with 1

  // faster analog read - set prescale to 16
  ADCSRA |= _BV(ADPS2);
  ADCSRA &= ~_BV(ADPS1);
  ADCSRA &= ~_BV(ADPS0);

  // Setup SPI: Serial Peripheral Interface
  // SPCR: SPiControlRegister
  // | 7    | 6    | 5    | 4    | 3    | 2    | 1    | 0    |
  // | SPIE | SPE  | DORD | MSTR | CPOL | CPHA | SPR1 | SPR0 |
  // Default value = 0;
  SPCR = (1 << SPE) |  // SPE  - Enables the SPI when 1
         (0 << DORD) | // DORD - When zero, the MSB of the data word is transmitted first.
         (1 << MSTR) | // MSTR - Sets the Arduino in master mode when 1
         (1 << CPHA) | // CPHA - Samples data on the falling edge of the data clock when 1
         (1 << CPOL) |
         (0 << SPR1) |   // SPR1 SPR0 - Clock Rate Select
         (0 << SPR0);    // SPR1 and SPR0 - Sets the SPI speed, 00 is fastest (4MHz).
  SPSR = (1 << SPI2X);   // Double Speed (CK/2) Master SPI Mode (8MHz)
  EIMSK |= (1 << INT0);  // Enable INT0 interrupt (EIMSK External Interrupt Mask Register) Hsync
  EIMSK |= (1 << INT1);  // Enable INT1 interrupt (EIMSK External Interrupt Mask Register) Vsync
  EICRA |= (1 << ISC01); // EICRA External Interrupt Control Register A
  EICRA |= (1 << ISC00); // INT0 (Hsync); INT1 (Vsync)
  EICRA |= (1 << ISC11);
  EICRA |= (0 << ISC10); // 00: LOW | 11: RISING | 01: ANY CHANGE | 10: FALLING
  clearVideoOut;         // disable all timers irq
  TIMSK0 = 0;
  TIMSK1 = 0;
  TIMSK2 = 0;
  set_sleep_mode(SLEEP_MODE_IDLE); // energy saving

  Serial.begin(115200);
}

uint32_t currTime = 0;

void loop()
{
  while (1)
  { // read serial data from UART DATA register no interrupt
    clearVideoOut;
    if (a20mSecTimer - currTime > 50)
    {
      currTime = a20mSecTimer;
      // Serial.print("camera resolution: ");
      // Serial.println(CAMERA_V_RESOLUTION12);
    }
    if (readEnable == 1)
    {
      if (synchronization == 2)
      {
        // processSerialData();
        requestId++;
        synchronization = 0;
      }
      else if (UCSR0A & (1 << RXC0))
      {
        RS232_TX_Buffer[dataCount] = UDR0;
        dataCount++;
        if (RS232_TX_Buffer[3] + HAF_RESPONSE_SIZE == dataCount)
        { // end of frame
          synchronization = 1;
        }
      }
    }
    sleep_enable();
    sleep_mode();
  }
}

///////////////////////////////////
// R o u t i n e s
///////////////////////////////////

void setupPinMode()
{
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(14, INPUT);
  pinMode(15, INPUT);
  pinMode(16, INPUT);
  pinMode(17, INPUT);
}

// fill spaces to videoBuffer
static void clearVideoBuffer(uint16_t endByte)
{
  for (uint16_t x = 0; x < endByte; x++)
    VideoBuffer[x] = 0x00;
}

static void PrintLayoutToBuffer()
{
  uint16_t poc = 0;

  for (uint16_t pos = 0; pos < LAYOUT; pos++)
  {
    uint16_t charId = pgm_read_byte(&layout[pos]);
    charId = charId * 7;

    for (uint16_t x = charId; x < charId + 7; x++)
    {
      VideoBuffer[poc] = pgm_read_byte(&letters[x]);
      poc++;
    }
  }

  // reset bat1 voltage
  buffering2BigNumbers(BAT1_BIGNUMBAR_START, 0, 0);
  buffering2BigNumbers(SPEED_BIGNUMBAR_START, 0, 0);
  buffering3BigNumbers(ALTITUDE_BIGNUMBAR_START, 0, 0, 0);
}

static void PrintHeaderToBuffer()
{
  uint16_t poc = 0;

  for (uint16_t pos = 0; pos < HEADER; pos++)
  {
    uint16_t charId = pgm_read_byte(&header[pos]);
    charId = charId * 7;
    for (uint16_t x = charId; x < charId + 7; x++)
    {
      VideoBuffer[poc] = pgm_read_byte(&letters[x]);
      poc++;
    }
  }
}

static void PrintMenuPageToBuffer(uint8_t pageNumber)
{
  uint16_t poc = 0;

  if (pageNumber < 1)
    pageNumber = 1;

  uint16_t temp = ONE_LINE_IN_MENU * MAX_MENU_ITEMS;

  for (uint16_t pos = temp * (pageNumber - 1); pos < temp * pageNumber; pos++)
  {
    uint16_t charId = pgm_read_byte(&menuPages[pos]);
    charId = charId * 7;
    for (uint16_t x = charId; x < charId + 7; x++)
    {
      VideoBuffer[poc] = pgm_read_byte(&letters[x]);
      poc++;
    }
  }
}

// lower memory consumation
static void bufferingArrow(uint16_t arrowPointer, uint8_t arrowType)
{
  for (uint8_t j = 0, count = 0; count < 32; count++, j += 2, j %= 31)
  { // refactoring (first column then second column) and write to video buffer
    VideoBuffer[arrowPointer + count] = pgm_read_byte(&homeArrows[arrowType + j]);
  }
}

// lower memory consumation
static void buffering2BigNumbers(uint16_t barPointer, uint8_t id1, uint8_t id2)
{
  uint16_t temp = id1 * 32;
  uint16_t temp1;

  for (uint8_t j = 0, count = 0; count < 32; count++, j += 2, j %= 31)
  { // refactor number (first column then second column) and write to video buffer
    VideoBuffer[barPointer + count] = pgm_read_byte(&bigNumbers[temp + j]);
  }

  temp = id2 * 32;
  temp1 = barPointer + 32;

  for (uint8_t j = 0, count = 0; count < 32; count++, j += 2, j %= 31)
  { // refactor number (first column then second column) and write to video buffer
    VideoBuffer[temp1 + count] = pgm_read_byte(&bigNumbers[temp + j]);
  }
}

// lower memory consumation
static void buffering3BigNumbers(uint16_t barPointer, uint8_t id1, uint8_t id2, uint8_t id3)
{
  uint16_t temp = id1 * 32;
  uint16_t temp1;

  for (uint8_t j = 0, count = 0; count < 32; count++, j += 2, j %= 31)
  { // refactor number (first column then second column) and write to video buffer
    VideoBuffer[barPointer + count] = pgm_read_byte(&bigNumbers[temp + j]);
  }

  temp = id2 * 32;
  temp1 = barPointer + 32;

  for (uint8_t j = 0, count = 0; count < 32; count++, j += 2, j %= 31)
  { // refactor number (first column then second column) and write to video buffer
    VideoBuffer[temp1 + count] = pgm_read_byte(&bigNumbers[temp + j]);
  }

  temp = id3 * 32;
  temp1 = barPointer + 64;

  for (uint8_t j = 0, count = 0; count < 32; count++, j += 2, j %= 31)
  { // refactor number (first column then second column) and write to video buffer
    VideoBuffer[temp1 + count] = pgm_read_byte(&bigNumbers[temp + j]);
  }
}

static void buffering(uint16_t barPointer, uint8_t pos, uint8_t id)
{
  // start in 0 line
  uint16_t posTemp = barPointer + (pos * 7); // positioning in video buffer
  uint16_t idTemp = id * 7;

  if (id <= 32)
  {
    for (uint16_t x = posTemp; x < posTemp + 7; x++)
    {
      VideoBuffer[x] = lettersInRam[idTemp];
      idTemp++;
    }
  }
  else
  {
    for (uint16_t x = posTemp; x < posTemp + 7; x++)
    {
      VideoBuffer[x] = pgm_read_byte(&letters[idTemp]);
      idTemp++;
    }
  }
}

static void bufferingInMenu(uint8_t whoLine, uint8_t pos, uint8_t id)
{
  // start in 0 line
  uint16_t posTemp = (pos * 7) + (whoLine * ONE_LINE_IN_MENU_SIZE); // positioning in video buffer
  uint16_t idTemp = id * 7;

  if (id <= 32)
  {
    for (uint16_t x = posTemp; x < posTemp + 7; x++)
    {
      VideoBuffer[x] = lettersInRam[idTemp];
      idTemp++;
    }
  }
  else
  {
    for (uint16_t x = posTemp; x < posTemp + 7; x++)
    {
      VideoBuffer[x] = pgm_read_byte(&letters[idTemp]);
      idTemp++;
    }
  }
}

static void printOneLineInMenu(uint8_t delayMicros)
{
  DDRB |= 0x08;
  delayMicroseconds(LEFT_MARGIN_IN_MENU - delayMicros);
  dimOn;
  for (uint16_t osdChar = varOSD; osdChar < temp1; osdChar += 7)
  {
    SPDR = VideoBuffer[osdChar];
    clearVideoOut;
    _delay_loop_1(2);
  }
  dimOff;
  varOSD++;
}

void writeEEPROM(uint8_t length)
{
  for (uint8_t i = 1; i < length; i++)
  {
    EEPROM.write(i, eepromBuffer[i]);
  }

  EEPROM.write(0, VERSION);
}

void readEEPROM(uint8_t length)
{
  if (EEPROM.read(0) == VERSION)
  { // correct eeprom data
    for (uint8_t i = 1; i < length; i++)
    {
      eepromBuffer[i] = EEPROM.read(i);
    }
    maxRSSI = (eepromBuffer[RSSI_CALIB + 1] << 8) | eepromBuffer[RSSI_CALIB];
  }
  else
  {

    // default values
    eepromBuffer[BAT1_LEVEL] = DEFAULT_BAT1_CRITICAL_VOLTAGE;
    eepromBuffer[BAT2_LEVEL] = DEFAULT_BAT2_CRITICAL_VOLTAGE;
    eepromBuffer[RSSI_CALIB] = maxRSSI & 0x00FF;
    eepromBuffer[RSSI_CALIB + 1] = (maxRSSI & 0xFF00) >> 8;
    eepromBuffer[AUX_SW] = DEFAULT_AUX_SWITCH;
    eepromBuffer[RC_SENS] = DEFAULT_RC_SENS;
    eepromBuffer[BAT1_CORRECTION] = DEFAULT_BAT1_CORRECTION;
    eepromBuffer[BAT2_CORRECTION] = DEFAULT_BAT2_CORRECTION;
    eepromBuffer[CURR_SENS_TYPE] = DEFAULT_CURRENT_TYPE;
    eepromBuffer[ROLL_SENSITIVITY] = DEFAULT_ROLL_SENSITIVITY;
    eepromBuffer[PITCH_SENSITIVITY] = DEFAULT_PITCH_SENSITIVITY;
  }
}

//________________________________________________________________________________________________

uint32_t read32(uint8_t id)
{
  uint32_t t = read16(id);
  t += (uint32_t)read16(id + 2) << 16;
  return t;
}

uint16_t read16(uint8_t id)
{
  uint16_t t = read8(id);
  t += (uint16_t)read8(id + 1) << 8;
  return t;
}

uint8_t read8(uint8_t id)
{
  return RS232_TX_Buffer[id] & 0xFF;
}

// send request to MWC
static void sendDataRequest(uint8_t request, uint8_t dataId, uint8_t fSize)
{
  uint8_t checksum = 0;;

  // header
  for (uint8_t i = 0; i < 3; i++)
  {
    while (!(UCSR0A & (1 << UDRE0)))
      ; // Wait for empty transmit buffer
    UDR0 = mwcRequestHeader[i];
  }

  // size
  while (!(UCSR0A & (1 << UDRE0)))
    ;
  UDR0 = fSize;
  checksum ^= fSize;

  // request id
  while (!(UCSR0A & (1 << UDRE0)))
    ; // Wait for empty transmit buffer
  UDR0 = request;
  checksum ^= request;

  // data
  if (dataId != 0)
  {

    if (dataId == 3)
      fSize /= 2;

    for (uint8_t j = 0; j < fSize; j++)
    {
      while (!(UCSR0A & (1 << UDRE0)))
        ; // Wait for empty transmit buffer
      switch (dataId)
      {
      case 1: // PID
        UDR0 = pidBuffer[j + 1];
        checksum ^= pidBuffer[j + 1];
        break;
      case 2: // RC TUNNING
        UDR0 = pidBuffer[((j * 3) + 1) + PID_DATA_BYTES];
        checksum ^= pidBuffer[((j * 3) + 1) + PID_DATA_BYTES];
        break;
      case 3: // BOX
        UDR0 = (uint8_t)(activate[j] & 0xFF);
        checksum ^= (uint8_t)(activate[j] & 0xFF);
        while (!(UCSR0A & (1 << UDRE0)))
          ; // Wait for empty transmit buffer
        UDR0 = (uint8_t)((activate[j] >> 8) & 0xFF);
        checksum ^= (uint8_t)((activate[j] >> 8) & 0xFF);
        break;
      }
    }
  }
  // checksum
  while (!(UCSR0A & (1 << UDRE0)))
    ; // Wait for empty transmit buffer
  UDR0 = checksum;
}

// for accData & altitude
static void printMainPanelToBuffer(uint8_t centerLine, uint8_t centerPoint, uint8_t rollAngle)
{

  int8_t temp3 = pitchAngleRelative_temp - rollAngle;
  // uint8_t temp4 = pitchAngleRelative_temp + rollAngle;
  int8_t temp5;
  uint8_t temp6 = rollAngle / 3;
  uint8_t angleType = 0; // 0=> angle%3 == 0; 1=> angle+1%3 == 0 ; 2=> angle-1%3 == 0

  if (rollAngle % 3 == 1)
  { // angle 4 ,7 ,10, 13 ...
    angleType = 2;
  }
  else if (rollAngle % 3 == 0)
  { // 3, 6, 9, 12 ...
    angleType = 0;
  }
  else
  { // 2, 5, 8, 11 ...
    temp6 += 1;
    angleType = 1;
  }

  if (temp3 < 0)
  {
    // if line go out from window
    if (segmentCounter == 0)
    {
      pointYpos = temp3;
    }
    temp5 = segmentCounter;
  }
  else
    temp5 = segmentCounter - temp3;

  for (uint8_t i = 0; i < 13; i++)
  { // null curve buffer
    curveBuffer[i] = 0;
  }

  if (rollAngle > 2)
  { // 3 and more deg
    for (uint8_t i = 0; i < 13; i++)
    {
      if (temp5 == pointYpos || pointYpos < 0)
      {
        if (i == pointXpos || pointYpos < 0)
        {
          if (pointYpos >= 0)
          {
            curveBuffer[i] = 0b00011110;
            if (pointXpos != centerPoint)
            {
              if (startPos_temp == 12)
                curveBuffer[i - 1] = 0b00011110;
              else
                curveBuffer[i + 1] = 0b00011110;
            }
          }
          angleType_count++;
          if (angleType_count == 3 || angleType_count == 4)
          {
            if (angleType == 1)
              pointYpos += (temp6 - 1);
            else if (angleType == 2)
              pointYpos += (temp6 + 1);
            else
              pointYpos += temp6;
          }
          else
            pointYpos += temp6;

          if (startPos_temp == 12)
          {
            if (pointXpos == centerPoint)
              pointXpos--;
            else
              pointXpos -= 2;
          }
          else
          {
            if (pointXpos == centerPoint)
              pointXpos++;
            else
              pointXpos += 2;
          }
        }
      }
    }
  }
  else if (rollAngle > 0)
  { // 0-2 deg
    uint8_t p = startPos_temp;
    while (p >= 0 && p <= 12)
    {
      if (temp5 == pointYpos || pointYpos < 0)
      {
        if (p == pointXpos || pointYpos < 0)
        {
          if (pointYpos >= 0)
          {
            curveBuffer[p] = 0b00011110;
            if (pointXpos != centerPoint)
            {
              if (startPos_temp == 12)
                curveBuffer[p - 1] = 0b00011110;
              else
                curveBuffer[p + 1] = 0b00011110;
            }
          }

          // x axis moving
          if (startPos_temp == 12)
          {
            if (pointXpos == centerPoint)
              pointXpos--;
            else
              pointXpos -= 2;
          }
          else
          {
            if (pointXpos == centerPoint)
              pointXpos++;
            else
              pointXpos += 2;
          }

          // y axis moving  0 > { }  1 > {6,7} 2 > {4,6,7,9}
          if (pointXpos == 5 || pointXpos == 6 || pointXpos == 7)
          {
            pointYpos++;
          }
          else if ((pointXpos == 3 || pointXpos == 4) || (pointXpos == 8 || pointXpos == 9))
          {
            if (rollAngle == 2)
              pointYpos++;
          }
        }
      }

      if (startPos_temp == 12)
        p--;
      else
        p++;
    }
  }
  else if (segmentCounter == pitchAngleRelative_temp)
  { // angle 0
    for (uint8_t p = 0; p < 13; p++)
    {
      curveBuffer[p] = 0b00011110;
    }
  }
  // center +
  if (segmentCounter == centerLine)
  { // Hline
    curveBuffer[4] = 0b00001111;
    curveBuffer[5] = 0b11111111;
    curveBuffer[7] = 0b11111111;
    curveBuffer[8] = 0b11111110;
  }
  else if (segmentCounter >= CLMR && segmentCounter <= CLPR)
    curveBuffer[6] = 0b00001110; // Vline
}

static void printFlightPanel()
{

  DDRB |= 0x08;
  _delay_loop_1(27);

// center column
#ifdef VERTICAL_LINES
  if (math1)
  { // long line
    dimOn;
    SPDR = 0b11111110;
  }
  else
  { // short line
    dimOn;
    SPDR = 0b00011110;
  }
  clearVideoOut;
  dimOff;
#else
  delayMicroseconds(1);
#endif

  delayMicroseconds(10);
  _delay_loop_1(3);

  // curve
  for (uint8_t i = 0; i < 13; i++)
  {
    SPDR = curveBuffer[i];
    clearVideoOut;
    _delay_loop_1(3);
    delay1;
  }

  delayMicroseconds(10);
  _delay_loop_1(3);

#ifdef VERTICAL_LINES
  if (math2)
  { // long line
    dimOn;
    SPDR = 0b11111110;
  }
  else
  { // short line
    dimOn;
    SPDR = 0b11110000;
    delay2;
  }
  // clearVideoOut;
  dimOff;
#else
  delayMicroseconds(1);
#endif
}

static void swap(int16_t *a, int16_t *b)
{
  int16_t temp = *a;
  *a = *b;
  *b = temp;
}

static void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1)
{
  volatile uint8_t steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep)
  {
    swap(&x0, &y0);
    swap(&x1, &y1);
  }

  if (x0 > x1)
  {
    swap(&x0, &x1);
    swap(&y0, &y1);
  }
  int8_t deltax = x1 - x0;
  int8_t deltay = abs(y1 - y0);
  int8_t error = deltax / 2;
  int8_t ystep;
  int8_t y = y0;

  if (y0 < y1)
  {
    ystep = 1;
  }
  else
  {
    ystep = -1;
  }

  for (int16_t x = x0; x <= x1; ++x)
  {
    if (y >= 0 && y < GRAPHIC_SIZE)
    {
      if (steep)
        writePixel(y, x, 1);
      else
        writePixel(x, y, 1);
    }
    error = error - deltay;
    if (error < 0)
    {
      y = y + ystep;
      error = error + deltax;
    }
  }
}

static void drawCircle(uint8_t center_x, uint8_t center_y, uint8_t radius)
{
  int f = 1 - radius;
  int ddF_x = 1;
  int ddF_y = -2 * radius;
  int x = 0;
  int y = radius;

  writePixel(center_x, center_y + radius, 1);
  writePixel(center_x, center_x - radius, 1);
  writePixel(center_x + radius, center_y, 1);
  writePixel(center_x - radius, center_y, 1);

  while (x < y)
  {
    if (f >= 0)
    {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;

    uint8_t cxpx = center_x + x;
    uint8_t cxmx = center_x - x;
    uint8_t cypy = center_y + y;
    uint8_t cymy = center_y - y;
    uint8_t cxpy = center_x + y;
    uint8_t cypx = center_y + x;
    uint8_t cxmy = center_x - y;
    uint8_t cymx = center_y - x;

    writePixel(cxpx, cypy, 1);
    writePixel(cxmx, cypy, 1);
    writePixel(cxpx, cymy, 1);
    writePixel(cxmx, cymy, 1);
    writePixel(cxpy, cypx, 1);
    writePixel(cxmy, cypx, 1);
    writePixel(cxpy, cymx, 1);
    writePixel(cxmy, cymx, 1);
  }
}

static void drawCharacter(uint8_t x, uint8_t y, const unsigned char *bmp, uint16_t charIndex, uint8_t lines)
{
  charIndex = charIndex * lines;
  uint8_t charByte;
  for (uint8_t i = 0; i < lines; i++)
  {
    charByte = pgm_read_byte(bmp + charIndex++);
    writePixel(x - 1, y + i, 0); // left right character border
    for (uint8_t j = 0; j < 8; j++)
    {
      if ((charByte & (0x80 >> j)) > 0)
      {
        writePixel(x + j, y + i, 1);
      }
      else
      {
        writePixel(x + j, y + i, 0);
      }
    }
    writePixel(x + 8, y + i, 0); // left right character border
  }
  for (uint8_t j = 0; j < 8; j++)
  { // top bottom character border
    writePixel(x + j, y - 1, 0);
    writePixel(x + j, y + 7, 0);
  }
}

static void writePixel(uint8_t x, uint8_t y, uint8_t color)
{
  uint8_t tmp = x / 8;
  uint16_t tmp1 = (y * GRAPHIC_SIZE_18) + tmp;

  if (color == 0)
    VideoBuffer[tmp1] &= ~0x80 >> (x & 7);
  else
    VideoBuffer[tmp1] |= 0x80 >> (x & 7); // x: x pos of point, y: line of point
}

static void readVcc()
{
#ifdef BAT2
#ifdef MOBIDRONEOSD_V2
  sensVcc = analogRead(1) - compCorrection;
#else
  sensVcc = analogRead(1);
#endif
  sensVcc *= 0.0147; // 0.0049 * 3 (1:3) measure voltage on 2s - 3s lipo + ---10KOhm---10KOhm---APIN----10KOhm---GND
  sensVcc += (float)((int8_t)(eepromBuffer[BAT2_CORRECTION] - NULL_CORRECTION)) / 10.0;
  if (sensVcc < 0)
    sensVcc = 0.0;
#endif
}

#ifdef CURRENT_SENSOR
static void readCurrent()
{
#ifdef MOBIDRONEOSD_V2
  float currentValue = analogRead(3) - compCorrection; // raw data reading
#else
  float currentValue = analogRead(3);
#endif

  switch (eepromBuffer[CURR_SENS_TYPE])
  {
  case 1:                                        // ACS758 100A bidirectional
    currentValue = (currentValue - 512) * 0.244; // offset 512 - default value 2,5V for bidirectional sensor ACS758
    break;                                       // 1024 / 5V  = 205 resolution per Volt ; 100A sensor / 2V = 50A per Volt; 50/205 = 0.244 A per sample  => 244mA
  case 2:                                        // ACS758 50A unidirectional
    currentValue = (currentValue - 123) * 0.061; // offset 123 - default value 0.6V for single sensor ACS758
    break;
  case 3: // Flytron Ultralight 50A Current Sensor
#ifdef MOBIDRONEOSD_V2
    currentValue += compCorrection;
#endif
    currentValue *= 0.049;
    break;
  case 4: // Flytron Ultralight 25A Current Sensor
#ifdef MOBIDRONEOSD_V2
    currentValue += compCorrection;
#endif
    currentValue *= 0.0245;
    break;
  case 5:                                        // ACS758 200A bidirectional
    currentValue = (currentValue - 512) * 0.488; // offset 512 - default value 2,5V for bidirectional sensor ACS758
    break;
  case 6:                                        // ACS756 50A BiDirectional
    currentValue = (currentValue - 418) * 0.121; //  sensor ACS756
    break;
  }

  if (currentValue < 0)
    currentValue = 0;

  currentAverage += currentValue;
  currIndex++;

#ifdef PAL
  consumedmAh += ((currentAverage / currIndex) / 25.714); // PAL read rate 7*20 = 140 mSec ; 1000mSec/140 = 7.142 * 3600 = 25714.3 samples in 1 Hour (example 244mA/25714.3samples in Hour = 0.244/25.7143 mA/H )
#else                                                     // NTSC
  consumedmAh += ((currentAverage / currIndex) / 30.856); // NTSC read time 7*16.67 = 116.67 mSec ; 30856 samples in 1 Hour  / 1000 = 30.856
#endif

  uint16_t curTemp = (currentAverage / currIndex) * 100;
  uint16_t mAhTemp = consumedmAh;

  if (currIndex >= 10)
  {
    currIndex = 0;
    currentAverage = 0.0;
  }

  // Logger
  if (highAmperage < curTemp)
    highAmperage = curTemp;
  ///

  if (screenNumber != 1)
    return;

  buffering(CURRENTVALUEBAR_START, 0, curTemp / 1000);
  buffering(CURRENTVALUEBAR_START, 1, curTemp / 100 - (curTemp / 1000) * 10);
  buffering(CURRENTVALUEBAR_START, 3, curTemp / 10 - (curTemp / 100) * 10);

  if (curTemp / 1000 == 0)
    buffering(CURRENTVALUEBAR_START, 0, 10);

  uint8_t capacityBar[5];

  capacityBar[0] = mAhTemp / 10000;
  capacityBar[1] = mAhTemp / 1000 - (mAhTemp / 10000) * 10;
  capacityBar[2] = mAhTemp / 100 - (mAhTemp / 1000) * 10;
  capacityBar[3] = mAhTemp / 10 - (mAhTemp / 100) * 10;
  capacityBar[4] = mAhTemp - (mAhTemp / 10) * 10;

  uint8_t startTemp = 0;
  for (uint8_t i = 0; i < 4; i++)
  {
    if (capacityBar[i] == 0 && startTemp == 0)
      buffering(CAPACITYVALUEBAR_START, i, 10);
    else
    {
      buffering(CAPACITYVALUEBAR_START, i, capacityBar[i]);
      startTemp = 1;
    }
  }

  buffering(CAPACITYVALUEBAR_START, 4, capacityBar[4]);
}
#endif

static void readRSSI()
{
  sensRSSI = analogRead(0);

  if (screenNumber == 6)
  {
    if (sensRSSI < 10 && pageNumber == INTERNAL_SETTINGS1)
    {

      bufferingInMenu(3, 15, 'E' - 54);
      bufferingInMenu(3, 16, 'R' - 54);
      bufferingInMenu(3, 17, 'R' - 54);
    }
    return;
  }

#ifdef RSSISIGNAL
  int rangeRSSI = constrain(sensRSSI, minRSSI, maxRSSI);
  uint8_t RSSI = map(rangeRSSI, minRSSI, maxRSSI, 0, 99);
  uint8_t rssiLevel;

  if (RSSI >= RSSI_5)
  {
    rssiLevel = 5;
  }
  else if (RSSI >= RSSI_4)
  {
    rssiLevel = 4;
  }
  else if (RSSI >= RSSI_3)
  {
    rssiLevel = 3;
  }
  else if (RSSI >= RSSI_2)
  {
    rssiLevel = 2;
  }
  else
    rssiLevel = 1;

  for (uint8_t i = 1; i <= 5; i++)
  {
    if (i <= rssiLevel)
      buffering(RSSIBAR_START, i, i + 51);
    else
      buffering(RSSIBAR_START, i, 10);
  }

  buffering(RSSIBAR_START, 6, RSSI / 10);
  buffering(RSSIBAR_START, 7, RSSI % 10);
#endif
}

//________________________________________________________________________________________________
static void calcTime()
{
  calcTimes++;
  mSeconds = (calcTimes * 160);

  if (armed)
  {
    calcFlyTimes++;
    seconds = ((calcFlyTimes * 160) / 1000);
    MinTime_dump = (seconds / 60);
    SecTime_dump = (seconds % 60);
  }

  if (screenNumber == 1)
  {

    seconds = (mSeconds / 1000);
    uint8_t MinTime = (seconds / 60);
    uint8_t SecTime = (seconds % 60);

    // start time
    // buffering2BigNumbers(STARTTIME_BIGNUMBAR_START, MinTime / 10, MinTime % 10);
    buffering3BigNumbers(STARTTIME_BIGNUMBAR_START, 6, 6, 6);

    buffering(TIMEBAR_START, 1, SecTime / 10);
    buffering(TIMEBAR_START, 2, SecTime % 10);

    // if(MinTime/10 == 0)
    //    buffering(TIMEBAR_START,4,10);

    // fly time
    buffering2BigNumbers(FLYTIME_BIGNUMBAR_START, MinTime_dump / 10, MinTime_dump % 10);

    buffering(FLYTIMEBAR_START, 1, SecTime_dump / 10);
    buffering(FLYTIMEBAR_START, 2, SecTime_dump % 10);

    // if(MinTime_dump/10 == 0)
    //    buffering(FLYTIMEBAR_START,4,10);

#ifdef BAT2
    uint8_t tempV = sensVcc; // remove decimal part and convert to int
    float bat2_critical_level = eepromBuffer[BAT2_LEVEL] / 10.0;

    // if(tempV/10 == 0)
    //   buffering(BAT2VOLTAGEBAR_START,0,10);
    // else
    //   buffering(BAT2VOLTAGEBAR_START,0,tempV/10);

    buffering2BigNumbers(BAT2_BIGNUMBAR_START, tempV / 10, tempV % 10);
    buffering(BAT2VOLTAGEBAR_START, 1, ((sensVcc * 10) - (tempV * 10)));

    if (sensVcc < bat2_critical_level)
    {
      if (mSeconds - bat2Timer >= 200)
      { // ~200 mSec timer
        if (bat2Blink == 1)
        {
          buffering(BAT2BAR_START, 0, 61);
          buffering(BAT2BAR_START, 1, 58);
          buffering(BAT2BAR_START, 2, 60);
          bat2Blink = 0;
        }
        else
        {
          buffering(BAT2BAR_START, 0, 10);
          buffering(BAT2BAR_START, 1, 10);
          buffering(BAT2BAR_START, 2, 10);
          bat2Blink = 1;
        }
        bat2Timer = mSeconds;
      }
    }
    else
    {
      if (sensVcc >= BAT2_LEVEL_2)
      {
        buffering(BAT2BAR_START, 0, 57);
        buffering(BAT2BAR_START, 1, 57);
        buffering(BAT2BAR_START, 2, 59);
      }
      else if (sensVcc >= BAT2_LEVEL_1)
      {
        buffering(BAT2BAR_START, 0, 57);
        buffering(BAT2BAR_START, 1, 57);
        buffering(BAT2BAR_START, 2, 60);
      }
      else if (sensVcc >= bat2_critical_level)
      {
        buffering(BAT2BAR_START, 0, 57);
        buffering(BAT2BAR_START, 1, 58);
        buffering(BAT2BAR_START, 2, 60);
      }
    }
#endif
  }
}

//________________________________________________________________________________________________
static void fastUpdate() // duration 20mSec * 7
{
  ftoken = a20mSecTimer & 7;
  switch (ftoken)
  {
  case 0:
    calcTime();
    break;
  case 1:
    analogRead(3);
    break;
  case 2:
#ifdef CURRENT_SENSOR
    readCurrent();
#endif
    break;
  case 3:
    break;
  case 4:
    break;
  case 5:
    break;
  case 6:
    break;
  case 7:
    break;
  default: // default exit;
    break;
  }
}

static void slowUpdate() // duration 20ms * 127
{
  stoken = a20mSecTimer & 127;
  switch (stoken)
  {
  case 0:
    analogRead(1);
    break;
  case 24:
    readVcc();
    break;
  case 26:
    analogRead(0);
    break;
  case 50:
    readRSSI();
    break;
  default:
    break;
  }
}
