
// Rev 2.0 has the MCU mounted to the circuit board.
// DIODE DIRECTION: Anode on ROW, Cathode on COLUMN

/**************************************************************
 * 1. User must download all libraries listed at top of code  *
 * 2. Diode direction is set up in keypad library. See below  *
 *    for default direction. You will have to change the code *
 *    in the keypad library to change direction via software  *
 * 3. MAKE SURE TO ADJUST ALL PINS USED BEFORE FLASHING       *
 * 4. To add gifs, see gif code below: ADD GIF                *
 * 5. If something doesn't work, there might be some errors   *
 *    in code. I havn't checked everything yet.               *
 * 6. Use profileSave button to set default tft screen mode.  *
 * 7. Brightness defaults to 0 on initial "factory" startup.  *
 **************************************************************/

/*
Short term goals:
- Add screen text for play/pause, birghtness, sleep mode
- Add BLE timout that restarts on keypress

Mid term goals:
- Make sleep wakeup from any keypress? (maybe use rtc_gpio_wakeup_enable(gpio_num_t gpio_num, gpio_int_type_t intr_type see https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/gpio.html#api-reference-rtc-gpio)
- Make it easy to integrate macros.
- Make it easier to load gifs/images right from sd card rather than going into the raw code. probably switch
- Add wired version of keyboard


Long term ideal goals/nice to haves
- Make encoder work after 2 digits rather than 4 (1 tick rather than 2)
- Bluetooth pairing mode or disconnect feature
- Find a better gif decoder or a find a way to make this one have seemless loops. There is currently still a small delay each time it is loaded.
- Integrate a user menu to change picture delay or gif loops with the rotory encoder in the function menu.
  - Could also add way to add/remove existing pictures/gifs from this menu (might need to make switch case into if/else if statements, using var + 1 as a condition to keep sequence)
- Add pong to the screen
- Clean up this messy massive piece of code
- Optimize code
- Add batery and screen display for batery level, time, version/info. 
  - Ideally using a 3.7v li-ion bat that is regulated to 3.3v for esp32. Need to have a protected charging circuit -> power too low...tried and wouldn't boot
*/

// Keyboard
#include <Arduino.h>
#include <BleKeyboard.h> //for bluetooth
#include <Keypad.h>

// images/animated gif includes start
#include <Preferences.h>
#include <AnimatedGIF.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789 (tft driver)
#include <SD.h>

// sleep pin libraries
//#include "driver/gpio.h" //use these two for normal gpio holds during sleep and etc.
//#include "soc/soc.h" //disable brownout problems
#include "driver/rtc_io.h"
#include "soc/rtc_cntl_reg.h" //disable brownout problems
#include "soc/rtc.h"
#include "esp_sleep.h"



//Config defines
#define encoderEnable //default enables both encoders
#define TFTEnable //default enables GIFs
#define troubleshoot // enables serial communication. Uncomment messages in matrix scan if needed. Uncomment messages in gif loop if needed.
//#define getLoops //shows the loops/second of the matrix scan of Keyboard when troubleshoot is defined


// animated gif
#define TFT_CS         16 //pin
#define TFT_RST        4 //pin
#define TFT_DC         0 //pin ||RTCIO
#define TFT_MOSI       23 //pin
#define TFT_CLK        18 //pin
#define TFT_MISO       19 //pin
#define TFT_LIT        15 //brightness pin, pwm || RTCIO

#define DISPLAY_WIDTH 240
#define DISPLAY_HEIGHT 135

#ifndef BUILTIN_SDCARD
#define BUILTIN_SDCARD 0
#endif

#ifdef encoderEnable 
//Encoder 2
#define ENC_A2 22 //CLK pin
#define ENC_B2 21 //DT pin
unsigned long _lastIncReadTime = micros(); 
unsigned long _lastDecReadTime = micros();
int _pauseLength = 25000;
int _fastIncrement = 5; // Changes the multiplier of the fast encoder turning (when using count)
volatile int counter = 0;  // Uncomment for counter use with encoder

//Encoder 1
#define ENC_A1 13 //CLK pin
#define ENC_B1 9 //DT pin
unsigned long _lastIncReadTime1 = micros(); 
unsigned long _lastDecReadTime1 = micros(); 
int _pauseLength1 = 25000;
int _fastIncrement1 = 5; // Changes the multiplier of the fast encoder turning (when using count)
volatile int counter1 = 0;  // Uncomment for counter use with encoder
#endif

//Keyboard

#define Row1 32 //pin RTCIO
#define Row2 12 //pin RTCIO
#define Row3 14 //pin RTCIO
#define Row4 27 //pin RTCIO
#define Row5 26 //pin RTCIO
#define Row6 10 //pin RTCIO

#define Col1 33 //pin
#define Col2 25 //pin 
#define Col3 5 //pin
#define Col4 17 //pin

const byte ROWS = 6;
const byte COLS = 4;
int activeLayer = 0; //default layer

int batVInput = 34; // pin for battery input to check percentage
int battVal = 0; // ADC value of battery voltage
int battPercent = 0; // current battery charge percent

char keys[ROWS][COLS] =
  /* Default
    *   Col1 | Col2 | Col3 | Col4
    * ,----------------------------.
    * | ENC1 | ENC2 |    Screen    | Row1
    * |------+------+------+-------|      
    * | FUNC |   /  |   *  |BackSpc| Row2
    * |------+------+------+-------|
    * |   7  |   8  |   9  |   -   | Row3
    * |------+------+------+-------|
    * |   4  |   5  |   6  |   +   | Row4
    * |------+------+------+-------|
    * |   1  |   2  |   3  |   =   | Row5
    * |------+------+------+-------|
    * |   0  |  00  |  .   | Enter | Row6
    * `----------------------------'
    */    
/*

To use other media keys use a custom key hexcode provided below. Then adjust/add
code to the matrix scan of the Keyboard in the loop() function if successful.
Check bleKeyboard library for media key names.

Custom keys that should work (have no other keybinds):
0x01 - 0x07
0x0B - 0x1F
0x88 - 0x8B
0xEE - 0xFF

Use key names from the keypad or bleKeyboard library when changing keymap. Some keys on bleKeyboard are finicky. See above for media keys.
*/

{              //0x00 = null | Left encoder = 0x8A | Right encoder = 0x8B |don't change encoder keypress here. Change them in the matrix scan by comment "keypress for encoder"
    {0x00      , 0x00          , 0x8A             , 0x8B},
    {0xEE      , KEY_NUM_SLASH , KEY_NUM_ASTERISK , KEY_BACKSPACE}, //0xDB = NumLk | 0xEE = layer key
    {KEY_NUM_7 , KEY_NUM_8     , KEY_NUM_9        , KEY_NUM_MINUS},
    {KEY_NUM_4 , KEY_NUM_5     , KEY_NUM_6        , KEY_NUM_PLUS},
    {KEY_NUM_1 , KEY_NUM_2     , KEY_NUM_3        , '='},
    {KEY_NUM_0 , KEY_RIGHT_ALT , KEY_NUM_PERIOD   , KEY_NUM_ENTER    },
};

char keys1[ROWS][COLS] =
  /* Default
    *   Col1 | Col2 | Col3 | Col4
    * ,----------------------------.
    * |  ENC | ENC1 |    Screen    | Row1
    * |------+------+------+-------|      
    * | FUNC | BATT |  ___ |  DEL  | Row2
    * |------+------+------+-------|
    * | NumLk|  ___ |  ___ |  ___  | Row3
    * |------+------+------+-------|
    * | BLEON|BLEOFF|  ___ |  ___  | Row4 //BLE inop currently
    * |------+------+------+-------|
    * |  ___ |  ___ |  ___ |profsav| Row5
    * |------+------+------+-------|
    * | SLEEP|  ___ |  ___ |profsav| Row6
    * `----------------------------'
    */

/*
Custom keys that should work (have no other keybinds):
0x01 - 0x07
0x0B - 0x1F
0x88 - 0x8B
0xEE - 0xFF
*/

{              //0x00 = null | Left encoder = 0x8A | Right encoder = 0x8B |don't change encoder keypress here. Change them in the matrix scan by comment "keypress for encoder"
    {0x00 , 0x00 , 0x8A , 0x8B },
    {0xEE , 0x05 , 0x00 , KEY_DELETE }, //0x05 = Battery %
    {0xDB , 0x00 , 0x00 , 0x00 }, //0xDB = NumLk
    {0x03 , 0x04 , 0x00 , 0x00 }, //0x03 = BLE on (inop currently) || 0x04 = BLE off
    {0x00 , 0x00 , 0x00 , 0x01 }, //
    {0x02 , 0x00 , 0x00 , 0x01 }, //0x02 = Deep sleep || 0x01 = profileSave for preferences
};

byte rowPins[ROWS] = {Row1, Row2, Row3, Row4, Row5, Row6}; //diode direction is rows --> columns : annode on row side, cathode on column side
byte colPins[COLS] = {Col1, Col2, Col3, Col4};


Keypad kpd = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS ); //initiate keypad
Keypad kpd1 = Keypad( makeKeymap(keys1), rowPins, colPins, ROWS, COLS ); //initiate keypad

//Used for cycle count
#ifdef troubleshoot
unsigned long loopCount;
unsigned long startTime;
#endif
String msg;

BleKeyboard bleKeyboard; //declare ble Keyboard name



// animated gif code portion start 1/3
#ifdef TFTEnable

int timer = 0;
int i = 0;
int imageCount = 7; // change number of images loaded here.
int gifCount = 3; // change number of gifs loaded here.

int gifNumber; //put in retentive memory
int screen; //put in retentive memory
int image; //put in retentive memory
int brightness; //put in retentive memory
bool play; //put in retentive memory

bool cycleImage = 1; //must be high for initial image load
bool cycleGif = 1;

Adafruit_ST7789      tft    = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST); //initiate tft screen

TaskHandle_t Task1; //lable for task used for gif

AnimatedGIF gif; //name for gif
File f; //name for file

Preferences preferences; //name for preferences (retentive memory)

// #define BUTTON_PIN_BITMASK 0x1000 // 2^12 in hex || 200000000 // 2^33 in hex used for ext1 wakup, not used here

//RTC_DATA_ATTR int bootCount = 0; //boot counter for sleep wakeup troubleshooting

void * GIFOpenFile(const char *fname, int32_t *pSize)
{
  f = SD.open(fname);
  if (f)
  {
    *pSize = f.size();
    return (void *)&f;
  }
  return NULL;
} /* GIFOpenFile() */

void GIFCloseFile(void *pHandle)
{
  File *f = static_cast<File *>(pHandle);
  if (f != NULL)
     f->close();
} /* GIFCloseFile() */

int32_t GIFReadFile(GIFFILE *pFile, uint8_t *pBuf, int32_t iLen)
{
    int32_t iBytesRead;
    iBytesRead = iLen;
    File *f = static_cast<File *>(pFile->fHandle);
    // Note: If you read a file all the way to the last byte, seek() stops working
    if ((pFile->iSize - pFile->iPos) < iLen)
       iBytesRead = pFile->iSize - pFile->iPos - 1; // <-- ugly work-around
    if (iBytesRead <= 0)
       return 0;
    iBytesRead = (int32_t)f->read(pBuf, iBytesRead);
    pFile->iPos = f->position();
    return iBytesRead;
} /* GIFReadFile() */

int32_t GIFSeekFile(GIFFILE *pFile, int32_t iPosition)
{ 
  int i = micros();
  File *f = static_cast<File *>(pFile->fHandle);
  f->seek(iPosition);
  pFile->iPos = (int32_t)f->position();
  i = micros() - i;
  //  Serial.printf("Seek time = %d us\n", i);
  return pFile->iPos;
} /* GIFSeekFile() */

// Draw a line of image directly on the LCD
void GIFDraw(GIFDRAW *pDraw)
{
    uint8_t *s;
    uint16_t *d, *usPalette, usTemp[320];
    int x, y, iWidth;

    iWidth = pDraw->iWidth;
    if (iWidth + pDraw->iX > DISPLAY_WIDTH)
       iWidth = DISPLAY_WIDTH - pDraw->iX;
    usPalette = pDraw->pPalette;
    y = pDraw->iY + pDraw->y; // current line
    if (y >= DISPLAY_HEIGHT || pDraw->iX >= DISPLAY_WIDTH || iWidth < 1)
       return; 
    s = pDraw->pPixels;
    if (pDraw->ucDisposalMethod == 2) // restore to background color
    {
      for (x=0; x<iWidth; x++)
      {
        if (s[x] == pDraw->ucTransparent)
           s[x] = pDraw->ucBackground;
      }
      pDraw->ucHasTransparency = 0;
    }

    // Apply the new pixels to the main image
    if (pDraw->ucHasTransparency) // if transparency used
    {
      uint8_t *pEnd, c, ucTransparent = pDraw->ucTransparent;
      int x, iCount;
      pEnd = s + iWidth;
      x = 0;
      iCount = 0; // count non-transparent pixels
      while(x < iWidth)
      {
        c = ucTransparent-1;
        d = usTemp;
        while (c != ucTransparent && s < pEnd)
        {
          c = *s++;
          if (c == ucTransparent) // done, stop
          {
            s--; // back up to treat it like transparent
          }
          else // opaque
          {
             *d++ = usPalette[c];
             iCount++;
          }
        } // while looking for opaque pixels
        if (iCount) // any opaque pixels?
        {
          tft.startWrite();
          tft.setAddrWindow(pDraw->iX+x, y, iCount, 1);
          tft.writePixels(usTemp, iCount, false, false);
          tft.endWrite();
          x += iCount;
          iCount = 0;
        }
        // no, look for a run of transparent pixels
        c = ucTransparent;
        while (c == ucTransparent && s < pEnd)
        {
          c = *s++;
          if (c == ucTransparent)
             iCount++;
          else
             s--; 
        }
        if (iCount)
        {
          x += iCount; // skip these
          iCount = 0;
        }
      }
    }
    else
    {
      s = pDraw->pPixels;
      // Translate the 8-bit pixels through the RGB565 palette (already byte reversed)
      for (x=0; x<iWidth; x++)
        usTemp[x] = usPalette[*s++];
      tft.startWrite();
      tft.setAddrWindow(pDraw->iX, y, iWidth, 1);
      tft.writePixels(usTemp, iWidth, false, false);
      tft.endWrite();
    }
} /* GIFDraw() */
  #endif
//animated gif portion end 1/3

//Wakeup reason troubleshoot message
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

void setup() {

  setCpuFrequencyMhz(80);

  //Deep sleep
  rtc_gpio_hold_dis((gpio_num_t) TFT_LIT);
  //Serial.begin(115200); //good idea to increase delay a bit to give time to serial monitor to start up
  
  //Increment boot number and print it every reboot
  //++bootCount; //must undefine the initial bootCount setup above
  //Serial.println("Boot number: " + String(bootCount));

  //Print the wakeup reason for ESP32
  //print_wakeup_reason();

  esp_sleep_enable_ext0_wakeup(GPIO_NUM_13,0); //1 = High, 0 = Low || wake up from sleep when GPIO 32 goes low (Turn encoder 1)

  //open "last_state" preferences in read/write mode
  preferences.begin("last_state", false);
    //remove all preferences under the opened namespace
  //preferences.clear();
  
  //set varibles to previous saved value, if none then set 0
  gifNumber = preferences.getInt("gifNumber", 1);
  screen = preferences.getInt("screen", 0);
  image = preferences.getInt("image", 1);
  brightness = preferences.getInt("brightness", 150);
  play = preferences.getBool("play", 0);
  
  //close "last_state" preferences
  preferences.end();

  //Encoder setup
  #ifdef encoderEnable
  //encoder 0
  pinMode(ENC_A2, INPUT_PULLUP); //declare pin for encoder
  pinMode(ENC_B2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A2), read_encoder, CHANGE); //interupt for encoder state
  attachInterrupt(digitalPinToInterrupt(ENC_B2), read_encoder, CHANGE);
  //encoder 1
  pinMode(ENC_A1, INPUT_PULLUP);
  pinMode(ENC_B1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A1), read_encoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B1), read_encoder1, CHANGE);
  #endif
  
  #ifdef troubleshoot
    Serial.begin(115200);
    #ifdef getLoops
      loopCount = 0; 
      startTime = millis();
    #endif
  
  /*

  //retentive memory readout
  Serial.println(" ");
  Serial.print("gifCount = ");
  Serial.println(gifNumber);
  Serial.print("screen = ");
  Serial.println(screen);
  Serial.print("image = ");
  Serial.println(image);
  Serial.print("brightness = ");
  Serial.println(brightness);  
  Serial.print("play = ");
  Serial.println(play);
  */

  #endif
  msg = "";

  //Start Keyboard
    //bleKeyboard.begin(); //Start bluetooth Keyboard
 
  //animated gif portion start 2/3
  #ifdef TFTEnable

  analogWrite(TFT_LIT, brightness); //start tft at brigthness level

  if(!SD.begin(2)){ //SDCS pin
    Serial.println("SD Card mount failed!"); //more messages to uncomment if needed.
    return;
  }
  else{
    Serial.println("SD Card mount succeeded!");
  }

  tft.init(135, 240); //tft screen size
  tft.setRotation(3); //direction of tft screen
  tft.fillScreen(ST77XX_BLACK); //fills screen black
  gif.begin(LITTLE_ENDIAN_PIXELS);

  xTaskCreatePinnedToCore(Task1code, "Task1", 10000, 0, 1, &Task1, 0); //Creates a task to run on core 0 (esp32 has cpu cores: 0 and 1) 

  //Serial.println("Setup Complete");
  #endif
  //animated gif portion end 2/3


// put your setup code here, to run once:


} /* Setup */

//Keyboard scan
void loop() {

 /* //Lines for encoder 0 counter --- needs to be duplicated for encoder 1 use and have other counter lines uncommented    
  static int lastCounter = 0; //Uncomment for encoder counter use

  // If counter has changed print the new value to serial
  if(counter != lastCounter){
    Serial.println(counter);
    lastCounter = counter;
  }*/

  // Used to check loops per second, default should be around 103,000
  #ifdef getLoops
  loopCount++;
  if ( (millis()-startTime)>5000 ) {
    Serial.print("Average loops per second = ");
    Serial.println(loopCount/5);
    startTime = millis();
    loopCount = 0;
  }
  #endif

  // Fills kpd.key[] array with up-to 10 active keys.
  // Returns true if there are ANY active keys.
  if(activeLayer == 0){ //LAYER 0 (DEFAULT)
    if (kpd.getKeys())
    {
      for (int i=0; i<LIST_MAX; i++)   // Scan the whole key list.
      {
        if ( kpd.key[i].stateChanged )   // Only find keys that have changed state.
        {
          switch (kpd.key[i].kstate) {  // Report active key state : IDLE, PRESSED, HOLD, or RELEASED
            case PRESSED:
              //msg = " PRESSED."; //uncomment if checking keypress for porper function with keycheck code
              bleKeyboard.press(kpd.key[i].kchar);
              if(kpd.key[i].kchar == 0xEE){
                activeLayer = 1;
              }
    #ifdef encoderEnable
              if(kpd.key[i].kchar == 0x8A){ //used in lue of an ascii media key
                bleKeyboard.write(KEY_MEDIA_MUTE); //Keypress for encoder.
              }
              if(kpd.key[i].kchar == 0x8B){ //used in lue of an ascii media key
                bleKeyboard.write(KEY_MEDIA_PLAY_PAUSE); //Keypress for encoder1
              }
    #endif
          break;
            case HOLD:
            //msg = " HOLD."; //uncomment if checking keypress for porper function with keycheck code
          break;
            case RELEASED:
              //msg = " RELEASED.";
              bleKeyboard.release(kpd.key[i].kchar);
              if(kpd.key[i].kchar == 0xEE){
                activeLayer = 0;
              }        
          break;
            //case IDLE: //uncomment if checking keypress for porper function with keycheck code
            //msg = " IDLE."; 
          }
        }
      }
    }
  
  }else if(activeLayer == 1){ //LAYER 1 (FUNCTION)
    if (kpd1.getKeys())
    {
      for (int i=0; i<LIST_MAX; i++)   // Scan the whole key list.
      {
        if ( kpd1.key[i].stateChanged )   // Only find keys that have changed state.
        {
          switch (kpd1.key[i].kstate) {  // Report active key state : IDLE, PRESSED, HOLD, or RELEASED
            case PRESSED:
            //msg = " PRESSED."; //uncomment if checking keypress for porper function with keycheck code
            bleKeyboard.press(kpd1.key[i].kchar);
            if(kpd1.key[i].kchar == 0xEE){
              activeLayer = 1; //is this nessary on this layer?
            }
            if(kpd1.key[i].kchar == 0x01){ //Saves current profile (overwrites last values)
              preferences.begin("last_state", false);
              preferences.putInt("gifNumber", gifNumber);
              preferences.putInt("screen", screen);
              preferences.putInt("image", image);
              preferences.putInt("brightness", brightness);
              preferences.putBool("play", play);
              preferences.end();
            //add line here to show "Preferences Saved" on tft screen for a bit.
            }
            if(kpd1.key[i].kchar == 0x02){ //Put Keyboard into sleep mode
              Serial.println("Going to sleep now");

              analogWrite(TFT_LIT, 0); // set LIT low (tft backlight)
              delay(100); //must have this display or else the screen backlight will stay on
              rtc_gpio_hold_en((gpio_num_t) TFT_LIT); //hold the TFT_LIT pin at its last value, even during deep sleep

              tft.enableSleep(true);
              esp_deep_sleep_start(); //Initiate deep sleep mode
            }
            if(kpd1.key[i].kchar == 0x03){ // begin bluetooth
              bleKeyboard.begin();
            }
            if(kpd1.key[i].kchar == 0x04){
              ESP.restart();
              //btStop();
            }
            if(kpd1.key[i].kchar == 0x05){ //Display battery %
              battVal = analogRead(batVInput);
              Serial.println(battVal);
            
            /*
            using a 33k / 50k voltage divider
            min op v for 3.3v reg = 2.7v .........2.5v overdischarge protection on other circuit.
            charging = 4.2v for charging
            max batt = ?.?v
            2.7-4.2v operating range
            2.7v on batt = 1.782V on input = 2211.3
            4.2v on batt = 2.772V on input = 3439.8
            any value between these will be the battery %
            0v = 0 || 3.3v = 4095
            1240.909/1v
            range from 0% - 100% = 2211.3 - 3439.8
            Analog value - 2211.3 / 1228.5 * 100 = Percentage
            Probably round to nearest whole number
            */


            }
    #ifdef encoderEnable
            if(kpd1.key[i].kchar == 0x8A){ //used in lue of an ascii media key,     left encoder
              if(screen < 1){
                screen ++;
              }else {
                screen = 0;
              }
              cycleGif = 1;
              cycleImage = 1;
              Serial.println("changing screen");
            }
            if(kpd1.key[i].kchar == 0x8B){ //used in lue of an ascii media key,     right encoder
              if(play){
                play = 0;
              }else {
                play = 1;
                cycleGif = 1;
                cycleImage = 1;
              }
              Serial.println("plushing play");
            }
    #endif
          break;
            case HOLD:
            //msg = " HOLD."; //uncomment if checking keypress for porper function with keycheck code
          break;
            case RELEASED:
            msg = " RELEASED.";
            bleKeyboard.release(kpd1.key[i].kchar);
            if(kpd1.key[i].kchar == 0xEE){
              activeLayer = 0;
            }        
          break;
            //case IDLE: //uncomment if checking keypress for porper function with keycheck code
            //msg = " IDLE."; 
          }


          // KEY CHECK: Used to check for keypresses and proper - function uncomment above messages
    #ifdef troubleshoot
          Serial.print("Key ");
          Serial.print(kpd.key[i].kchar);
          Serial.println(msg);
    #endif
        }
      }
    }
  }
}// Matrix scan loop  
  //animated gif portion start 3/3
  
//GIF and Image Loop
#ifdef TFTEnable
void Task1code(void * pvParameters){ 
  for(;;){
    if(screen == 0){
      if(cycleGif){
        gif.close(); 
        switch(gifNumber){
          case 1:
            if(gif.open("/SleepyMoon2a.GIF", GIFOpenFile, GIFCloseFile, GIFReadFile, GIFSeekFile, GIFDraw)){        
            }/*else{
              Serial.printf("Error opening file = %d\n", gif.getLastError());
              while (1){
              }
            }*/
            break;  

          case 2:
            if(gif.open("/FlyingThroughClouds4.GIF", GIFOpenFile, GIFCloseFile, GIFReadFile, GIFSeekFile, GIFDraw)){         
            }/*else {
              Serial.printf("Error opening file = %d\n", gif.getLastError());
              while (1){
              }
            }*/
            break;

          case 3:
            if(gif.open("/Tantabus3.GIF", GIFOpenFile, GIFCloseFile, GIFReadFile, GIFSeekFile, GIFDraw)){         
            }/*else {
              Serial.printf("Error opening file = %d\n", gif.getLastError());
              while (1){
              }
            }*/
            tft.fillScreen(ST77XX_BLACK); //fills screen black for background because gif is not the full size of the tft screen
            break;
          
          //Add aditional gif's below this line

          default:
            tft.fillScreen(ST77XX_BLUE);
            break;


        }
        cycleGif = 0;
      }

    /*GIF case template: uncomment all aditional lines for trouble shooting
        case 3:
          if(gif.open("/<YourFileNameHere.GIF>", GIFOpenFile, GIFCloseFile, GIFReadFile, GIFSeekFile, GIFDraw)){         
          }//else {
            //Serial.printf("Error opening file = %d\n", gif.getLastError());
            //while (1){
            //}
          //}
          //tft.fillScreen(ST77XX_BLACK); // uncomment and change to desired colour (must be all caps). Most solid colours will work.
    //       break; //uncomment this line
    */


    /* ADD GIF (max 0-255)
    1. To add a gif, remove SD card and copy any gif file onto the root folder of the SD card. Super long gifs will most likely not work, but you can try.
      Default TFT size is 240 pixels wide by 135 high. https://ezgif.com is a great website to edit/create GIFs
    2. Copy the GIF case template, and paste it below the line to add aditional gifs
    3. Replace the number of teh case with the next in line (ex: if "case 2:" is the last case, use "case 3:" next)
    4. Change the gif.open file name ( ex: gif.open("/<YourFileNameHere.GIF>" )
    5. If your gif is not the full size of the screen, add a line to chose a background colour otherwise skip this step. Gif image will overlap with previous gif if it is smaller than full screen.
      (ex: tft.fillScreen(ST77XX_BLACK)
    6. Scroll down to the comment "Change number of GIFs here." Number should match the number of cases (ex. if there are 4 gif cases, put number 4 there. if there are 2, use 2)
    7. Make sure SD card is plugged back in.
    8. Flash numpad with new code (default microcontroller is: NodeMCU-32s. Make sure to pick your board before flashing)
    9. Observe and make sure things work. If they don't work, define troubleshoot at top of code and open serial monitor (top right corner of ArduinoIDE).
      If there is a continous stream of wierd looking stuff, the GIF is probably too long. Otherwise check any messages and try to fix it. Uncomment SD card messages if needed.

    */

      GIFINFO gi;

    /* //Reads off information of GIF in serial monitor
      Serial.printf("Successfully opened GIF; Canvas size = %d x %d\n", gif.getCanvasWidth(), gif.getCanvasHeight());
      if (gif.getInfo(&gi)) {
        Serial.printf("frame count: %d\n", gi.iFrameCount);
        Serial.printf("duration: %d ms\n", gi.iDuration);
        Serial.printf("max delay: %d ms\n", gi.iMaxDelay);
        Serial.printf("min delay: %d ms\n", gi.iMinDelay);
      }
    */
    //change the number in "i < 2" to increase or decrease loops of gif before switching to next gif
      for(i = 0; i < 2; i++){ //how many times 1 gif loops. Seems to help with loading delay when using this
        while (gif.playFrame(true, NULL) && screen == 0 && cycleGif == 0){
        }  
      }
      if(play){
        if(gifNumber < gifCount){
          gifNumber++;
        }else{
          gifNumber = 1;
        }
        cycleGif = 1;
      }   
    }else if(screen == 1){ //Draw sigle image (.gif type) and cycle through them when prompted.
      if(cycleImage){
        timer = 0; 
        gif.close();   
        switch(image){ //make sure image number matches the number of cases -> go to: Change number of images here
          case 1:
            if(gif.open("/FlutterShy.GIF", GIFOpenFile, GIFCloseFile, GIFReadFile, GIFSeekFile, GIFDraw)){        
            }/*else{
              Serial.printf("Error opening file = %d\n", gif.getLastError());
              while (1){
              }
            }*/
            break;

          case 2:
            if(gif.open("/Resting_Luna.GIF", GIFOpenFile, GIFCloseFile, GIFReadFile, GIFSeekFile, GIFDraw)){        
            }/*else{
              Serial.printf("Error opening file = %d\n", gif.getLastError());
              while (1){
              }
            }*/
            break; 

          case 3:
            if(gif.open("/Space_Luna.GIF", GIFOpenFile, GIFCloseFile, GIFReadFile, GIFSeekFile, GIFDraw)){        
            }/*else{
              Serial.printf("Error opening file = %d\n", gif.getLastError());
              while (1){
              }
            }*/
            break;

          case 4:
            if(gif.open("/GlassWings.GIF", GIFOpenFile, GIFCloseFile, GIFReadFile, GIFSeekFile, GIFDraw)){        
            }/*else{
              Serial.printf("Error opening file = %d\n", gif.getLastError());
              while (1){
              }
            }*/
            break;

          case 5:
            if(gif.open("/MistyMountain.GIF", GIFOpenFile, GIFCloseFile, GIFReadFile, GIFSeekFile, GIFDraw)){        
            }/*else{
              Serial.printf("Error opening file = %d\n", gif.getLastError());
              while (1){
              }
            }*/
            break;

          case 6:
            if(gif.open("/SpaceDragon.GIF", GIFOpenFile, GIFCloseFile, GIFReadFile, GIFSeekFile, GIFDraw)){        
            }/*else{
              Serial.printf("Error opening file = %d\n", gif.getLastError());
              while (1){
              }
            }*/
            break;

          case 7:
            if(gif.open("/SpaceEagle.GIF", GIFOpenFile, GIFCloseFile, GIFReadFile, GIFSeekFile, GIFDraw)){        
            }/*else{
              Serial.printf("Error opening file = %d\n", gif.getLastError());
              while (1){
              }
            }*/
            break;

        //Add aditional images below this line

          default:
            tft.fillScreen(ST77XX_BLUE);
            break;

        }
        cycleImage = 0;
        while (gif.playFrame(true, NULL)){
        }      
      }
      if(timer < 3000){ // loops || Max ~2,000,000 loops || @10ms delay 3,000 = 30 seconds / 15,000 = 2.5m / 30,000 = 5 min / @125ms delay 691,200 = 1 day
        delay(10); //time in ms, increase if desired. anything between 10 - 500 is recomended. prevents infinite looping forcing a reset of the MCU (this is a work around for runtime error?)
        timer ++;
      }else{
        timer = 0;
      }
      if(play && timer == 3000){
        if(image < imageCount){ //changes which image plays | Change number of images here
          image++;
        }else{
          image = 1;
        }
        cycleImage = 1;      
      }

    /* ADD IMAGE (max 0 - 255)
       almost the same steps as adding a gif, just make sure to use a single image .gif file, ideally with the 240 x 135 size.
       make sure image number matches the number of cases -> go to: imageCount
    */

    }
  }
}
#endif
//animated gif portion end 3/3

//Right encoder code (Enc2)
void read_encoder() {
  // Encoder interrupt routine for both pins. Updates counter
  // if they are valid and have rotated a full indent
 
  static uint8_t old_AB = 3;  // Lookup table index
  static int8_t encval = 0;   // Encoder value  
  static const int8_t enc_states[]  = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; // Lookup table

  old_AB <<=2;  // Remember previous state

  if (digitalRead(ENC_A2)) old_AB |= 0x02; // Add current state of pin A
  if (digitalRead(ENC_B2)) old_AB |= 0x01; // Add current state of pin B
  
  encval += enc_states[( old_AB & 0x0f )];

  // Update counter if encoder has rotated a full indent, that is at least 4 steps
  if( encval > 3 ) {        // Four steps forward
    /*int changevalue = 1;  // Uncomment for fast scroll option when using counter
    if((micros() - _lastIncReadTime) < _pauseLength) {
      changevalue = _fastIncrement * changevalue; 
    }*/
    _lastIncReadTime = micros();
    //counter = counter + changevalue;              // Update counter
    if(activeLayer == 0){
      bleKeyboard.write(KEY_MEDIA_VOLUME_UP); //Change encoder0 CW keypress here
    }else if(activeLayer == 1){
      if(screen == 1){ //manually cycle images
        if(!play && !cycleImage){
          if(image < imageCount){ 
            image ++;
          }else {
            image = 1;
          }
          cycleImage = 1;
        }
      }else if(screen == 0){ //for manual cycling GIFs
        if(!play && !cycleGif){
          if(gifNumber < gifCount){
            gifNumber ++;
          } else{
            gifNumber = 1;
          }
          cycleGif = 1;
        }
      }
    } 
    encval = 0;
  }
  else if( encval < -3 ) {        // Four steps backward
    /*int changevalue = -1;  // Uncomment for fast scroll option when using counter
    if((micros() - _lastDecReadTime) < _pauseLength) {
      changevalue = _fastIncrement * changevalue; 
    }*/
    _lastDecReadTime = micros();
    //counter = counter + changevalue;              // Update counter
    if(activeLayer == 0){    
      bleKeyboard.write(KEY_MEDIA_VOLUME_DOWN); //Change encoder0 CCW keypress here
    }else if(activeLayer == 1){
      if(screen == 1){ //manually cycle images
        if(!play && !cycleImage){
          if(image > 1){ 
            image --;
          }else {
            image = imageCount;
          }
        }
        cycleImage = 1;
      }else if(screen == 0){ //for manual cycling GIFs
        if(!play && !cycleGif){
          if(gifNumber > 1){
            gifNumber --;
          } else{
            gifNumber = gifCount;
          }
        }
        cycleGif = 1;
      }
    }
    encval = 0;
  }
}

//Left encoder code (Encf1)
void read_encoder1() {
  // Encoder interrupt routine for both pins. Updates counter
  // if they are valid and have rotated a full indent
 
  static uint8_t old_AB1 = 3;  // Lookup table index
  static int8_t encval1 = 0;   // Encoder value  
  static const int8_t enc_states1[]  = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; // Lookup table

  old_AB1 <<=2;  // Remember previous state

  if (digitalRead(ENC_A1)) old_AB1 |= 0x02; // Add current state of pin A
  if (digitalRead(ENC_B1)) old_AB1 |= 0x01; // Add current state of pin B
  
  encval1 += enc_states1[( old_AB1 & 0x0f )];

  // Update counter if encoder has rotated a full indent, that is at least 4 steps
  if( encval1 > 3 ) {        // Four steps forward
    /*int changevalue1 = 1;
    if((micros() - _lastIncReadTime1) < _pauseLength1) {  // Uncomment for fast scroll option when using counter
      changevalue1 = _fastIncrement1 * changevalue1; 
    }*/
    _lastIncReadTime1 = micros();
    //counter1 = counter1 + changevalue1;              // Update counter
    if(activeLayer == 0){
      bleKeyboard.write(KEY_PAGE_UP); //Change encoder1 CW keypress here
    }else if(activeLayer == 1){
      if(brightness < 255){
        brightness += 15;
        analogWrite(TFT_LIT, brightness);
      }
    }
    encval1 = 0;
  }
  else if( encval1 < -3 ) {        // Four steps backward
    /*int changevalue1 = -1;
    if((micros() - _lastDecReadTime1) < _pauseLength1) {  // Uncomment for fast scroll option when using counter
      changevalue1 = _fastIncrement1 * changevalue1; 
    }*/
    _lastDecReadTime1 = micros();
    //counter1 = counter1 + changevalue1;              // Update counter
    if(activeLayer == 0){
      bleKeyboard.write(KEY_PAGE_DOWN); //Change encoder1 CCW keypress here
    }else if(activeLayer == 1){
      if(brightness > 0){
        brightness -= 15;
        analogWrite(TFT_LIT, brightness);
      }
    }
    encval1 = 0;
  }
} 
