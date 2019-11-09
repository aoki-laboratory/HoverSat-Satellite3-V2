//------------------------------------------------------------------//
//Supported MCU:   ESP32 (M5Stack)
//File Contents:   HoverSat Satellite3
//Version number:  Ver.1.2
//Date:            2019.06.14
//------------------------------------------------------------------//
 
//This program supports the following boards:
//* M5Stack(Grey version)
 
//Include
//------------------------------------------------------------------//
#include <M5Stack.h>
#include <Servo.h>
#include <Wire.h>
#include <WiFi.h>
#include <time.h>
#include <EEPROM.h>
#include "BluetoothSerial.h"
#include "BasicStepperDriver.h"


//Define
//------------------------------------------------------------------//
#define   TIMER_INTERRUPT     10      // ms
#define   LCD
#define   ONE_ROTATION_LENGTH 78.5
#define   REDUCTION_RATIO 1.875

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 400
// Target RPM for cruise speed
#define RPM 120

// Microstepping mode. If you hardwired it to save pins, set to the same value here.
#define MICROSTEPS 1

#define DIR 26
#define STEP 19


// 2-wire basic config, microstepping is hardwired on the driver
BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP);

#define BufferRecords 16

#define NOOFPATTERNS  3

int parameters[NOOFPATTERNS][5] =
{
// Accel, Velocity, Decel, TIme
  {1000, 100, 100, 100, 10000 },
  {1000, 200, 100, 400, 10000 },
  {1000, 400, 100, 800, 10000 }
};



//Global
//------------------------------------------------------------------//
int     pattern = 0;
int     tx_pattern = 0;
int     rx_pattern = 0;
int     rx_val = 0;
bool    hover_flag = false;
bool    log_flag = false;
bool    telemetry_flag = false;
int     cnt10 = 0;

unsigned long time_ms;
unsigned long time_stepper = 0;
unsigned long time_buff = 0;
unsigned long time_buff2 = 0;
unsigned char current_time = 0; 
unsigned char old_time = 0;  

byte    counter;
char charBuf[100];
char charBuf2[100];
long  abslength = 0;
boolean inc_flag = false;
long steps;
float velocity;
boolean hasData = false;
String label = "Tick";
static const int Limit1Pin = 17;
static const int Limit2Pin = 34;
int  Limit1State = 1;
int  Limit2State = 1;

// Stepper
const int Stepper_Enable_Pin = 25;
int       Stepper_Enable = 1;
int       motor_accel = 666;
int       motor_decel = 666;

static const int TSND_121 = 13;
int  TSND_121_ENABLE = 0;

// progress
float  current_length;
float  current_velocity;
float  current_accel;
float  old_length=0;
char stepper_pattern=10;

BluetoothSerial bts;
String  bts_rx;
char bts_rx_buffer[16];
int bts_index = 0;

//String ssid_buff;
//String pass_buff;
//const char* ssid;
//const char* pass;

// Your WiFi credentials.
// Set password to "" for open networks.
//char ssid[] = "Buffalo-G-0CBA";
//char pass[] = "hh4aexcxesasx";
char ssid[] = "X1Extreme-Hotspot";
char pass[] = "5]6C458w";
//char ssid[] = "Macaw";
//char pass[] = "1234567890";


// Time
char ntpServer[] = "ntp.nict.jp";
const long gmtOffset_sec = 9 * 3600;
const int  daylightOffset_sec = 0;
struct tm timeinfo;
String dateStr;
String timeStr;

File file;
String fname_buff;
const char* fname;

String accel_buff;
const char* accel_out;

typedef struct {
    String  log_time;
    int     log_pattern;
    String  log_time_ms;
    float   log_length;
    float   log_velocity;
    float   log_accel;
} RecordType;

static RecordType buffer[2][BufferRecords];
static volatile int writeBank = 0;
static volatile int bufferIndex[2] = {0, 0};


// DuctedFan
static const int DuctedFanPin = 15;
Servo DuctedFan;

// Timer Interrupt
volatile int interruptCounter;
volatile int interruptCounterS;
int totalInterruptCounter;
int iTimer10;


hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Parameters
unsigned char hover_val = 70;
unsigned int ex_length = 2000;
unsigned int ex_velocity = 200;
unsigned int ex_accel = 5;
unsigned int ex_decel = 5;
unsigned char wait = 5;
unsigned char dir_flag = 1;
unsigned char ssid_pattern = 0;
unsigned char patternNo = 0;





//Global
//------------------------------------------------------------------//
void IRAM_ATTR onTimer(void);
void SendByte(byte addr, byte b);
void SendCommand(byte addr, char *ci);
void Timer_Interrupt( void );
void getTimeFromNTP(void);
void getTime(void);
void bluetooth_rx(void);
void bluetooth_tx(void);
void eeprom_write(void);
void eeprom_read(void);
void TSND121( void );


//Setup
//------------------------------------------------------------------//
void setup() {

  M5.begin();
  Wire.begin();
  EEPROM.begin(128);
  SD.begin(4, SPI, 24000000, "/sd");
  M5.Lcd.clear();
  M5.Lcd.drawJpgFile(SD, "/Image/Picture.jpg");
  M5.Lcd.setTextColor(BLACK);
  M5.Lcd.setTextSize(3);
  M5.Lcd.setCursor(88, 160);
  M5.Lcd.println("HoverSat");
  M5.Lcd.setCursor(82, 200);
  M5.Lcd.println("Satellite3");

  //eeprom_read();
  ex_length = parameters[0][0];
  ex_velocity = parameters[0][1];
  ex_accel = parameters[0][2];
  ex_decel = parameters[0][3];
  
  delay(1000);

  M5.Lcd.setTextSize(3);
  M5.Lcd.fillScreen(BLACK);

  Serial.begin(115200);
  bts.begin("M5Stack Satellite3");
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    M5.Lcd.print(".");
  }

  // timeSet
  getTimeFromNTP();
  getTime();
  fname_buff  = "/log/Satellite3_log_"
              +(String)(timeinfo.tm_year + 1900)
              +"_"+(String)(timeinfo.tm_mon + 1)
              +"_"+(String)timeinfo.tm_mday
              +"_"+(String)timeinfo.tm_hour
              +"_"+(String)timeinfo.tm_min
              +".csv";
  fname = fname_buff.c_str();

  pinMode(Limit1Pin, INPUT);
  pinMode(Limit2Pin, INPUT);
  pinMode(TSND_121, OUTPUT);

  pinMode(Stepper_Enable_Pin, OUTPUT);

  // Initialize Timer Interrupt
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, TIMER_INTERRUPT * 1000, true);
  timerAlarmEnable(timer);

  //file = SD.open(fname, FILE_APPEND);
  //if( !file ) {
  //  M5.Lcd.setCursor(5, 160);
  //  M5.Lcd.println("Failed to open sd");
  //}
  file = SD.open("/Satellite1_Log.csv", FILE_APPEND);
  if( !file ) {
    M5.Lcd.setCursor(25, 160);
    M5.Lcd.println("FailedToOpenLog");
  }

  stepper.begin(RPM, MICROSTEPS); 
  digitalWrite( Stepper_Enable_Pin, 1);
}




//Main
//------------------------------------------------------------------//
void loop() {

  Timer_Interrupt();
  //ReceiveStepperData();
  bluetooth_rx();
  bluetooth_tx();

  int readBank = !writeBank;

  if (bufferIndex[readBank] >= BufferRecords) {
    static RecordType temp[BufferRecords];

    memcpy(temp, buffer[readBank], sizeof(temp));
    bufferIndex[readBank] = 0;
    file = SD.open(fname, FILE_APPEND);
    for (int i = 0; i < BufferRecords; i++) {
        file.print(temp[i].log_time);
        file.print(",");
        file.print(temp[i].log_pattern);
        file.print(",");
        file.print(temp[i].log_time_ms);
        file.print(",");
        file.print(temp[i].log_length);
        file.print(",");
        file.print(temp[i].log_velocity);
        file.print(",");
        file.print(temp[i].log_accel);
        file.println(",");
    }
    file.close();
  }

  if( telemetry_flag ) {
    bts.print(time_ms);
    bts.print(", ");
    bts.print(pattern);
    bts.print(", ");
    bts.print(current_length);
    bts.print(", ");
    bts.print(current_velocity);
    bts.print(", ");
    bts.println(current_accel);
    telemetry_flag = false;
  }

  switch (pattern) {
    case 0:
      break;

    case 11:    
      digitalWrite( Stepper_Enable_Pin, 0);
      stepper.setSpeedProfile(stepper.LINEAR_SPEED, ex_accel*MOTOR_STEPS*REDUCTION_RATIO/ONE_ROTATION_LENGTH, ex_decel*MOTOR_STEPS*REDUCTION_RATIO/ONE_ROTATION_LENGTH);
      stepper.setRPM(ex_velocity*60*REDUCTION_RATIO/ONE_ROTATION_LENGTH);
      stepper.move(ex_length*MOTOR_STEPS*REDUCTION_RATIO/ONE_ROTATION_LENGTH); 
      time_buff2 = millis();
      pattern = 12;
      break;

    case 12:
      if( millis() - time_buff2 >= 1000 ) {
        digitalWrite( Stepper_Enable_Pin, 1);
        pattern = 0;
      }
      break;

      case 21:    
      digitalWrite( Stepper_Enable_Pin, 0);
      stepper.setSpeedProfile(stepper.LINEAR_SPEED, ex_accel*MOTOR_STEPS*REDUCTION_RATIO/ONE_ROTATION_LENGTH, ex_decel*MOTOR_STEPS*REDUCTION_RATIO/ONE_ROTATION_LENGTH);
      stepper.setRPM(ex_velocity*60*REDUCTION_RATIO/ONE_ROTATION_LENGTH);
      stepper.move((ex_length+300)*MOTOR_STEPS*REDUCTION_RATIO/ONE_ROTATION_LENGTH*-1); 
      time_buff2 = millis();
      pattern = 22;
      break;

    case 22:
      if( millis() - time_buff2 >= 1000 ) {
        digitalWrite( Stepper_Enable_Pin, 1);
        pattern = 0;
      }
      break;

    // CountDown
    case 111:    
      if( current_time >= 52  ) {
        time_buff2 = millis();
        pattern = 113;      
        hover_flag = true;
        M5.Lcd.clear();
        DuctedFan.attach(DuctedFanPin);
        DuctedFan.write(0);
        break;
      }
      M5.Lcd.setCursor(180, 100);
      M5.Lcd.clear();
      M5.Lcd.println(60 - current_time);
      bts.println( 60 - current_time );
      break;

    case 112:     
      if( current_time < 1 ) {
        pattern = 111;
        break;
      }
      bts.println( 60 - current_time + 60 );
      M5.Lcd.setCursor(180, 100);
      M5.Lcd.clear();
      M5.Lcd.println(60 - current_time);
      break;

    case 113:    
      if( millis() - time_buff2 >= 3000 ) {
        DuctedFan.write(hover_val);
        bts.println(" - Start within 5 seconds -");
        time_buff2 = millis();
        log_flag = false;
        pattern = 122;
        break;
      }    
      M5.Lcd.setCursor(180, 100);
      M5.Lcd.clear();
      M5.Lcd.println(60 - current_time);
      bts.println( 60 - current_time );
      break;

    case 122:   
      if( millis() - time_buff2 >= 3000 ) {
        time_buff2 = millis();
        pattern = 114;
        TSND121();
        bts.println( "\n - Log start -" );
        break;
      }        
      break;

    case 114:   
      if( millis() - time_buff2 >= 5000 ) {
        time_buff = millis();
        pattern = 115;
        bts.println( "\n - Sequence start -" );
        break;
      }        
      break;

    case 115:   
      time_stepper = time_ms;
      digitalWrite( Stepper_Enable_Pin, 0);
      if( dir_flag == 1 ){
        stepper.setSpeedProfile(stepper.LINEAR_SPEED, ex_accel*MOTOR_STEPS*REDUCTION_RATIO/ONE_ROTATION_LENGTH, ex_decel*MOTOR_STEPS*REDUCTION_RATIO/ONE_ROTATION_LENGTH);
        stepper.setRPM(ex_velocity*60*REDUCTION_RATIO/ONE_ROTATION_LENGTH);
        stepper.move(ex_length*MOTOR_STEPS*REDUCTION_RATIO/ONE_ROTATION_LENGTH);
      } else {
        stepper.setSpeedProfile(stepper.LINEAR_SPEED, ex_accel*MOTOR_STEPS*REDUCTION_RATIO/ONE_ROTATION_LENGTH, ex_decel*MOTOR_STEPS*REDUCTION_RATIO/ONE_ROTATION_LENGTH);
        stepper.setRPM(ex_velocity*60*REDUCTION_RATIO/ONE_ROTATION_LENGTH);
        stepper.move(ex_length*MOTOR_STEPS*REDUCTION_RATIO/ONE_ROTATION_LENGTH*-1);
      }
      time_buff2 = millis();
      pattern = 116;
      break;

    case 116:   
      if( millis() - time_buff2 >= wait*1000 ) {
        pattern = 118;
        break;
      }
      break;

    case 117:
      digitalWrite( Stepper_Enable_Pin, 0);
      stepper.setSpeedProfile(stepper.LINEAR_SPEED, ex_accel*MOTOR_STEPS*REDUCTION_RATIO/ONE_ROTATION_LENGTH, ex_decel*MOTOR_STEPS*REDUCTION_RATIO/ONE_ROTATION_LENGTH);
      stepper.setRPM(ex_velocity*60*REDUCTION_RATIO/ONE_ROTATION_LENGTH);
      stepper.move((ex_length+300)*MOTOR_STEPS*REDUCTION_RATIO/ONE_ROTATION_LENGTH*-1);
      digitalWrite( Stepper_Enable_Pin, 1);
      pattern = 118;
      time_buff2 = millis();
      break;

    case 118:
      if( millis() - time_buff2 >= 5000 ) {
        log_flag = false;
        pattern = 0;
        tx_pattern = 0;
        TSND121();
        hover_flag = false;
        M5.Lcd.clear();
        DuctedFan.detach();
        break;
      }
      break;
       
  }

      
  // Button Control
  M5.update();
  if (M5.BtnA.wasPressed()) {
    hover_flag = !hover_flag;
    // Hover Control
    if(hover_flag) {
      M5.Lcd.clear();
      DuctedFan.attach(DuctedFanPin);
      DuctedFan.write(0);
      delay(3000);
      DuctedFan.write(hover_val);
    } else {
      M5.Lcd.clear();
      DuctedFan.detach();
    } 
  } else if (M5.BtnB.wasPressed() && pattern == 0) {  
    patternNo++;
    M5.Lcd.fillScreen(BLACK);
    if( patternNo >= NOOFPATTERNS ) {
      patternNo = 0;
    }
    ex_length = parameters[patternNo][0];
    ex_velocity = parameters[patternNo][1];
    ex_accel = parameters[patternNo][2];
    ex_decel = parameters[patternNo][3];
    
  } else if (M5.BtnC.wasPressed() && pattern == 0) { 
    M5.Lcd.clear();
    M5.Lcd.setCursor(82, 100);
    if( current_time >= 52 ) {   
      pattern = 112;
    } else {
      pattern = 111;
    }
  }

 
}



// Timer Interrupt
//------------------------------------------------------------------//
void Timer_Interrupt( void ){
  if (interruptCounter > 0) {

    portENTER_CRITICAL(&timerMux);
    interruptCounter--;
    portEXIT_CRITICAL(&timerMux);

    cnt10++;
    time_ms = millis()-time_buff;
    
    getTime();

    if (bufferIndex[writeBank] < BufferRecords && log_flag) {
      RecordType* rp = &buffer[writeBank][bufferIndex[writeBank]];
      rp->log_time = timeStr;
      rp->log_pattern = pattern;
      rp->log_time_ms = time_ms;
      rp->log_length = current_length;
      rp->log_velocity = current_velocity;
      rp->log_accel = current_accel;
      if (++bufferIndex[writeBank] >= BufferRecords) {
          writeBank = !writeBank;
      }      
    }

    Limit1State = digitalRead(Limit1Pin);
    Limit2State = digitalRead(Limit2Pin);

    iTimer10++;
    switch( iTimer10 ) {
    case 1:
      if(hover_flag) {
        //M5.Lcd.fillScreen(BLACK);
        M5.Lcd.fillRect(0, 0, 80, 80, TFT_WHITE);
        M5.Lcd.fillRect(80, 0, 240, 80, TFT_DARKGREY);
        M5.Lcd.fillRect(0, 80, 80, 160, TFT_DARKGREY);
        M5.Lcd.setTextSize(5);
        M5.Lcd.setCursor(13, 23);
        M5.Lcd.setTextColor(BLACK);
        M5.Lcd.print("S3");
        M5.Lcd.setTextSize(3);
        M5.Lcd.setCursor(96, 30);
        M5.Lcd.setTextColor(WHITE);
        M5.Lcd.printf("DctF     %3d", hover_val);
        M5.Lcd.setCursor(15, 120);
        M5.Lcd.print("No.");
        M5.Lcd.setTextSize(5);
        M5.Lcd.setCursor(28, 160);
        M5.Lcd.print(patternNo+1);

        M5.Lcd.setTextSize(2);
        M5.Lcd.setCursor(96, 92);
        M5.Lcd.printf("Move Length   %4d", parameters[patternNo][0]);
        M5.Lcd.setCursor(96, 132);
        M5.Lcd.printf("Acceleration  %4d", parameters[patternNo][1]);
        M5.Lcd.setCursor(96, 172);
        M5.Lcd.printf("Velocity      %4d", parameters[patternNo][2]);
        M5.Lcd.setCursor(96, 212);
        M5.Lcd.printf("Deceleration  %4d", parameters[patternNo][3]);
      } else {
        M5.Lcd.fillRect(0, 0, 80, 80, TFT_WHITE);
        M5.Lcd.fillRect(80, 0, 240, 80, TFT_DARKGREY);
        M5.Lcd.fillRect(0, 80, 80, 160, TFT_DARKGREY);
        M5.Lcd.setTextSize(5);
        M5.Lcd.setCursor(13, 23);
        M5.Lcd.setTextColor(BLACK);
        M5.Lcd.print("S3");
        M5.Lcd.setTextSize(3);
        M5.Lcd.setCursor(96, 30);
        M5.Lcd.setTextColor(WHITE);
        M5.Lcd.print("DctF Disable");
        M5.Lcd.setCursor(15, 120);
        M5.Lcd.print("No.");
        M5.Lcd.setTextSize(5);
        M5.Lcd.setCursor(28, 160);
        M5.Lcd.print(patternNo+1);

        M5.Lcd.setTextSize(2);
        M5.Lcd.setCursor(96, 92);
        M5.Lcd.printf("Move Length   %4d", parameters[patternNo][0]);
        M5.Lcd.setCursor(96, 132);
        M5.Lcd.printf("Acceleration  %4d", parameters[patternNo][1]);
        M5.Lcd.setCursor(96, 172);
        M5.Lcd.printf("Velocity      %4d", parameters[patternNo][2]);
        M5.Lcd.setCursor(96, 212);
        M5.Lcd.printf("Deceleration  %4d", parameters[patternNo][3]);

      }
    break;

    case 2:
      if( tx_pattern == 11 ) {
        telemetry_flag = true;
      }
      break;
    
    case 10:
      iTimer10 = 0;
      break;

    }

  }
}


// EEPROM Write
//------------------------------------------------------------------// 
void eeprom_write(void) {
  EEPROM.write(0, hover_val);
  EEPROM.write(1, (ex_length & 0xFF));
  EEPROM.write(2, (ex_length>>8 & 0xFF));
  EEPROM.write(3, (ex_length>>16 & 0xFF));
  EEPROM.write(4, (ex_length>>24 & 0xFF));
  EEPROM.write(5, (ex_velocity & 0xFF));
  EEPROM.write(6, (ex_velocity>>8 & 0xFF));
  EEPROM.write(7, (ex_velocity>>16 & 0xFF));
  EEPROM.write(8, (ex_velocity>>24 & 0xFF));
  EEPROM.write(9, (ex_accel & 0xFF));
  EEPROM.write(10, (ex_accel>>8 & 0xFF));
  EEPROM.write(11, (ex_accel>>16 & 0xFF));
  EEPROM.write(12, (ex_accel>>24 & 0xFF));
  EEPROM.write(13, wait);
  EEPROM.write(14, dir_flag);
  EEPROM.commit();
}

// EEPROM Read
//------------------------------------------------------------------// 
void eeprom_read(void) {
    hover_val = EEPROM.read(0);
    ex_length = EEPROM.read(1) + (EEPROM.read(2)<<8) + (EEPROM.read(3)<<16) + (EEPROM.read(4)<<24);
    ex_velocity = EEPROM.read(5) + (EEPROM.read(6)<<8) + (EEPROM.read(7)<<16) + (EEPROM.read(8)<<24);
    ex_accel = EEPROM.read(9) + (EEPROM.read(10)<<8) + (EEPROM.read(11)<<16) + (EEPROM.read(12)<<24);
    wait = EEPROM.read(13);
    dir_flag = EEPROM.read(14);
}


// Bluetooth RX
//------------------------------------------------------------------//
void bluetooth_rx(void) {

  while (bts.available() > 0) {
    bts_rx_buffer[bts_index] = bts.read();
    bts.write(bts_rx_buffer[bts_index]);
    
    if( bts_rx_buffer[bts_index] == '/' ) {
      bts.print("\n\n"); 
      if( tx_pattern == 1 ) {
        rx_pattern = atoi(bts_rx_buffer);
      } else {
        rx_val = atof(bts_rx_buffer);
      }
      bts_index = 0;
      
      switch ( rx_pattern ) {
          
      case 0:
        tx_pattern = 0;
        break;
        
      case 11:
        rx_pattern = 0;
        tx_pattern = 11;
        break;

      case 20:
        rx_pattern = 0;
        tx_pattern = 20;
        file = SD.open("/Satellite1_Log.csv", FILE_APPEND);
        file.print("0");
        file.print(",");
        file.print(ex_length);
        file.print(",");
        file.print(ex_velocity);
        file.print(",");
        file.print(ex_accel);
        file.println(",");
        file.close();

        if( current_time >= 52 ) {   
          pattern = 112;
          break;
        } else {
          pattern = 111;
          break;
        }
        break;
        
      case 21:
        rx_pattern = 0;
        tx_pattern = 21;
        hover_flag = !hover_flag;
        if(hover_flag) {
          M5.Lcd.clear();
          DuctedFan.attach(DuctedFanPin);
          DuctedFan.write(0);
          delay(3000);
          DuctedFan.write(hover_val);
        } else {
          M5.Lcd.clear();
          DuctedFan.detach();
        }
        break;

      case 22:
        rx_pattern = 0;
        tx_pattern = 22;    
        inc_flag = true;
        pattern = 11;
        break;

      case 23:
        rx_pattern = 0;
        tx_pattern = 23;    
        inc_flag = true;
        pattern = 21;
        break;

      case 24:
        rx_pattern = 0;
        tx_pattern = 24;    
        pattern = 0;
        break;

      case 31:
        tx_pattern = 31;
        rx_pattern = 41;
        break;

      case 41:
        hover_val = rx_val;
        eeprom_write();
        tx_pattern = 0;
        rx_pattern = 0;
        break;

      case 32:
        tx_pattern = 32;
        rx_pattern = 42;
        break;

      case 42:
        ex_length = rx_val;
        eeprom_write();
        tx_pattern = 0;
        rx_pattern = 0;
        break;

      case 33:
        tx_pattern = 33;
        rx_pattern = 43;
        break;

      case 43:
        ex_velocity = rx_val;
        eeprom_write();
        tx_pattern = 0;
        rx_pattern = 0;
        break;

      case 34:
        tx_pattern = 34;
        rx_pattern = 44;
        break;

      case 44:
        ex_accel = rx_val;
        eeprom_write();
        tx_pattern = 0;
        rx_pattern = 0;
        break;

      case 35:
        tx_pattern = 35;
        rx_pattern = 45;
        break;

      case 45:
        wait = rx_val;
        eeprom_write();
        tx_pattern = 0;
        rx_pattern = 0;
        break;

      case 36:
        tx_pattern = 36;
        rx_pattern = 46;
        break;

      case 46:
        dir_flag = rx_val;
        eeprom_write();
        tx_pattern = 0;
        rx_pattern = 0;
        break;
          

      }
      
    } else {
        bts_index++;
    }
  }


}


// Bluetooth TX
//------------------------------------------------------------------//
void bluetooth_tx(void) {

    switch ( tx_pattern ) {
            
    case 0:
      delay(30);
      bts.print("\n\n\n\n\n\n");
      bts.print(" HoverSat Satellite3 (M5Stack version) "
                         "Test Program Ver1.20\n");
      bts.print("\n");
      bts.print(" Satellite control\n");
      bts.print(" 11 : Telemetry\n");
      bts.print(" 12 : Read log\n");
      bts.print("\n");
      bts.print(" 20 : Sequence Control\n");
      bts.print(" 21 : Start/Stop Hovering\n");
      bts.print(" 22 : Start Extruding\n");
      bts.print(" 23 : Start Winding\n");
      bts.print(" 24 : Pause\n");
      bts.print("\n");
      bts.print(" Set parameters  [Current val]\n");
      bts.print(" 31 : DuctedFan Output [");
      bts.print(hover_val);
      bts.print("%]\n");
      bts.print(" 32 : Moving Distance [");
      bts.print(ex_length);
      bts.print("mm]\n");
      bts.print(" 33 : Moving Speed [");
      bts.print(ex_velocity);
      bts.print("mm/s]\n");
      bts.print(" 34 : Movement Acceleration [");
      bts.print(ex_accel);
      bts.print("mm/s^2]\n");
      bts.print(" 35 : Sequence Wait [");
      bts.print(wait);
      bts.print("s]\n");
      bts.print(" 36 : Direction of movement [");
      bts.print(dir_flag);
      bts.print("]\n");
      
      bts.print("\n");
      bts.print(" Please enter 11 to 35  ");
      
      tx_pattern = 1;
      break;
        
    case 1: 
      break;
        
    case 2:
      break;
        
    case 11:
      //Telemetry @ Interrupt
      break;

    case 20:
      bts.print(" Starting Sequence...\n");
      tx_pattern = 1;
      break;

    case 21:
      if(hover_flag) {
        bts.print(" Start Hovering...\n");
      } else {
        bts.print(" Stop Hovering...\n");
      }
      delay(1000);
      tx_pattern = 0;
      break;

    case 22:
      bts.print(" Start Extruding...\n");
      tx_pattern = 0;
      break;

    case 23:
      bts.print(" Start Winding...\n");
      tx_pattern = 0;
      break;

    case 24:
      bts.print(" Pause...\n");
      tx_pattern = 0;
      break;

              
    case 31:
      bts.print(" DuctedFan Output [%] -");
      bts.print(" Please enter 0 to 100 ");
      tx_pattern = 2;
      break;

    case 32:
      bts.print(" Extension Length [mm] -");
      bts.print(" Please enter 0 to 10,000 ");
      tx_pattern = 2;
      break;

    case 33:
      bts.print(" Extension velocity [mm/s] -");
      bts.print(" Please enter 0 to 500 ");
      tx_pattern = 2;
      break;

    case 34:
      bts.print(" Extension Accel [mm/s^2] -");
      bts.print(" Please enter 1 to 50 ");
      tx_pattern = 2;
      break;

    case 35:
      bts.print(" Sequence Wait [s] -");
      bts.print(" Please enter 0 to 255 ");
      tx_pattern = 2;
      break;

    case 36:
      bts.print(" LimitSwitch Enable -");
      bts.print(" Please enter 0 or 1 ");
      tx_pattern = 2;
      break;
                 
    }
}


// TSND 121
//------------------------------------------------------------------//
void TSND121( void ) {
    TSND_121_ENABLE = 1;
    digitalWrite( TSND_121, HIGH );
    delay(2000);
    digitalWrite( TSND_121, LOW );

}

// IRAM
//------------------------------------------------------------------//
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter=1;
  portEXIT_CRITICAL_ISR(&timerMux);
}



//Get Time From NTP
//------------------------------------------------------------------//
void getTimeFromNTP(void){
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  while (!getLocalTime(&timeinfo)) {
    delay(1000);
  }
}

//Get Convert Time
//------------------------------------------------------------------//
void getTime(void){
  getLocalTime(&timeinfo);
  dateStr = (String)(timeinfo.tm_year + 1900)
          + "/" + (String)(timeinfo.tm_mon + 1)
          + "/" + (String)timeinfo.tm_mday;
  timeStr = (String)timeinfo.tm_hour
          + ":" + (String)timeinfo.tm_min
          + ":" + (String)timeinfo.tm_sec;
  current_time = timeinfo.tm_sec;
}


//Write SD Initial Data
//------------------------------------------------------------------//
void writeDataInitial(void) {
  file = SD.open(fname, FILE_APPEND);
  file.println("Tether extension experiment");
  file.println("Parameters");
  file.println("Time, Pattern, Pattern");
  file.close();
}

