
#include <SdFat.h>
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include <avr/sleep.h>
#include <Wire.h>
#include <SparkFun_MS5803_I2C.h>

#define crx 3
#define ctx 2
#define grx 8
#define gtx 7
#define tempProbePin 6
#define chipSelect 10
#define ledPin 13
#define GPSECHO  false
enum state {
  GPS_READ,
  GPS_WRITE,
  CONDUCTIVITY_READ,
  CONDUCTIVITY_WRITE,
  TEMP_PRESS_READ_WRITE
};
String sensorstring = "";
byte current_state;
File logfile;
SdFat SD;
//Init GPS Shield
//HardwareSerial gpsSerial = Serial;
SoftwareSerial gpsSerial(grx,gtx);                    
Adafruit_GPS GPS(&gpsSerial);
//Init Conducivity Sensor
//SoftwareSerial conductSerial(crx, ctx);                      //define how the soft serial port is going to work
 //sss conductSerial();
//HardwareSerial conductSerial = Serial;
//Init Temp Sensors
OneWire oneWire(tempProbePin);
DallasTemperature tempSensor(&oneWire);
//Init Pressure Sensor
MS5803 psensor(ADDRESS_HIGH);
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
void setup() {
  // put your setup code here, to run once:
  //conductSerial.begin(9600);
 // Serial.begin(115200);
  //Serial.println("S");
  sssBegin();
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  useInterrupt(true);

  pinMode(10, OUTPUT);
  if (!SD.begin(chipSelect)) {      // if you're using an UNO, you can use this line instead
  //  Serial.println("Card init. failed!");
  //  error(2);
  //Serial.print("C");
  }
  char filename[15];
  strcpy(filename, "CTDLOG00.TXT");
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = '0' + i/10;
    filename[7] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }
  logfile = SD.open(filename, FILE_WRITE);
  if ( ! logfile ) {
    //Serial.print("O");
    //Serial.println(filename);
    //error(3);
  }
 // Serial.println("Starting sensors");
  psensor.reset();
  psensor.begin();
  tempSensor.begin();
  delay(250);
  current_state = GPS_READ;
}

void loop() {
  switch(current_state){
 //Serial.println("L");
  // put your main code here, to run repeatedly:
  //
  //Log Format MM/dd/YYYY,HH:MM:SS.mmmmm,lat,latdeg,long,longdeg,EC,TDS,SAL,GRAV,tempa,tempb,tempc,temppressure,pressureabs
  // read data from the GPS in the 'main loop'
  case GPS_READ:
    if (GPS.newNMEAreceived()) {
      GPS.parse(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
    }
    current_state = GPS_WRITE;
     // Serial.println("R");

    break;
  case GPS_WRITE:
      logfile.print(GPS.month, DEC);
      logfile.print("/");
      logfile.print(GPS.day, DEC);
      logfile.print("/20");
      logfile.print(GPS.year, DEC);
      logfile.print(",");
      logfile.print(GPS.hour, DEC); // Time is GMT
      logfile.print(":");
      logfile.print(GPS.minute, DEC);
      logfile.print(":");
      logfile.print(GPS.seconds, DEC);
      logfile.print(".");
      logfile.print(GPS.milliseconds);
      if(GPS.fix){
      logfile.print(",");
      logfile.print(GPS.latitude, 4);
      logfile.print(",");
      logfile.print(GPS.latitudeDegrees, 4);
      logfile.print(",");
      logfile.print(GPS.longitude, 4);
      logfile.print(",");
      logfile.print(GPS.longitudeDegrees, 4);
      }
      current_state = CONDUCTIVITY_READ;
      //  Serial.println("W");

      break;
    case CONDUCTIVITY_READ:
    if (sssAvailable() > 0){
       // Serial.println("CR");

      char inchar = (char)sssRead();              //get the char we just received
      sensorstring += inchar;                           //add the char to the var called sensorstring
      if (inchar == '\r') {                             //if the incoming character is a <CR>
        current_state = CONDUCTIVITY_WRITE;                  //set the flag
       //   Serial.println("CW");

      }
    }
    break;
    case CONDUCTIVITY_WRITE:
    if (isdigit(sensorstring[0])) {
        logfile.print(",");
        sensorstring.trim();
        logfile.print(sensorstring); 
  //        char sensorstring_array[30]; 
//        char *EC;                                           //char pointer used in string parsing
//        char *TDS;                                          //char pointer used in string parsing
//        char *SAL;                                          //char pointer used in string parsing
//        char *GRAV; 
//        sensorstring.toCharArray(sensorstring_array, 30);   //convert the string to a char array 
//        logfile.print(strtok(sensorstring_array, ","));
//        logfile.print(",");
//        logfile.print(strtok(NULL, ","));
//        logfile.print(",");
//        logfile.print(strtok(NULL, ","));
//        logfile.print(",");
//        logfile.print(strtok(NULL, ","));
//        logfile.print(",");
       //   Serial.println("CWW");

    }
    sensorstring="";
    current_state = TEMP_PRESS_READ_WRITE;
   // Serial.println("TMP");

    break;
    case TEMP_PRESS_READ_WRITE:
      tempSensor.requestTemperatures();
      logfile.print(",");
      logfile.print(tempSensor.getTempCByIndex(0));
      logfile.print(",");
      logfile.print(tempSensor.getTempCByIndex(1));
      logfile.print(",");
      logfile.print(tempSensor.getTempCByIndex(2));
      logfile.print(",");
      logfile.print(psensor.getTemperature(CELSIUS, ADC_512));
      logfile.print(",");
      logfile.println(psensor.getPressure(ADC_4096));
      logfile.flush();
      current_state = GPS_READ;
      delay(50);
      //Serial.println("D");
      break;
  }
}



//void print_EC_data(void) {                            //this function will pars the string
////  char *EC;                                           //char pointer used in string parsing
////  char *TDS;                                          //char pointer used in string parsing
////  char *SAL;                                          //char pointer used in string parsing
////  char *GRAV;                                         //char pointer used in string parsing
////  EC = strtok(sensorstring_array, ",");               //let's pars the array at each comma
////  TDS = strtok(NULL, ",");                            //let's pars the array at each comma
////  SAL = strtok(NULL, ",");                            //let's pars the array at each comma
////  GRAV = strtok(NULL, ",");                           //let's pars the array at each comma
//  logfile.print(strtok(sensorstring_array, ","));
//  logfile.print(",");
//  logfile.print(strtok(NULL, ","));
//  logfile.print(",");
//  logfile.print(strtok(NULL, ","));
//  logfile.print(",");
//  logfile.print(strtok(NULL, ","));
//  logfile.print(",");
//
//}
//void error(uint8_t errno) {
//  while (1) {
//    uint8_t i;
//    for (i = 0; i < errno; i++) {
//      digitalWrite(ledPin, HIGH);
//      delay(100);
//      digitalWrite(ledPin, LOW);
//      delay(100);
//    }
//    for (i = errno; i < 10; i++) {
//      delay(200);
//    }
//  }
//}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

