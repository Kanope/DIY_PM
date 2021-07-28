#include "SdsDustSensor.h"  //PM SENSOR LIBRARY
#include "ArduinoLowPower.h"  //ARDUINO LOW POWER LIBRARY
#include "arduino_secrets.h"  //LORAWAN ID
//#include <Wire.h>
#include "Adafruit_HTU21DF.h" //TEMPERATURE/HUMIDITY SENSOR LIBRARY
#include <RTCZero.h>          //RTC LIBRARY
#include <SensirionI2CScd4x.h>
#include <MKRWAN.h>            //LORAWAN LIBRARY
#include "sgp40_voc_index.h"


//Please note that, if the processor is sleeping, a new sketch can't be uploaded. To overcome this, manually reset the board (usually with a single or double tap to the RESET button)

//Please enter your sensitive data in the Secret tab or arduino_secrets.h
String appEui = SECRET_APP_EUI;
String appKey = SECRET_APP_KEY;

#define PULSEFREQUENCY 60 //PULSE FREQUENCY IN s
#define PULSEDURATION 200 //PULSE DURATION IN ms
#define MEASUREFREQUENCY  1800//MEASUREMENT FREQUENCY IN s ==> 30min * 60s = 1800 secondes
#define MEASUREDURATION  120//MEASUREMENT DURATION IN s ==> 120 secondes

#define ACTIVEPM 9  //GPIO 9 ACTIVE PM SENSOR
#define ACTIVETH 10 //GPIO 10 ACTIVE TEMPERATURE/HUMIDITY SENSOR
#define RESET 8     //GPIO 8 RESET PIN
#define PULSE 7     //GPIO 7 PEEK PULSE PIN

//Global variables
SdsDustSensor sds(Serial1);                     //Sensor PM declaration
bool toggle = 0;                                //toggle variable for led blink
Adafruit_HTU21DF htu = Adafruit_HTU21DF();      //Humidity and temperature sensor declaration
int count = 0;                                  //count variable declaration to count the time during measurement 
int countC02 = 0;                   
String msg = "TEST";                            //string message declaration
String stringOne = msg;                         //string message declaration
float humidity = 0;                             //Humidity variable declaration
float temperature = 0;                          //Temperature variable declaration
int co2 = 0;
uint16_t co2_scd4x = 0;
float pm25 = 0;                                 //pm25 variable declaration
float pm10 = 0;                                 //pm10 variable declaration
float moyTemp = 0;                              //moyTemp variable declaration
float moyHum = 0;                               //moyHum variable declaration
float moyPM25 = 0;                              //moyPM25 variable declaration
float moyPM10 = 0;                              //moyPM10 variable declaration
float moyCo2 = 0;                              //moyPM10 variable declaration
//Function definitions
void dopulse(void); //Pulse methode definition
void sendLoraMessage(void);//Send message using Lora definition
void ledBlink(void);


SensirionI2CScd4x scd4x;

void printUint16Hex(uint16_t value) {
    Serial.print(value < 4096 ? "0" : "");
    Serial.print(value < 256 ? "0" : "");
    Serial.print(value < 16 ? "0" : "");
    Serial.print(value, HEX);
}

void printSerialNumber(uint16_t serial0, uint16_t serial1, uint16_t serial2) {
    Serial.print("Serial: 0x");
    printUint16Hex(serial0);
    printUint16Hex(serial1);
    printUint16Hex(serial2);
    Serial.println();
}


//Function declarations
void ledBlink(void)
{
   //Led Blink process
  if(toggle == 1)
  {
    toggle = 0;
    digitalWrite(LED_BUILTIN, 1);
  }
  else
  {
    toggle = 1;
    digitalWrite(LED_BUILTIN, 0);
  }
}

//Send message using Lora declaration
void sendLoraMessage(void)
{
    LoRaModem modem;
    count = 0;
    delay(500);
    char buff[5] = {0};
    buff[0] = 1;//moyTemp * 100;
    buff[1] = 2;//moyHum * 100;
    buff[2] = 3;//moyPM25 * 100;
    buff[3] = 4;//moyPM10 * 100;
    buff[4] = 5;//moyCo2;
    //stringOne =  String(moyTemp, 2) + " " + String(moyHum, 2) + " " + String(moyPM25, 2) + " " + String(moyPM10, 2);
    int err;
    if (!modem.begin(EU868)) 
    {
      Serial.println("Failed to start module");
    }
    
    Serial.print("Your module version is: ");
    Serial.println(modem.version());
    Serial.print("Your device EUI is: ");
    Serial.println(modem.deviceEUI());
    
    int connected = modem.joinOTAA(appEui, appKey);
    if (!connected) 
    {
      Serial.println("Something went wrong; are you indoor? Move near a window and retry");
    }
    
    modem.minPollInterval(60);
    modem.beginPacket();
    //modem.print(stringOne);
    modem.write(int(moyTemp * 100) | ((int(moyHum * 100))<<16));
    modem.write(int(moyPM25 * 100) | ((int(moyPM10 * 100))<<16));
    modem.write(int(moyCo2));
    err = modem.endPacket(true);
    if (err > 0) 
    {
      Serial.println("Message sent correctly!");
    } 
    else 
    {
      Serial.println("Error sending message :(");
      Serial.println("(you may send a limited amount of messages per minute, depending on the signal strength");
      Serial.println("it may vary from 1 message every couple of seconds to 1 message every minute)");
    }

    delay(5000);
    digitalWrite(LORA_RESET, LOW);
    digitalWrite(ACTIVEPM, LOW);
    digitalWrite(ACTIVETH, LOW);
    digitalWrite(LED_BUILTIN, 0);
}

void dopulse(void) //Pulse methode declaration
{
    int index = 0;
    int indexMax = 0;
    indexMax = MEASUREFREQUENCY/((PULSEDURATION*0.001)+PULSEFREQUENCY);  // example: 1800 / (1 + 10) = 59 index
    for(index = 0; index <indexMax; index ++)
    {
      digitalWrite(ACTIVEPM, HIGH); 
      digitalWrite(ACTIVETH, HIGH);
      digitalWrite(PULSE, HIGH);
      delay(PULSEDURATION);    //Pulse duration
      digitalWrite(PULSE, LOW);
      digitalWrite(ACTIVEPM, LOW);
      digitalWrite(ACTIVETH, LOW);
      LowPower.sleep((PULSEFREQUENCY*1000) - PULSEDURATION); //Pulse frequency
      Serial.println(index);
      Serial.print(", index = ");
    }
    Serial.print("RESET");  //PRINT ON TERMINAL
    digitalWrite(RESET, LOW); //ACTIVE RESET PIN  //Reset when the timer reach 30 min
}

void setup() 
{
  Wire.begin();
  int16_t err;
  uint16_t error;
  char errorMessage[256];
  pinMode(RESET, OUTPUT);                 //RESET pin OUTPUT configuration
  digitalWrite(RESET, HIGH);              //RESET pin to HIGH (disable)
  pinMode(PULSE, OUTPUT);                 //PULSE pin OUTPUT configuration
  digitalWrite(PULSE, LOW);               //PULSE pin to LOW (disable)
  delay(200);                             //200ms Delay



    
  //pinMode(RESET, OUTPUT); 
  Serial.begin(115200);                   //Serial port declaration
  pinMode(ACTIVEPM, OUTPUT);              //ACTIVEPM pin OUTPUT configuration
  pinMode(ACTIVETH, OUTPUT);              //ACTIVETH pin OUTPUT configuration
  digitalWrite(ACTIVEPM, HIGH);           //ACTIVEPM pin to HIGH (disable)
  digitalWrite(ACTIVETH, HIGH);           //ACTIVETH pin to HIGH (disable)
  pinMode(LED_BUILTIN, OUTPUT);           //LED_BUILTIN pin OUTPUT configuration
  sds.begin();                            //PM sensor declaration
  if (!htu.begin())                       //Humidity and temperature sensor declaration
  {
    Serial.println("Couldn't find sensor!");
  }
  delay(1000);                            //1000ms Delay
  PmResult pm = sds.readPm();             //Read PM sensor
  Serial.println(sds.queryFirmwareVersion().toString()); // prints firmware version
  Serial.println(sds.setActiveReportingMode().toString()); // ensures sensor is in 'active' reporting mode
  Serial.println(sds.setContinuousWorkingPeriod().toString()); // ensures sensor has continuous working period - default but not recommended
  
    /* Initialize I2C bus, SHT, SGP and VOC Engine */
  while ((err = sensirion_init_sensors())) {
    Serial.print("initialization failed: ");
    Serial.println(err);
    sensirion_sleep_usec(1000000); /* wait one second */
  }
  Serial.println("initialization successful");
  
  
  scd4x.begin(Wire);

      // stop potentially previously started measurement
  error = scd4x.stopPeriodicMeasurement();
  if (error) 
  {
    Serial.print("Error trying to execute stopPeriodicMeasurement(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }

  uint16_t serial0;
  uint16_t serial1;
  uint16_t serial2;
  error = scd4x.getSerialNumber(serial0, serial1, serial2);
  if (error) 
  {
      Serial.print("Error trying to execute getSerialNumber(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
  } 
  else 
  {
      printSerialNumber(serial0, serial1, serial2);
  }
  
  // Start Measurement
  error = scd4x.startPeriodicMeasurement();
  if (error) 
  {
      Serial.print("Error trying to execute startPeriodicMeasurement(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
  }
  
  Serial.println("Waiting for first measurement... (5 sec)");
  delay(5000);
}

//Main process looping each secondes
void loop()
{
    int16_t err;
  int32_t voc_index;
  int32_t temperature_celsius;
  int32_t relative_humidity_percent;
  err = sensirion_measure_voc_index_with_rh_t(
          &voc_index, &relative_humidity_percent, &temperature_celsius );
  if (err == STATUS_OK) {
    Serial.print("VOCindex:");
    Serial.print(voc_index);
    Serial.print("\t");
    Serial.print("Humidity[%RH]:");
    Serial.print(relative_humidity_percent * 0.001f);
    Serial.print("\t");
    Serial.print("Temperature[degC]:");
    Serial.println(temperature_celsius * 0.001f);
  } else {
    Serial.print("error reading signal: ");
    Serial.println(err);
  }
  
  
  uint16_t error;
  char errorMessage[256];
  float temperature_scd4x;
  float humidity_scd4x;
  error = scd4x.readMeasurement(co2_scd4x, temperature_scd4x, humidity_scd4x);
  if (error) 
  {
    Serial.print("Error trying to execute readMeasurement(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  } 
  else if (co2_scd4x == 0) 
  {
    Serial.println("Invalid sample detected, skipping.");
  } 
  else 
  {

  }
  Serial.print("Co2:");
  Serial.print(co2_scd4x);
  Serial.print("\t");
  PmResult pm = sds.readPm();  //Read PM sensor
  if (pm.isOk())               //Check if PM sensor is ok
  {
    Serial.print("PM2.5 = ");
    Serial.print(pm.pm25);
    Serial.print(", PM10 = ");
    Serial.println(pm.pm10);
    //if you want to just print the measured values, you can use toString() method as well
    Serial.println(pm.toString());
  } 
  else 
  {
    Serial.print("Could not read values from sensor, reason: ");
    Serial.println(pm.statusToString());
  }
  
  ledBlink();
  
  //Read humidity and temperature values
  float temp = htu.readTemperature();
  float rel_hum = htu.readHumidity();
  Serial.print("Temp: "); Serial.print(temp); Serial.print(" C");
  Serial.print("\t\t");
  Serial.print("Humidity: "); Serial.print(rel_hum); Serial.println(" \%");

  //Save all measurements
  humidity = humidity + rel_hum;
  temperature = temperature + temp;
  pm25 = pm25 + pm.pm25;
  pm10 = pm10 + pm.pm10;
  co2 = co2 + co2_scd4x;
  Serial.println(co2);
  count = count + 1;
  if(co2_scd4x > 0)  countC02 = countC02 + 1;
  if(count >= MEASUREDURATION) //When the process reach the end time MEASUREDURATION
  {
    //Average process + print
    if(count > 0)
    {
      moyTemp = temperature/count;
      moyHum = humidity/count;
      moyPM25 = pm25/count;
      moyPM10 = pm10/count;
      moyCo2 = co2/countC02;
    }
    Serial.print(", moyPM10 = ");
    Serial.println(moyPM10);
    Serial.print(", moyPM25 = ");
    Serial.println(moyPM25);
    Serial.print(", moyTemp = ");
    Serial.println(moyTemp);
    Serial.print(", moyHum = ");
    Serial.println(moyHum);
    Serial.print(", moyCo2 = ");
    Serial.println(moyCo2);
    count = 0;
    //Send the data to server using lorawan
    sendLoraMessage();
    delay(200);
    //Pulse process during sleep
    dopulse();
  }
  delay(1000);
}
