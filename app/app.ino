#include "SdsDustSensor.h"  //PM SENSOR LIBRARY
#include "ArduinoLowPower.h"  //ARDUINO LOW POWER LIBRARY
#include "arduino_secrets.h"  //LORAWAN ID
//#include <Wire.h>
#include "Adafruit_HTU21DF.h" //TEMPERATURE/HUMIDITY SENSOR LIBRARY
#include <MKRWAN.h>           //LORAWAN LIBRARY
#include <RTCZero.h>          //RTC LIBRARY

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
String msg = "TEST";                            //string message declaration
String stringOne = msg;                         //string message declaration
float humidity = 0;                             //Humidity variable declaration
float temperature = 0;                          //Temperature variable declaration
float pm25 = 0;                                 //pm25 variable declaration
float pm10 = 0;                                 //pm10 variable declaration
float moyTemp = 0;                              //moyTemp variable declaration
float moyHum = 0;                               //moyHum variable declaration
float moyPM25 = 0;                              //moyPM25 variable declaration
float moyPM10 = 0;                              //moyPM10 variable declaration

//Function definitions
void dopulse(void); //Pulse methode definition
void sendLoraMessage(void);//Send message using Lora definition
void ledBlink(void);



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
    stringOne =  String(moyTemp, 2) + " " + String(moyHum, 2) + " " + String(moyPM25, 2) + " " + String(moyPM10, 2);
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
    modem.print(stringOne);
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
}

//Main process looping each secondes
void loop()
{
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

  count = count + 1;
  if(count >= MEASUREDURATION) //When the process reach the end time MEASUREDURATION
  {
    //Average process + print
    if(count > 0)
    {
      moyTemp = temperature/count;
      moyHum = humidity/count;
      moyPM25 = pm25/count;
      moyPM10 = pm10/count;
    }
    Serial.print(", moyPM10 = ");
    Serial.println(moyPM10);
    Serial.print(", moyPM25 = ");
    Serial.println(moyPM25);
    Serial.print(", moyTemp = ");
    Serial.println(moyTemp);
    Serial.print(", moyHum = ");
    Serial.println(moyHum);
    count = 0;
    //Send the data to server using lorawan
    sendLoraMessage();
    delay(200);
    //Pulse process during sleep
    dopulse();
  }
  delay(1000);
}
