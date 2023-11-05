
// Libraries / Header Files
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"
#include "Adafruit_GFX.h"
#include <Blynk.h>
#include <BlynkSimpleEsp32.h>
#include "OakOLED.h"

// Aliases
OakOLED oled;
MAX30105 particleSensor;

// Macros
#define REPORTING_PERIOD_MS 1000 // frequency of updates sent to blynk app in ms
#define BLYNK_AUTH_TOKEN "nN0y_X1zx3HqPtNpBT1YdTewgf8aqg5n" // LED for no finger

/***********************************************  GLOBAL VARIABLES  *************************************************/

// Blynk IoT Connection

char auth[] = BLYNK_AUTH_TOKEN;   // Authentication token of out blynk interface
char ssid[] = "iQOO Z7 Pro 5G";    // Wifi credentials 
char pass[] = "koderchit";         // Wifi Password

uint32_t tsLastReport = 0;        //stores the time the last update was sent to the blynk app

// Sensor Variables
uint32_t irBuffer[100];           //Infrared LED sensor data
uint32_t redBuffer[100];          //red LED sensor data
int32_t bufferLength;             //data length
const int irThresh = 50000;

// Maxim Variables
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

// Reading Calculations [BPM]
float beatsPerMinute;
int beatAvg;
float floatAvg;
const byte RATE_SIZE = 100; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

// Reading Calculations [Spo2]
const byte RATE_SIZE_OX = 100;
byte rates_OX[RATE_SIZE_OX];
byte rateSpot_OX = 0;
float spo2Avg;

// LED
//byte pulseLED = 11; //Must be on PWM pin
//byte readLED = 13; //Blinks with each data read

// Button to Initialise
const int buttonPin = 18;

// Initial Countdown
long preva = 50;
long a;

//No finger Variables
long NOFingerTime= 5000;

// Finger Detection
bool finger = true;

// Heart Animation Bitmap
const unsigned char bitmap [] PROGMEM =
{
  0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 0x18, 0x00, 0x0f, 0xe0, 0x7f, 0x00, 0x3f, 0xf9, 0xff, 0xc0,
  0x7f, 0xf9, 0xff, 0xc0, 0x7f, 0xff, 0xff, 0xe0, 0x7f, 0xff, 0xff, 0xe0, 0xff, 0xff, 0xff, 0xf0,
  0xff, 0xf7, 0xff, 0xf0, 0xff, 0xe7, 0xff, 0xf0, 0xff, 0xe7, 0xff, 0xf0, 0x7f, 0xdb, 0xff, 0xe0,
  0x7f, 0x9b, 0xff, 0xe0, 0x00, 0x3b, 0xc0, 0x00, 0x3f, 0xf9, 0x9f, 0xc0, 0x3f, 0xfd, 0xbf, 0xc0,
  0x1f, 0xfd, 0xbf, 0x80, 0x0f, 0xfd, 0x7f, 0x00, 0x07, 0xfe, 0x7e, 0x00, 0x03, 0xfe, 0xfc, 0x00,
  0x01, 0xff, 0xf8, 0x00, 0x00, 0xff, 0xf0, 0x00, 0x00, 0x7f, 0xe0, 0x00, 0x00, 0x3f, 0xc0, 0x00,
  0x00, 0x0f, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// Heart Beating Animation
void HeartAnimation()
{
  oled.drawBitmap( 82, 18, bitmap, 28, 28, 1);
  oled.display();
}

void BlynkSetup();
void DisplayScreenInitial();
void InitialiseSensor();
void WaitForButtonPress();
void SetupLED();
void clearAllVar();

void setup()
{
  Serial.begin(115200);
  pinMode(buttonPin, INPUT_PULLUP);
  DisplayScreenInitial(); // To display the first two screens: Project, Initialising
  BlynkSetup();
  InitialiseSensor();

  SetupLED();
}

void loop()
{
victor:
  clearAllVar();
  WaitForButtonPress();
  Blynk.run();
  // bufferLength = 100000; //buffer length of 100 stores 4 seconds of samples running at 25sps
  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps
  long initialTime = millis(); // paperboy
  long waitingTime = 6000;

  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    long irValue = irBuffer[i];

    Serial.print("IR=");
    Serial.print(irValue);
    Serial.print(", BPM=");
    Serial.print(heartRate);
    Serial.print(", Avg BPM=");
    Serial.print(beatAvg);
    Serial.print(", Cal. BPM=");
    Serial.print(floatAvg);
    Serial.print(", Cal. spO2=");
    Serial.print(spo2Avg);
    bool f=1;
    int startTimeVal=-1;
    while (irValue < irThresh) {
      Serial.print(" No finger?");
      oled.clearDisplay();
      oled.setTextSize(2);
      oled.setTextColor(1);
      oled.setCursor(10, 24);
      oled.println("NO FINGER");
      oled.display();
      Blynk.virtualWrite(V0, 1);
      delay(1000);
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data
      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      irValue = irBuffer[i];

      
      if(f)
        {
          f=0;
          startTimeVal= millis();
        }
        else{
          if(millis()-startTimeVal>NOFingerTime)
          {
            goto victor;        
          }
        }
    }



    Serial.println();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

    // Serial.print(F("red="));
    // Serial.print(redBuffer[i], DEC);
    // Serial.print(F(", ir="));
    // Serial.println(irBuffer[i], DEC);

    if (millis() - initialTime < waitingTime) {
      a = waitingTime / 1000 - (millis() - initialTime) / 1000;
      if (preva != a) {
        oled.clearDisplay();
        oled.setTextSize(1);
        oled.setTextColor(1);
        oled.setCursor(0, 0);
        oled.println("Please wait...");
        oled.setCursor(0, 36);
        oled.println("Time Left: ");
        oled.setCursor(0, 47);
        oled.setTextSize(2);
        oled.print(a);
        oled.display();
        HeartAnimation();
      }
      preva = a;
    }
  }
  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  if (heartRate < 255 && heartRate > 20)
  {
    rates[rateSpot++] = (byte)heartRate; //Store this reading in the array
    rateSpot %= RATE_SIZE; //Wrap variable

    //Take average of readings
    beatAvg = 0;
    for (byte x = 0 ; x < RATE_SIZE ; x++)
      beatAvg += rates[x];
    beatAvg /= RATE_SIZE;
    floatAvg = float(beatAvg) * (93.00 / 136.00);
  }

  if (spo2 > 0)
  {
    rates_OX[rateSpot_OX++] = (byte)spo2; //Store this reading in the array
    rateSpot_OX %= RATE_SIZE_OX; //Wrap variable

    //Take average of readings
    spo2Avg = 0;
    for (byte x = 0 ; x < RATE_SIZE_OX ; x++)
      spo2Avg += rates_OX[x];
    spo2Avg /= RATE_SIZE_OX;
  }

  while (1)
  {
    finger = true;
    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }


    //take 25 sets of samples before calculating the heart rate.
    for (byte i = 75; i < 100; i++)
    {
      redBuffer[i - 75] = redBuffer[i - 50];
      irBuffer[i - 75] = irBuffer[i - 50];
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();


      long irValue = irBuffer[i];
      maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
      if (heartRate < 255 && heartRate > 20)
      {
        rates[rateSpot++] = (byte)heartRate; //Store this reading in the array
        rateSpot %= RATE_SIZE; //Wrap variable

        //Take average of readings
        beatAvg = 0;
        for (byte x = 0 ; x < RATE_SIZE ; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
        floatAvg = float(beatAvg) * (93.00 / 136.00);
      }

      if (spo2 > 0)
      {
        rates_OX[rateSpot_OX++] = (byte)spo2; //Store this reading in the array
        rateSpot_OX %= RATE_SIZE_OX; //Wrap variable

        //Take average of readings
        spo2Avg = 0;
        for (byte x = 0 ; x < RATE_SIZE_OX ; x++)
          spo2Avg += rates_OX[x];
        spo2Avg /= RATE_SIZE_OX;
      }

      Serial.print("IR=");
      Serial.print(irValue);
      Serial.print(", BPM=");
      Serial.print(heartRate);
      Serial.print(", Avg BPM=");
      Serial.print(beatAvg);
      Serial.print(", Cal. BPM=");
      Serial.print(floatAvg);
      Serial.print(F(", SPO2="));
      Serial.print(spo2, DEC);
      Serial.print(", Cal. spo2=");
      Serial.print(spo2Avg);
      bool f=1;
      int startTimeVal=-1;
      while (irValue < irThresh) {
        Serial.print(" No finger?");
        finger = false;
        oled.clearDisplay();
        oled.setTextSize(2);
        oled.setTextColor(1);
        oled.setCursor(10, 24);
        oled.println("NO FINGER");
        oled.display();
        Blynk.virtualWrite(V0, 1);
        delay(1000);
        while (particleSensor.available() == false) //do we have new data?
          particleSensor.check(); //Check the sensor for new data
        redBuffer[i] = particleSensor.getRed();
        irBuffer[i] = particleSensor.getIR();
        irValue = irBuffer[i];

        if(f)
        {
          f=0;
          startTimeVal= millis();
        }
        else{
          if(millis()-startTimeVal>NOFingerTime)
          {
            goto victor;        
          }
        }
      }

      Serial.println();

      particleSensor.nextSample(); //We're finished with this sample so move to next sample

      // //send samples and calculation result to terminal program through UART
      // Serial.print(F("red="));
      // Serial.print(redBuffer[i], DEC);
      // Serial.print(F(", ir="));
      // Serial.print(irBuffer[i], DEC);

      // Serial.print(F(", HR="));
      // Serial.print(heartRate, DEC);

      // Serial.print(F(", HRvalid="));
      // Serial.print(validHeartRate, DEC);


      // Serial.print(F(", SPO2Valid="));
      // Serial.println(validSPO2, DEC);

      if (millis() - tsLastReport > REPORTING_PERIOD_MS)
      {
        Blynk.virtualWrite(V0, !finger);

        if (millis() - initialTime < waitingTime) {
          a = waitingTime / 1000 - (millis() - initialTime) / 1000;
          if (preva != a) {
            oled.clearDisplay();
            oled.setTextSize(1);
            oled.setTextColor(1);
            oled.setCursor(0, 0);
            oled.println("Please wait...");
            oled.setCursor(0, 36);
            oled.println("Time Left: ");
            oled.setCursor(0, 47);
            oled.setTextSize(2);
            oled.print(a);
            oled.display();
            HeartAnimation();
          }
          preva = a;
        } else {
          if (!finger) {

            //Code will never come in this part for execution
            oled.clearDisplay();
            oled.setTextSize(2);
            oled.setTextColor(1);
            oled.setCursor(10, 24);
            oled.println("NO FINGER");
            oled.display();
            Blynk.virtualWrite(V0, 1);
            delay(1000);
//            goto victor;
          } else {

            Blynk.virtualWrite(V3, int(spo2Avg));
            Blynk.virtualWrite(V4, int(floatAvg));

            if (finger && int(spo2Avg) < 90) {
              Blynk.logEvent("lowspo2", "Your SPO2 is below 90%");
            }

            oled.clearDisplay();

            oled.setTextSize(2);
            oled.setTextColor(1);
            oled.setCursor(0, 0);
            oled.println("BPM:");

            oled.setTextSize(2);
            oled.setTextColor(1);
            oled.setCursor(0, 15);
            oled.println(int(floatAvg));

            oled.setTextSize(2);
            oled.setTextColor(1);
            oled.setCursor(0, 31);
            oled.println("SPO2:");

            oled.setTextSize(2);
            oled.setTextColor(1);
            oled.setCursor(0, 46);

            oled.println(int(spo2Avg));
            oled.display();

            HeartAnimation();
          }
        }

        tsLastReport = millis();
      }
    }

    //After gathering 25 new samples recalculate HR and SP02
    //    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  }
}

void DisplayScreenInitial() {
  oled.begin();
  oled.clearDisplay();
  oled.setTextColor(1);
  oled.setTextSize(2);
  oled.setCursor(22, 14);
  oled.println("PROJECT");
  oled.setTextSize(1);
  oled.setCursor(22, 32);
  oled.println("under PRM Sir");
  oled.setCursor(22, 43);
  oled.println("SPO2 & BPM");
  oled.display();
  delay(1000); // paperboy
}

void BlynkSetup() {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(1);
  oled.setCursor(22, 25);
  oled.println("Connecting");
  oled.setCursor(15, 36);
  oled.println("to WiFi [Blynk]...");
  oled.display();
  delay(1000); // paperboy
  Serial.println("START");
  Blynk.begin(auth, ssid, pass, "blynk.cloud", 80);
  Serial.println("DONE");

  Blynk.virtualWrite(V3, 0);
  Blynk.virtualWrite(V4, 0);
  Blynk.virtualWrite(V0, 0);
}

void InitialiseSensor() {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(1);
  oled.setCursor(22, 25);
  oled.println("Initializing");
  oled.setCursor(15, 36);
  oled.println("pulse oximeter..");
  oled.display();
  delay(1000); // paperboy

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) // Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    oled.clearDisplay();
    oled.setTextSize(2);
    oled.setTextColor(1);
    oled.setCursor(23, 17);
    oled.println("FAILURE");
    oled.setCursor(28, 37);
    oled.setTextSize(1);
    oled.println("CHECK WIRING");
    oled.display();
    while (1);
  }
  else {
    oled.clearDisplay();
    oled.setTextSize(2);
    oled.setTextColor(1);
    oled.setCursor(23, 17);
    oled.println("SUCCESS");
    oled.setCursor(20, 37);
    oled.setTextSize(1);
    oled.println("Sensor Detected");
    oled.display();
    Serial.println("SENSOR FOUND SUCCESSFULLY");
    delay(1000); // paperboy
  }
}

void WaitForButtonPress() {
  oled.clearDisplay();
  oled.setTextSize(2);
  oled.setTextColor(1);
  oled.setCursor(32, 4);
  oled.println("Attach");
  oled.setCursor(32, 20);
  oled.println("Finger");
  oled.setTextSize(1);
  oled.setCursor(50, 42);
  oled.println("AND");
  oled.setCursor(0, 55);
  oled.println("Press the button");
  oled.display();
  Serial.println(F("Attach sensor to finger with rubber band. Press any key to start conversion"));

  int buttonValue = digitalRead(buttonPin);
  while (buttonValue == 1) {
    buttonValue = digitalRead(buttonPin);
  }
}

void SetupLED() {
  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
}

void clearAllVar() {
  tsLastReport = 0;  //stores the time the last update was sent to the blynk app

  // Sensor Variables
  for (int i = 0; i < 100; i++) {
    irBuffer[i] = 0;
    redBuffer[i] = 0;
  }

  // Maxim Variables
  spo2 = 98; //SPO2 value
  validSPO2 = 0; //indicator to show if the SPO2 calculation is valid
  heartRate = 125; //heart rate value
  validHeartRate = 0; //indicator to show if the heart rate calculation is valid

  // Reading Calculations [BPM]
  beatsPerMinute = 0;
  beatAvg = 0;
  floatAvg = 0.0;
  for (int i = 0; i < RATE_SIZE; i++) {
    rates[i] = heartRate;
  }
  rateSpot = 0;
  lastBeat = 0; //Time at which the last beat occurred

  // Reading Calculations [Spo2]
  for (int i = 0; i < RATE_SIZE_OX ; i++) {
    rates_OX[i] = spo2;
  }
  rateSpot_OX = 0;
  spo2Avg = 0.0;

  preva = 50;
  a = 0;

  // Finger Detection
  finger = true;

  Blynk.virtualWrite(V3, 0);
  Blynk.virtualWrite(V4, 0);
  Blynk.virtualWrite(V0, 0);
}