#include <Adafruit_GPS.h>

#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"

// turn on only the second sentence (GPRMC)
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
// turn on GPRMC and GGA
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn on ALL THE DATA
#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn off output
#define PMTK_SET_NMEA_OUTPUT_OFF "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"

#define PMTK_Q_RELEASE "$PMTK605*31"

int scale = 3; // 3 (±3g) for ADXL337, 200 (±200g) for ADXL377
boolean micro_is_5V = true; // Set to true if using a 5V microcontroller such as the Arduino Uno, false if using a 3.3V microcontroller, this affects the interpretation of the sensor data

void setup() {
  while (!Serial); // wait for leo to be ready

  Serial.begin(57600); // this baud rate doesn't actually matter!
  Serial1.begin(9600);
  delay(2000);
  Serial.println("Get version!");
  Serial1.println(PMTK_Q_RELEASE);

  // you can send various commands to get it started
  Serial1.println(PMTK_SET_NMEA_OUTPUT_RMCONLY);

  Serial1.println(PMTK_SET_NMEA_UPDATE_10HZ);
}


void loop() {

  // Get raw accelerometer data for each axis
  int rawX = analogRead(A0);
  int rawY = analogRead(A1);
  int rawZ = analogRead(A2);

  // Scale accelerometer ADC readings into common units
  // Scale map depends on if using a 5V or 3.3V microcontroller
  float scaledX, scaledY, scaledZ; // Scaled values for each axis
  if (micro_is_5V) // Microcontroller runs off 5V
  {
    scaledX = mapf(rawX, 0, 675, -scale, scale); // 3.3/5 * 1023 =~ 675
    scaledY = mapf(rawY, 0, 675, -scale, scale);
    scaledZ = mapf(rawZ, 0, 675, -scale, scale);
  }
  else // Microcontroller runs off 3.3V
  {
    scaledX = mapf(rawX, 0, 1023, -scale, scale);
    scaledY = mapf(rawY, 0, 1023, -scale, scale);
    scaledZ = mapf(rawZ, 0, 1023, -scale, scale);
  }

  if (Serial.available()) {
    char c = Serial.read();
    Serial.write(c);
    Serial1.write(c);

  }
  if (Serial1.available()) {
    char c = Serial1.read();
    Serial.print(c);
    // c is being compared with ASCII newline DEC of 10
    if (c == 10) {
      Serial.print("X: "); Serial.println(rawX);
      Serial.print("Y: "); Serial.println(rawY);
      Serial.print("Z: "); Serial.println(rawZ);
    }
  }
}

// Same functionality as Arduino's standard map function, except using floats
float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

