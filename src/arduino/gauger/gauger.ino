/*
 * Commands
 *
 * 71 - Set mode
 *
 *  :<id>:01 <mode>;
 *
 * 72 - Set declination angle
 *
 *  :<id>:02 <radians>;
 */
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <SoftwareSerial.h> 
#include <TinyGPS.h>


//#include <LiquidCrystal_I2C.h>
////LiquidCrystal_I2C  lcd(0x27,2,1,0,4,5,6,7); // 0x27 is the I2C bus address for an unmodified module
//LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

// Modes
// 1: quiet
// 2: stream all
// 3: stream gps
#define maxMode 3
byte mode = 1;

// Motor Controller
#define mcStatePin 5
#define mcRxPin 6
#define mcTxPin 7
SoftwareSerial mcSerial(mcRxPin, mcTxPin); //rx, tx
#define mcTimeout 2000
byte mcState = LOW;

// GPS
#define gspRxPin 8
#define gpsTxPin 9 // not functional
SoftwareSerial gpsSerial(gspRxPin, gpsTxPin); //rx, tx
TinyGPS gps;
float gps_lat = 1000;
float gps_lon = 1000;

// Mag

/* Assign a unique ID to this sensor */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(49138);
// https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
float declinationAngle = 0.23; // radians
boolean isMagInit = false;
float mag_x = 0; // micro-Tesla (uT)
float mag_y = 0;
float mag_z = 0;
float mag_heading = -1; // degrees

void setup() {
  //lcd.begin(20, 4);
  //lcd.clear();
  //lcd.backlight();
  pinMode(mcStatePin, INPUT);
  Serial.begin(9600);
  gpsSerial.begin(9600);
  mcSerial.begin(9600);
  if (mag.begin()) {
    isMagInit = true;
  }

  
}

void loop() {

  if (mode == 2) {
    readAll();
    writeAll(Serial);
  } else if (mode == 3) {
    streamGps(Serial);
  }
  //updateDisplay();
  takeCommand(Serial, Serial);
  delay(1000);
}

/*
void updateDisplay() {
  lcd.setCursor(0, 0);
  lcd.print("Heading: ");
  lcd.print(mag_heading, 2);
}
*/
void takeCommand(Stream &input, Stream &output) {

  if (!input.available()) {
    return;
  }
  
  byte firstByte = input.read();

  // ignore trailing \n
  if (firstByte == '\n') {
    return;
  }
  if (firstByte != ':') {
    return;
  }

  long id = input.parseInt();

  if (input.read() != ':') {
    writeAck(id, output, true);
    output.write("=40\n");
    return;
  }

  String command = input.readStringUntil(' ');
  if (command.toInt() < 70) {

    // forward to motorcontroller

    readMcState();
    if (mcState != HIGH) {
      input.readStringUntil(';');
      writeAck(id, output, true);
      output.write("=04\n");
      return;
    }

    mcSerial.write(':');
    mcSerial.print(command);
    mcSerial.write(' ');
    mcSerial.print(input.readStringUntil(';'));
    mcSerial.write(';');

    int d = 0;
    while (!mcSerial.available()) {
      delay(1);
      d += 1;
      if (d > mcTimeout) {
        writeAck(id, output, true);
        output.write("=02\n");
        return;
      }
    }

    String res = mcSerial.readStringUntil("\n");
    writeAck(id, output, false);
    output.print(res);
    output.write("\n");

    return;
  }

  writeAck(id, output, true);

  if (command.equals("71")) {
    // set mode
    byte newMode = input.readStringUntil(';').toInt();
    if (newMode < 1 || newMode > maxMode) {
      output.write("=49\n");
      return;
    }
    mode = newMode;
    output.write("=00\n");
  } else if (command.equals("72")) {
    // set declination angle
    float newValue = input.readStringUntil(';').toFloat();
    if (newValue > 7 || newValue < -7) {
      output.write("=49\n");
      return;
    }
    declinationAngle = newValue;
    output.write("=00\n");
  } else {
    output.write("=44\n");
  }
}

void writeAck(long &id, Stream &output, boolean withColon) {
  output.write("ACK:");
  output.print(id, DEC);
  if (withColon) {
    output.write(':');
  }
}

void writeAll(Stream &output) {
  output.write("GPS:");
  writeGps(output);
  output.write("\n");
  if (isMagInit) {
    output.write("MAG:");
    writeMag(output);
    output.write("\n");
  }
}

void writeGps(Stream &output) {
  output.print(gps_lat, 6);
  output.write('|');
  output.print(gps_lon, 6);
}

void writeMag(Stream &output) {
  output.print(mag_heading, 4);
  output.write('|');
  output.print(mag_x, 4);
  output.write('|');
  output.print(mag_y, 4);
  output.write('|');
  output.print(mag_z, 4);
  output.write('|');
  output.print(declinationAngle, 4);
}

void readAll() {
  readMcState();
  readGps();
  if (isMagInit) {
    readMag();
  }
}

void readMcState() {
  mcState = digitalRead(mcStatePin);
}

void readGps() {
  while (gpsSerial.available()) {
    if (gps.encode(gpsSerial.read())) {
      gps.f_get_position(&gps_lat, &gps_lon);
    }
  }
}

void streamGps(Stream &output) {
  while (gpsSerial.available()) {
    output.write(gpsSerial.read());
  }
}

void readMag() {
  sensors_event_t event; 
  mag.getEvent(&event);
  // magnetic vector values are in micro-Tesla (uT)
  mag_x = event.magnetic.x;
  mag_y = event.magnetic.y;
  mag_z = event.magnetic.z;
  
  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);

  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0) {
    heading += 2*PI;
  }

  // Check for wrap due to addition of declination.
  if(heading > 2*PI) {
    heading -= 2*PI;
  }
   
  // Convert radians to degrees for readability.
  mag_heading = heading * 180/M_PI; 
}
