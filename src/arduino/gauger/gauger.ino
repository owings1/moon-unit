/*
 * Commands
 *
 * 71 - Set mode
 *
 *  :<id>:71 <mode>;
 *
 * 72 - Set declination angle
 *
 *  :<id>:72 <radians>;
 *
 * 73 - Set loop delay in milliseconds
 *
 *  :<id>:73 <milliseconds>;
 */
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <SoftwareSerial.h> 
#include <TinyGPS.h>

// hardware enable
#define gpsEnabled true
#define orientationEnabled false
#define magEnabled true

// Modes
// 1: quiet
// 2: stream all
// 3: stream gps
#define maxMode 3
byte mode = 1;

// Loop delay in milliseconds
long loopDelay = 250;

// Motor Controller
#define mcStatePin 5
#define mcRxPin 6
#define mcTxPin 7
SoftwareSerial mcSerial(mcRxPin, mcTxPin); //rx, tx
#define mcTimeout 10000
byte mcState = LOW;

// Orientation sensor

// https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/arduino-code
Adafruit_BNO055 bno = Adafruit_BNO055(55);
// whether it is initialized properly
boolean isOrientationInit = false;
boolean isOrientationCalibrated = false;
// latest read
float orientation_x;
float orientation_y;
float orientation_z;
// calibration
uint8_t cal_system = 0;
uint8_t cal_gyro = 0;
uint8_t cal_accel = 0;
uint8_t cal_mag = 0;

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
  pinMode(mcStatePin, INPUT);
  Serial.begin(9600);
  
  mcSerial.begin(9600);
  if (gpsEnabled) {
    gpsSerial.begin(9600);
  }
  if (magEnabled && mag.begin()) {
    isMagInit = true;
  }
  if (orientationEnabled && bno.begin()) {
    isOrientationInit = true;
    bno.setExtCrystalUse(true);
  }
}

void loop() {

  if (mode == 2) {
    readAll();
    writeAll(Serial);
  } else if (mode == 3 && gpsEnabled) {
    streamGps(Serial);
  }
  takeCommand(Serial, Serial);
  delay(loopDelay);
}

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

    mcSerial.listen();

    String mcBody = String(":");
    mcBody.concat(command);
    mcBody.concat(" ");
    mcBody.concat(input.readStringUntil(';'));
    mcBody.concat(";");
    mcSerial.print(mcBody);
    //output.print("mcBody#");
    //output.println(mcBody);
    //mcSerial.write(':');
    //mcSerial.print(command);
    //mcSerial.write(' ');
    //mcSerial.print(input.readStringUntil(';'));
    //mcSerial.write(';');

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
    writeAck(id, output, true);
    output.print(res);
    output.write("\n");

    //delay(100);
 
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
  } else if (command.equals("73")) {
    // set loop delay
    long newValue = input.readStringUntil(';').toInt();
    if (newValue < 1) {
      output.write("=49\n");
      return;
    }
    loopDelay = newValue;
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

  output.write("MOD:");
  writeModules(output);
  output.write("\n");

  if (isOrientationInit) {
    output.write("ORI:");
    writeOrientation(output);
    output.write("\n");
  }
  
  if (gpsEnabled) {
    output.write("GPS:");
    writeGps(output);
    output.write("\n");
  }

  if (isMagInit) {
    output.write("MAG:");
    writeMag(output);
    output.write("\n");
  }
}

void writeModules(Stream &output) {

  boolean doPrefix = false;

  if (isOrientationInit) {
    if (doPrefix) {
      output.write('|');
    }
    output.write("ORI");
    doPrefix = true;
  }

  if (gpsEnabled) {
    if (doPrefix) {
      output.write('|');
    }
    output.write("GPS");
    doPrefix = true;
  }

  if (isMagInit) {
    if (doPrefix) {
      output.write('|');
    }
    output.write("MAG");
    doPrefix = true;
  }
}

void writeOrientation(Stream &output) {
  // x|y|z|cal_system|cal_gyro|cal_accel|cal_mag|isCalibrated
  output.print(orientation_x, 4);
  output.write('|');
  output.print(orientation_y, 4);
  output.write('|');
  output.print(orientation_z, 4);
  output.write('|');
  output.print(cal_system);
  output.write('|');
  output.print(cal_gyro);
  output.write('|');
  output.print(cal_accel);
  output.write('|');
  output.print(cal_mag);
  output.write('|');
  output.write(isOrientationCalibrated ? 'T' : 'F');
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
  if (isOrientationInit) {
    readOrientation();
  }
  if (gpsEnabled) {
    readGps();
  }
  if (isMagInit) {
    readMag();
  }
}

void readMcState() {
  mcState = digitalRead(mcStatePin);
}

void readOrientation() {
  /* Get a new sensor event */ 
  sensors_event_t event; 
  bno.getEvent(&event);

  orientation_x = event.orientation.x;
  orientation_y = event.orientation.y;
  orientation_z = event.orientation.z;
  if (!isOrientationCalibrated) {
    bno.getCalibration(&cal_system, &cal_gyro, &cal_accel, &cal_mag);
    if (cal_system == 3 && cal_gyro == 3 && cal_accel == 3 && cal_mag == 3) {
      isOrientationCalibrated = true;
    }
  }
}

void readGps() {
  gpsSerial.listen();
  while (gpsSerial.available()) {
    if (gps.encode(gpsSerial.read())) {
      gps.f_get_position(&gps_lat, &gps_lon);
    }
  }
}

void streamGps(Stream &output) {
  gpsSerial.listen();
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
