/*
 * Commands
 *
 * 71 - Set mode
 *
 *  :<id>:71 <mode>;
 *
 * 73 - Set loop delay in milliseconds
 *
 *  :<id>:73 <milliseconds>;
 *
 * 74 - Set MCI check interval in milliseconds
 *
 *  :<id>:74 <milliseconds>;
 */

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <utility/imumaths.h>
#include <Wire.h>

/******************************************/
/* Hardware Enable                        */
/******************************************/
#define gpsEnabled true
#define orientationEnabled true
#define magEnabled true

/******************************************/
/* Constants                              */
/******************************************/

#define BAUD_RATE 9600L
#define DEG_NULL 1000.00

/******************************************/
/* Behavior                               */
/******************************************/

#define maxMode 3
// 1: quiet, 2: stream all, 3: stream gps
byte mode = 1;
// Loop delay in milliseconds
unsigned long loopDelay = 50;

/******************************************/
/* Module                                 */
/******************************************/

struct Module {
  char label[4];
  boolean isEnabled;
  boolean isInit;
  boolean hasData;
};

/******************************************/
/* Motor Controller Serial                */
/******************************************/

#define mccStatePin 5
#define mccRxPin 6
#define mccTxPin 7
#define mccReadTimeout 250L
#define mccWriteTimeout 10000L
#define mccBaudRate 9600L

SoftwareSerial mccSerial(mccRxPin, mccTxPin);

struct MotorControllerSerial {

  Module module;

  byte statePin;
  byte state;

  unsigned long readTimeout;
  unsigned long writeTimeout;
  
  String statusStr;
};

MotorControllerSerial mcc;

/******************************************/
/* Motor Controller I2C                   */
/******************************************/

#define mciAddress 0x9
#define mciMessageLength 18
#define mciDefaultCheckInterval 2000L

struct MotorControllerI2C {

  Module module;

  byte address;
 
  unsigned long checkInterval;
  unsigned long lastCheckTime;

  String statusStr;
};

MotorControllerI2C mci;

/******************************************/
/* Orientation Sensor                     */
/******************************************/

#define oriAddress 0x28
#define orfAddress 0x29

struct Orientation {

  Module module;

  // https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/arduino-code
  Adafruit_BNO055 sensor;
  byte address;

  boolean isCalibrated;

  float x;
  float y;
  float z;
  float qw;
  float qx;
  float qy;
  float qz;

  int8_t temp;

  byte cal_system;
  byte cal_gyro;
  byte cal_accel;
  byte cal_mag;
};

Orientation ori;
Orientation orf;

/******************************************/
/* GPS                                    */
/******************************************/

#define gpsBaudRate 9600L
#define gspRxPin 8
#define gpsTxPin 9 // not functional

SoftwareSerial gpsSerial(gspRxPin, gpsTxPin); //rx, tx

struct Gps {

  Module module;

  TinyGPS helper;

  float lat;
  float lon;
};

Gps gps;

/******************************************/
/* Magnetometer                           */
/******************************************/

#define magCheckAddress (0x3C >> 1)
#define magDeviceId 49138
//#define defaultDeclinationRad 0.23

struct Mag {

  Module module;

  Adafruit_HMC5883_Unified sensor;

  int deviceId;
  byte checkAddress;
  
  //float declinationRad;

  // micro-Tesla (uT)
  float x;
  float y;
  float z;

  // degrees
  float heading;
};

Mag mag;


/******************************************/
/* Setup                                  */
/******************************************/

void setup() {
  pinMode(mccStatePin, INPUT);
  Serial.begin(BAUD_RATE);
  Wire.begin();
  setupModules();
}

void setupModules() {

  strcpy(ori.module.label, "ORI");
  ori.module.isEnabled = orientationEnabled;
  ori.address = oriAddress;
  ori.sensor = Adafruit_BNO055(55, ori.address);
  ori.x = DEG_NULL;
  ori.y = DEG_NULL;
  ori.z = DEG_NULL;
  ori.qw = DEG_NULL;
  ori.qx = DEG_NULL;
  ori.qy = DEG_NULL;
  ori.qz = DEG_NULL;
  if (ori.module.isEnabled && ori.sensor.begin()) {
    ori.module.isInit = true;
    ori.sensor.setExtCrystalUse(true);
  }
  ori.module.hasData = ori.module.isInit;

  strcpy(orf.module.label, "ORF");
  orf.module.isEnabled = orientationEnabled;
  orf.address = orfAddress;
  orf.sensor = Adafruit_BNO055(55, orf.address);
  orf.x = DEG_NULL;
  orf.y = DEG_NULL;
  orf.z = DEG_NULL;
  orf.qw = DEG_NULL;
  orf.qx = DEG_NULL;
  orf.qy = DEG_NULL;
  orf.qz = DEG_NULL;
  if (orf.module.isEnabled && orf.sensor.begin()) {
    orf.module.isInit = true;
    orf.sensor.setExtCrystalUse(true);
  }
  orf.module.hasData = orf.module.isInit;

  strcpy(mcc.module.label, "MCC");
  mcc.module.isEnabled = true;
  mcc.readTimeout = mccReadTimeout;
  mcc.writeTimeout = mccWriteTimeout;
  mcc.statePin = mccStatePin;
  mccSerial.begin(mccBaudRate);
  mcc.module.isInit = checkMccConnected(mccSerial);

  strcpy(mci.module.label, "MCI");
  mci.module.isEnabled = true;
  mci.address = mciAddress;
  mci.checkInterval = mciDefaultCheckInterval;
  mci.module.isInit = checkMciConnected(mci);

  strcpy(gps.module.label, "GPS");
  gps.module.isEnabled = gpsEnabled;
  gps.lat = DEG_NULL;
  gps.lon = DEG_NULL;
  if (gps.module.isEnabled) {
    gpsSerial.begin(gpsBaudRate);
    gps.module.isInit = checkGpsConnected(gpsSerial);
  }
  gps.module.hasData = gps.module.isInit;

  strcpy(mag.module.label, "MAG");
  mag.module.isEnabled = magEnabled;
  mag.deviceId = magDeviceId; // set a unique id
  // the address is hard-coded in the sensor library, so
  // this is just for checking whether it is connected.
  mag.checkAddress = magCheckAddress;
  mag.sensor = Adafruit_HMC5883_Unified(mag.deviceId);
  mag.heading = DEG_NULL;
  //mag.declinationRad = defaultDeclinationRad;
  if (mag.module.isEnabled) {
    mag.module.isInit = mag.sensor.begin() && checkMagConnected(mag);
  }
  mag.module.hasData = mag.module.isInit;
}

void loop() {

  takeCommand(Serial, Serial);
  if (mode == 2) {
    readAll();
    writeAll(Serial);
  } else if (mode == 3 && gpsEnabled) {
    streamGps(Serial, gpsSerial);
  }
  // take command twice, since readAll takes time.
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

    if (!mcc.module.isInit) {
      input.readStringUntil(';');
      writeAck(id, output, true);
      output.write("=01\n");
      return;
    }
    // forward to motorcontroller

    readMccState(mcc);

    if (mcc.state != HIGH) {
      input.readStringUntil(';');
      writeAck(id, output, true);
      output.write("=04\n");
      return;
    }

    mccSerial.listen();

    String mcBody = String(":");
    mcBody.concat(command);
    mcBody.concat(" ");
    mcBody.concat(input.readStringUntil(';'));
    mcBody.concat(";");
    mccSerial.print(mcBody);

    int d = 0;
    while (!mccSerial.available()) {
      delay(1);
      d += 1;
      if (d > mcc.timeout) {
        writeAck(id, output, true);
        output.write("=02\n");
        return;
      }
    }

    String res = mccSerial.readStringUntil("\n");
    writeAck(id, output, true);
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
  } else if (command.equals("73")) {
    // set loop delay
    long newValue = input.readStringUntil(';').toInt();
    if (newValue < 1) {
      output.write("=49\n");
      return;
    }
    loopDelay = newValue;
    output.write("=00\n");
  } else if (command.equals("74")) {
    // set mci check interval
    long newValue = input.readStringUntil(';').toInt();
    if (newValue < 1) {
      output.write("=49\n");
      return;
    }
    mci.checkInterval = newValue;
    output.write("=00\n");
  } else {
    output.write("=44\n");
  }
}

void writeAck(long &id, Stream &output, boolean withColon) {
  // clear with newline, for initialization and mode change
  output.write("\nACK:");
  output.print(id, DEC);
  if (withColon) {
    output.write(':');
  }
}

void writeAll(Stream &output) {

  output.write("MOD:");
  writeModules(output);
  output.write("\n");

  if (ori.module.isInit) {
    output.write(ori.module.label);
    output.write(':');
    writeOrientation(ori, output);
    output.write("\n");
  }

  if (orf.module.isInit) {
    output.write(orf.module.label);
    output.write(':');
    writeOrientation(orf, output);
    output.write("\n");
  }
 
  if (gps.module.isInit) {
    output.write(gps.module.label);
    output.write(':');
    writeGps(gps, output);
    output.write("\n");
  }

  if (mag.module.isInit) {
    output.write(mag.module.label);
    output.write(':');
    writeMag(mag, output);
    output.write("\n");
  }

  if (mcc.module.isInit) {
    output.write(mcc.module.label);
    output.write(':');
    writeMccStatus(mcc, output);
    output.write("\n");
  }

  if (mci.module.isInit) {
    output.write(mci.module.label);
    output.write(':');
    writeMciStatus(mci, output);
    output.write("\n");
  }
}

void writeModules(Stream &output) {

  boolean doPrefix = false;

  if (ori.module.hasData) {
    if (doPrefix) {
      output.write('|');
    }
    output.write(ori.module.label);
    doPrefix = true;
  }

  if (orf.module.hasData) {
    if (doPrefix) {
      output.write('|');
    }
    output.write(orf.module.label);
    doPrefix = true;
  }

  if (gps.module.hasData) {
    if (doPrefix) {
      output.write('|');
    }
    output.write(gps.module.label);
    doPrefix = true;
  }

  if (mag.module.hasData) {
    if (doPrefix) {
      output.write('|');
    }
    output.write(mag.module.label);
    doPrefix = true;
  }

  if (mcc.module.hasData) {
    if (doPrefix) {
      output.write('|');
    }
    output.write(mcc.module.label);
    doPrefix = true;
  }

  if (mci.module.hasData) {
    if (doPrefix) {
      output.write('|');
    }
    output.write(mci.module.label);
    doPrefix = true;
  }
  
}

void writeMccStatus(MotorControllerSerial &m, Stream &output) {
  output.print(m.statusStr);
}

void writeMciStatus(MotorControllerI2C &m, Stream &output) {
  output.print(m.statusStr);
}

void writeOrientation(Orientation &o, Stream &output) {
  output.print(o.x, 4);
  output.write('|');
  output.print(o.y, 4);
  output.write('|');
  output.print(o.z, 4);
  output.write('|');
  output.print(o.qw, 4);
  output.write('|');
  output.print(o.qx, 4);
  output.write('|');
  output.print(o.qy, 4);
  output.write('|');
  output.print(o.qz, 4);
  output.write('|');
  output.print(o.temp);
  output.write('|');
  output.print(o.cal_system);
  output.write('|');
  output.print(o.cal_gyro);
  output.write('|');
  output.print(o.cal_accel);
  output.write('|');
  output.print(o.cal_mag);
  output.write('|');
  output.write(o.isCalibrated ? 'T' : 'F');
}

void writeGps(Gps &g, Stream &output) {
  output.print(g.lat, 6);
  output.write('|');
  output.print(g.lon, 6);
}

void writeMag(Mag &m, Stream &output) {
  output.print(m.heading, 4);
  output.write('|');
  output.print(m.x, 4);
  output.write('|');
  output.print(m.y, 4);
  output.write('|');
  output.print(m.z, 4);
  output.write('|');
  output.print(m.declinationRad, 4);
}

void readAll() {
  readMccState(mcc);
  if (mcc.module.isInit) {
    readMccStatus(mcc, mccSerial);
  }
  if (mci.module.isInit) {
    readMciStatus(mci);
  }
  if (ori.module.isInit) {
    readOrientation(ori);
  }
  if (orf.module.isInit) {
    readOrientation(orf);
  }
  if (gps.module.isInit) {
    readGps(gps, gpsSerial);
  }
  if (mag.module.isInit) {
    readMag(mag);
  }
}

void readMccState(MotorControllerSerial &m) {
  m.state = digitalRead(m.statePin);
}

void readMccStatus(MotorControllerSerial &m, SoftwareSerial &ser) {
  if (m.state != HIGH) {
    return;
  }
  ser.listen();
  ser.write(":18 ;");
  int d = 0;
  while (!ser.available()) {
    delay(1);
    d += 1;
    if (d > m.readTimeout) {
      return;
    }
  }
  String codeStr = ser.readStringUntil(';');
  if (!codeStr.equals("=00")) {
    return;
  }
  // this is what used to take so long, until
  // we replaced "\n" with '\n' !
  m.statusStr = ser.readStringUntil('\n');
  m.statusStr.trim();
  m.module.hasData = m.statusStr.length() > 0;
}

// should only do this occasionally, since it will slow
// motor operations down.
// read I2C
void readMciStatus(MotorControllerI2C &m) {
  if (millis() - m.lastCheckTime < m.checkInterval) {
    return;
  }
  m.lastCheckTime = millis();
  Wire.beginTransmission(m.address);
  Wire.write(0x0);
  if (Wire.endTransmission() != 0) {
    return;
  }
  Wire.requestFrom((uint8_t) m.address, (uint8_t) mciMessageLength);
  char buf[19];
  byte i = 0;
  while (Wire.available()) {
    char c = Wire.read();
    
    if (c == 13) {
      break;
    }
    if (c < 13) {
      continue;
    }
    buf[i] = c;
    i++;
  }
  m.statusStr = String(buf);
  m.statusStr.trim();
  m.module.hasData = m.statusStr.length() > 0;
}

void readOrientation(Orientation &o) {

  o.temp = o.sensor.getTemp();

  /* Get a new sensor event */ 
  sensors_event_t event; 
  o.sensor.getEvent(&event);

  // TODO: figure out whether we can ever go out of calibration
  if (!o.isCalibrated) {
    o.sensor.getCalibration(&o.cal_system, &o.cal_gyro, &o.cal_accel, &o.cal_mag);
    if (o.cal_system + o.cal_gyro + o.cal_accel + o.cal_mag == 12) {
      o.isCalibrated = true;
    } else {
      // do not set values if not calibrated
      return;
    }
  }

  o.x = event.orientation.x;
  o.y = event.orientation.y;
  o.z = event.orientation.z;

  imu::Quaternion q = o.sensor.getQuat();
  o.qw = q.w();
  o.qx = q.x();
  o.qy = q.y();
  o.qz = q.z();
}

void readGps(Gps &g, SoftwareSerial &ser) {
  ser.listen();
  // we have to set a delay here after listen, otherwise we
  // never read values. this value MUST be at least 50ms.
  delay(60);
  while (ser.available()) {
    if (g.helper.encode(ser.read())) {
      g.helper.f_get_position(&g.lat, &g.lon);
    }
  }
}

void streamGps(Stream &output, SoftwareSerial &ser) {
  ser.listen();
  while (ser.available()) {
    output.write(ser.read());
  }
}

/***************************************************************************
  Written by Kevin Townsend for Adafruit Industries with some heading example from
  Love Electronics (loveelectronics.co.uk)
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the version 3 GNU General Public License as
 published by the Free Software Foundation.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.

 ***************************************************************************/
void readMag(Mag &m) {
  sensors_event_t event; 
  m.sensor.getEvent(&event);
  // magnetic vector values are in micro-Tesla (uT)
  m.x = event.magnetic.x;
  m.y = event.magnetic.y;
  m.z = event.magnetic.z;
  
  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);

  // move the declination offset calculation to the application layer
  //heading += m.declinationRad;
  
  // Correct for when signs are reversed.
  if (heading < 0) {
    heading += 2 * PI;
  }

  // Check for wrap due to addition of declination.
  if (heading > 2 * PI) {
    heading -= 2 * PI;
  }
   
  // Convert radians to degrees for readability.
  m.heading = heading * 180 / M_PI; 
}

// Utilities

// the begin method in the Adafruit library just writes to
// the device address and returns true. We need to check for
// a response at address 0x3C >> 1
boolean checkMagConnected(Mag &m) {
  Wire.beginTransmission(m.checkAddress);
  return Wire.endTransmission() == 0;
}

boolean checkMciConnected(MotorControllerI2C m) {
  Wire.beginTransmission(m.address);
  return Wire.endTransmission() == 0;
}

// send a status request with a 2 second timeout
boolean checkMccConnected(SoftwareSerial &ser) {
  ser.listen();
  ser.write(":18 ;");
  // timeout 2 seconds
  int d = 0;
  while (!ser.available()) {
    delay(1);
    d += 1;
    if (d > 2000) {
      return false;
    }
  }
  ser.readStringUntil("\n");
  return true;
}

boolean checkGpsConnected(SoftwareSerial &ser) {
  ser.listen();
  // timeout 3 seconds
  int d = 0;
  while (!ser.available()) {
    delay(1);
    d += 1;
    if (d > 3000) {
      return false;
    }
  }
  return true;
}
