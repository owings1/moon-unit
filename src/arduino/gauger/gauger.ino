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
 *
 * 75 - Reinit MCC & MCI module
 *
 *  :<id>:75 ;
 *
 * 76 - Send motor stop signal
 *
 *  :<id>:76 ;
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
#define mStopPin D9

/******************************************/
/* Constants                              */
/******************************************/
#define BAUD_RATE 9600L
#define DEG_NULL 1000.00
#define STATE_READY HIGH
#define STATE_BUSY LOW
 
/******************************************/
/* I2C                                    */
/******************************************/
#define mainWire Wire
#define SDA_MAIN D14
#define SCL_MAIN D13

/******************************************/
/* Behavior                               */
/******************************************/
#define maxMode 3
#define MODE_QUIET 1
#define MODE_STREAM_ALL 2
#define MODE_STREAM_GPS 3
byte mode = MODE_QUIET;
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
#define mccStatePin D10
#define mccSerial Serial2
#define mccReadTimeout 250L
#define mccWriteTimeout 10000L
#define mccBaudRate 9600L

struct MotorControllerSerial {
  Stream &stream;
  Module module;

  byte statePin;
  byte state;

  unsigned long readTimeout;
  unsigned long writeTimeout;
  
  String statusStr;
};

MotorControllerSerial mcc = {mccSerial};

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
#define gpsRxPin D6

SoftwareSerial gpsSerial(gpsRxPin, -1); //rx, tx

struct Gps {
  Stream &stream;
  Module module;

  TinyGPS helper;

  float lat;
  float lon;
};

Gps gps = {gpsSerial};

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
  pinMode(mccStatePin, INPUT_PULLUP);
  pinMode(mStopPin, OUTPUT);
  Serial.begin(BAUD_RATE);
  mainWire.setSDA(SDA_MAIN);
  mainWire.setSCL(SCL_MAIN);
  mainWire.begin();
  mccSerial.begin(mccBaudRate);
  if (gpsEnabled) {
    gpsSerial.begin(gpsBaudRate);
  }
  setupModules();
}

void setupModules() {
  strcpy(ori.module.label, "ORI");
  setupOrientationModule(ori, oriAddress);
  strcpy(orf.module.label, "ORF");
  setupOrientationModule(orf, orfAddress);

  strcpy(mcc.module.label, "MCC");
  mcc.module.isEnabled = true;
  mcc.readTimeout = mccReadTimeout;
  mcc.writeTimeout = mccWriteTimeout;
  mcc.statePin = mccStatePin;
  if (mcc.module.isEnabled) {
    mcc.module.isInit = checkMccConnected(mcc.stream);
  }

  strcpy(mci.module.label, "MCI");
  mci.module.isEnabled = true;
  mci.address = mciAddress;
  mci.checkInterval = mciDefaultCheckInterval;
  if (mci.module.isEnabled) {
    mci.module.isInit = checkMciConnected(mci);
  }

  strcpy(gps.module.label, "GPS");
  gps.module.isEnabled = gpsEnabled;
  gps.lat = DEG_NULL;
  gps.lon = DEG_NULL;
  if (gps.module.isEnabled) {
    gps.module.isInit = checkGpsConnected(gps.stream);
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

void setupOrientationModule(Orientation &m, byte address) {
  m.module.isEnabled = orientationEnabled;
  m.address = address;
  m.sensor = Adafruit_BNO055(55, m.address);
  m.x = DEG_NULL;
  m.y = DEG_NULL;
  m.z = DEG_NULL;
  m.qw = DEG_NULL;
  m.qx = DEG_NULL;
  m.qy = DEG_NULL;
  m.qz = DEG_NULL;
  if (m.module.isEnabled && m.sensor.begin()) {
    m.module.isInit = true;
    m.sensor.setExtCrystalUse(true);
  }
  m.module.hasData = m.module.isInit;
}

void loop() {
  takeCommand(Serial);
  if (mode == MODE_STREAM_ALL) {
    readAll();
    writeAll(Serial);
  } else if (mode == MODE_STREAM_GPS && gps.module.isEnabled) {
    pipeStream(Serial, gps.stream);
  }
  // take command twice, since readAll takes time.
  takeCommand(Serial);
  delay(loopDelay);
}

void takeCommand(Stream &ioStream) {
  takeCommand(ioStream, ioStream);
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
  long cmdId = command.toInt();

  if (cmdId < 70) {

    if (!mcc.module.isInit) {
      input.readStringUntil(';');
      writeAck(id, output, true);
      output.write("=01\n");
      return;
    }
    // forward to motorcontroller

    readMccState(mcc);

    if (mcc.state != STATE_READY) {
      input.readStringUntil(';');
      writeAck(id, output, true);
      output.write("=04\n");
      return;
    }

    String mcBody = String(":");
    mcBody.concat(command);
    mcBody.concat(" ");
    mcBody.concat(input.readStringUntil(';'));
    mcBody.concat(";");
    mcc.stream.print(mcBody);

    // output.println(mcBody);
    // output.write('\n');

    int d = 0;
    while (!mcc.stream.available()) {
      delay(1);
      d += 1;
      if (d > mcc.writeTimeout) {
        writeAck(id, output, true);
        output.write("=02\n");
        return;
      }
    }

    String res = mcc.stream.readStringUntil('\n');
    writeAck(id, output, true);
    output.print(res);
    output.write('\n');
 
    return;
  }

  writeAck(id, output, true);

  if (cmdId == 71) {
    // set mode
    byte newMode = input.readStringUntil(';').toInt();
    if (newMode < 1 || newMode > maxMode) {
      output.write("=49\n");
      return;
    }
    mode = newMode;
    output.write("=00\n");
  } else if (cmdId == 73) {
    // set loop delay
    long newValue = input.readStringUntil(';').toInt();
    if (newValue < 1) {
      output.write("=49\n");
      return;
    }
    loopDelay = newValue;
    output.write("=00\n");
  } else if (cmdId == 74) {
    // set mci check interval
    long newValue = input.readStringUntil(';').toInt();
    if (newValue < 1) {
      output.write("=49\n");
      return;
    }
    mci.checkInterval = newValue;
    output.write("=00\n");
  } else if (cmdId == 75) {
    // Reinit MCC & MCI module
    input.readStringUntil(';');
    if (mcc.module.isEnabled) {
      mcc.module.isInit = checkMccConnected(mcc.stream);
    }
    if (mci.module.isEnabled) {
      mci.module.isInit = checkMciConnected(mci);
    }
    output.write("=00\n");
  } else if (cmdId == 76) {
    // Send motor stop signal
    input.readStringUntil(';');
    digitalWrite(mStopPin, HIGH);
    output.write("=00\n");
    delay(100);
    digitalWrite(mStopPin, LOW);
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
  if (ori.module.isInit) {
    writeModulePrefix(ori.module, output);
    writeOrientation(ori, output);
  }
  if (orf.module.isInit) {
    writeModulePrefix(orf.module, output);
    writeOrientation(orf, output);
  }
  if (gps.module.isInit) {
    writeModulePrefix(gps.module, output);
    writeGps(gps, output);
  }
  if (mag.module.isInit) {
    writeModulePrefix(mag.module, output);
    writeMag(mag, output);
  }
  if (mcc.module.isInit) {
    writeModulePrefix(mcc.module, output);
    writeMccStatus(mcc, output);
  }
  if (mci.module.isInit) {
    writeModulePrefix(mci.module, output);
    writeMciStatus(mci, output);
  }
  output.write('\n');
}

void writeModulePrefix(Module &m, Stream &output) {
  output.write('\n');
  output.write(m.label);
  output.write(':');
}

void writeModules(Stream &output) {
  boolean doPrefix = false;
  writeModuleLabel(ori.module, output, doPrefix);
  writeModuleLabel(orf.module, output, doPrefix);
  writeModuleLabel(gps.module, output, doPrefix);
  writeModuleLabel(mag.module, output, doPrefix);
  writeModuleLabel(mcc.module, output, doPrefix);
  writeModuleLabel(mci.module, output, doPrefix);
}

void writeModuleLabel(Module &m, Stream &output, bool &doPrefix) {
  if (m.hasData) {
    if (doPrefix) {
      output.write('|');
    }
    output.write(m.label);
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
}

void readAll() {
  readMccState(mcc);
  if (mcc.module.isInit) {
    readMccStatus(mcc);
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
    readGps(gps);
  }
  if (mag.module.isInit) {
    readMag(mag);
  }
}

void readMccState(MotorControllerSerial &m) {
  m.state = digitalRead(m.statePin);
}

void readMccStatus(MotorControllerSerial &m) {
  if (m.state != STATE_READY) {
    return;
  }
  // Send command Get full status
  m.stream.write(":18 ;");
  int d = 0;
  while (!m.stream.available()) {
    delay(1);
    d += 1;
    if (d > m.readTimeout) {
      return;
    }
  }
  String codeStr = m.stream.readStringUntil(';');
  if (!codeStr.equals("=00")) {
    return;
  }
  m.statusStr = m.stream.readStringUntil('\n');
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
  if (!checkMciConnected(m)) {
    return;
  }
  mainWire.requestFrom(m.address, mciMessageLength);
  char buf[19];
  byte i = 0;
  while (mainWire.available()) {
    char c = mainWire.read();
    buf[i] = c;
    i++;
    if (c == 13) {
      break;
    }
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

void readGps(Gps &g) {
  // we have to set a delay here after listen, otherwise we
  // never read values. this value MUST be at least 50ms.
  delay(60);
  while (g.stream.available()) {
    if (g.helper.encode(g.stream.read())) {
      g.helper.f_get_position(&g.lat, &g.lon);
    }
  }
}

void pipeStream(Stream &output, Stream &source) {
  while (source.available()) {
    output.write(source.read());
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
  mainWire.beginTransmission(m.checkAddress);
  return mainWire.endTransmission() == 0;
}

boolean checkMciConnected(MotorControllerI2C m) {
  mainWire.beginTransmission(m.address);
  return mainWire.endTransmission() == 0;
}

// send a status request with a 2 second timeout
boolean checkMccConnected(Stream &stream) {
  stream.write(":18 ;");
  // timeout 2 seconds
  int d = 0;
  while (!stream.available()) {
    delay(1);
    d += 1;
    if (d > 2000) {
      return false;
    }
  }
  stream.readStringUntil('\n');
  return true;
}

boolean checkGpsConnected(Stream &stream) {
  // timeout 3 seconds
  int d = 0;
  while (!stream.available()) {
    delay(1);
    d += 1;
    if (d > 3000) {
      return false;
    }
  }
  return true;
}
