/*
 * Commands
 *
 * 71 - Set mode
 *
 *  :<id>:71 <mode>;
 *
 * 73 - Set stream delay in milliseconds
 *
 *  :<id>:73 <milliseconds>;
 *
 * 74 - Set MCI busy check interval in milliseconds
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
 *
 * 77 - Reset motorcontroller
 *
 *  :<id>:77 ;
 *
 * 78 - Reset self
 *
 *  :<id>:78 ;
 */
#include <Adafruit_BNO055.h>
#include <Adafruit_GPS.h>
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
#define magEnabled true
#define mcBusyPin D8
#define mcStopPin D9
#define mcResetPin D10
#define selfResetPin D18

/******************************************/
/* Constants                              */
/******************************************/
#define BAUD_RATE 115200L
#define DEG_NULL 1000.00
#define POS_NULL 10000000UL

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
// Stream delay in milliseconds
unsigned long streamDelay = 50;
unsigned long lastStreamAt = 0;

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
#define mccSerial Serial2
#define mccBaudRate 9600L

struct MotorControllerSerial {
  Module module;
  Stream &stream;
  unsigned long readTimeout = 250UL;
  unsigned long writeTimeout = 10000UL;
};

MotorControllerSerial mcc = {
  {"MCC", true},
  mccSerial,
};

/******************************************/
/* Motor Controller I2C                   */
/******************************************/
#define mciMessageLengthFull 84
#define mciMessageLengthShort 20

struct MotorControllerI2C {
  Module module;
  byte address;
  unsigned long checkInterval = 2000UL;
  unsigned long lastCheckTime;
  String statusStr;
};

MotorControllerI2C mci = {
  {"MCI", true},
  0x9,
};

/******************************************/
/* Orientation Sensor                     */
/******************************************/

struct Orientation {
  Module module;
  byte address;
  // https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/arduino-code
  Adafruit_BNO055 sensor;

  boolean isCalibrated;

  float x = DEG_NULL;
  float y = DEG_NULL;
  float z = DEG_NULL;
  float qw = DEG_NULL;
  float qx = DEG_NULL;
  float qy = DEG_NULL;
  float qz = DEG_NULL;

  int8_t temp;

  byte cal_system;
  byte cal_gyro;
  byte cal_accel;
  byte cal_mag;
};

Orientation ori = {{"ORI", true}, 0x28};
Orientation orf = {{"ORF", false}, 0x29};

/******************************************/
/* GPS                                    */
/******************************************/
struct Gps {
  Module module;
  Adafruit_GPS helper;
  byte address;
  unsigned long checkInterval = 1000;
  unsigned long lastCheckTime;
  boolean fix;
  float lat;
  float lon;
  float angle;
};

Gps gps = {{"GPS", true}, Adafruit_GPS(&mainWire), 0x10};

/******************************************/
/* Magnetometer                           */
/******************************************/
// #define magCheckAddress (0x3C >> 1)
// #define magDeviceId 49138
//#define defaultDeclinationRad 0.23

struct Mag {
  Module module;
  int deviceId; // set a unique id
  // the address is hard-coded in the sensor library, so
  // this is just for checking whether it is connected.
  byte checkAddress;
  Adafruit_HMC5883_Unified sensor;
  //float declinationRad;
  // micro-Tesla (uT)
  float x;
  float y;
  float z;
  // degrees
  float heading = DEG_NULL;
};

Mag mag = {{"MAG", true}, 49138, (0x3C >> 1)};

/******************************************/
/* Setup                                  */
/******************************************/
void setup() {
  pinMode(mcBusyPin, INPUT_PULLDOWN);
  pinMode(mcStopPin, OUTPUT);
  Serial.begin(BAUD_RATE);
  mainWire.setSDA(SDA_MAIN);
  mainWire.setSCL(SCL_MAIN);
  mainWire.begin();
  mccSerial.begin(mccBaudRate);
  setupModules();
}

void setupModules() {
  setupOrientationModule(ori);
  setupOrientationModule(orf);
  if (mcc.module.isEnabled) {
    mcc.module.isInit = checkMccConnected(mcc.stream);
  }
  if (mci.module.isEnabled) {
    mci.module.isInit = checkMciConnected(mci);
  }
  if (gps.module.isEnabled) {
    gps.module.isInit = gps.helper.begin(gps.address);
    if (gps.module.isInit) {
      gps.helper.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
      gps.helper.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    }
  }
  gps.module.hasData = gps.module.isInit;

  mag.sensor = Adafruit_HMC5883_Unified(mag.deviceId);
  if (mag.module.isEnabled) {
    mag.module.isInit = mag.sensor.begin() && checkMagConnected(mag);
  }
  mag.module.hasData = mag.module.isInit;
}

void setupOrientationModule(Orientation &m) {
  m.sensor = Adafruit_BNO055(55, m.address);
  if (m.module.isEnabled && m.sensor.begin()) {
    m.module.isInit = true;
    m.sensor.setExtCrystalUse(true);
  }
  m.module.hasData = m.module.isInit;
}

void loop() {
  takeCommand(Serial);
  if (mode == MODE_STREAM_ALL) {
    if (lastStreamAt + streamDelay < millis()) {
      lastStreamAt = millis();
      readAll();
      writeAll(Serial);
    }
  }
  if (gps.module.isInit) {
    if (gps.helper.available()) {
      byte c = gps.helper.read();
      if (mode == MODE_STREAM_GPS) {
        Serial.write(c);
      }
    }
  }
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

  const long id = input.parseInt(SKIP_NONE);

  if (input.read() != ':') {
    writeAck(id, output, true);
    sendCommandErr(input, output, "40");
    return;
  }

  const long cmdId = input.parseInt(SKIP_NONE);
  if (cmdId < 70) {

    if (!mcc.module.isInit) {
      writeAck(id, output, true);
      sendCommandErr(input, output, "01");
      return;
    }
    // forward to motorcontroller

    if (digitalRead(mcBusyPin)) {
      writeAck(id, output, true);
      sendCommandErr(input, output, "04");
      return;
    }

    String mcBody = String(":");
    mcBody.concat(String(cmdId));
    mcBody.concat(input.readStringUntil(';'));
    mcBody.concat(";");
    mcc.stream.print(mcBody);

    int d = 0;
    while (!mcc.stream.available()) {
      delay(1);
      d += 1;
      if (d > mcc.writeTimeout) {
        writeAck(id, output, true);
        sendCommandErr(input, output, "02");
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
    const byte newMode = input.parseInt(SKIP_WHITESPACE);
    if (newMode < 1 || newMode > maxMode) {
      sendCommandErr(input, output, "49");
      return;
    }
    input.readStringUntil(';');
    mode = newMode;
    output.write("=00\n");
  } else if (cmdId == 73) {
    // set loop delay
    long newValue = input.parseInt(SKIP_WHITESPACE);
    if (newValue < 1) {
      sendCommandErr(input, output, "49");
      return;
    }
    input.readStringUntil(';');
    streamDelay = newValue;
    output.write("=00\n");
  } else if (cmdId == 74) {
    // set mci check interval
    long newValue = input.parseInt(SKIP_WHITESPACE);
    if (newValue < 1) {
      sendCommandErr(input, output, "49");
      return;
    }
    input.readStringUntil(';');
    mci.checkInterval = newValue;
    output.write("=00\n");
  } else if (cmdId == 75) {
    // Reinit MCC & MCI module
    input.readStringUntil(';');
    mccSerial.begin(mccBaudRate);
    mcc.module.isInit = checkMccConnected(mcc.stream);
    mci.module.isInit = checkMciConnected(mci);
    output.write("=00\n");
  } else if (cmdId == 76) {
    // Send motor stop signal
    input.readStringUntil(';');
    digitalWrite(mcStopPin, HIGH);
    delay(100);
    digitalWrite(mcStopPin, LOW);
    output.write("=00\n");
  } else if (cmdId == 77) {
    // Reset motorcontroller
    input.readStringUntil(';');
    tripResetPin(mcResetPin);
    output.write("=00\n");
  } else if (cmdId == 78) {
    // Reset self
    input.readStringUntil(';');
    output.write("=00\n");
    tripResetPin(selfResetPin);
  } else {
    sendCommandErr(input, output, "44");
  }
}

void sendCommandErr(Stream &input, Stream &output, const char *errCode) {
  if (input.available() && input.findUntil(";", ":")) {
    input.readStringUntil(';');
  }
  output.write('=');
  output.write(errCode);
  output.write('\n');
}

void tripResetPin(byte pin) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    delay(100);
    digitalWrite(pin, HIGH);
    pinMode(pin, INPUT_PULLUP);
}

void writeAck(const long id, Stream &output, boolean withColon) {
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
  output.print((int) g.fix);
  output.write('|');
  output.print(g.lat, 4);
  output.write('|');
  output.print(g.lon, 4);
  output.write('|');
  output.print(g.angle, 4);
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

// should only do this occasionally, since it will slow
// motor operations down.
// read I2C
void readMciStatus(MotorControllerI2C &m) {
  boolean mcBusy = digitalRead(mcBusyPin);
  if (mcBusy && millis() - m.lastCheckTime < m.checkInterval) {
    return;
  }
  m.lastCheckTime = millis();
  if (!checkMciConnected(m)) {
    return;
  }
  mainWire.requestFrom(m.address, mcBusy ? mciMessageLengthShort : mciMessageLengthFull);
  m.statusStr = mainWire.readStringUntil('\n');
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
  if (millis() - g.lastCheckTime < g.checkInterval) {
    return;
  }
  g.lastCheckTime = millis();
  g.helper.parse(g.helper.lastNMEA());
  g.fix = g.helper.fix;
  if (g.fix) {
    g.lat = g.helper.latitude;
    g.lon = g.helper.longitude;
    g.angle = g.helper.angle;
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
  stream.write(":14 ;");
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

