/*
 * Commands
 * 
 * 1 - Move single motor n steps in a given direction
 * 
 *  :<id>:1 <motorId> <direction> <steps>;
 * 
 * 2 - Set max speed for a motor
 * 
 *  :<id>:2 <motorId> <speed>;
 * 
 * 3 - Set acceleration for a motor
 * 
 *  :<id>:3 <motorId> <acceleration>;
 *
 * 6 - Home a single motor
 *
 *  :<id>:6 <motorId>;
 *
 * 7 - Home all motors
 *
 *  :<id>:7;
 *
 * 8 - End a single motor
 *
 *  :<id>:8 <motorId>;
 *
 * 9 - End all motors
 *
 *  :<id>:9;
 *
 * 10 - Move both motors by steps. Last param is arrive at same time.
 *
 *  :<id>:10 <direction_1> <steps_1> <direction_2> <steps_2> <1|0>;
 *
 * 17 - Set limit switch enablement for a motor
 *
 *  :<id>:17 <motorId> <1|0>;
 *
 * 18 - Set homing speed for a motor
 *
 *  :<id>:18 <motorId> <speed>;
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
 * 75 - Reinit MCI module
 *
 *  :<id>:75;
 *
 * 76 - Stop all motors
 *
 *  :<id>:76;
 *
 * 77 - Reset motorcontroller
 *
 *  :<id>:77;
 */
#include <Adafruit_BNO055.h>
#include <Adafruit_GPS.h>
#include <Adafruit_HMC5883_U.h>
#include <utility/imumaths.h>
#include <Wire.h>

/******************************************/
/* Hardware Enable                        */
/******************************************/
#define gpsEnabled true
#define magEnabled true
#define mcResetPin D10

/******************************************/
/* Constants                              */
/******************************************/
#define BAUD_RATE 115200L
#define DEG_NULL 1000.00
#define POS_NULL 10000000UL
#define MAX_MOTORID 4

/******************************************/
/* I2C                                    */
/******************************************/
#define mainWire Wire1
#define SDA_MAIN D4
#define SCL_MAIN D5

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

typedef enum {
  OK = 0,
  MODULE_UNAVAILABLE = 32,
  MALFORMED_COMMAND = 40,
  UNKNOWN_COMMAND = 44,
  INVALID_MOTORID = 45,
  INVALID_DIRECTION = 46,
  INVALID_DISTANCE = 47,
  INVALID_SPEED_OR_ACCELERATION = 48,
  INVALID_OTHER = 49,
} ResCode;

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
/* Motor Controller I2C                   */
/******************************************/
struct MotorControllerI2C {
  Module module;
  byte address;
  byte numMotors;
  TwoWire& wire = mainWire;
  unsigned long checkInterval = 500;
  unsigned long lastCheckTime;
  String statusStr;
};

MotorControllerI2C mci = {
  {"MCI", true},
  0x9,
  2,
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
  Serial.begin(BAUD_RATE);
  mainWire.setSDA(SDA_MAIN);
  mainWire.setSCL(SCL_MAIN);
  mainWire.begin();
  setupModules();
}

void setupModules() {
  setupOrientationModule(ori);
  setupOrientationModule(orf);
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
    sendCommandErr(id, input, output, MALFORMED_COMMAND);
    return;
  }

  const long cmdId = input.parseInt(SKIP_NONE);

  if (cmdId == 1 || cmdId == 2 || cmdId == 3 || cmdId == 6 || cmdId == 8 || cmdId == 17 || cmdId == 18) {
    // Single motor command
    byte reg = 0x2 << 0x6;
    byte buf[4];
    boolean hasBuf = false;
    // 1: Move a motor n steps in a direction
    // 2: Set max speed for motor
    // 3: Set acceleration for motor
    // 6: Home a motor
    // 8: End a motor
    // 17: Set limit switch enablement for a motor
    // 18: Set homing speed for motor
    const byte motorId = readMotorIdFromInput(input);
    if (motorId == 0) {
      sendCommandErr(id, input, output, INVALID_MOTORID);
      return;
    }
    reg |= (motorId - 1) << 0x4;
    if (cmdId == 1) {
      // 1: Move a motor n steps in a direction
      // second param is direction 1: clockwise, 2: anti-clockwise
      const int dir = input.parseInt(SKIP_WHITESPACE);
      if (dir != 1 && dir != 2) {
        sendCommandErr(id, input, output, INVALID_DIRECTION);
        return;
      }
      reg |= 0x7 + dir;
    }
    if (cmdId == 6 || cmdId == 8) {
      input.readStringUntil(';');
      if (cmdId == 6) {
        reg |= 0x1;
      } else {
        reg |= 0x2;
      }
    } else if (cmdId == 17) {
      input.readStringUntil(' ');
      const int param = getBoolParam(input.readStringUntil(';'));
      if (param < 0) {
        sendCommandErr(id, input, output, INVALID_OTHER);
        return;
      }
      reg |= 0x4 - param;
    } else {
      const long value = input.parseInt(SKIP_WHITESPACE);
      if (value < 1) {
        sendCommandErr(id, input, output, cmdId == 1 ? INVALID_DISTANCE : INVALID_SPEED_OR_ACCELERATION);
        return;
      }
      input.readStringUntil(';');
      packLong(value, buf);
      hasBuf = true;
      if (cmdId == 2) {
        reg |= 0xa;
      } else if (cmdId == 3) {
        reg |= 0xb;
      } else if (cmdId == 18) {
        reg |= 0xc;
      }
    }
    if (!mci.module.isInit) {
      sendCommandErr(id, input, output, MODULE_UNAVAILABLE);
      return;
    }
    writeAck(id, output, true);
    output.write('=');
    mci.wire.beginTransmission(mci.address);
    mci.wire.write(reg);
    if (hasBuf) {
      mci.wire.write(buf, 4);
    }
    mci.wire.endTransmission();
    mci.wire.requestFrom(mci.address, 1);
    output.print(mci.wire.read());
    output.write('\n');
    return;
  }

  if (cmdId == 76 || cmdId == 7 || cmdId == 9) {
    input.readStringUntil(';');
    byte reg = 0x3 << 0x6;
    if (cmdId == 76) {
      reg |= 0x1;
    } else if (cmdId == 7) {
      reg |= 0x2;
    } else if (cmdId == 9) {
      reg |= 0x3;
    }
    if (!mci.module.isInit) {
      sendCommandErr(id, input, output, MODULE_UNAVAILABLE);
      return;
    }
    writeAck(id, output, true);
    output.write('=');
    mci.wire.beginTransmission(mci.address);
    mci.wire.write(reg);
    mci.wire.endTransmission();
    mci.wire.requestFrom(mci.address, 1);
    output.print(mci.wire.read());
    output.write('\n');
    return;
  }

  if (cmdId == 10) {
    byte reg = 0x3 << 0x6;
    byte buf1[4];
    byte buf2[4];
    // move both motors by steps
    // first param is direction_1
    const int dir1 = input.parseInt(SKIP_WHITESPACE);
    if (dir1 != 1 && dir1 != 2) {
      sendCommandErr(id, input, output, INVALID_DIRECTION);
      return;
    }
    // second param is steps_1
    const long howMuch1 = input.parseInt(SKIP_WHITESPACE);
    if (howMuch1 < 1) {
      sendCommandErr(id, input, output, INVALID_DISTANCE);
      return;
    }
    // third param is direction_2
    const int dir2 = input.parseInt(SKIP_WHITESPACE);
    if (dir2 != 1 && dir2 != 2) {
      sendCommandErr(id, input, output, INVALID_DIRECTION);
      return;
    }
    // fourth param is steps_2
    const long howMuch2 = input.parseInt(SKIP_WHITESPACE);
    if (howMuch2 < 1) {
      sendCommandErr(id, input, output, INVALID_DISTANCE);
      return;
    }
    // fifth param is isSameTime
    input.readStringUntil(' ');
    const int isSameTime = getBoolParam(input.readStringUntil(';'));
    if (isSameTime < 0) {
      sendCommandErr(id, input, output, INVALID_OTHER);
      return;
    }
    if (!isSameTime) {
      if (dir1 == 1 && dir2 == 1) {
        reg |= 0x11;
      } else if (dir1 == 2 && dir2 == 2) {
        reg |= 0x12;
      } else if (dir1 == 1 && dir2 == 2) {
        reg |= 0x13;
      } else {
        reg |= 0x14;
      }
    } else {
      if (dir1 == 1 && dir2 == 1) {
        reg |= 0x15;
      } else if (dir1 == 2 && dir2 == 2) {
        reg |= 0x16;
      } else if (dir1 == 1 && dir2 == 2) {
        reg |= 0x17;
      } else {
        reg |= 0x18;
      }
    }
    if (!mci.module.isInit) {
      sendCommandErr(id, input, output, MODULE_UNAVAILABLE);
      return;
    }
    packLong(howMuch1, buf1);
    packLong(howMuch2, buf2);
    writeAck(id, output, true);
    output.write('=');
    mci.wire.beginTransmission(mci.address);
    mci.wire.write(reg);
    mci.wire.write(buf1, 4);
    mci.wire.write(buf2, 4);
    mci.wire.endTransmission();
    mci.wire.requestFrom(mci.address, 1);
    output.print(mci.wire.read());
    output.write('\n');
    return;
  }

  writeAck(id, output, true);

  if (cmdId == 71) {
    // set mode
    const byte newMode = input.parseInt(SKIP_WHITESPACE);
    if (newMode < 1 || newMode > maxMode) {
      sendCommandErr(input, output, INVALID_OTHER);
      return;
    }
    mode = newMode;
    sendCommandErr(input, output, OK);
  } else if (cmdId == 73) {
    // set loop delay
    long newValue = input.parseInt(SKIP_WHITESPACE);
    if (newValue < 1) {
      sendCommandErr(input, output, INVALID_OTHER);
      return;
    }
    streamDelay = newValue;
    sendCommandErr(input, output, OK);
  } else if (cmdId == 74) {
    // set mci check interval
    long newValue = input.parseInt(SKIP_WHITESPACE);
    if (newValue < 1) {
      sendCommandErr(input, output, INVALID_OTHER);
      return;
    }
    mci.checkInterval = newValue;
    sendCommandErr(input, output, OK);
  } else if (cmdId == 75) {
    // Reinit MCI module
    mci.module.isInit = checkMciConnected(mci);
    sendCommandErr(input, output, OK);
  } else if (cmdId == 77) {
    // Reset motorcontroller
    tripResetPin(mcResetPin);
    sendCommandErr(input, output, OK);
  } else {
    sendCommandErr(input, output, UNKNOWN_COMMAND);
  }
}

void sendCommandErr(const long &id, Stream &input, Stream &output, const ResCode &errCode) {
  writeAck(id, output, true);
  sendCommandErr(input, output, errCode);
}

void sendCommandErr(Stream &input, Stream &output, const ResCode &errCode) {
  if (input.available() && input.findUntil(";", ":")) {
    input.readStringUntil(';');
  }
  output.write('=');
  output.print(errCode);
  output.write('\n');
}

byte readMotorIdFromInput(Stream &input) {
  const byte motorId = input.parseInt(SKIP_WHITESPACE);
  if (motorId > 0 && motorId <= MAX_MOTORID) {
    return motorId;
  }
  return 0;
}

int getBoolParam(String value) {
  if (value.length() == 1) {
    const char c = value.charAt(0);
    if (c == 'T' || c == '1') {
      return 1;
    }
    if (c == 'F' || c == '0') {
      return 0;
    }
  }
  return -1;
}

void tripResetPin(byte pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delay(100);
  digitalWrite(pin, HIGH);
  pinMode(pin, INPUT_PULLUP);
}

void writeAck(const long &id, Stream &output, boolean withColon) {
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
  output.print((int) o.isCalibrated);
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

// read I2C
void readMciStatus(MotorControllerI2C &m) {
  if (millis() - m.lastCheckTime < m.checkInterval) {
    return;
  }
  m.lastCheckTime = millis();
  if (!checkMciConnected(m)) {
    return;
  }
  String s;
  unsigned int flag = 0;
  for (byte mId = 1; mId <= m.numMotors; ++mId) {
    m.wire.beginTransmission(m.address);
    m.wire.write((1 << 0x6) | ((mId - 1) << 0x4) | 0x0);
    m.wire.endTransmission();
    m.wire.requestFrom(m.address, 1);
    byte mFlag = m.wire.read();
    flag |= mFlag << (0x4 + 0x8 * (mId - 1));
    // set bit 0x1 of overall flag (mc busy) if motor is moving (bit 0x4)
    flag |= (mFlag & 0x4) == 0x4;
  }
  s.concat(String(flag, HEX));
  // read positions
  byte buf[4];
  for (byte mId = 1; mId <= m.numMotors; ++mId) {
    m.wire.beginTransmission(m.address);
    m.wire.write((1 << 0x6) | ((mId - 1) << 0x4) | 0x2);
    m.wire.endTransmission();
    m.wire.requestFrom(m.address, 4);
    m.wire.readBytes(buf, 4);
    s.concat('|');
    s.concat(String(unpackLong(buf), HEX));
  }
  if ((flag & 0x1) == 0x0) {
    // mc not busy, read extended data
    for (byte mId = 1; mId <= m.numMotors; ++mId) {
      for (byte reg = 0x3; reg <= 0xb; ++reg) {
        m.wire.beginTransmission(m.address);
        m.wire.write((1 << 0x6) | ((mId - 1) << 0x4) | reg);
        m.wire.endTransmission();
        m.wire.requestFrom(m.address, 4);
        m.wire.readBytes(buf, 4);
        s.concat('|');
        s.concat(String(unpackLong(buf), HEX));
      }
    }
  }
  m.statusStr = s;
  m.module.hasData = true;
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
  m.wire.beginTransmission(m.address);
  return m.wire.endTransmission() == 0;
}

void packLong(const unsigned long value, byte *buf) {
  buf[0] = (byte) ((value >> 0x18) & 0xff);
  buf[1] = (byte) ((value >> 0x10) & 0xff);
  buf[2] = (byte) ((value >> 0x8) & 0xff);
  buf[3] = (byte) (value & 0xff);
}

unsigned long unpackLong(byte *buf) {
  return (
    buf[0] << 0x18 |
    buf[1] << 0x10 |
    buf[2] << 0x8 |
    buf[3]
  );
}
