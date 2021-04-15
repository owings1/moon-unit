/*
 * Commands
 *
 * 01 - Set mode
 *
 *  :<id>:01 <mode>;
 *
 */
#include <SoftwareSerial.h> 
#include <TinyGPS.h>

// Modes
// 1: quiet
// 2: stream all
// 3: stream gps
byte mode = 1;

SoftwareSerial gpsSerial(8,9); //rx, tx
TinyGPS gps;
float gps_lat = -1;
float gps_lon = -1;

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
}

void loop() {

  if (mode == 2) {
    readAll();
    writeAll(Serial);
  } else if (mode == 3) {
    streamGps(Serial);
  }
  takeCommand(Serial, Serial);
  delay(1000);
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

  output.write("ACK:");
  output.print(id, DEC);
  output.write(':');


  if (input.read() != ':') {
    output.write("=40\n");
    return;
  }

  String command = input.readStringUntil(' ');

  if (command.equals("01")) {
    // set mode
    byte newMode = input.readStringUntil(';').toInt();
    if (newMode != 1 && newMode != 2) {
      output.write("=49\n");
      return;
    }
    mode = newMode;
    output.write("=00\n");
  } else {
    output.write("=44\n");
  }
}

void writeAll(Stream &output) {
  output.write("GPS:");
  writeLatLong(output);
  output.write("\n");
}

void writeLatLong(Stream &output) {
  output.print(gps_lat, 6);
  output.write('|');
  output.print(gps_lon, 6);
}

void readAll() {
  readGps();
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