#include <SoftwareSerial.h> 
#include <TinyGPS.h>

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

  readGps();
  if (mode == 2) {
    writeAll(Serial);
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
  output.print(id, 0);


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
  output.write('|');
}

void readGps() {
  if (gpsSerial.available()) {
    if (gps.encode(gpsSerial.read())) {
      gps.f_get_position(&gps_lat, &gps_lon);
    }
  }
}
