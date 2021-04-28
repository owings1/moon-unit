// attempt make an I2C slave device that transmits the
// debounced rotary encoder position
#include <Wire.h>

#define WIRE_ADDRESS 0x8

// encoder CLK
#define pinLeft 2
// encoder DT
#define pinRight 3
#define pinButton 4

volatile byte mode = 0;

int num = 0;    // track absolute position
volatile int change = 0; // change is reset when 0x0 is sent through I2C
volatile boolean isPressed = false;

// rotary reading from: https://www.pinteric.com/rotary.html

uint8_t lrmem = 3;
int lrsum = 0;

void setup() {

  Serial.begin(9600);

  pinMode(pinLeft, INPUT);
  pinMode(pinRight, INPUT);
  pinMode(pinButton, INPUT);

  pinMode(pinLeft, INPUT_PULLUP);
  pinMode(pinRight, INPUT_PULLUP);
  pinMode(pinButton, INPUT_PULLUP);

  // join bus
  Wire.begin(WIRE_ADDRESS);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
}

void loop() {

  int8_t res = rotary();
  
  isPressed = digitalRead(pinButton) == LOW;

  if (res != 0) {
    num += res;
    change += res;
  }
  
  /*
  // debug
  if (isPressed || change) {
    delay(200);
    Serial.println(getChangeByte(), BIN);
    change = 0;
  }
  */
}

void requestEvent() {
  if (mode == 0) {
    byte changeByte = getChangeByte();
    Wire.write(changeByte);
    change = 0;
  }
}

void receiveEvent(int howMany) {
  mode = Wire.read();
}

byte getChangeByte() {
  // first bit MSB is button pressed flag
  byte value = isPressed << 7;
  // second bit is sign, positive=1 negative=0
  if (change >= 0) {
    value |= 1 << 6;
    // last six bits is quantity
    if (change < 64) {
      value += change;
    } else {
      // don't overfill
      value += 63;
    }
  } else {
    if (change > -64) {
      value -= change;
    } else {
      // don't overfill
      value += 63;
    }
  }
  return value;
}

// from: https://www.pinteric.com/rotary.html
int8_t rotary() {
  static int8_t TRANS[] = {0,-1,1,14,1,0,14,-1,-1,14,0,1,14,1,-1,0};
  int8_t l, r;
  
  l = digitalRead(pinLeft);
  r = digitalRead(pinRight);
  
  lrmem = ((lrmem & 0x03) << 2) + 2*l + r;
  lrsum = lrsum + TRANS[lrmem];
  // encoder not in the neutral state
  if (lrsum % 4 != 0) {
    return 0;
  }
  // encoder in the neutral state
  if (lrsum == 4) {
    lrsum = 0;
    return 1;
  }
  if (lrsum == -4) {
    lrsum = 0;
    return -1;
  }
  // lrsum > 0 if the impossible transition
  lrsum = 0;
  return 0;
}
