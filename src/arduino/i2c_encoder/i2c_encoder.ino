// attempt make an I2C slave device that transmits the
// debounced rotary encoder position
#include <Wire.h>

#define WIRE_ADDRESS 0x8

// rotary reading from: https://www.pinteric.com/rotary.html

#define LEFT 5//2
#define RIGHT 6//3
#define PUSH 7//4

uint8_t lrmem = 3;
int lrsum = 0;
int num = 0;
boolean push = false;

void setup() {

  Serial.begin(9600);

  pinMode(LEFT, INPUT);
  pinMode(RIGHT, INPUT);
  pinMode(PUSH, INPUT);

  pinMode(LEFT, INPUT_PULLUP);
  pinMode(RIGHT, INPUT_PULLUP);
  pinMode(PUSH, INPUT_PULLUP);

  // join bus
  Wire.begin(WIRE_ADDRESS);
  Wire.onRequest(requestEvent);
}

void loop() {

  int8_t res = rotary();
  
  push = digitalRead(PUSH) == LOW;
  if (res!=0) {
    num += res;
    writeStatus(Serial);
  }
}

void requestEvent() {
  writeStatus(Wire);
}

void writeStatus(Stream &output) {
  output.write('^');
  output.print(num);
  output.write('|');
  output.print(push);
  output.write("\n");
}

int8_t rotary() {
  static int8_t TRANS[] = {0,-1,1,14,1,0,14,-1,-1,14,0,1,14,1,-1,0};
  int8_t l, r;
  
  l = digitalRead(LEFT);
  r = digitalRead(RIGHT);
  
  lrmem = ((lrmem & 0x03) << 2) + 2*l + r;
  lrsum = lrsum + TRANS[lrmem];
  // encoder not in the neutral state
  if(lrsum % 4 != 0) return(0);
  // encoder in the neutral state
  if (lrsum == 4) {
    lrsum=0;
    return(1);
  }
  if (lrsum == -4) {
    lrsum=0;
    return(-1);
  }
  // lrsum > 0 if the impossible transition
  lrsum=0;
  return(0);
}
