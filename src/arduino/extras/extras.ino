

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#include <RotaryEncoder.h>
#include <LiquidCrystal_I2C.h>
// Rotary Encoder
#define encoderEnabled false
// LCD Display
#define lcdEnabled false
// Orientation Sensor
#define orientationEnabled true

// Rotary Encoder

// These cannot be changed without addressing interrupt as below
#define encoderPin1 A2
#define encoderPin2 A3

RotaryEncoder encoder(encoderPin1, encoderPin2);

// This cannot be changed since we are attaching to interrupt 0 (pin 2)
#define encoderSwitchPin 2
#define debouncing_time 15 //Debouncing Time in Milliseconds

// The encoder switch changes the "program"
volatile unsigned long debounceLastMicros;
#define numPrograms 3
volatile int programNum = 1;

// LCD Display

// https://github.com/fmalpartida/New-LiquidCrystal
LiquidCrystal_I2C  lcd(0x27,2,1,0,4,5,6,7); // 0x27 is the I2C bus address for an unmodified module

boolean displaySleepEnabled = true;
unsigned long displaySleepTimeout = 10000L;
unsigned long lastDisplayActionTime = millis();
volatile boolean displayUpdateNeeded = false;
boolean isDisplayLightOn = false;


// Orientation sensor

// https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/arduino-code
Adafruit_BNO055 bno = Adafruit_BNO055(55);
// whether it is initialized properly
boolean isOrientation = false;
boolean isOrientationCalibrated = false;
// latest read
float orientation_x;
float orientation_y;
float orientation_z;
// calibration
// 
uint8_t cal_system;
uint8_t cal_gyro;
uint8_t cal_accel;
uint8_t cal_mag;




void setup() {
  //setupStatePin();
  //setState(STATE_BUSY);
  //Serial.begin(baudRate);
  //setupMotors();
  setupEncoder();
  setupDisplay();
  setupOrientation();
  //setupStopPin();
  //setState(STATE_READY);
}

void loop() {


  static int pos = 0;

  if (encoderEnabled) {
    int newPos = encoder.getPosition();
  
    if (pos != newPos) {
      if (shouldTakeInput()) {
        takeKnobInput(pos, newPos);
      }
      // if we are not taking input, this will ignore changes to knob position
      pos = newPos;
    }
  }

  //if (runMotorsIfNeeded()) {
  //  setState(STATE_BUSY);
  //} else {
    updateDisplay();
    checkDisplaySleep();
    //checkMotorSleep();
    readOrientation();
    //setState(STATE_READY);
    //takeCommand(Serial, Serial);
    //}
}

/*
 * 05 - RETIRED - Get state of limit switches and stop pin
 *
 *  :05 ;
 *
 *    example response:
 *      =00;TFFT|F
else if (command.equals("05")) {

    // read limit switch and stop pin states

    input.readStringUntil(';');

    char states[7] = {
      isLimit_m1_cw  ? 'T' : 'F',
      isLimit_m1_acw ? 'T' : 'F',
      isLimit_m2_cw  ? 'T' : 'F',
      isLimit_m2_acw ? 'T' : 'F',
      '|',
      shouldStop ? 'T' : 'F'
    };

    output.write("=00;");
    output.write(states);
    output.write("\n");

  }*/
/*

 * 12 - RETIRED - Get motor positions
 *
 *  :12 <format>;
 *
 *    example responses:
 *      =00;8500|1200
 *      =00;1000|130.195
 *      =00;1000|1000
 *
 *    NB: 1000 degrees means no value
 *        -1 steps means no value

else if (command.equals("12")) {

    // get motor positions

    // param is format
    int format = input.readStringUntil(';').toInt();

    if (format != 1 && format != 2) {
      output.write("=49\n");
      return;
    }

    output.write("=00;");

    writePositions(output, format);


    output.write("\n");

  }*/ 
/*
 * 15 - RETIRED - Get position in degrees, followed by limits enabled
 *
 *  :15 ;
else if (command.equals("15")) {

   // get position in degrees, followed by limits enabled
   input.readStringUntil(';');

   output.write("=00;");

   writePositions(output, 2);

   output.write('|');
   output.write(limitsEnabled_m1 ? 'T' : 'F');

   output.write('|');
   output.write(limitsEnabled_m2 ? 'T' : 'F');

   output.write("\n");

 }*/
void writeOrientation(Stream &output) {
  if (isOrientation) {
    writeString(output, String(orientation_x, 4));
    output.write('|');
    writeString(output, String(orientation_y, 4));
    output.write('|');
    writeString(output, String(orientation_z, 4));
  } else {
    output.write("?|?|?");
  }
  output.write('|');
  output.write(isOrientationCalibrated ? 'T' : 'F');
}

// ----------------------------------------------
// Rotary Encoder functions
// ----------------------------------------------

void takeKnobInput(int oldPos, int newPos) {
  // TODO: this is just a placeholder, write a real encoder UI.
  if (oldPos != newPos) {
    turnOnDisplayLight();
    if (newPos > oldPos) {
      jumpBoth(800 * programNum);
    } else {
      jumpBoth(-800 * programNum);
    }
    registerDisplayAction();
  }
}

boolean shouldTakeInput() {
  return stepper_m1.distanceToGo() == 0 && stepper_m2.distanceToGo() == 0;
}

void onEncoderSwitchChange() {
  if (!shouldTakeInput()) {
    // ignore switch input
    return;
  }
  if (programNum >= numPrograms) {
    programNum = 1;
  } else {
    programNum += 1;
  }
  displayUpdateNeeded = true;
  registerDisplayAction();
}

// ----------------------------------------------
// LCD display functions
// ----------------------------------------------

void updateDisplay() {
  if (lcdEnabled && shouldUpdateDisplay()) {
    lcd.setCursor(0, 0);
    lcd.print("Program: ");
    lcd.print(programNum);
    displayUpdateNeeded = false;
  }
}

boolean shouldUpdateDisplay() {
  return lcdEnabled && displayUpdateNeeded;
}

void turnOnDisplayLight() {
  if (lcdEnabled) {
    lcd.setBacklightPin(3, NEGATIVE);
  }
}

void turnOffDisplayLight() {
  if (lcdEnabled) {
    lcd.setBacklightPin(3, POSITIVE);
  }
}

void registerDisplayAction() {
  lastDisplayActionTime = millis();
}

void checkDisplaySleep() {
  if (!lcdEnabled || !displaySleepEnabled) {
    return;
  }
  unsigned long elapsed = millis() - lastDisplayActionTime;
  if (elapsed > displaySleepTimeout) {
    turnOffDisplayLight();
  } else {
    // for button, so we don't have to do this in an interrupt
    turnOnDisplayLight();
  }
}

// ----------------------------------------------
// Orientation functions
// ----------------------------------------------

void readOrientation() {
  if (!isOrientation) {
    return;
  }
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

void setupOrientation() {
  cal_system = cal_gyro = cal_accel = cal_mag = 0;
  if (!orientationEnabled) {
    return;
  }
  if (bno.begin()) {
    isOrientation = true;
    bno.setExtCrystalUse(true);
  }
}

void setupDisplay() {
  if (!lcdEnabled) {
    return;
  }
  lcd.begin(16, 2);
  lcd.clear();
  displayUpdateNeeded = true;
  turnOnDisplayLight();
  updateDisplay();
}

void setupEncoder() {

  if (!encoderEnabled){
    return;
  }

  pinMode(encoderSwitchPin, OUTPUT);
  // use pullup resistor
  digitalWrite(encoderSwitchPin, HIGH);
  attachInterrupt(digitalPinToInterrupt(encoderSwitchPin), debounceInterrupt, RISING);

  // from https://github.com/mathertel/RotaryEncoder/blob/master/examples/InterruptRotator/InterruptRotator.ino#L35
  // You may have to modify the next 2 lines if using other pins than A2 and A3
  PCICR |= (1 << PCIE1);    // This enables Pin Change Interrupt 1 that covers the Analog input pins or Port C.
  PCMSK1 |= (1 << PCINT10) | (1 << PCINT11);  // This enables the interrupt for pin 2 and 3 of Port C.
}

// The Interrupt Service Routine for Pin Change Interrupt 1
// This routine will only be called on any signal change on A2 and A3: exactly where we need to check.
ISR(PCINT1_vect) {
  if (encoderEnabled) {
    encoder.tick(); // just call tick() to check the state.
  }
}



// ----------------------------------------------
// Miscellaneous Util
// ----------------------------------------------

// https://www.instructables.com/id/Arduino-Software-debouncing-in-interrupt-function/
void debounceInterrupt() {
  //Serial.println("Debounce interrupt");
  if ((long)(micros() - debounceLastMicros) >= debouncing_time * 1000) {
    onEncoderSwitchChange();
    debounceLastMicros = micros();
  }
}