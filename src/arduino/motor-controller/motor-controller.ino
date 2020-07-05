// - add commands
//    - set acceleration
//    - move motor by degress

#include <AccelStepper.h>
#include <MultiStepper.h>
#include <RotaryEncoder.h>
#include <LiquidCrystal_I2C.h>

// Motor

#define dirPin_m1 5
#define dirPin_m2 8
#define stepPin_m1 6
#define stepPin_m2 9
#define enablePin_m1 7
#define enablePin_m2 10
#define maxSpeed_m1 2000L
#define maxSpeed_m2 2000L
#define degreesPerStep_m1 0.45
#define degreesPerStep_m2 0.045
#define maxAcceleration 4000L
#define motorSleepTimeout 2000L

unsigned long lastMotorActionTime = millis();
boolean isMotorEnabled = false;

AccelStepper stepper_m1(AccelStepper::FULL2WIRE, stepPin_m1, dirPin_m1);
AccelStepper stepper_m2(AccelStepper::FULL2WIRE, stepPin_m2, dirPin_m2);

// Encoder

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

// https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home
LiquidCrystal_I2C  lcd(0x27,2,1,0,4,5,6,7); // 0x27 is the I2C bus address for an unmodified module

#define displaySleepTimeout 10000L

unsigned long lastDisplayActionTime = millis();
volatile boolean displayUpdateNeeded = false;
boolean isDisplayLightOn = false;

void setup() {
  Serial.begin(115200L);
  setupMotors();
  setupEncoder();
  setupDisplay();
}

void loop() {

  static int pos = 0;

  int newPos = encoder.getPosition();
  
  if (pos != newPos) {
    if (shouldTakeInput()) {
      takeKnobInput(pos, newPos);
    }
    // if we are not taking input, this will ignore changes to knob position
    pos = newPos;
  }
  if (!runMotorsIfNeeded()) {
    updateDisplay();
    checkDisplaySleep();
    checkMotorSleep();
    takeCommand(Serial, Serial);
  }
}

void takeCommand(Stream &input, Stream &output) {

  if (!input.available()) {
    return;
  }
  
  byte firstByte = input.read();

  if (firstByte != ':') {
    output.write("=40\n");
    return;
  }

  String command = input.readStringUntil(' ');

  if (command.equals("01")) {

    // Move a motor n steps in a direction

    // first param is the motor id, 1 or 2

    int motorId = input.readStringUntil(' ').toInt();
    if (motorId != 1 && motorId != 2) {
      output.write("=46\n");
      return;
    }

    // second param is direction 1: clockwise, 2: anti-clockwise

    int dir = input.readStringUntil(' ').toInt();

    int dirMult = 0;
    if (dir == 1) {
      dirMult -= 1;
    } else if (dir == 2) {
      dirMult += 1;
    } else {
      output.write("=47\n");
      return;
    }

    // third param is how many steps

    long howMuch = input.readStringUntil(';').toInt() * dirMult;
    if (howMuch == 0) {
      output.write("=48\n");
      return;
    }

    // perform action

    if (motorId == 1) {
      // TODO: figure out why we couldn't dynamically assign this -- need to learn this language better
      jumpOne(stepper_m1, howMuch);
    } else if (motorId == 2) {
      jumpOne(stepper_m2, howMuch);
    }

    output.write("=00\n");

  } else if (command.equals("02")) {

    // Set max speed for motor

    // TODO: make DRY

    // first param is the motor id, 1 or 2

    int motorId = input.readStringUntil(' ').toInt();
    if (motorId != 1 && motorId != 2) {
      output.write("=46\n");
      return;
    }

    // second param is the speed
    long newSpeed = input.readStringUntil(';').toInt();
    if (newSpeed == 0) {
      output.write("=49\n");
      return;
    }

    if (motorId == 1) {
      // TODO: figure out why we couldn't dynamically assign this, as above
      stepper_m1.setMaxSpeed(min(newSpeed, maxSpeed_m1));
    } else if (motorId == 2) {
      stepper_m2.setMaxSpeed(min(newSpeed, maxSpeed_m2));
    }

    output.write("=00\n");
  } else if (command.equals("03")) {

    // Set acceleration for motor

    // TODO: make DRY

    // first param is the motor id, 1 or 2

    int motorId = input.readStringUntil(' ').toInt();
    if (motorId != 1 && motorId != 2) {
      output.write("=46\n");
      return;
    }

    // second param is the acceleration
    long newAccel = input.readStringUntil(';').toInt();
    if (newAccel == 0) {
      output.write("=50\n");
      return;
    }

    if (motorId == 1) {
      // TODO: figure out why we couldn't dynamically assign this, as above
      stepper_m1.setAcceleration(min(newAccel, maxAcceleration));
    } else if (motorId == 2) {
      stepper_m2.setAcceleration(min(newAccel, maxAcceleration));
    }

    output.write("=00\n");

  } else if (command.equals("04")) {
    // Move a motor n degrees in a direction

    // first param is the motor id, 1 or 2

    int motorId = input.readStringUntil(' ').toInt();
    if (motorId != 1 && motorId != 2) {
      output.write("=46\n");
      return;
    }

    // second param is direction 1: clockwise, 2: anti-clockwise

    int dir = input.readStringUntil(' ').toInt();

    int dirMult = 0;
    if (dir == 1) {
      dirMult -= 1;
    } else if (dir == 2) {
      dirMult += 1;
    } else {
      output.write("=47\n");
      return;
    }

    // third param is how many steps

    float howMuch = input.readStringUntil(';').toFloat() * dirMult;
    if (howMuch == 0) {
      output.write("=48\n");
      return;
    }

    // perform action

    jumpOneByDegrees(motorId, howMuch);


    output.write("=00\n");
  } else {
    output.write("=44\n");
  }
}

boolean shouldTakeInput() {
  return stepper_m1.distanceToGo() == 0 && stepper_m2.distanceToGo() == 0;
}

boolean shouldUpdateDisplay() {
  return displayUpdateNeeded;
}

boolean runMotorsIfNeeded() {
  boolean isRun = false;
  if (stepper_m1.distanceToGo() != 0) {
    stepper_m1.run();
    registerMotorAction();
    isRun = true;
  }
  if (stepper_m2.distanceToGo() != 0) {
    stepper_m2.run();
    registerMotorAction();
    isRun = true;
  }
  return isRun;
}

void takeKnobInput(int oldPos, int newPos) {
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

void onSwitchChange() {
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

void jumpBoth(long howMuch) {
  stepper_m1.move(howMuch);
  stepper_m2.move(howMuch);
  enableMotors();
}

void jumpOne(AccelStepper &stepper, long howMuch) {
  stepper.move(howMuch);
  enableMotors();
}

void jumpOneByDegrees(int motorId, float howMuch) {
  long steps;
  
  if (motorId == 1) {
    steps = howMuch / degreesPerStep_m1;
    stepper_m1.move(steps);
  } else if (motorId == 2) {
    steps = howMuch / degreesPerStep_m2;
    stepper_m2.move(steps);
  } else {
    return;
  }
  enableMotors();
}
void enableMotors() {
  if (!isMotorEnabled) {
    digitalWrite(enablePin_m1, LOW);
    digitalWrite(enablePin_m2, LOW);
    isMotorEnabled = true;
    delay(2);
  }
  registerMotorAction();
}

void disableMotors() {
  if (isMotorEnabled) {
    digitalWrite(enablePin_m1, HIGH);
    digitalWrite(enablePin_m2, HIGH);
    isMotorEnabled = false;
  }
}

void updateDisplay() {
  if (shouldUpdateDisplay()) {
    lcd.setCursor(0, 0);
    lcd.print("Program: ");
    lcd.print(programNum);
    displayUpdateNeeded = false;
  }
}

void turnOnDisplayLight() {
  lcd.setBacklightPin(3, NEGATIVE);
}

void turnOffDisplayLight() {
  lcd.setBacklightPin(3, POSITIVE);
}

void registerMotorAction() {
  lastMotorActionTime = millis();
}

void registerDisplayAction() {
  lastDisplayActionTime = millis();
}

void setupMotors() {

  // Declare pins as output:
  pinMode(stepPin_m1, OUTPUT);
  pinMode(stepPin_m2, OUTPUT);
  pinMode(dirPin_m1, OUTPUT);
  pinMode(dirPin_m2, OUTPUT);
  pinMode(enablePin_m1, OUTPUT);
  pinMode(enablePin_m2, OUTPUT);

  // set initial state of motor to disabled
  digitalWrite(enablePin_m1, HIGH);
  digitalWrite(enablePin_m2, HIGH);
  isMotorEnabled = false;

  // AccelStepper
  stepper_m1.setMaxSpeed(maxSpeed_m1);
  stepper_m2.setMaxSpeed(maxSpeed_m2);
  stepper_m1.setAcceleration(maxAcceleration);
  stepper_m2.setAcceleration(maxAcceleration);
}

void checkMotorSleep() {
  unsigned long elapsed = millis() - lastMotorActionTime;
  if (elapsed > motorSleepTimeout) {
    disableMotors();
  }
}

void checkDisplaySleep() {
  unsigned long elapsed = millis() - lastDisplayActionTime;
  if (elapsed > displaySleepTimeout) {
    turnOffDisplayLight();
  } else {
    // for button, so we don't have to do this in an interrupt
    turnOnDisplayLight();
  }
}

void setupDisplay() {
  lcd.begin(16, 2);
  lcd.clear();
  displayUpdateNeeded = true;
  turnOnDisplayLight();
  updateDisplay();
}

void setupEncoder() {

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
  encoder.tick(); // just call tick() to check the state.
}

// https://www.instructables.com/id/Arduino-Software-debouncing-in-interrupt-function/
void debounceInterrupt() {
  //Serial.println("Debounce interrupt");
  if ((long)(micros() - debounceLastMicros) >= debouncing_time * 1000) {
    onSwitchChange();
    debounceLastMicros = micros();
  }
}
