//joystick processing library
#include <Arduino.h>
#include "joystickHelper.h"

joystickHelper::joystickHelper(int xPin, int yPin, int swPin) {
  joystickHelper::xPin = (int)xPin;
  joystickHelper::yPin = (int)yPin;
  joystickHelper::swPin = (int)swPin;
  
  joystickHelper::deadband = abs(joystickHelper::deadband); //keep deadband positive

  pinMode(xPin, INPUT);
  pinMode(yPin, INPUT);
  pinMode(swPin, INPUT_PULLUP); //use internal pullup reistors from arduino
}

boolean joystickHelper::isPressed() {
  joystickHelper::update();
  return joystickHelper::pressed;
}

joystickPosition joystickHelper::getPosition() {
  joystickHelper::update();
  return {x: joystickHelper::x, y: joystickHelper::y};
}

void joystickHelper::update() {
  //reset
  joystickHelper::up = false;
  joystickHelper::down = false;
  joystickHelper::left = false;
  joystickHelper::right = false;
  joystickHelper::movement = false;
  
  //STEP 1: deal with button (debounce logic)
  /*int swReading = digitalRead(joystickHelper::swPin);
  if (swReading != joystickHelper::lastPressState) { //debouncing stuff
    joystickHelper::lastPressTime = millis();
  }
  if ((millis() - joystickHelper::lastPressTime) > 50) { //50ms debounce delay
    if (swReading != joystickHelper::pressed) {
      joystickHelper::pressed = swReading;
    }
  }*/
  int tB = digitalRead(joystickHelper::swPin);
  if (joystickHelper::pressed && ((millis() - joystickHelper::lastPressTime) > 50)) {
    joystickHelper::pressed = false; //disable press after 100ms
  }

  if (tB == HIGH) { //when not pressed reset detection
    joystickHelper::allowPress = true;
  } else if (tB == LOW && joystickHelper::allowPress) { //just pressed (detect rising edge)
    joystickHelper::pressed = true;
    joystickHelper::allowPress = false;
    joystickHelper::lastPressTime = millis();
  }

  //STEP 2: deal with xPos
  int tX = map(analogRead(joystickHelper::xPin), 0, 1023, -100, 100); //map to percentages
  if (tX > -joystickHelper::deadband && tX < joystickHelper::deadband) { //deadband
    joystickHelper::x = 0; //just set to 0
  } else {
    joystickHelper::x = tX;
  }
  if (tX > joystickHelper::deadband) {
    joystickHelper::right = true;
  } else if (tX < -joystickHelper::deadband) {
    joystickHelper::left = true;
  }

  //STEP 3: deal with yPos
  int tY = map(analogRead(joystickHelper::yPin), 0, 1023, 100, -100); //map to percentages
  if (tY > -joystickHelper::deadband && tX < joystickHelper::deadband) { //deadband
    joystickHelper::y = 0; //just set to 0
  } else {
    joystickHelper::y = tY;
  }
  if (tY > joystickHelper::deadband) {
    joystickHelper::down = true;
  } else if (tY < -joystickHelper::deadband) {
    joystickHelper::up = true;
  }

  if (joystickHelper::left || joystickHelper::right || joystickHelper::up || joystickHelper::down) {
    joystickHelper::movement = true;
  }
}
  
