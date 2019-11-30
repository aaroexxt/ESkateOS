#ifndef joystickHelper_h
#define joystickHelper_h

struct joystickPosition {
  int x;
  int y;
};

class joystickHelper {
  public:
    joystickHelper(int xPin, int yPin, int swPin);
    int x;
    int y;
    boolean pressed;
    boolean isPressed();
    joystickPosition getPosition();
    void update();
    int deadband = 10;
    boolean right, left, up, down, movement;
  private:
    int xPin;
    int yPin;
    int swPin;
    boolean allowPress = true;
    long lastPressTime = -1;
};

#endif
