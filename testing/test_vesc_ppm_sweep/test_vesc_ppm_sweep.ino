

#include <ServoTimer2.h>

ServoTimer2 ESC_RIGHT; //Create FSESC "servo" output
#define ESC_R_PIN 15
#define ESC_MIN 800
#define ESC_MAX 2000

int scroll = (ESC_MIN+ESC_MAX)/2;
boolean direction = true;

void setup() {
	pinMode(ESC_R_PIN, OUTPUT);
	ESC_RIGHT.attach(ESC_R_PIN);
  	ESC_RIGHT.write(scroll);
}

void loop() {
	if (direction) {
		scroll++;
	} else {
		scroll--;
	}

	if (scroll > ESC_MAX || scroll < ESC_MIN) {
		direction = !direction; //flip flop and skip cycle
		return;
	}

	ESC_RIGHT.write(scroll);

  delay(10);
}
