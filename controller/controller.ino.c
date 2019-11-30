/***************
 ______     ______     __  __     ______     ______   ______        ______     ______     __   __     ______   ______     ______     __         __         ______     ______    
/\  ___\   /\  ___\   /\ \/ /    /\  __ \   /\__  _\ /\  ___\      /\  ___\   /\  __ \   /\ "-.\ \   /\__  _\ /\  == \   /\  __ \   /\ \       /\ \       /\  ___\   /\  == \   
\ \  __\   \ \___  \  \ \  _"-.  \ \  __ \  \/_/\ \/ \ \  __\      \ \ \____  \ \ \/\ \  \ \ \-.  \  \/_/\ \/ \ \  __<   \ \ \/\ \  \ \ \____  \ \ \____  \ \  __\   \ \  __<   
 \ \_____\  \/\_____\  \ \_\ \_\  \ \_\ \_\    \ \_\  \ \_____\     \ \_____\  \ \_____\  \ \_\\"\_\    \ \_\  \ \_\ \_\  \ \_____\  \ \_____\  \ \_____\  \ \_____\  \ \_\ \_\ 
  \/_____/   \/_____/   \/_/\/_/   \/_/\/_/     \/_/   \/_____/      \/_____/   \/_____/   \/_/ \/_/     \/_/   \/_/ /_/   \/_____/   \/_____/   \/_____/   \/_____/   \/_/ /_/ 
                                                                                                                                                                                                                                                                                             
****************

  By Aaron Becker
  V1 Nov/Dec 2019
*/

#include <SoftwareServo.h> 
int potentiometer=A7;
int potval;
int curval;


SoftwareServo ESC;

void setup() {
  
  pinMode(potentiometer, INPUT);
  ESC.attach(9);    
  Serial.begin(9600);  
  curval=0;
  ESC.setMinimumPulse(800);
  ESC.setMaximumPulse(2000);
}

void loop() {

  potval=analogRead(potentiometer);
  potval=map(potval,0,1023,0,180);
  
  while(curval<potval){
    potval=analogRead(potentiometer);
    potval=map(potval,0,1023,0,180);
    curval=curval+1;
    ESC.write(curval);
    SoftwareServo::refresh();
    Serial.println(curval);
    delay(50);}

  while(curval>potval){
    potval=analogRead(potentiometer);
    potval=map(potval,0,1023,0,180);
    curval=curval-1;
    ESC.write(curval);
    SoftwareServo::refresh();
    Serial.println(curval);
    delay(50);}

    ESC.write(curval);
    SoftwareServo::refresh();
    Serial.println(curval);
}

