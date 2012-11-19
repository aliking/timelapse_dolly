/*
Stepper Motor control for a timelapse dolly controller
Controls the speed of a stepper motor in two directions
using a potentiometer, where the middle range of the pot
zero, negative directions are anything less than that and
positive are more than that.

*/
#include <AccelStepper.h>

AccelStepper stepper(1,3,2);
//digital pins
const int BUTTON_1 = 8;
const int BUTTON_2 = 9;
const int LED_1_PIN = 12;
const int LED_2_PIN = 13;
//analog pins
const int POT_IN = 0;
//potentiometer control
const int DEAD_ZONE_SIZE = 100;
const int POT_MAX = 1023;
const int OVERDRIVE_ZONE_SIZE = 5;
const int OVERDRIVE_MULTIPLIER = 10;
//speed control
const long RUN_CHUNK = 500; //approx milliseconds for each blocking movement of the motor
const int MAX_SPEED = 100;
//loop control
const int LOOP_BASE = 0;
const int LOOP_MOVE = 1;
const int LOOP_PAUSE = 2;
const int LOOP_CALIBRATION = 3;
//Also BUTTON_1 and 2 are used as looping identifiers. That's probably a terrible idea.
int looper = LOOP_BASE;
int lastLoop = LOOP_BASE;
//button debouncing
const int NONE = 20;
int lastButton = NONE;
long lastButtonTime;
boolean buttonUnlock = true;
long debounceDelay = 100;  // the debounce time; increase if the output flickers
//LED states (bitwise)
const byte LED_1 = byte(1);
const byte LED_2 = byte(2);


int oldRaw = 0;
float oldSpeed = 0;
long maxStepperPos = 1000000;



void setup() {
  Serial.begin(9600);
  //set pin modes
  pinMode(LED_1_PIN, OUTPUT);
  pinMode(LED_2_PIN, OUTPUT);
  pinMode(BUTTON_1, INPUT_PULLUP);
  pinMode(BUTTON_2, INPUT_PULLUP);

  //stepper
  stepper.setMaxSpeed(3000);
  stepper.setAcceleration(500);
  stepper.setCurrentPosition(0);
}

void loop(){
  switch (looper) {
    case LOOP_BASE:
      delay(100);
      break;
    case LOOP_PAUSE:
      led(LED_2);
      delay(100);
      break; 
    case LOOP_MOVE:
      led(0);
      set(get_speed());
      stepper.runSpeedToPosition();
      //Serial.println(stepper.currentPosition());
      break;
    case BUTTON_1:
      if (lastLoop == LOOP_BASE){
        stepper.setCurrentPosition(0);
        looper = LOOP_CALIBRATION;
        Serial.println("Calibrating...");
      }
      else if (lastLoop == LOOP_CALIBRATION){
        Serial.println("Setting max postion");
        maxStepperPos = stepper.targetPosition()+50;
        Serial.println(maxStepperPos);
        looper = LOOP_MOVE;
      }
      else {
        looper = lastLoop; 
      }
      break;
    case BUTTON_2:
      if (lastLoop == LOOP_PAUSE){
        looper = LOOP_MOVE;
      }
      else {
        looper = LOOP_PAUSE;
      }
      break;
    case LOOP_CALIBRATION:
      led(LED_1);
      set(MAX_SPEED*OVERDRIVE_MULTIPLIER);
      stepper.runSpeedToPosition();
      break;
  }
  //Serial.println(looper);
  check_buttons();
}

void led(int ledState) {
  if (LED_1 & ledState){
    digitalWrite(LED_1_PIN, HIGH);
  }
  else{
    digitalWrite(LED_1_PIN, LOW); 
  }
  if (LED_2 & ledState){
    digitalWrite(LED_2_PIN, HIGH);
  }
  else{
    digitalWrite(LED_2_PIN, LOW); 
  } 
}

void set(float speed){
  if(stepper.distanceToGo() == 0) {
    if (speed == 0) {
      stepper.moveTo(stepper.currentPosition());
    }
    else {
      float frac = RUN_CHUNK / 1000.0;
      float runDist = (speed * frac);
      //int relMove = (speed > 0)? runDist:0-runDist;
      //Serial.println(runDist);
      long target = stepper.currentPosition() + runDist;
      
      //override move if it is beyond the bounds of movement
      if (target < 0 || target > maxStepperPos){
        target = stepper.currentPosition();
      }
      
      speed = abs(speed);
      stepper.setSpeed(speed);
      stepper.moveTo(target);
     // Serial.println(stepper.currentPosition());
      
    }
  }
}

float get_speed() {
  int raw_pos = analogRead(POT_IN);
  if (raw_pos != oldRaw) {
    oldRaw = raw_pos;
    int reverse_limit = (POT_MAX - DEAD_ZONE_SIZE)/2;
    int forward_limit = reverse_limit + DEAD_ZONE_SIZE;
    boolean overdrive = (raw_pos < OVERDRIVE_ZONE_SIZE || raw_pos > (POT_MAX - OVERDRIVE_ZONE_SIZE))?true:false;
    int ret = 0;
    
    if (raw_pos < reverse_limit) {
      ret = map(raw_pos, 0, reverse_limit, (0-MAX_SPEED), 0);
      
    }
      else if (raw_pos > forward_limit) {
      ret = map(raw_pos, forward_limit, POT_MAX, 0, MAX_SPEED);
      
    }
    ret = overdrive? ret * OVERDRIVE_MULTIPLIER:ret;
    oldSpeed = ret;
    Serial.println(ret);
    return ret;
  }
  return oldSpeed;
}

void check_buttons() {
  boolean b1 = digitalRead(BUTTON_1) == LOW? true: false;
  boolean b2 = digitalRead(BUTTON_2) == LOW? true: false;
  
  if (b1 || b2 ){
    if (buttonUnlock){
      if (b1) {
        if (lastButton != BUTTON_1) {
          lastButtonTime = millis();
        }
        lastButton = BUTTON_1;
      } 
      if (b2) {
        if (lastButton != BUTTON_2) {
          lastButtonTime = millis();
        }
        lastButton = BUTTON_2;
      }
      if ((millis() - lastButtonTime) > debounceDelay){
        lastLoop = looper;
        looper = lastButton;
        buttonUnlock = false;
        //Serial.println("button");
      }
    } 
  }
  else {
    lastButton = NONE;
    lastButtonTime = millis();
    buttonUnlock = true;
  }

}
