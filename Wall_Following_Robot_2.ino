/* This program does stuff
 * Fill out rest of header here
 */
 
//Includes
#include "Arduino.h"
#include "RunningMedian.h"
 
#define DEBUG
 
//Constants
const int BASE_SPEED = 110;
const int MAX_BATTERY = 1000;
const int MIN_BATTERY = 0;
const int BASE_DISTANCE = 30;
const int MAX_DISTANCE = 60;
const int FRONT_DISTANCE = 30;
const int a = 1;
//Pins
const unsigned int L_MOTOR_CURRENT_PIN = A0;
const unsigned int R_MOTOR_CURRENT_PIN = A1;
const unsigned int L_MOTOR_PIN = 11;
const unsigned int R_MOTOR_PIN = 3;
const unsigned int L_MOTOR_DIRECTION_PIN = 12;
const unsigned int R_MOTOR_DIRECTION_PIN = 13;
const unsigned int F_SENSOR_TRIG_PIN = 5;
const unsigned int F_SENSOR_ECHO_PIN = 6;
const unsigned int R_SENSOR_TRIG_PIN = 7;
const unsigned int R_SENSOR_ECHO_PIN = 10;

typedef enum {
  SEARCH,
  FOLLOW,
  LOST,
  DANCE
} State;

//File globals
int _batLevel;
int _frontSensor, _rightSensor;
int _leftMotor = BASE_SPEED, _rightMotor = BASE_SPEED;
State _state = FOLLOW;
RunningMedian _sensorMedian = RunningMedian(5);

/**
 * Check currents of motors
 */
 
void setup()
{
  #ifdef DEBUG
  Serial.begin(9600);
  #endif
  
  pinMode(L_MOTOR_CURRENT_PIN, INPUT);
  pinMode(R_MOTOR_CURRENT_PIN, INPUT);
  pinMode(L_MOTOR_PIN, OUTPUT);
  pinMode(R_MOTOR_PIN, OUTPUT);
  pinMode(L_MOTOR_DIRECTION_PIN, OUTPUT);
  pinMode(R_MOTOR_DIRECTION_PIN, OUTPUT);
  pinMode(F_SENSOR_TRIG_PIN, OUTPUT);
  pinMode(F_SENSOR_ECHO_PIN, INPUT);
  pinMode(R_SENSOR_TRIG_PIN, OUTPUT);
  pinMode(R_SENSOR_ECHO_PIN, INPUT);
  
  pinMode(L_MOTOR_DIRECTION_PIN, OUTPUT);
  pinMode(R_MOTOR_DIRECTION_PIN, OUTPUT);
  digitalWrite(L_MOTOR_DIRECTION_PIN, HIGH);
  digitalWrite(R_MOTOR_DIRECTION_PIN, HIGH);
  
  //do some sensor init
  for(int i = 0; i < 5; i++)
  {
    _sensorMedian.add(getSensorDistance(R_SENSOR_TRIG_PIN, R_SENSOR_ECHO_PIN));
  }
}

void loop()
{
  //read input
  //pulse sensors and read the echo
  _sensorMedian.add(getSensorDistance(R_SENSOR_TRIG_PIN, R_SENSOR_ECHO_PIN));
  _rightSensor = _sensorMedian.getMedian();
  _frontSensor = getSensorDistance(F_SENSOR_TRIG_PIN, F_SENSOR_ECHO_PIN);
     if (_rightSensor > 200) {
      _rightSensor = 200;
    }
  //State checking:
    switch(_state) {
    case FOLLOW:
      follow();
      break;
    case SEARCH:
      search();
      break;
    case DANCE:
      boogie();
      break;
    case LOST:
      lost();
      break;
    }
    
    //output signal
    analogWrite(L_MOTOR_PIN, _leftMotor);
    analogWrite(R_MOTOR_PIN, _rightMotor);
 
    #ifdef DEBUG
    debug();
    #endif   
}


//Returns the distance to the object in cm
long getSensorDistance(unsigned int pinTrig, unsigned int pinEcho){
  //Converts duration of signal into centimeters
  digitalWrite(pinTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(pinTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pinTrig, LOW);
  
  double duration = pulseIn(pinEcho, HIGH);
  return (duration/2) / 29.1;
}

void debug()
{
    unsigned long t;
    
    switch(_state) {
    case FOLLOW:
      Serial.print("FOLLOWING");
      break;
    case SEARCH:
      Serial.print("BONUS STAGE");
      break;
    case DANCE:
      Serial.print("BREAK IT DOWN");
      break;
    case LOST:
      Serial.print("LOST");
      break;
    }

   Serial.print("\tSens:");
    Serial.print("\tF: ");
    Serial.print(_frontSensor);
    Serial.print("\tR: ");
    Serial.print(_rightSensor);
    Serial.print("\tRM: ");
    Serial.print(_rightMotor);
    Serial.print("\tLM: ");
    Serial.print(_leftMotor);
    
    Serial.print("\n");
}

void follow() {
  if (_frontSensor < FRONT_DISTANCE) {
    //try not to hit the wall by doing an evasive maneuver to the left
    _rightMotor = BASE_SPEED;
    _leftMotor = BASE_SPEED / 2;
  }
  else if(_rightSensor > MAX_DISTANCE) {
    _state = LOST;
  }
  else if (_rightSensor > (BASE_DISTANCE *1.5)) {
    _rightMotor = BASE_SPEED;
    
    int mod2 = (_rightSensor - BASE_SPEED);
    if (mod2<20) {
      mod2 = 20;
    }
    
    _leftMotor = BASE_SPEED + mod2; 
  }
  else if (_rightSensor < (BASE_DISTANCE * 0.75)) {
    int mod1 = (BASE_DISTANCE -_rightSensor);   
    if (mod1<20) {
      mod1 = 20;
    }
    
    _rightMotor = BASE_SPEED + mod1;

     
    _leftMotor = BASE_SPEED;
  }
  else {
     _rightMotor = BASE_SPEED;
     _leftMotor = BASE_SPEED;
   }
}
  
void lost() {
  _rightMotor = BASE_SPEED / 2;
  _leftMotor = BASE_SPEED;
  if(_rightSensor < MAX_DISTANCE) {
     _state = FOLLOW;
   }
}

void search(){}
void boogie(){}
