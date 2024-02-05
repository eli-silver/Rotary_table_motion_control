#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <DueTimer.h>
#include "pid.h"

// Rotary Encoder Inputs
#define QA 12
#define QB 13
#define MOT_PWM 8
#define MOT_DIR 9
#define MOT_EN  10

// encoder mm per pulses:
float mmpp = 300/1000.0;

int enc_count = 0;
int prev_enc_count = 0;
int currentStateQA;
int currentStateQB;
String currentDir ="";

unsigned long prev_time = 0;
unsigned long curr_time = prev_time;

float curr_speed = 0;
float target_speed = 0;

int curr_pwm = 0;
bool hold_pwm = false;

LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address 0x27, 16 column and 2 rows

/* Motor speed PID parameters */
#define PID_KP 0.0f
#define PID_KI 0.075f
#define PID_KD 0.0f
#define PID_KFF 0.0025f

#define PID_TAU 0.1f

#define PID_LIM_MIN -1.0f
#define PID_LIM_MAX 1.0f

#define PID_T 0.001f

PIDController pid = { PID_KP, PID_KI, PID_KD, PID_KFF, PID_TAU, PID_LIM_MIN, PID_LIM_MAX, PID_T };

void setMotor(float duty_cycle){
  //Serial.println(duty_cycle);
  if(duty_cycle <= 0 ){
    digitalWrite(MOT_DIR, 1);
  }else{
    digitalWrite(MOT_DIR, 0);
  }

  int pwm = (int)fabs(duty_cycle * 4095);
  curr_pwm = pwm;
  analogWrite(MOT_PWM, pwm);
}

void updateEncoder(){
	//Read the current state of QA
  //currentStateQA = digitalRead(QA);
  currentStateQB = digitalRead(QB);

  if(currentStateQB){
    enc_count--;
  }
  else if( !currentStateQB){
    enc_count++;
  }else{}
}

void updateController(){
  unsigned long curr_time =  millis();
  int dT = curr_time - prev_time;
  int num_pulses = enc_count - prev_enc_count;
  float measurement_period = 1000.0/(dT);

  curr_speed = (num_pulses * measurement_period * mmpp);
  if(!hold_pwm){
    setMotor(PIDController_Update(&pid, target_speed, curr_speed));
  }

  Serial.print(curr_speed);
  Serial.print(" ");
  Serial.println(curr_pwm);

  prev_enc_count = enc_count;
  prev_time = curr_time;
}

void updateLCD(){
  lcd.setCursor(2,0);
  lcd.print("SPEED (mm/s):");
  lcd.setCursor(4,1);
  lcd.print(curr_speed);
  lcd.print("     ");
}




void setup() {


	// Set encoder pins as inputs
	pinMode(QA,INPUT_PULLUP);
	pinMode(QB,INPUT_PULLUP);

  // setup motor outputs
  analogWriteResolution(12);
  pinMode(MOT_PWM, OUTPUT);
  pinMode(MOT_DIR, OUTPUT);
  pinMode(MOT_EN, OUTPUT);

	// Setup Serial Monitor
	Serial.begin(9600);

  // initialize the lcd
  lcd.init(); 
  lcd.backlight();

  PIDController_Init(&pid);

  prev_time = millis();
  curr_time = millis();

  Timer3.attachInterrupt(updateController);
	Timer3.start(500000); // Calls every 500ms

  Timer4.attachInterrupt(updateLCD);
  Timer4.start(3000000); // Calls every 3s

  // Call updateEncoder() when any rising edge changed seen on pin QA
	attachInterrupt(digitalPinToInterrupt(QA), updateEncoder, RISING);

  Serial.println("Interrupts Set");
}

void loop() {
  if(Serial.available()){
    int new_speed = Serial.parseInt();
    Serial.print(" New Speed: ");
    Serial.println(new_speed);
    if(new_speed == 0){
      digitalWrite(MOT_EN, 0);
      target_speed = 0;
      setMotor(0);
    }else if(new_speed == 123){
      hold_pwm = !hold_pwm;
      Serial.print("hold pwm: ");
      Serial.println(hold_pwm);
    }
    else{
      digitalWrite(MOT_EN, 1);
      if(new_speed >= -4095 && new_speed <= 4095){
        if(hold_pwm){
          setMotor(new_speed/4095.0); // allows you to type in a PWM value directly when in hold mode
          Serial.print("PWM set to: ");
          Serial.println(new_speed);
        }else{
          target_speed = new_speed;
        }
      }else{
        Serial.println("invalid target speed. Must be mm/s int");
      }
    }
     Serial.print(" Target speed set to: ");
     Serial.println(target_speed);
  }
  delay(10);
}

