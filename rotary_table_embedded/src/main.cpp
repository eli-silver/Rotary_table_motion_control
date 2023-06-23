#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

// Rotary Encoder Inputs
#define QA 12
#define QB 13

// encoder mm per pulses:
float mmpp = 300/1000.0;

int loop_time = 500; //repeat time in ms
int count = 0;
int prev_count = 0;
int currentStateQA;
int currentStateQB;
int lastStateQA;
String currentDir ="";
unsigned long prev_time = 0;

LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address 0x27, 16 column and 2 rows

void updateEncoder(){
	// Read the current state of QA
  //currentStateQA = digitalRead(QA);
  currentStateQB = digitalRead(QB);

  if(currentStateQB){
    count--;
  }
  else if( !currentStateQB){
    count++;
  }else{}
  

}

void setup() {

	// Set encoder pins as inputs
	pinMode(QA,INPUT_PULLUP);
	pinMode(QB,INPUT_PULLUP);

	// Setup Serial Monitor
	Serial.begin(9600);

  // initialize the lcd
  lcd.init(); 
  lcd.backlight();

	// Read the initial state of QA
	lastStateQA = digitalRead(QA);
	
	// Call updateEncoder() when any high/low changed seen
	// on interrupt 0 (pin 2), or interrupt 1 (pin 3)
	attachInterrupt(digitalPinToInterrupt(QA), updateEncoder, RISING);
	//attachInterrupt(digitalPinToInterrupt(QB), updateEncoder, CHANGE);
  Serial.println("Interrupts Set");
  prev_time = millis();
}

void loop() {
  unsigned long curr_time =  millis();
  int dT = curr_time - prev_time;
  int ppl = count - prev_count;
  float lps = 1000.0/(dT);
  //float pps = (count - prev_count) / (curr_time - prev_time) * (1000.0 / loop_time) ;
  Serial.print("Distance from start: ");
  Serial.println(count * mmpp);
  //Serial.print("Speed: ");
  //Serial.println(ppl * lps * mmpp);
  //Serial.println(" ");
  
  lcd.setCursor(2,0);
  lcd.print("SPEED (mm/s):");
  //lcd.setCursor(4,1);
  //lcd.print("        ");
  lcd.setCursor(4,1);
  lcd.print(ppl * lps * mmpp);
  lcd.print("     ");
  prev_time = curr_time;
  prev_count = count;
  delay(loop_time);
}

