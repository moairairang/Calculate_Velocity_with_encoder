#include <RP2040_PWM.h>
#include <arduino-timer.h>

RP2040_PWM* PWM_Instance;

#define ENCODER_COUNTS_PER_REV 6400  // replace with your encoder's value

#define CLK_PIN 21
#define DT_PIN 22
#define MOTOR_PIN_INA 12
#define MOTOR_PIN_INB 3
const int MOTOR_PIN_PWM = 2;
#define POT_PIN 26

volatile int counter = 0;
int encoderState = 0;
int motorSpeed = 0;
int lastCounter = 0;
unsigned long lastMeasureTime = 0;

long prevT = 0;
int posPrev = 0;
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

float v1Filt = 0;
float v1Prev = 0;
float eintegral = 0;

void setup() {
  Serial.begin(115200);

  pinMode(CLK_PIN, INPUT_PULLUP);
  pinMode(DT_PIN, INPUT_PULLUP);

  encoderState = (digitalRead(DT_PIN) << 1) | digitalRead(CLK_PIN);

  attachInterrupt(digitalPinToInterrupt(CLK_PIN), readEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DT_PIN), readEncoder, CHANGE);

  pinMode(MOTOR_PIN_INA, OUTPUT);
  pinMode(MOTOR_PIN_INB, OUTPUT);
  pinMode(MOTOR_PIN_PWM, OUTPUT);
  pinMode(POT_PIN, INPUT);

  PWM_Instance = new RP2040_PWM(MOTOR_PIN_PWM, 20000, 0);
}

void loop() {
  unsigned long currentMicros = micros();
  Serial.print(currentMicros);
  Serial.print("\t");
  Serial.print(lastMeasureTime);
  Serial.print("\t");
  Serial.println(currentMicros - lastMeasureTime);
  while(currentMicros - lastMeasureTime < 200){
    currentMicros = micros();
    delayMicroseconds(1);
    
  }
  if (currentMicros - lastMeasureTime == 200) { //samping time microsec
    
    int pos = 0;
    noInterrupts();
    pos = pos_i;
    interrupts();

    long currT = micros();
    float deltaT = ((float)(currT - prevT)) / 1.0e6;
    float velocity1 = (counter - posPrev) / deltaT;
    posPrev = counter;
    prevT = currT;

    float v1 = velocity1 / 600.0 * 60.0;
    v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
    v1Prev = v1;

    Serial.print(v1Filt/2);
    Serial.print("    ");
    Serial.println(v1/2);

    
  }
  
  analogReadResolution(16);
  int Val = analogRead(POT_PIN);
  motorSpeed = map(Val, 0, 65535, 0, 100);
  digitalWrite(MOTOR_PIN_INA, HIGH);
  digitalWrite(MOTOR_PIN_INB, LOW);
  PWM_Instance->setPWM(MOTOR_PIN_PWM, 20000, motorSpeed);
  lastMeasureTime = currentMicros;
}

void readEncoder() {
  int newState = (digitalRead(DT_PIN) << 1) | digitalRead(CLK_PIN);

  if ((encoderState == 0b00 && newState == 0b01) || (encoderState == 0b01 && newState == 0b11) || (encoderState == 0b11 && newState == 0b10) || (encoderState == 0b10 && newState == 0b00)) {
    counter++;
  } else if ((encoderState == 0b00 && newState == 0b10) || (encoderState == 0b10 && newState == 0b11) || (encoderState == 0b11 && newState == 0b01) || (encoderState == 0b01 && newState == 0b00)) {
    counter--;
  }
  
  encoderState = newState;
  pos_i = pos_i + counter;
}
