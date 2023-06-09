#include <Arduino.h>
#include <PID.h>
#include <parsing.h> //Delimiter (;)


struct velocity_param
{
    float kecepatan = 0;
    float encoder = 0;
    float rpm = 0;
    float min_speed = 0; //minimum input kecepatan
    float pulse_rev = 100; //encoder per rotation
    unsigned long curr_time = 0;
    unsigned long prev_time = 0;
    float inc = 0;
    float avg_rpm = 0;
    float act_speed = 0; // Kecepatan dalam m/s
    int act_rpm = 0; //Kecepatan input RPM
    int u = 0;
    float delta_time = 0;
    float pos = 0;
    float posPrev = 0;
};

//Header LIB PID
PID_params param[2];
velocity_param vel[2];

//Function
void encoderA();
void initialize();
void encoderB();
void M1_maju(int speed_m);
void M2_maju(int speed_m);
void M1_mundur(int speed_m);
void M2_mundur(int speed_m);
void velocity1(int i);
void velocity(int i);
void avg_fill(int i, int N);

// Pin Motor DC
#define pwm_m1 29
#define pwm_m2 30
#define dir_m1 A12
#define dir_m2 A13

//Encoder
const int encoderB1 = 7;
const int encoderA2 = 5;
const int encoderB2 = 8;
const int encoderA1 = 6;
int enc[2] = {0,0};
float countA = 0;
float countB = 0;
float c = 0; //Keliling Roda

//PID
int control_PID[2] = {0,0};
double SP = 0.0;
float KP1 = 0, KI1 = 0, KD1 = 0;
float KP2 = 0, KI2 = 0, KD2 = 0;


//Pin BTN
#define btnUp A18
#define btnOk A20
#define btnDown A19
#define btnBack 26

//Pin LED
#define led1 A0
#define led2 A1
#define led3 A2
#define led4 A3

//Pin Buzzer
#define buzzer 13

//Parameter Motor DC
float motorSpeed = 300;
int motorOffset = 10;
int turnSpeed = 20;

//Parameter Kecepatan Motor DC
float M1_speed = 0;
float maxControl = 40;
float minControl = -40;
float kecepatan = 0;
float encoder = 0;
unsigned long curr_time = 0;
unsigned long prev_time = 0;
#define ts 100
#define phi 3.14
float inc = 0;
float delta_time = 0;
float r = 0.0335;
float track = 0.17;
float gear_ratio = 0;

void initialize(){
  digitalWrite(buzzer, HIGH); digitalWrite(led1, HIGH); digitalWrite(led4, HIGH);
  delay(100);
  digitalWrite(buzzer, LOW); digitalWrite(led2, HIGH); digitalWrite(led3, HIGH);
  delay(100);
  digitalWrite(buzzer, HIGH); digitalWrite(led4, LOW); digitalWrite(led2, LOW);
  delay(100);
  digitalWrite(buzzer, LOW); digitalWrite(led1, LOW); digitalWrite(led3, LOW);
  delay(100);
  digitalWrite(buzzer, HIGH); digitalWrite(led1, HIGH); digitalWrite(led4, HIGH);
  digitalWrite(led2, HIGH); digitalWrite(led3, HIGH);
  delay(100);
  digitalWrite(buzzer, LOW); digitalWrite(led4, LOW); digitalWrite(led2, LOW);
  digitalWrite(led1, LOW); digitalWrite(led3, LOW);
}

void setup(){
  // Serial.begin(57600);
  Serial.begin(115200);
  pinMode(pwm_m1, OUTPUT);
  pinMode(pwm_m2, OUTPUT);
  pinMode(dir_m1, OUTPUT);
  pinMode(dir_m2, OUTPUT);

  pinMode(buzzer, OUTPUT);

  pinMode(btnUp,   INPUT_PULLUP);
  pinMode(btnOk,   INPUT_PULLUP);
  pinMode(btnDown, INPUT_PULLUP);
  pinMode(btnBack, INPUT_PULLUP);

  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  pinMode(led4, OUTPUT);

  pinMode(encoderA1, INPUT);
  pinMode(encoderA2, INPUT);
  pinMode(encoderB1, INPUT);
  pinMode(encoderB2, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderA2),encoderA,RISING);
  attachInterrupt(digitalPinToInterrupt(encoderB2),encoderB,RISING);

  param[0].KP = 0;
  param[0].KI = 0;
  param[0].KD = 0;
  param[1].KP = 0;
  param[1].KI = 0;
  param[1].KD = 0;
  
  // initialize();
  

}

void loop(){
  // M1_maju(30);
  // M2_maju(30);
  // M1_mundur(50);
  // M2_mundur(50);
  if (Serial.available() > 0){
    Serial.setTimeout(100);
    READ_DATA_UNTIL('\n');
    // data.replace(',','.');
    parseString();

    SP = DATA_STR(0).toFloat();
    KP1 = DATA_STR(1).toFloat();
    KI1 = DATA_STR(2).toFloat();
    KD1 = DATA_STR(3).toFloat();
    KP2 = DATA_STR(4).toFloat();
    KI2 = DATA_STR(5).toFloat();
    KD2 = DATA_STR(6).toFloat();

    Serial.flush();

  }
    // Serial.println(String(param[0].set_point)+","+String(vel[0].avg_rpm)+","+String(control_PID[0])+",");
    // Serial.println(String(SP)+","+String(vel[0].avg_rpm)+","+String(vel[1].avg_rpm)+",");
    // Serial.println(String(SP)+","+String(param[0].feedback)+","+String(param[1].feedback)+",");
    // Serial.println(String(SP)+","+String(vel[0].act_speed)+","+String(vel[1].act_speed)+",");
  
  if (millis() - curr_time > ts){
      curr_time = millis();
      vel[0].pos = 0;
      vel[1].pos = 0;
      noInterrupts();
      vel[0].pos = vel[0].encoder;
      vel[1].pos = vel[1].encoder;
      interrupts();

      velocity(0);
      velocity(1);

      // param[0].set_point = param[1].set_point = SP;

      param[0].KP = KP1;
      param[0].KI = KI1;
      param[0].KD = KD1;
      param[1].KP = KP2;
      param[1].KI = KI2;
      param[1].KD = KD2;

      //Konversi m/s ke RPM
      c = phi*(r*2); //0.21
      vel[0].act_rpm = ((SP/c) * 60.0);
      vel[1].act_rpm = ((SP/c) * 60.0);

      param[0].set_point = vel[0].act_rpm;
      param[1].set_point = vel[1].act_rpm;

      param[0].feedback = vel[0].avg_rpm/2;
      param[1].feedback = vel[1].avg_rpm/2;

      //Konversi RPM ke m/s
      vel[0].act_speed = (vel[0].avg_rpm*c)/60;
      vel[1].act_speed = (vel[1].avg_rpm*c)/60;

      control_PID[0] = (int) PID(param[0]);
      control_PID[1] = (int) PID(param[1]);

      if ((SP <= 0.09) && (SP >= -0.09)){
        M1_maju(0);
        M2_maju(0);
      }

      // if (control_PID[0] > 0) M1_maju(control_PID[0]);
      // if (control_PID[0] < 0) M1_mundur(control_PID[0]*-1);
        
      // if (control_PID[1] > 0) M2_maju(control_PID[1]);
      // if (control_PID[1] < 0) M2_mundur(control_PID[1]*-1);
      Serial.println(String(SP)+","+String(control_PID[0])+","+String(control_PID[1])+",");
      // Serial.println(String(vel[0].act_rpm)+","+String(vel[0].avg_rpm)+","+String(vel[1].avg_rpm)+",");
      // Serial.println(String(SP)+","+String(vel[0].act_speed)+","+String(vel[1].act_speed)+",");
  }
      

}

//M1 = motor kiri, M2 = motor kanan

void M1_mundur(int speed_m){
  analogWrite(pwm_m1, speed_m);
  digitalWrite(dir_m1, LOW);
}

void M2_mundur(int speed_m){
  int pwm_ = 255 - speed_m;
  analogWrite(pwm_m2, pwm_);
  digitalWrite(dir_m2, HIGH);
}

void M1_maju(int speed_m){
  int pwm_ = 255 - speed_m;
  analogWrite(pwm_m1, pwm_);
  digitalWrite(dir_m1, HIGH);
}

void M2_maju(int speed_m){
  analogWrite(pwm_m2, speed_m);
  digitalWrite(dir_m2, LOW);
}

void velocity1(int i){
  vel[i].curr_time =  micros();
  vel[i].delta_time = (float)(vel[i].curr_time - vel[i].prev_time)/1.0e6;
  vel[i].kecepatan = vel[i].inc/vel[i].delta_time;
  vel[i].prev_time = vel[i].curr_time;
}

void velocity(int i){
    vel[i].curr_time =  micros();
    vel[i].delta_time = (float)(vel[i].curr_time - vel[i].prev_time)/1.0e6;
    vel[i].kecepatan = (vel[i].pos-vel[i].posPrev)/vel[i].delta_time;
    // vel[i].rpm = (vel[i].kecepatan*100)/60; //RPM
    vel[i].rpm = vel[i].kecepatan;
    avg_fill(i, 600);
    avg_fill(i, 600);
    vel[i].posPrev = vel[i].pos;
    vel[i].prev_time = vel[i].curr_time;
}

void avg_fill(int i, int N){
  vel[i].avg_rpm = 0;
  for(vel[i].u = 0 ; vel[i].u < N ; vel[i].u++){
    vel[i].avg_rpm += vel[i].rpm;
  }
  vel[i].avg_rpm = vel[i].avg_rpm/N;
}

//Encoder Motor Kiri
void encoderA(){
  enc[0] = digitalRead(encoderA1);
  vel[0].inc = 0;
  if (enc[0]>0){vel[0].inc=1;}
  else{vel[0].inc=-1;}
  vel[0].encoder += vel[0].inc;
  // velocity1(0);
}

//Encoder Motor Kanan
void encoderB(){
  enc[1] = digitalRead(encoderB1);
  vel[1].inc = 0;
  if (enc[1]>0){vel[1].inc=1;}
  else{vel[1].inc=-1;}
  vel[1].encoder += vel[1].inc;
  // velocity1(1);
}
