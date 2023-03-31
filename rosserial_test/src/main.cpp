#include <Arduino.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <PID.h>
// #include <parsing.h>  //Delimiter (;)

struct encoder_param
{
    float encoder = 0;
    float inc = 0;
};

ros::NodeHandle nh;
int update_nh = 0;

//Variabel Linear dan Angular Motor
float demandx = 0, demandz = 0;
float demand_speed[2] = {0,0};
float demand_speed_left = 0, demand_speed_right = 0;

//Variabel Posisi Robot
float pos_left = 0;
float pos_right = 0;

//Variabel msgs String
unsigned char lw[6];
unsigned char rw[6];

//Fungsi Subcriber
void cmd_vel_cb(const geometry_msgs::Twist& twist){
  demandx = twist.linear.x;
  demandz = twist.angular.z;
}

//Setting Subcriber dan Publisher
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb);
geometry_msgs::Vector3Stamped speed_msg;
ros::Publisher speed_pub("speed", &speed_msg);
// std_msgs::UInt16 left_wheel_msg;
// ros::Publisher left_wheel_pub("lwheel", &left_wheel_msg);
// std_msgs::UInt16 right_wheel_msg;
// ros::Publisher right_wheel_pub("rwheel", &right_wheel_msg);

struct velocity_param
{
    float kecepatan = 0;
    float encoder = 0;
    float rpm = 0;
    float pulse_rev = 48;
    unsigned long curr_time = 0;
    unsigned long prev_time = 0;
    float inc = 0;
    float avg_rpm = 0;
    int u = 0;
    float delta_time = 0;
    float act_rpm = 0;
    float act_speed = 0;
};

//Header LIB PID
PID_params param[2];
velocity_param vel[2];
encoder_param coder[2];

//Function
void encoderA();
void initialize();
void encoderB();
void M1_maju(int speed_m);
void M2_maju(int speed_m);
void M1_mundur(int speed_m);
void M2_mundur(int speed_m);
void velocity1(int i);
float velocity(int i);
void avg_fill(int i, int N);
void publishPos(double time);
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);

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

//PID
int control_PID[2] = {0,0};
float SP = 0;
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

//Parameter Kecepatan Motor DC
float M1_speed = 0;
float maxControl = 50;
float minControl = -50;
float kecepatan = 0;
float encoder = 0;
float pulse_rev = 48;
unsigned long curr_time = 0;
unsigned long prev_time = 0;
#define ts 100
#define phi 3.14
float inc = 0;
float delta_time = 0;
float r = 0.0335;

//Parameter Kecepatan dengan menyamakan jumlah encoder
float encoder_diff[2] = {0, 0};
float encoder_prev[2] = {0, 0};
float speed_left = 0;
float speed_right = 0;
float c = 0;

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

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup(){
  nh.getHardware()->setBaud(115200);
  // Serial.begin(57600);
  // Serial.begin(115200);
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

  param[0].KP = 1.5; //0
  param[0].KI = 0.0; //0
  param[0].KD = 0.0; //0
  param[1].KP = 1.6; //0
  param[1].KI = 0.0; //0
  param[1].KD = 0; //0

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(speed_pub);
  // nh.advertise(left_wheel_pub);
  // nh.advertise(right_wheel_pub);

  // initialize();

}

void loop(){
  if (millis() - curr_time > ts){
    curr_time = millis();

    demand_speed_left = demandx - (demandz*0.0875);
    demand_speed_right = demandx + (demandz*0.0875);

    //Konversi m/s ke RPM
    c = phi*(r*2);
    vel[0].act_rpm = int((demand_speed_left/c) * 60.0);
    vel[1].act_rpm = int((demand_speed_right/c) * 60.0);

    param[0].set_point = vel[0].act_rpm;
    param[1].set_point = vel[1].act_rpm;

    vel[0].rpm = (vel[0].kecepatan*48)/60; //RPM
    vel[1].rpm = (vel[1].kecepatan*48)/60; //RPM
    avg_fill(0, 600);
    avg_fill(1, 600);
    param[0].feedback = vel[0].avg_rpm/2;
    param[1].feedback = vel[1].avg_rpm/2;

    //Konversi RPM ke m/s
    vel[0].act_speed = (vel[0].avg_rpm*c)/60;
    vel[1].act_speed = (vel[1].avg_rpm*c)/60;

    control_PID[0] = (int) PID(param[0]);
    control_PID[1] = (int) PID(param[1]);

    encoder_diff[0] = vel[0].encoder - encoder_prev[0];
    encoder_diff[1] = vel[1].encoder - encoder_prev[1];
    speed_left = encoder_diff[0]/50.0;
    speed_right = encoder_diff[1]/50.0;
    encoder_prev[0] = vel[0].encoder;
    encoder_prev[1] = vel[1].encoder;

    if (control_PID[0] >= 0) M1_maju(control_PID[0]);
    if (control_PID[0] < 0) M1_mundur(control_PID[0]*-1);
     
    
    if (control_PID[1] >= 0) M2_maju(control_PID[1]);
    if (control_PID[1] < 0) M2_mundur(control_PID[1]*-1);

    // if (update_nh > 10){
    //   nh.spinOnce();
    //   update_nh = 0;
    // }
    // else {update_nh++;}
    nh.spinOnce();
    publishPos(ts);
  } 
}

//Pengiriman data odometry ke Jetson Nano
void publishPos(double time){
  speed_msg.header.stamp = nh.now();
  speed_msg.vector.x = speed_left;
  speed_msg.vector.y = speed_right;
  speed_msg.vector.z = time/1000;
  speed_pub.publish(&speed_msg);
  // left_wheel_msg.data = param[0].feedback;
  // right_wheel_msg.data = param[1].feedback;
  // left_wheel_pub.publish(&left_wheel_msg);
  // right_wheel_pub.publish(&right_wheel_msg);
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

float velocity(int i){
    vel[i].curr_time = millis();
    if(vel[i].curr_time - vel[i].prev_time > ts){
        vel[i].prev_time = vel[i].curr_time;
        vel[i].kecepatan = (float)(vel[i].encoder*600/vel[i].pulse_rev);
        // Serial.print(vel[i].kecepatan);
        vel[i].encoder = 0;
    }
    return 0;
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
  velocity1(0);
}

//Encoder Motor Kanan
void encoderB(){
  enc[1] = digitalRead(encoderB1);
  vel[1].inc = 0;
  if (enc[1]>0){vel[1].inc=1;}
  else{vel[1].inc=-1;}
  vel[1].encoder += vel[1].inc;
  velocity1(1);
}
