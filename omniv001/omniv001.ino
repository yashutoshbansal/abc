#include "MPU9250.h"
MPU9250 IMU;
double t;
int16_t gyro[3] = {0, 0, 0};
int16_t acc[3] = {0, 0, 0};
float gyroBias[3]  = {0, 0, 0};
float accelBias[3] = {0, 0, 0};
int dt;
long Time;
double r= 5.00;
double s=0;
double x = 0.0, y = 0.0;
double cosine , sine;
double offset;
double angle = 0.00;
double first_reading=0;
double angleX, angleY;
double x1, y1;
//double x1, y1;
double angle1 =0.00;
int Status;
int sda= 20;
int scl= 21;
int encoderPinA1 = 2;
int encoderPinB1 = 10;
int encoderPinA2 = 3;
int encoderPinB2 = 11;
//int t = 0;
//int avg=0;
int v1=150, v2=150, v3=150; 
int motorDir1 =1 ;
int PWM1 = 6;
int motorDir2 =4;
int PWM2 = 7;
int motorDir3 = 8 ;
int PWM3 = 5 ;
double error1= 0.00, previouserror1= 0.00;
double correction1= 0.00;
double error2= 0.00, previouserror2= 0.00;
double correction2= 0.00;
double tempX = 0.00, counterX = 0.00;
double tempY = 0.00, counterY = 0.00;
double counter= 0.00;
double Kp1 = 1.00 ,Kd1 =0.50;
double Kp2 = 6.00 ,Kd2 =3.50;
void setup() {
  // put your setup code here, to run once:
pinMode(motorDir1, OUTPUT);
pinMode(PWM1, OUTPUT);
pinMode(motorDir2, OUTPUT);
pinMode(PWM2, OUTPUT);
pinMode(motorDir3, OUTPUT);
pinMode(PWM3, OUTPUT);
pinMode(sda, INPUT);
pinMode(scl, INPUT);
pinMode(encoderPinA1, INPUT_PULLUP);
pinMode(encoderPinB1, INPUT_PULLUP);
attachInterrupt(0,enA1, CHANGE);
//attachInterrupt(1,enB1, CHANGE);
//int encoderPinA2= 2;
//int encoderPinB2= 3;
attachInterrupt(1,enA2, CHANGE);
//attachInterrupt(1,enB2, CHANGE);
//Serial.begin(9600);
//Status = IMU.begin();
////analogWrite(PWM, angle);
// for(int i=0; i<=1000; i++){
//    avg= avg+IMU.getGyroX_rads();
//  }
// // status = IMU.calibrateGyroX();
//   t = millis();
 Serial.begin(9600);
//  IMU.calibrateMPU9250(gyroBias,accelBias);
  for(int i=0;i<1000;i++)
  { getgyro();
    
    s+=gyro[2];
  }
  s=s/1000;
 // while(millis()<=1100)
//{
   IMU.readGyroData(gyro);
  dt = micros() - Time;
  angle += (gyro[2]-s)*dt/131.0/1000000.0;
  Time = micros();
  first_reading=angle;  
  Serial.println("fr");
  Serial.println(first_reading);
   angle=angle-first_reading;
//}
}
void loop() {
  // put your main code here, to run repeatedly:
  if(tempX!=counterX){
    angleX= ((counterX-tempX)*360)/1200;
    tempX = counterX;
  }
  Serial.println(angleX);
  if(tempY!=counterY){
    
    angleY= ((counterY-tempY)*360)/1200;
    tempY = counterY;
  }
  Serial.println(angleY);
  updatePid1();
// if(correction1< 0){
//    digitalWrite(motorDir1, HIGH);
//    digitalWrite(motorDir2, HIGH);
//    digitalWrite(motorDir3, HIGH);
//  }
//  else{
//    digitalWrite(motorDir1, LOW);
//    digitalWrite(motorDir2, LOW);
//    digitalWrite(motorDir3, LOW);
//  }
  
  Serial.println(angle);
  //angle_();
  getgyro();
Serial.println(angle);
//  Serial.println(s);
  updatePid2();
  Run();
  //IMU.readSensor();
  IMU.readGyroData(gyro);
  x1= x1+(tempX*r)*cos(angle1)+ (tempY*r)*sin(angle1);
  y1= y1+(tempX*r)*sin(angle1)- (tempY*r)*cos(angle1);
}
void updatePid1()
{
  error1=( y1-(x1*tan(angle1)));
  correction1 =(Kp1*error1)+ (Kd1*(error1-previouserror1));
  previouserror1 = error1;
  Serial.print("error1=");
  Serial.println(error1);
  Serial.print("correction1=");
  Serial.println(correction1);
}
void enA1(){
  if(digitalRead(encoderPinA1) == digitalRead(encoderPinB1)) counter ++;
  else counter--;
}
//void enB1(){
//  if (digitalRead(encoderPinB1)!=digitalRead(encoderPinA1)) counter++;
//  else counter--;
//}

void enA2(){
  if(digitalRead(encoderPinA2) == digitalRead(encoderPinB2)) counter ++;
  else counter--;
}


//void enB2(){
//  if (digitalRead(encoderPinB2)!=digitalRead(encoderPinA2)) counter++;
//  else counter--;
//}


//void angle_(){
//  IMU.readSensor();
//  angle = angle + (millis()-t)*(IMU.getGyroX_rads()-avg);
//  t = millis();
//  Serial.print("angle=");
//  Serial.print(angle);
//}
void getgyro()
{
  IMU.readGyroData(gyro);
  dt = micros() - Time;

  angle += (gyro[2]-s)*dt/131.0/1000000.0;
  Time = micros();
}


void updatePid2()
{
  error2= (angle);
  correction2 =(Kp2*error2)+ (Kd2*(error2-previouserror2));
  previouserror2 = error2;
  Serial.print("error2=");
  Serial.println(error2);
  Serial.print("correction2=");
  Serial.println(correction2);
}
void Run()
{
  int left_motorspeed= v1*cos(-angle1+(5*PI/6))-correction1-correction2;
  {
  if(left_motorspeed<0){
    analogWrite(PWM1,(-1)*left_motorspeed);
    digitalWrite(motorDir1,LOW);
  }
  else{
    analogWrite(PWM1,left_motorspeed);
    digitalWrite(motorDir1, HIGH);
  }
  Serial.print("left_motorspeed=");
  Serial.println(left_motorspeed);
  }
  int right_motorspeed= v2*cos(angle1-(PI/6))+correction1+correction2;
  {
  if(right_motorspeed<0){
    analogWrite(PWM2,(-1)*right_motorspeed);
    digitalWrite(motorDir2,HIGH);
  }
  else{
   analogWrite(PWM2,right_motorspeed);
   digitalWrite(motorDir2,LOW); 
  }
  Serial.print("right_motorspeed=");
  Serial.println(right_motorspeed);
  }
  Serial.println("v3=");
  Serial.println(v3);
  int front_motorspeed= v3*cos(angle1)+correction1+correction2;
  {
  if(front_motorspeed<0){
    analogWrite(PWM3,(-1)*front_motorspeed);
    digitalWrite(motorDir3,HIGH); 
  }
  else{
    analogWrite(PWM3,front_motorspeed);
    digitalWrite(motorDir3,LOW); 
  }
  Serial.print("front_motorspeed=");
  Serial.println(front_motorspeed);
}
}
