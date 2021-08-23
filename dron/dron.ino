#include <Wire.h>
#include <LSM303.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <Servo.h>

const uint64_t pipeIn = 0xE8E8F0F0E1LL; //Remember that this code is the same as in the transmitter

RF24 radio(9, 10);
LSM303 compass;

Servo ESC_L_F;
Servo ESC_R_F;
Servo ESC_L_B;
Servo ESC_R_B;   

struct Data {
  byte AxisX1;
  byte AxisY1;
  byte AxisX2;
  byte AxisY2;
};

Data data;

int relay = A3;

int min_ESC_value = 700;
int max_ESC_value = 2000;

float time_now, last_time, time_change;
float PWM_L_F, PWM_R_F, PWM_L_B, PWM_R_B;
unsigned long lastRecvTime = 0;

//wartosc PID dla regulatgora os x
int Set_angle_x, Gyro_angle_x;
float kp_x = 1;
float ki_x = 1;
float kd_x = 1;
float x_error, x_error_sum, x_last_error, x_d_error, x_p_out, x_i_out, x_d_out, x_PWM_value;

//wartosc PID dla regulatgora os y
int Set_angle_y, Gyro_angle_y;
float kp_y = 1;
float ki_y = 1;
float kd_y = 1;
float y_error, y_error_sum, y_last_error, y_d_error, y_p_out, y_i_out, y_d_out, y_PWM_value;

void resetData()
{ 
  data.AxisX1 = 127;
  data.AxisY1 = 127;
  data.AxisX2 = 127;
  data.AxisY2 = 127;
}

void ESC_Calibrate(int relay)
{
  //Switch off power to ESC driver
  digitalWrite(relay, LOW);
  //Set minimal value to control driver
  ESC_L_F.writeMicroseconds(max_ESC_value);
  ESC_R_F.writeMicroseconds(max_ESC_value);
  ESC_L_B.writeMicroseconds(max_ESC_value);
  ESC_R_B.writeMicroseconds(max_ESC_value);
  //wait 5s
  delay(5000);

  //Switch on powet to ESC driver
  digitalWrite(relay, HIGH);
  //Set maximum value to control driver
  ESC_L_F.writeMicroseconds(min_ESC_value);
  ESC_R_F.writeMicroseconds(min_ESC_value);
  ESC_L_B.writeMicroseconds(min_ESC_value);
  ESC_R_B.writeMicroseconds(min_ESC_value);
  //Wait 1s
  delay(1000);
}

void recvData()
{
  while (radio.available()) 
  {
    radio.read(&data, sizeof(Data));
    lastRecvTime = millis(); //here we receive the data
  }
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  compass.init();
  compass.enableDefault();
  
  resetData();
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);

  radio.openReadingPipe(1, pipeIn);
  //we start the radio comunication
  radio.startListening();

  ESC_L_F.attach(2);
  ESC_R_F.attach(3);
  ESC_L_B.attach(4);
  ESC_R_B.attach(5);      
}

void loop() {
  last_time = time_now;
  time_now = millis();
  time_change = time_now - last_time;
  
  recvData();
  //Here we check if we've lost signal, if we did we reset the values 
  if ( time_change > 1000 ) 
   {
    // Signal lost?
    resetData();
  }
  
  Set_angle_x = map(float(data.AxisX2), 0, 255, -10, 10);
  Gyro_angle_x = compass.a.x / 251.1;

  Set_angle_y = map(float(data.AxisY2), 0, 255, -10, 10);
  Gyro_angle_y = compass.a.y / 251.1;

 //PID regulator
  if (time_change >= 1000)
  {
    x_error = Set_angle_x - Gyro_angle_x;
    y_error = Set_angle_y - Gyro_angle_y;
    x_error_sum = x_error_sum + (x_error + x_last_error) * 0.5;
    y_error_sum = y_error_sum + (y_error + y_last_error) * 0.5;
    x_d_error = (x_error - x_last_error);
    y_d_error = (y_error - y_last_error);
    x_p_out = kp_x * x_error;
    y_p_out = kp_y * y_error;
    x_i_out = ki_x * x_error_sum;
    y_i_out = ki_y * y_error_sum;
    x_d_out = kd_x * x_d_error;
    y_d_out = kd_y * y_d_error;

    x_PWM_value = x_p_out + x_i_out + x_d_out;
    y_PWM_value = y_p_out + y_i_out + y_d_out;
    if (x_PWM_value < -400)
    {
      x_PWM_value = -400;
    }else if (x_PWM_value > 400)
    {
      x_PWM_value = 400;
    }
    x_last_error = x_error; 

    if (y_PWM_value < -400)
    {
      y_PWM_value = -400;
    }else if (y_PWM_value > 400)
    {
      y_PWM_value = 400;
    }
    x_last_error = x_error;
    y_last_error = y_error;  
  }
  PWM_L_F = 1000 - x_PWM_value - y_PWM_value;
  PWM_R_F = 1000 + x_PWM_value - y_PWM_value;
  PWM_L_B = 1000 - x_PWM_value + y_PWM_value;
  PWM_R_B = 1000 + x_PWM_value + y_PWM_value;

  if (PWM_L_B < 1000)
  {
    PWM_L_B = 1000;
  }
  if (PWM_L_B > 2000)
  {
    PWM_L_B = 2000;
  }


  if (PWM_R_B < 1000)
  {
    PWM_R_B = 1000;
  }
  if (PWM_R_B > 2000)
  {
    PWM_R_B = 2000;
  }
  
  ESC_L_B.writeMicroseconds(PWM_L_B);
  ESC_L_B.writeMicroseconds(PWM_R_B);
}
