#include "Motor_Control_Gyems.h"
#include "Speedgoat_Comm.h"
#include "ads1292r.h"
int gear_ratio = 8;
float kT = 3.0; // Constant torque [Nm/A] Max current: 7 A
int CAN_ID = 0;    // CAN port from Teensy. Teensy has two CAN port. We use CAN port 1
int Motor_ID = 1;  // Motor Can Bus ID
Motor_Control_Gyems m1(Motor_ID, CAN_ID); //Create motor object see Motor_Control_Gyems.h
Speedgoat_Comm SG; //Create Speedgoat COM object see Speefgoat_Comm.h
ads1292r torque_sensor1;  

float ref_signal_fl = 0.;
int ref_signal_min = -7;
int ref_signal_max = +7;

float LC_fl = 0.;
int LC_int = 0;
int LC_min  = -20;
int LC_max  = +20;

float mot_current_fl = 0.;
int mot_current_int = 0;
int mot_current_min = -33;
int mot_current_max = +33;

//float torqueSensorGain = 0.0003446 * (1) * 2 / 0.116;
//float torqueSensorGain = 0.0003446 * (1) * 2 / 0.04;
float torqueSensorGain = 0.0003446 * (1) * 2 / .58;
//float torqueSensorGain = 0.0003446 * (1) * 2 / 1.85;
unsigned long t_i = 0;
double t;
unsigned long t_pr1 = 0;
unsigned long t_pr2 = 0;
unsigned long Delta_T1 = 5000; // [micros]
unsigned long Delta_T2 = 900; // [micros]
double current_command;
int current_command_int = 0;
double current_limit = 5.0;
byte Array[]  = {1,2,3,4,5,6,7,8};

void setup() 
{
  SG.ref_signal_int=0;
  Serial3.begin(115200);
  Serial.begin(115200);
  Serial3.setTimeout(50);
  Serial.setTimeout(50);
  m1.init_motor_CAN();           // Start the CAN bus communication & Motor Control
  m1.read_PID();
  //m1.write_PID_RAM(30, 5, 30, 5, 30, 5);
  m1.write_PID_RAM(200, 5, 30, 5, 30, 5);
  torque_sensor1.Torque_sensor_initial(); //initial the torque sensor see ads1292r.cpp.
  //torque_sensor1.Torque_sensor_gain(0.0003446 * (-1) * 2 * 0.32, 0.0003446 * (-1) * 2 * 0.32); 
  //torque_sensor1.Torque_sensor_gain(0.0003446 * (-1) * 2 / 1.049, 0.0003446 * (-1) * 2 / 1.049); 
  torque_sensor1.Torque_sensor_gain(torqueSensorGain, torqueSensorGain); 
  torque_sensor1.Torque_sensor_offset_calibration();
}


void loop() 
{
  SG.Read_Data();
   
  t_i = micros();
  t = t_i/1000000.0;
   
  if (t_i - t_pr1 > Delta_T1)
  {
    t_pr1 = t_i;

    m1.read_motor_status_2();
    mot_current_fl = m1.iq_A;
    mot_current_int = SG.float_to_uint(mot_current_fl, mot_current_min, mot_current_max, 16);
    
    //Serial.print("encoder: "); 
    //Serial.print(mot_angle_fl); Serial.print("   ");
    Serial.print("loadcell: ");
    Serial.print(LC_fl); Serial.print("   ");
    Serial.print("current_command: ");
    Serial.print(current_command); Serial.print("   ");
    //Serial.print(LC_fl); Serial.print("   ");
    //Serial.print("current: ");
    //Serial.print(mot_current_fl); Serial.print("   ");
    Serial.println();
  }

  if (t_i - t_pr2 > Delta_T2)
  {    
    t_pr2 = t_i;        
    
    //position_command = 45 * sin(2 * 3.1415 * t_i / 1000000);//(uint degree  pre gear)
    SG.read_ref_signal(ref_signal_min, ref_signal_max);
    current_command = SG.ref_signal_fl;
    current_command_int = SG.float_to_uint(current_command*2.03, -32, 32, 16);
    
    //Serial.println(current_command);
    //current_command = max(current_command, -current_limit);
    //current_command = min(current_command, current_limit);
    m1.send_current_command(current_command);
    
    torque_sensor1.Torque_sensor_read();                   
    LC_fl = torque_sensor1.torque[0];
    LC_int = SG.float_to_uint(LC_fl, LC_min, LC_max, 16);
    
//    Array[0] = LC_int >> 8;
//    Array[1] = LC_int & 0xFF;
//    Array[2] = mot_current_int >> 8;
//    Array[3] = mot_current_int & 0xFF;

    Array[0] = LC_int >> 8;
    Array[1] = LC_int & 0xFF;
    Array[2] = current_command_int >> 8;
    Array[3] = current_command_int & 0xFF;
    
    Serial3.write(0xff);      //255
    Serial3.write(0x88);      //136
    Serial3.write(Array[0]);
    Serial3.write(Array[1]);
    Serial3.write(Array[2]);
    Serial3.write(Array[3]);
    Serial3.write(0x77);      //119

  }
    

}
