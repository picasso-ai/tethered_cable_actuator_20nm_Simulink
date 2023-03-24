#include "Motor_Control_Geminic.h"

Motor_Control_Geminic::Motor_Control_Geminic(uint8_t id, int Can_id)
{
  ID = id;
}
Motor_Control_Geminic::~Motor_Control_Geminic()
{}
void Motor_Control_Geminic::init_motor_CAN()
{
  //initial CAN Bus
  Can0.begin(1000000, defaultMask, 1, 1);
  delay(3000);
  pinMode(28, OUTPUT);
  digitalWrite(28, LOW);
  Serial.println("Can bus setup done...");
  Serial.println("*******************************initial motor model***********************************");
  //clear Motor
  close_motor();
  delay(1000);
  stop_motor();
  delay(1000);
  clear_motor_error();

  delay(1000);
  //ReStar Motor
  start_motor();
  delay(1000);
  //Read Initial Position
  //read_PID();  //send read all PI Gain command
  delay(10);
  read_encoder();
  delay(10);
  read_multi_turns_angle();
  delay(10);
  read_single_turns_angle();
  delay(10);
  read_acceleration();
  delay(10);
  read_motor_status();
  delay(10);
  read_motor_status_2();
  delay(10);
  read_motor_status_3();
  delay(10);
}
//////////////////Check if there is a CAN message, then read it///////////////
void Motor_Control_Geminic::receive_CAN_data()
{
  while (Can0.available() > 0)
  {
    Can0.read(msgR);
    DataExplanation(msgR);
  }
}
////////////////////////////////////////////////////////////////////////////
////////////////Received CAN Message Decoding////////////////////////////////
void Motor_Control_Geminic::DataExplanation(CAN_message_t msgR2)
{
  int len = msgR2.len;
  if (len == 8)
  {
    switch(msgR2.buf[0])
    {
      case 0x30://1.Read PID gain
        anglePidKp = msgR2.buf[2];
        anglePidKi = msgR2.buf[3];
        speedPidKp = msgR2.buf[4];
        speedPidKi = msgR2.buf[5];
        iqPidKp = msgR2.buf[6];
        iqPidKi = msgR2.buf[7];
        Serial.println("Success to read PID gain");
        Serial.print("anglePidKp:");
        Serial.print(anglePidKp);
        Serial.print("; anglePidKi:");
        Serial.print(anglePidKi);
        Serial.print("; speedPidKp:");
        Serial.print(speedPidKp);
        Serial.print("; speedPidKi:");
        Serial.print(speedPidKi);
        Serial.print("; iqPidKp:");
        Serial.print(iqPidKp);
        Serial.print("; iqPidKi:");
        Serial.println(iqPidKi);
        break;
      case 0x31: //2.write PID to RAM
        anglePidKp = msgR2.buf[2];
        anglePidKi = msgR2.buf[3];
        speedPidKp = msgR2.buf[4];
        speedPidKi = msgR2.buf[5];
        iqPidKp = msgR2.buf[6];
        iqPidKi = msgR2.buf[7];
        Serial.println("Success to write PID to RAM");
        Serial.print("anglePidKp:");
        Serial.print(anglePidKp);
        Serial.print("; anglePidKi:");
        Serial.print(anglePidKi);
        Serial.print("; speedPidKp:");
        Serial.print(speedPidKp);
        Serial.print("; speedPidKi:");
        Serial.print(speedPidKi);
        Serial.print("; iqPidKp:");
        Serial.print(iqPidKp);
        Serial.print("; iqPidKi:");
        Serial.println(iqPidKi);
        break;
      case 0x32: //3.write PID to ROM 
        anglePidKp = msgR2.buf[2];
        anglePidKi = msgR2.buf[3];
        speedPidKp = msgR2.buf[4];
        speedPidKi = msgR2.buf[5];
        iqPidKp = msgR2.buf[6];
        iqPidKi = msgR2.buf[7];
        Serial.println("Success to write PID to ROM");
        Serial.print("anglePidKp:");
        Serial.print(anglePidKp);
        Serial.print("; anglePidKi:");
        Serial.print(anglePidKi);
        Serial.print("; speedPidKp:");
        Serial.print(speedPidKp);
        Serial.print("; speedPidKi:");
        Serial.print(speedPidKi);
        Serial.print("; iqPidKp:");
        Serial.print(iqPidKp);
        Serial.print("; iqPidKi:");
        Serial.println(iqPidKi);
        break;
      case 0x33: //4.read Accel
        acceleration_uint32 = (uint32_t)(((uint32_t)msgR2.buf[7] << 24) | ((uint32_t)msgR2.buf[6] << 16) | ((uint32_t)msgR2.buf[5] << 8) | ((uint32_t)msgR2.buf[4]));
        acceleration_int32 = (int32_t)acceleration_uint32;
        Accel = acceleration_int32; //unit 1dps/s dps(degree per second)
        Serial.println("Success to read parameter Accel");
        Serial.print("Accel:");
        Serial.println(Accel);        
        break;
      case 0x34: //5.write Accel to RAM
        acceleration_uint32 = (uint32_t)(((uint32_t)msgR2.buf[7] << 24) | ((uint32_t)msgR2.buf[6] << 16) | ((uint32_t)msgR2.buf[5] << 8) | ((uint32_t)msgR2.buf[4]));
        acceleration_int32 = (int32_t)acceleration_uint32;
        Accel = acceleration_int32; //unit 1dps/s dps(degree per second)
        Serial.println("Success to write parameter Accel to RAM");
        Serial.print("Accel:");
        Serial.println(Accel);
        break;
      case 0x90: //6: read encoder (encoder, encoderRaw,encoderOffset)
        encoder = (uint16_t)(((uint16_t)msgR2.buf[3] << 8) | ((uint16_t)msgR2.buf[2]));
        encoderRaw = (uint16_t)(((uint16_t)msgR2.buf[5] << 8) | ((uint16_t)msgR2.buf[4]));
        encoderOffset = (uint16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));
        Serial.println("Success to read encoder parameters:");
        Serial.print("encoder:");
        Serial.print(encoder);  
        Serial.print("; encoderRaw:");
        Serial.print(encoderRaw);
        Serial.print("; encoderOffset:");
        Serial.println(encoderOffset);      
        break;
      case 0x91: //7:write certain position as zero position to ROM
        encoderOffset = (uint16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));
        Serial.print("Success to write the encoderOffset as ");
        Serial.println(encoderOffset);
        break;
      case 0x19: //8:write current position as zero position to ROM
        encoderOffset = (uint16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));
        Serial.println("Success to write the current position as zero");
        Serial.print("encoderOffset is ");
        Serial.println(encoderOffset);
        break;
      case 0x92: //9: read multi-turn motorAngle (it is int32 datatype ; datasheet is wrong)
//        motorAngle_uint64 = (uint64_t)( (0x00 << 54)|((uint64_t)msgR2.buf[7] << 48) | ((uint64_t)msgR2.buf[6] << 40) | ((uint64_t)msgR2.buf[5] << 32) | ((uint64_t)msgR2.buf[4] << 24) | ((uint64_t)msgR2.buf[3] << 16) | ((uint64_t)msgR2.buf[2] << 8) | ((uint64_t)msgR2.buf[1]));        
//        int64_t motorAngle_int64=(int64_t)motorAngle_uint64;
//        motorAngle = ((double)motorAngle_int64*0.01);
        motorAngle_int32 = (int32_t)( ((uint32_t)msgR2.buf[4] << 24) | ((uint32_t)msgR2.buf[3] << 16) | ((uint32_t)msgR2.buf[2] << 8) | ((uint32_t)msgR2.buf[1] )) ;
        motorAngle= ((double) motorAngle_int32*0.01);
     
        break;
      case 0x94: //10: read single turn circle angle
        circleAngle = (uint16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));
        Serial.print("Success to read position: "); Serial.print("circleAngle is "); Serial.println(circleAngle);
        break;
      case 0x95: //11: clear angle command
        Serial.println("clear all command and set current position as zero");
        break;
      case 0x9A: //12: read motor status temperature, voltage, errorState
        temperature = (int8_t)msgR2.buf[1];
        voltage = (uint16_t)(((uint16_t)msgR2.buf[4] << 8) | ((uint16_t)msgR2.buf[3]));
        errorState = msgR2.buf[7];
        Serial.println("successful to read motor status");
        Serial.print("temperature is ");
        Serial.print(temperature); 
        Serial.print("; voltage is ");
        Serial.print(voltage);
        Serial.print("; errorState is ");
        Serial.println(errorState);
        if (errorState & 0x01)
        {
          Serial.println("low voltage protection");
        }
        else if (errorState & 0x08)
        {
          Serial.println("Over temperature protection");
        }
        break;
      case 0x9B: //13: clear motor error status return temperature, voltage, errorState
        temperature = (int8_t)msgR2.buf[1];
        voltage = (uint16_t)(((uint16_t)msgR2.buf[4] << 8) | ((uint16_t)msgR2.buf[3]));
        errorState = msgR2.buf[7];
        Serial.println("successful to clear motor error");
        Serial.print("temperature is ");
        Serial.print(temperature); 
        Serial.print("; voltage is ");
        Serial.print(voltage);
        Serial.print("; errorState is ");
        Serial.println(errorState);
        if (errorState & 0x01)
        {
          Serial.println("low voltage protection");
        }
        else if (errorState & 0x08)
        {
          Serial.println("Over temperature protection");
        }
        break;
      case 0x9C: //14: read motor status 2 return temperature, iq current(-2048~2048 mapping-33A to 33A), speed (1dps/LSB), encoder (0~16383)
        temperature = (int8_t)msgR2.buf[1];
        iq = (int16_t)(((uint16_t)msgR2.buf[3] << 8) | ((uint16_t)msgR2.buf[2]));
        iq_A = ((double)iq) * 33 / 2048;
        speed_value = (int16_t)(((uint16_t)msgR2.buf[5] << 8) | ((uint16_t)msgR2.buf[4]));
        encoder = (uint16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));
//        Serial.println("successful to read motor status 2");
//        Serial.print("temperature is ");
//        Serial.print(temperature); 
//        Serial.print("; iq_A is ");
//        Serial.print(iq_A);
//        Serial.print("; speed_value is ");
//        Serial.print(speed_value);
//        Serial.print("; encoder is ");
//        Serial.println(encoder);        
        break;
      case 0x9D: //15: read motor status 3 return temperature, phase A B C current
        temperature = (int8_t)msgR2.buf[1];
        iA = (int16_t)(((uint16_t)msgR2.buf[3] << 8) | ((uint16_t)msgR2.buf[2]));
        iA_A = ((double)iA) / 64;
        iB = (int16_t)(((uint16_t)msgR2.buf[5] << 8) | ((uint16_t)msgR2.buf[4]));
        iB_A = ((double)iB) / 64;
        iC = (int16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));
        iC_A = ((double)iC) / 64;
        Serial.println("successful to read motor status 3");
        Serial.print("temperature is ");
        Serial.print(temperature); 
        Serial.print("; iA_A is ");
        Serial.print(iA_A);
        Serial.print("; iB_A is ");
        Serial.print(iB_A);
        Serial.print("; iC_A is ");
        Serial.println(iC_A);
        break;
      case 0x80: //16: close motor and clear all command
        Serial.println("successful to close motor and clear all command");
        break;
      case 0x81: //17: stop motor and keep all command
        Serial.println("successful to stop motor and keep all command");
        break;
      case 0x88: //18: start motor
        Serial.println("successful to start motor and resume previous control method");
        break;
      case 0xA1: //19: send current command and return temperature, iq, speed, encoder
        if (msgR2.id == 0x141)
        {
          temperature = (int8_t)msgR2.buf[1];
          iq = (int16_t)(((uint16_t)msgR2.buf[3] << 8) | ((uint16_t)msgR2.buf[2]));
          iq_A = ((double)iq) * 33 / 2048;
          speed_value = (int16_t)(((uint16_t)msgR2.buf[5] << 8) | ((uint16_t)msgR2.buf[4]));
          encoder = (uint16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));
        }
        else if(msgR2.id == 0x142)
        {
          temperature2 = (int8_t)msgR2.buf[1];
          iq2 = (int16_t)(((uint16_t)msgR2.buf[3] << 8) | ((uint16_t)msgR2.buf[2]));
          iq_A2 = ((double)iq) * 33 / 2048;
          speed_value2 = (int16_t)(((uint16_t)msgR2.buf[5] << 8) | ((uint16_t)msgR2.buf[4]));
          encoder2 = (uint16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));
        }
        else if(msgR2.id == 0x143)
        {
          temperature3 = (int8_t)msgR2.buf[1];
          iq3 = (int16_t)(((uint16_t)msgR2.buf[3] << 8) | ((uint16_t)msgR2.buf[2]));
          iq_A3 = ((double)iq) * 33 / 2048;
          speed_value3 = (int16_t)(((uint16_t)msgR2.buf[5] << 8) | ((uint16_t)msgR2.buf[4]));
          encoder3 = (uint16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));
        }
        else if(msgR2.id == 0x144)
        {
          temperature4 = (int8_t)msgR2.buf[1];
          iq4 = (int16_t)(((uint16_t)msgR2.buf[3] << 8) | ((uint16_t)msgR2.buf[2]));
          iq_A4 = ((double)iq) * 33 / 2048;
          speed_value4 = (int16_t)(((uint16_t)msgR2.buf[5] << 8) | ((uint16_t)msgR2.buf[4]));
          encoder4 = (uint16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));
        }        
        break;
      case 0xA2: //20: send speed command and return temperature, iq, speed, encoder
        temperature = (int8_t)msgR2.buf[1];
        iq = (int16_t)(((uint16_t)msgR2.buf[3] << 8) | ((uint16_t)msgR2.buf[2]));
        iq_A = ((double)iq) * 33 / 2048;
        speed_value = (int16_t)(((uint16_t)msgR2.buf[5] << 8) | ((uint16_t)msgR2.buf[4]));
        encoder = (uint16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));
        break;
      case 0xA3: //21: send position command and return temperature, iq, speed, encoder
        temperature = (int8_t)msgR2.buf[1];
        iq = (int16_t)(((uint16_t)msgR2.buf[3] << 8) | ((uint16_t)msgR2.buf[2]));
        iq_A = ((double)iq) * 33 / 2048;
        speed_value = (int16_t)(((uint16_t)msgR2.buf[5] << 8) | ((uint16_t)msgR2.buf[4]));
        encoder = (uint16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));
        //Serial.print("Success reading position: "); Serial.println(encoder);
        break;
      case 0xA4: //22: send position command 2(mulitturn command) and return temperature, iq, speed, encoder
        temperature = (int8_t)msgR2.buf[1];
        iq = (int16_t)(((uint16_t)msgR2.buf[3] << 8) | ((uint16_t)msgR2.buf[2]));
        iq_A = ((double)iq) * 33 / 2048;
        speed_value = (int16_t)(((uint16_t)msgR2.buf[5] << 8) | ((uint16_t)msgR2.buf[4]));
        encoder = (uint16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));
        break;
      case 0xA6: //23: send position command 3(single turn command) and return temperature, iq, speed, encoder
        temperature = (int8_t)msgR2.buf[1];
        iq = (int16_t)(((uint16_t)msgR2.buf[3] << 8) | ((uint16_t)msgR2.buf[2]));
        iq_A = ((double)iq) * 33 / 2048;
        speed_value = (int16_t)(((uint16_t)msgR2.buf[5] << 8) | ((uint16_t)msgR2.buf[4]));
        encoder = (uint16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));
        break;
      default:
        break;
    }
  }
}
/////////////////////////////////////////////////////////////////////

//******1.Read PID data command******//
void Motor_Control_Geminic::read_PID()
{
  msgW.id = 0x140 + ID;
  msgW.ext = 0;
  msgW.len = 8;
  msgW.rtr = 0;
  msgW.buf[0] = 0x30;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can0.write(msgW);
  delay(1);
  receive_CAN_data();
}
//******2.Write PID gain to RAM******//
void Motor_Control_Geminic::write_PID_RAM(uint8_t Kp_pos,uint8_t Ki_pos,uint8_t Kp_vel,uint8_t Ki_vel,uint8_t Kp_cur,uint8_t Ki_cur)
{
  msgW.id = 0x140 + ID;
  msgW.ext = 0;
  msgW.len = 8;
  msgW.rtr = 0;
  msgW.buf[0] = 0x31;
  msgW.buf[1] = 0;
  msgW.buf[2] = Kp_pos;
  msgW.buf[3] = Ki_pos;
  msgW.buf[4] = Kp_vel;
  msgW.buf[5] = Ki_vel;
  msgW.buf[6] = Kp_cur;
  msgW.buf[7] = Ki_cur;
  Can0.write(msgW);
  delay(1);
  receive_CAN_data();
}

//******3.Write PID gain to ROM******//
void Motor_Control_Geminic::write_PID_ROM(uint8_t Kp_pos,uint8_t Ki_pos,uint8_t Kp_vel,uint8_t Ki_vel,uint8_t Kp_cur,uint8_t Ki_cur)
{
  msgW.id = 0x140 + ID;
  msgW.ext = 0;
  msgW.len = 8;
  msgW.rtr = 0;
  msgW.buf[0] = 0x32;
  msgW.buf[1] = 0;
  msgW.buf[2] = Kp_pos;
  msgW.buf[3] = Ki_pos;
  msgW.buf[4] = Kp_vel;
  msgW.buf[5] = Ki_vel;
  msgW.buf[6] = Kp_cur;
  msgW.buf[7] = Ki_cur;
  Can0.write(msgW);
  delay(1);
  receive_CAN_data();
}

//******4.Read Acceleration******//
void Motor_Control_Geminic::read_acceleration()
{
  msgW.id = 0x140 + ID;
  msgW.ext = 0;
  msgW.len = 8;
  msgW.rtr = 0;
  msgW.buf[0] = 0x33;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can0.write(msgW);
  delay(1);
  receive_CAN_data();
}
//******5.Write Acceleration RAM******//
void Motor_Control_Geminic::write_acceleration_RAM()
{
  msgW.id = 0x140 + ID;
  msgW.ext = 0;
  msgW.len = 8;
  msgW.rtr = 0;
  msgW.buf[0] = 0x34;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = *(uint8_t*)(&Accel);
  msgW.buf[5] = *((uint8_t*)(&Accel) + 1);
  msgW.buf[6] = *((uint8_t*)(&Accel) + 2);
  msgW.buf[7] = *((uint8_t*)(&Accel) + 3);
  Can0.write(msgW);
  delay(1);
  receive_CAN_data();
}
//******6.Read encoder Position******//
void Motor_Control_Geminic::read_encoder()
{
  msgW.id = 0x140 + ID;
  msgW.ext = 0;
  msgW.len = 8;
  msgW.rtr = 0;
  msgW.buf[0] = 0x90;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can0.write(msgW);
  receive_CAN_data();
}
//******7.Write ENCODER OFFSET ROM******//
void Motor_Control_Geminic::write_encoder_offset_RAM(uint16_t encoder_Offset )
{
  encoderOffset = encoder_Offset;
  msgW.id = 0x140 + ID;
  msgW.ext = 0;
  msgW.len = 8;
  msgW.rtr = 0;
  msgW.buf[0] = 0x91;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = *(uint8_t*)(&encoderOffset);
  msgW.buf[7] = *((uint8_t*)(&encoderOffset) + 1);
  Can0.write(msgW);
  delay(1);
  receive_CAN_data();
}
//******8.Write current postioton as Zero degree******//
void Motor_Control_Geminic::write_current_position_as_zero_position()
{
  msgW.id = 0x140 + ID;
  msgW.ext = 0;
  msgW.len = 8;
  msgW.rtr = 0;
  msgW.buf[0] = 0x19;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can0.write(msgW);
  delay(1);
  receive_CAN_data();
}
//******9.Read multi turns angle command******//
void Motor_Control_Geminic::read_multi_turns_angle()
{
  msgW.id = 0x140 + ID;
  msgW.ext = 0;
  msgW.len = 8;
  msgW.rtr = 0;
  msgW.buf[0] = 0x92;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can0.write(msgW);
  //delayMicroseconds(500);
  receive_CAN_data();
}
//******10.Read single circle angle command******//
void Motor_Control_Geminic::read_single_turns_angle()
{
  msgW.id = 0x140 + ID;
  msgW.ext = 0;
  msgW.len = 8;
  msgW.rtr = 0;
  msgW.buf[0] = 0x94;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can0.write(msgW);
  //delay(1);
  receive_CAN_data();
}
//(current cannot use it)******11.clear all angle command and offset currnet position as zero command (unit 0.01deg/LSB)******//
void Motor_Control_Geminic::clear_motor_angle_command()
{
  msgW.id = 0x140 + ID;
  msgW.ext = 0;
  msgW.len = 8;
  msgW.rtr = 0;
  msgW.buf[0] = 0x95;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can0.write(msgW);
  delay(1);
  receive_CAN_data();
}
//******12.read motor status and error (temperature(1degreeC/LSB), voltage(0.1V/LSB), errorState:(0: normal, 1:abnormal)0bit Voltage 3bit temperature) ******//
void Motor_Control_Geminic::read_motor_status()
{
  msgW.id = 0x140 + ID;
  msgW.ext = 0;
  msgW.len = 8;
  msgW.rtr = 0;
  msgW.buf[0] = 0x9A;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can0.write(msgW);
  delay(1);
  receive_CAN_data();
}
//******13.clear motot error and reset motor******//
void Motor_Control_Geminic::clear_motor_error()
{
  msgW.id = 0x140 + ID;
  msgW.ext = 0;
  msgW.len = 8;
  msgW.rtr = 0;
  msgW.buf[0] = 0x9B;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can0.write(msgW);
  delay(1);
  receive_CAN_data();
}
//******14.read motor status 2 (temperature 1degreeC/LSB, iq(-2048~2048 mapping to -33A ~33A), speed(1dps/LSB), 14 bit encoder value(0~16383))******//
void Motor_Control_Geminic::read_motor_status_2()
{
  msgW.id = 0x140 + ID;
  msgW.ext = 0;
  msgW.len = 8;
  msgW.rtr = 0;
  msgW.buf[0] = 0x9C;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can0.write(msgW);
  //delay(1);
  receive_CAN_data();
}
//******15.read motor status 3 (temperature 1degreeC/LSB,A phase current(1A/64LSB),B phase current(1A/64LSB),C phase current(1A/64LSB) )******//
void Motor_Control_Geminic::read_motor_status_3()
{
  msgW.id = 0x140 + ID;
  msgW.ext = 0;
  msgW.len = 8;
  msgW.rtr = 0;
  msgW.buf[0] = 0x9D;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can0.write(msgW);
  delay(1);
  receive_CAN_data();
}
//******16.close motor and clear all command******//
void Motor_Control_Geminic::close_motor()
{
  msgW.id = 0x140 + ID;
  msgW.ext = 0;
  msgW.len = 8;
  msgW.rtr = 0;
  msgW.buf[0] = 0x80;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can0.write(msgW);
  delay(1);
  receive_CAN_data();
}
//******17.stop motor but don't clear any previous command******//
void Motor_Control_Geminic::stop_motor()
{
  msgW.id = 0x140 + ID;
  msgW.ext = 0;
  msgW.len = 8;
  msgW.rtr = 0;
  msgW.buf[0] = 0x81;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can0.write(msgW);
  delay(1);
  receive_CAN_data();
}
//******18.start motor******//
void Motor_Control_Geminic::start_motor()
{
  msgW.id = 0x140 + ID;
  msgW.ext = 0;
  msgW.len = 8;
  msgW.rtr = 0;
  msgW.buf[0] = 0X88;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  if (Can0.write(msgW))
  {
    Serial.println("Succeful to send start motor command");
  }
  else
  {
    Serial.println("Fail to send start motor command");
  }
  delay(1);
  receive_CAN_data();
}
//******19.current control: send current command current unit A(not limited by maximum Torque Current)******//
void Motor_Control_Geminic::send_current_command(double current)
{
  current = current * 2000 / 32;
  iqControl = (int16_t)current;
  msgW.id = 0x140 + ID;
  msgW.ext = 0;
  msgW.len = 8;
  msgW.rtr = 0;
  msgW.buf[0] = 0xA1;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = *(uint8_t*)(&iqControl);
  msgW.buf[5] = *((uint8_t*)(&iqControl) + 1);
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can0.write(msgW);
  //delay(1);
  //receive_CAN_data();
}
//******20.speed control: send speed command speed unit dps******//
void Motor_Control_Geminic::send_speed_command(double speedvalue)
{
  speedvalue = speedvalue * 100;
  speedControl = (int32_t)speedvalue;
  msgW.id = 0x140 + ID;
  msgW.ext = 0;
  msgW.len = 8;
  msgW.rtr = 0;
  msgW.buf[0] = 0xA2;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = *(uint8_t*)(&speedControl);
  msgW.buf[5] = *((uint8_t*)(&speedControl) + 1);
  msgW.buf[6] = *((uint8_t*)(&speedControl) + 2);
  msgW.buf[7] = *((uint8_t*)(&speedControl) + 3);
  Can0.write(msgW);
  delay(1);
  receive_CAN_data();
}
//******21.position control:send position command (angle unit degree)******//
void Motor_Control_Geminic::send_position_command(double angle)
{
  angle = angle * 100;
  angleControl = (int32_t)angle;
  msgW.id = 0x140 + ID;
  msgW.ext = 0;
  msgW.len = 8;
  msgW.rtr = 0;
  msgW.buf[0] = 0xA3;
  msgW.buf[1] = 0x00;
  msgW.buf[2] = 0x00;
  msgW.buf[3] = 0x00;
  msgW.buf[4] = ((uint8_t*)(&angleControl))[0];
  msgW.buf[5] = ((uint8_t*)(&angleControl))[1];
  msgW.buf[6] = ((uint8_t*)(&angleControl))[2];
  msgW.buf[7] = ((uint8_t*)(&angleControl))[3];
  Can0.write(msgW);
  //delayMicroseconds(1);
  //receive_CAN_data();
}
//******22.position control 2:send multi-turns position command (angle unit degree)
//(limited by maximum speed unit 1dps)******//
//(limited by maximum angle)//
//(limited by maximum current)//
void Motor_Control_Geminic::send_position_command_2(double angle, double max_speed)
{
  angle = angle * 100;
  angleControl = (int32_t)angle;
  maxiSpeed = (int16_t)max_speed;
  msgW.id = 0x140 + ID;
  msgW.ext = 0;
  msgW.len = 8;
  msgW.rtr = 0;
  msgW.buf[0] = 0xA4;
  msgW.buf[1] = 0x00;
  msgW.buf[2] = *(uint8_t*)(&maxiSpeed);
  msgW.buf[3] = *((uint8_t*)(&maxiSpeed) + 1);
  msgW.buf[4] = *(uint8_t*)(&angleControl);
  msgW.buf[5] = *((uint8_t*)(&angleControl) + 1);
  msgW.buf[6] = *((uint8_t*)(&angleControl) + 2);
  msgW.buf[7] = *((uint8_t*)(&angleControl) + 3);
  Can0.write(msgW);
  delay(1);
  receive_CAN_data();
}
//******23.position control 3:send single-turn position command (angle unit degree 0~359.99)
//(limited by maximum acceleration unit)******//
//(limited by maximum speed unit 1dps)******//
//(limited by maximum angle)//
//(limited by maximum current)//
void Motor_Control_Geminic::send_position_command_3(double angle, uint8_t spinDirection)
{
  angle = angle * 100;
  angleControl = (uint8_t)angle;
  msgW.id = 0x140 + ID;
  msgW.ext = 0;
  msgW.len = 8;
  msgW.rtr = 0;
  msgW.buf[0] = 0xA6;
  msgW.buf[1] = spinDirection;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = *(uint8_t*)(&angleControl);
  msgW.buf[5] = *((uint8_t*)(&angleControl) + 1);
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can0.write(msgW);
  delay(1);
  receive_CAN_data();
}
//******24.position control 4:send single-turn position command (angle unit degree 0~359.99)
//(limited by maximum acceleration unit)******//
//(limited by maximum speed unit 1dps)******//
//(limited by maximum angle)//
//(limited by maximum current)//
void Motor_Control_Geminic::send_position_command_4(double angle, double max_speed, uint8_t spinDirection)
{
  angle = angle * 100;
  angleControl = (uint8_t)angle;
  maxiSpeed = (uint8_t)max_speed;
  msgW.id = 0x140 + ID;
  msgW.ext = 0;
  msgW.len = 8;
  msgW.rtr = 0;
  msgW.buf[0] = 0xA6;
  msgW.buf[1] = spinDirection;
  msgW.buf[2] = *(uint8_t*)(&maxiSpeed);
  msgW.buf[3] = *(uint8_t*)(&maxiSpeed);

  msgW.buf[4] = *(uint8_t*)(&angleControl);
  msgW.buf[5] = *((uint8_t*)(&angleControl) + 1);
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can0.write(msgW);
  delay(1);
  receive_CAN_data();
}
//******25.multi-motors current control
//ID must be #1~#4 for four motor
void Motor_Control_Geminic::send_multi_motor_current_command(double Motor1_current, double Motor2_current, double Motor3_current, double Motor4_current)
{
  iqControl_1 = (int16_t)Motor1_current;
  iqControl_2 = (int16_t)Motor2_current;
  iqControl_3 = (int16_t)Motor3_current;
  iqControl_4 = (int16_t)Motor4_current;
  msgW.id = 0x280;
  msgW.ext = 0;
  msgW.len = 8;
  msgW.rtr = 0;
  msgW.buf[0] = *(uint8_t *)(&iqControl_1);
  msgW.buf[1] = *((uint8_t *)(&iqControl_1) + 1);
  msgW.buf[2] = *(uint8_t *)(&iqControl_2);
  msgW.buf[3] = *((uint8_t *)(&iqControl_2) + 1);
  msgW.buf[4] = *(uint8_t *)(&iqControl_3);
  msgW.buf[5] = *((uint8_t *)(&iqControl_3) + 1);
  msgW.buf[6] = *(uint8_t *)(&iqControl_4);
  msgW.buf[7] = *((uint8_t *)(&iqControl_4) + 1);
  Can0.write(msgW);
  delay(1);
  receive_CAN_data();
}
