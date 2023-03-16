#include "Speedgoat_Comm.h"

Speedgoat_Comm::Speedgoat_Comm()
{}

Speedgoat_Comm::~Speedgoat_Comm()
{}

void Speedgoat_Comm::read_ref_signal(int min_sig, int max_sig)
{  
  ref_signal_int = (uint16_t)((S2<<8)|(S1));
  ref_signal_fl = uint_to_float(ref_signal_int, min_sig, max_sig, 16);
}

int Speedgoat_Comm::float_to_uint(float x, float x_min,float x_max, uint8_t nbits)
{
  float span = x_max - x_min;
  if(x<x_min){ x=x_min;}
  else if(x>x_max){ x=x_max;}
  return (int)((x-x_min)*((float)((1<<nbits)-1)/span));
}

float Speedgoat_Comm::uint_to_float(int x_int, float x_min, float x_max, uint8_t nbits)
{
  float span = x_max-x_min;
  float offset_value = x_min;
  return ((float)x_int)*span/((float)((1<<nbits)-1))+offset_value;
}

void Speedgoat_Comm::Decode_Data(uint8_t c)
{
  switch (st)
  {
  case 0: //Read 1st Byte
//    Serial.print("Case 0, data: "); Serial.print(c); Serial.print(", count: "); Serial.println(read_count); 
    if (c == 0xff)
    {
      st = 1;
      Datain[read_count] = c;
      read_count += 1;
    }
    break;
  case 1: //Read 2nd Byte
//    Serial.print("Case 1, data: "); Serial.print(c); Serial.print(", count: "); Serial.println(read_count); 
    if (c == 0x88 && Datain[0] == 0xff)
    {
      st = 2;
      Datain[read_count] = c;
      read_count += 1;
    }
    else
    {
      st = 0;
      read_count = 0;
    }
    break;
  case 2: //Read 2nd Byte
//    Serial.print("Case 2, data: "); Serial.print(c); Serial.print(", count: "); Serial.println(read_count); 
    if (c == 0x77 && Datain[0] == 0xff && Datain[1] == 0x88 )
    {
      st = 3;
      Datain[read_count] = c;
      read_count += 1;
    }
    else
    {
      st = 0;
      read_count = 0;
    }
    break;
  case 3: //Read 3rd Byte
//    Serial.print("Case 3, data: "); Serial.print(c); Serial.print(", count: "); Serial.println(read_count);
    if (Datain[0] == 0xff && Datain[1] == 0x88 && Datain[2] == 0x77)
    {    
      Datain[read_count] = c;
      read_count += 1;
      if (read_count == 7)
      {
        st = 0;
        read_count = 0;
        if (Datain[6] == 0x11)
        {
        S1=Datain[3];
        S2=Datain[4];
        S3=Datain[5];
        //Serial.print("Successfully received data: "); Serial.print(S1); Serial.print(" "); Serial.print(S2); Serial.print(" "); Serial.println(S3);
        //m1.read_multi_turns_angle();
        //m1.read_single_turns_angle();
        //delayMicroseconds(10);
        //Serial.println("Here");
        //Serial.println(m1.motorAngle);
        }
      }
    }
    else
    {
      st = 0;
      read_count = 0;
    }
    break;
  default:
    st = 0;
    break;
  }
}

void Speedgoat_Comm::Read_Data()
{
  if (Serial3.available())
  {
    ch = Serial3.read();
    Decode_Data(ch);
  }
}
