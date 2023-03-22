//#ifndef _SPEEDGOAT_COMM_H
//#define _SPEEDGOAT_COMM_H

#include <Arduino.h>

class Speedgoat_Comm
{
  public:
    int count = 0;
    uint8_t st = 0;
    int read_count = 0;
    uint8_t ch;
    uint8_t Datain[7];   
    int S2=0x7F;
    int S1=0xFF;
    int S3;
    int ref_signal_int;
    float ref_signal_fl;
    Speedgoat_Comm();
    ~Speedgoat_Comm();
    int float_to_uint(float x, float x_min, float x_max, uint8_t nbits);
    float uint_to_float(int x_int, float x_min, float x_max, uint8_t nbits);
    void Read_Data();
    void Decode_Data(uint8_t c); 
    void read_ref_signal(int min_sin, int max_sig);
};

//#endif
