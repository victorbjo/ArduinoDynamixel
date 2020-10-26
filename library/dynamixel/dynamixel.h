#ifndef Morse_h
#define Morse_h

#include "Arduino.h"

class dynamixel
{
  public:
    dynamixel(uint8_t id);
    void dot();
    void dash();
    void action();
    void ledOn();
    void ledOff();
    void torgueEnable();
    void torgueDisable();
    void goalVel(int velocity);
    void sendPckg(uint8_t msg[], int msgSize);
    void changeMode(int mode);
    void goalPos(uint16_t pos);
    void velLimit(uint16_t limit);
    void pwmLimit(uint16_t limit);
    uint16_t compute_crc(uint16_t crc_accum, uint8_t* data_blk_ptr, size_t data_blk_size);
  private:
    uint8_t _id;
    uint16_t  crc_table[256];
};

#endif