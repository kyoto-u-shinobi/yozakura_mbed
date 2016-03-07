#include "mems.h"
#include "mbed.h"

MEMS::MEMS(PinName sda, PinName scl)
        : _mems(sda, scl) {

}

void MEMS::GetTemp(float* data) {
        
    char I2C_buffer[64];    // read-buffer for raw data
    short L_buffer;         // 
    short H_buffer;         //
    float PTAT;             // PTAT temperature data (not used)
    int PEC;                // Packet Error Check code (not used)

    // measure
    _mems.start();
    _mems.write(D6T_addr);
    _mems.write(D6T_cmd);
    _mems.read(D6T_addr,I2C_buffer,35);

    // data conversion  
    L_buffer = I2C_buffer[0];
    H_buffer = I2C_buffer[1];
    PTAT = 0.1 * (L_buffer + H_buffer * 256);
    for(int i=0; i<16; i++) {
        L_buffer = I2C_buffer[2*i+2];
        H_buffer = I2C_buffer[2*i+3];
        data[i] = 0.1 * (L_buffer + H_buffer * 256);
    }
    PEC = I2C_buffer[34];

}
