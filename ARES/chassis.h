#ifndef CHASSIS_H
#define CHASSIS_H
#include "main.h"
#include "usart.h"

//Copilot Suggest me to define so
typedef enum wheelPos{
    FRONT_LEFT=3,
    FRONT_RIGHT=2,
    BACK_LEFT=0,
    BACK_RIGHT=1,
}wheelPos;

/*
        x
        ^
        |
        |
y<---------------
        |
        |
        |
*/

typedef struct Chassis {
    volatile float   xSpeed;
    volatile float   ySpeed;
    volatile float   wSpeed;
    volatile int32_t timeout;
    float            r;
    float            wheelSpeed[4];
} Chassis;

extern void Chassis_setSpeed(float xSpeed, float ySpeed, float wSpeed, int32_t timeout);

#endif
