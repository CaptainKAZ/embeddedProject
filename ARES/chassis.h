#ifndef CHASSIS_H
#define CHASSIS_H
#include "main.h"
#include "usart.h"

//Copilot Suggest me to define so
typedef enum wheelPos{
    FRONT_LEFT=0,
    FRONT_RIGHT,
    BACK_LEFT,
    BACK_RIGHT
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
