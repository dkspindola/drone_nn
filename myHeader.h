#ifndef MYHEADER_H
#define MYHEADER_H

// your header file with global variables and functions
typedef struct VECTOR3
{
    float x;
    float y;
    float z;
}Vector3;

void fly();
void test();
void shutoff_motors();
void start_motors();
void changeState();

#endif