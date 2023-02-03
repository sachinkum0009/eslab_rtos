/**
 * @file
 * 
 * ESP32 Motor Library
 * 
 * Functions to set motor speed
 * 
 * Authors: Vipul Deshpande, Jaime Burbano
 */

#ifndef motor_h
#define motor_h
#define DELAY_TIME 0.0215 // 5714286
#define DISTANCE_TIME 0.5

enum Motors
{
  MotorA = 0,   /* Crawler Motor 1 */
  MotorB = 1    /* Crawler Motor 2 */
};

enum Direction
{
  Forward = 0,  /* Motor Forward */
  Backward = 1  /* Motor Backward */
};

class mclass {
  public:
    mclass();
    
    void SETUP();   /* Initialize the Motors */
    void SPEED(int motor_speed);
    void motor_direction(Motors motor_ch, Direction dir); /* set direction of rotation of the motors */
    void set_speed(Motors motor_ch, Direction dir, int new_speed);  /* set the speed of the motors */
    void turnAngle(const int16_t* angle);
    void moveRover(const int16_t* distance);
};

extern mclass motorobject;

#endif