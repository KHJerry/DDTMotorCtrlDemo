#include "motor.h"
/********************************************************
 *@brief: MIT Mode Send Function
 *@param: parameter1 - (motor_id) - CAN ID   - StdId
 *        parameter2 - (pos_ref ) - POS SET  - Round
 *        parameter3 - (vel_ref ) - VEL SET  - Round / sec
 *        parameter4 - (tor_ref ) - TOR SET  - Nm
 *        parameter6 - (  Kp    ) - KP  SET  - POS Proportion
 *        parameter7 - (  Ki    ) - Ki  SET  - VEL Differentiation
 *@birth: Created by Hkk on 2025-03-27
 *@ret:   None
 ********************************************************/
void MITSend(uint32_t motor_id,float pos_ref,float vel_ref,float tor_ref,float kp,float kd) {

    uint8_t txdata[8]={0,0,0,0,0,0,0,0};

    uint16_t pos_uint16 = 32768;//zero point
    uint16_t vel_uint16 = 2048; //zero velocity
    uint16_t tor_uint16 = 2048; //zero torque
    uint16_t kp_uint16  = 0;
    uint16_t kd_uint16  = 0;

    if(pos_ref <= -1.f)  pos_ref = -1.f;
    if(pos_ref >=  1.f)  pos_ref =  1.f;
    if(vel_ref <= -3.f)  vel_ref = -3.f;
    if(vel_ref >=  3.f)  vel_ref =  3.f;
    if(tor_ref <= -50.f) tor_ref = -50.f;
    if(tor_ref >=  50.f) tor_ref =  50.f;
    if(kp < 0.f)         kp = 0.f;
    if(kp > 500.f)       kp = 500.f;
    if(kd < 0.f)         kd = 0.f;
    if(kd > 5.f)         kd = 5.f;

    pos_ref/=1.f;
    pos_uint16+=((uint16_t)(pos_ref*32767));

    txdata[0] = (pos_uint16>>8)&0XFF;
    txdata[1] = (pos_uint16>>0)&0XFF;

    vel_ref/=3.f;
    vel_uint16+=((int16_t)(vel_ref*2047));

    txdata[2] = (vel_uint16>>4)&0XFF;
    txdata[3] = ((vel_uint16<<4)&0XF0);

    kp/=500.f;
    kp_uint16+=((int16_t)(kp*4095));

    txdata[3]|= ((kp_uint16>>8)&0X0F);
    txdata[4] = (kp_uint16>>0)&0XFF;

    kd/=5.f;
    kd_uint16+=((int16_t)(kd*4095));

    txdata[5] = (kd_uint16>>4)&0XFF;
    txdata[6] = (kd_uint16<<4)&0XF0;

    tor_ref   /=50.f;
    tor_uint16+=((int16_t)(tor_ref*2047));

    txdata[6] |= (tor_uint16>>8)&0X0F;
    txdata[7]  = (tor_uint16>>0)&0XFF;

    CAN_Send_Msg(motor_id,txdata);
}