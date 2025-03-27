/***************************************************************
 * @brief:    Direct Drive Tech Motor Control Methods
 * @birth:    Created by Hkk on 2025-03-27
 * @revision: Last revised by Hkk on 2025-03-27
 * @version:  V1.0.0
 ***************************************************************/


#ifndef MOTOR_H
#define MOTOR_H

#ifdef __cplusplus
extern "C"{
#endif

#include "can.h"

void MITSend(uint32_t motor_id,float pos_ref,float vel_ref,float tor_ref,float kp,float kd);

#ifdef __cplusplus
};
#endif


#endif