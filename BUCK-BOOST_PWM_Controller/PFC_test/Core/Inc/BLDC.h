/*
 * BLDC.h
 *
 *  Created on: Jun 20, 2024
 *      Author: chipi
 */

#ifndef INC_BLDC_H_
#define INC_BLDC_H_


#include "main.h"
#include "stdbool.h"
extern TIM_HandleTypeDef htim1;
void Phaze_A_ON(void);
void Phaze_A_OFF(void);
void Phaze_A_ZZ(void);

void Phaze_B_ON(void);
void Phaze_B_OFF(void);
void Phaze_B_ZZ(void);

void Phaze_C_ON(void);
void Phaze_C_OFF(void);
void Phaze_C_ZZ(void);

uint8_t SWITCH( bool coils[]);
void BLDC_MotorCommutation(uint8_t halls  );


#endif /* INC_BLDC_H_ */
