/*
 * BLDC.c
 *
 *  Created on: Jun 20, 2024
 *      Author: chipi
 */
#include "BLDC.h"

uint8_t SWITCH( bool coils[]){

uint8_t connector=0;

connector|=coils[0]&1;
connector|=(coils[1]&1)<<1;
connector|=(coils[2]&1)<<2;
return connector;
}

void Phaze_A_ON(void){HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1); HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);}
void Phaze_A_OFF(void){HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1); HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);}
void Phaze_A_ZZ(void){HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);}

void Phaze_B_ON(void){HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);}
void Phaze_B_OFF(void){HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2); }
void Phaze_B_ZZ(void){HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);}

void Phaze_C_ON(void){HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);  }
void Phaze_C_OFF(void){HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);}
void Phaze_C_ZZ(void){HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);}


void BLDC_MotorCommutation(uint8_t halls  ){

	 switch (halls) {

	case 1:

    Phaze_A_ON();
	Phaze_B_ZZ();
	Phaze_C_OFF();

	break;
	case 2:

    Phaze_A_OFF();
	Phaze_B_ON();
    Phaze_C_ZZ();

	break;
	case 3:

	Phaze_A_ZZ();
	Phaze_B_ON();
	Phaze_C_OFF();

	break;
	case 4:

	Phaze_A_ZZ();
	Phaze_B_OFF();
	Phaze_C_ON();


	break;
	case 5:

	Phaze_A_ON();
	Phaze_B_OFF();
	Phaze_C_ZZ();

	break;
	case 6:

	Phaze_A_OFF();
	Phaze_B_ZZ();
	Phaze_C_ON();

	break;

	default:
	break;}

}
