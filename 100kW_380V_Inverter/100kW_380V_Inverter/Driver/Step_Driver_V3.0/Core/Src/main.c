/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdint.h"
#include "stdio.h"
#include "math.h"
#include "stdbool.h"
#include "modbusDevice.h"
#include "modbusSlave.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TX_2    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10,GPIO_PIN_SET);
#define RX_2    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10,GPIO_PIN_RESET);

#define PORT1_ON    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11,GPIO_PIN_SET);
#define PORT1_OFF   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11,GPIO_PIN_RESET);

#define PORT2_ON    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,GPIO_PIN_SET);
#define PORT2_OFF   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,GPIO_PIN_RESET);

#define PORT3_ON    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,GPIO_PIN_SET);
#define PORT3_OFF   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,GPIO_PIN_RESET);

#define PORT4_ON    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_SET);
#define PORT4_OFF   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_RESET);
#define LED_1_ON    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15,GPIO_PIN_SET);
#define LED_1_OFF   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15,GPIO_PIN_RESET);
#define LED_2_ON    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0,GPIO_PIN_SET);
#define LED_2_OFF   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0,GPIO_PIN_RESET);

#define LED_3_ON    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1,GPIO_PIN_SET);
#define LED_3_OFF   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1,GPIO_PIN_RESET);

#define CLEAR HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)

//////////////////////////////////////////////////////////////////////////////////////////////

#define LCD_ADDR (0x27 << 1)       // адрес дисплея, сдвинутый на 1 бит влево (HAL работает с I2C-адресами, сдвинутыми на 1 бит влево)

#define PIN_RS    (1 << 0)         // если на ножке 0, данные воспринимаются как команда, если 1 - как символы для вывода
#define PIN_EN    (1 << 2)         // бит, по изменению сост. которого считывается информация
#define BACKLIGHT (1 << 3)         // управление подсветкой

#define LCD_DELAY_MS 5             // пауза перед высвечиванием символа

//////////////////////////////////////////////////////////////////////////////////////////////

#define I2C_ADDR 0x27 // I2C address of the PCF8574
#define RS_BIT 0 // Register select bit
#define EN_BIT 2 // Enable bit
#define BL_BIT 3 // Backlight bit
#define D4_BIT 4 // Data 4 bit
#define D5_BIT 5 // Data 5 bit
#define D6_BIT 6 // Data 6 bit
#define D7_BIT 7 // Data 7 bit

#define LCD_ROWS 4 // Number of rows on the LCD
#define LCD_COLS 20 // Number of columns on the LCD

#define N 20

#define I2C_SLAVE_ADDRESS   0x2D
/////////////////////////////////////////////////////////////

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc2;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
char str[112]={0, };
uint16_t count=0;
static uint16_t current[4]={0, };
static uint16_t hall_sens[5]={0, };
uint16_t pwm[]={10,10,10,10 };
uint8_t TxData[6] = {0, };
bool lcd_status=0;
bool lcd_update=0;

uint16_t limit=500;

uint16_t pwm_n=0;

volatile uint16_t pwm_cmd=0;
uint8_t pwm_tgr=0;
volatile uint8_t flag = 0;

 int period=0;
 uint16_t pulseWidth=0;
 uint16_t rpm=0;
 uint16_t f=0;
 uint8_t backlight_state = 1;
 uint8_t adc_count=0;
 uint16_t adc_filter[20]={0,};
 uint8_t flt=0;
 uint16_t filter=0;
 uint8_t page_num=0;
 uint8_t page_flag=0;
 uint16_t current_avg=0;
 uint16_t current_sum=0;
 uint8_t min=0;
 uint8_t hour=0;
 uint8_t sec=0;
 uint8_t status=0;
 uint16_t current_dc=5;
 uint16_t voltage_dc=25;

 bool coils[4]={0,};

 I2C_HandleTypeDef* i2cMasterHandler = &hi2c1;



 uint8_t rxFrame[28];
 uint8_t txFrame[255];
 uint8_t SLAVE_ID=1;
 uint16_t data_reg[32]={0,};
 uint16_t rcv_data_reg[16]={0,};


 uint32_t timing=0;


 RTC_TimeTypeDef sTime = {0};
 RTC_DateTypeDef sDate = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM8_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM15_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC3_Init(void);
/* USER CODE BEGIN PFP */

void ADC_reset_ch(void){

	ADC_ChannelConfTypeDef sConfig = {0};

		              sConfig.Channel = ADC_CHANNEL_1;
		              sConfig.Rank = 0;
		              HAL_ADC_ConfigChannel(&hadc3, &sConfig);
		              sConfig.Channel = ADC_CHANNEL_2;
		              sConfig.Rank =0;
		              HAL_ADC_ConfigChannel(&hadc3, &sConfig);
}


void ADC_Select_CH1 (void)
	     {
	     	ADC_ChannelConfTypeDef sConfig = {0};
	     	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	     	  */
	     	  sConfig.Channel = ADC_CHANNEL_1;
	     	  sConfig.Rank = 1;
	     	  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
	     	  HAL_ADC_ConfigChannel(&hadc3, &sConfig);
	     }

void ADC_Select_CH12 (void)
	     {
	     	ADC_ChannelConfTypeDef sConfig = {0};
	     	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	     	  */
	     	  sConfig.Channel = ADC_CHANNEL_12;
	     	  sConfig.Rank = 1;
	     	  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
	     	  HAL_ADC_ConfigChannel(&hadc3, &sConfig);
	     }



uint16_t ADC_read_1(void)

	  {
//	ADC_reset_ch();
	ADC_Select_CH1 ();
	current[1]=0;
	HAL_ADC_Start(&hadc3);

	 		  HAL_ADC_PollForConversion(&hadc3, 100);
	 		  current[1] = HAL_ADC_GetValue(&hadc3);
	 		  HAL_ADC_Stop(&hadc3);


	 		  return current[1];
	  }

uint16_t ADC_read_12(void)

	  {
//	ADC_reset_ch();
	ADC_Select_CH12 ();
	current[0]=0;
	HAL_ADC_Start(&hadc3);

	 		  HAL_ADC_PollForConversion(&hadc3, 100);
	 		  current[0] = HAL_ADC_GetValue(&hadc3);
	 		  HAL_ADC_Stop(&hadc3);


	 		  return current[0];
	  }


void PWM_correction(void){

for(uint8_t i=0;i<4;i++){


	pwm[i]=pwm_cmd;

	if((hall_sens[i]>=limit)&&(pwm[i]>=10)){
	pwm[i]=pwm[i]-10;
    LED_2_ON;
	}


	if(hall_sens[i]>=3800){

		            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
		            HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
		            HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);

		            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
		            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
		            HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_1);
		            HAL_TIMEx_PWMN_Stop(&htim15, TIM_CHANNEL_1);

		            pwm[0]=10;
		            pwm[1]=10;
		            pwm[2]=10;
		            pwm[3]=10;

		            LED_2_ON;}

	if((pwm[i]>=10)&&(hall_sens[i]<=10)){LED_2_OFF;}

}
pwm_n=(pwm[0]+pwm[1]+pwm[2]+pwm[3])/4;
}
////////////////////////////////////////////////////////////////////////////////////////////

void I2C_send(uint8_t data, uint8_t flags)
{
	HAL_StatusTypeDef res;
	 // бесконечный цикл
        for(;;) {
                // проверяем, готово ли устройство по адресу lcd_addr для связи
	        res = HAL_I2C_IsDeviceReady(&hi2c1, LCD_ADDR, 1, HAL_MAX_DELAY);
	         // если да, то выходим из бесконечного цикла
                if(res == HAL_OK) break;
	    }
        // операция �? с 1111 0000 приводит к обнулению бит с 0 по 3, остаются биты с 4 по 7
	uint8_t up = data & 0xF0;
        // то же самое, но data сдвигается на 4 бита влево
        uint8_t lo = (data << 4) & 0xF0;

	uint8_t data_arr[4];
         // 4-7 биты содержат информацию, биты 0-3 настраивают работу дисплея
	data_arr[0] = up|flags|BACKLIGHT|PIN_EN;
         // дублирование сигнала, на выводе Е в этот раз 0
	data_arr[1] = up|flags|BACKLIGHT;
	data_arr[2] = lo|flags|BACKLIGHT|PIN_EN;
	data_arr[3] = lo|flags|BACKLIGHT;

	HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, data_arr, sizeof(data_arr), HAL_MAX_DELAY);
	HAL_Delay(LCD_DELAY_MS);
}


void LCD_SendString(char *str)
{
    // *char по сути является строкой
	while(*str) {                                   // пока строчка не закончится
		I2C_send((uint8_t)(*str), 1);               // передача первого символа строки
        str++;                                      // сдвиг строки налево на 1 символ
    }
}

////////////////////////////////////////////////////////////////////////////////////////////

void lcd_write_nibble(uint8_t nibble, uint8_t rs) {
  uint8_t data = nibble << D4_BIT;
  data |= rs << RS_BIT;
  data |= backlight_state << BL_BIT; // Include backlight state in data
  data |= 1 << EN_BIT;
  HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDR << 1, &data, 1, 100);
  HAL_Delay(1);
  data &= ~(1 << EN_BIT);
  HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDR << 1, &data, 1, 100);
}

////////////////////////////////////////////////////////////////////////////////////////////

void lcd_send_cmd(uint8_t cmd) {
  uint8_t upper_nibble = cmd >> 4;
  uint8_t lower_nibble = cmd & 0x0F;
  lcd_write_nibble(upper_nibble, 0);
  lcd_write_nibble(lower_nibble, 0);
  if (cmd == 0x01 || cmd == 0x02) {
    HAL_Delay(2);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////

void lcd_send_data(uint8_t data) {
  uint8_t upper_nibble = data >> 4;
  uint8_t lower_nibble = data & 0x0F;
  lcd_write_nibble(upper_nibble, 1);
  lcd_write_nibble(lower_nibble, 1);
}

//////////////////////////////////////////////////////////////////////////////////////////////

void lcd_init() {
  HAL_Delay(50);
  lcd_write_nibble(0x03, 0);
  HAL_Delay(5);
  lcd_write_nibble(0x03, 0);
  HAL_Delay(1);
  lcd_write_nibble(0x03, 0);
  HAL_Delay(1);
  lcd_write_nibble(0x02, 0);
  lcd_send_cmd(0x28);
  lcd_send_cmd(0x0C);
  lcd_send_cmd(0x06);
  lcd_send_cmd(0x01);
  HAL_Delay(2);
}

//////////////////////////////////////////////////////////////////////////////////////////////

void lcd_write_string(char *str) {
  while (*str) {
    lcd_send_data(*str++);
  }
}
//////////////////////////////////////////////////////////////////////////

void lcd_set_cursor(uint8_t row, uint8_t column) {
    uint8_t address;
    switch (row) {
        case 1:
            address = 0x80;
            break;
        case 2:
            address = 0xC0;
            break;

        case 3:
            address = 0x94;
            break;
        case 4:
            address = 0xD4;
            break;


        default:
            address = 0x80;
    }
    address += column;
    lcd_send_cmd(0x80 | address);
}

//////////////////////////////////////////////////////////////////////////////////////////////////

void lcd_clear(void) {
	lcd_send_cmd(0x01);
    HAL_Delay(2);
}

//////////////////////////////////////////////////////////////////////////////////////////////////

void lcd_backlight(uint8_t state) {
  if (state) {
    backlight_state = 1;
  } else {
    backlight_state = 0;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////



void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART2)
	{

				 if (rxFrame[0] == SLAVE_ID) {

					//if( (calcCRC16ModBus(rxFrame, sizeof(rxFrame)))==   )
					  			 		uint8_t opCode = rxFrame[1];
					  			     switch (opCode) {

					  			   case   READ_DISC_INPUTs:


					  				handleReadInputRegs(&huart2,0xFF);
								   break;

					  			      // case READ_COILs:

					  			    	 break;
					  			       case  READ_INPUT_REGs:
					  			    	 handleReadHoldingRegs(&huart2,data_reg);


					  			    	   break;
					  			     case WRITE_SINGLE_REG:

					  			   					  			    //	 handleWriteSingleHandlingRegister (&huart2,data_reg);
					  			   					  			    	handleWriteMulyipleHandlingRegister (&huart2,rcv_data_reg);
					  			                                              pwm_cmd=rcv_data_reg[1];

					  			                                               break;

					  			       case WRITE_HOLDING_REGs:


					  			    	handleWriteMulyipleHandlingRegister (&huart2,rcv_data_reg);
					  			    	 pwm_cmd=rcv_data_reg[1];
					  			    	 limit=rcv_data_reg[2];

					  		sTime.Hours=(rcv_data_reg[3]>>8) & 0xff;
					  		sTime.Minutes= rcv_data_reg[3] & 0xff;
					  		sTime.Seconds=rcv_data_reg[4];
					  		HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD);



                                            break;


					  			       default:
					  			 				break;
					  			     }

					  			 }



		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *) rxFrame,sizeof(rxFrame));
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

	}
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */




void lcd_init();
void lcd_write_nibble(uint8_t nibble, uint8_t rs);
void lcd_send_cmd(uint8_t cmd);
void lcd_send_data(uint8_t data);
void lcd_write_string(char *str);
void lcd_set_cursor(uint8_t row, uint8_t column);
void lcd_clear(void);
void lcd_backlight(uint8_t state);


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM8_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_TIM15_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */
  LED_2_ON;
	TX_2;


 TIM1->CCR1=pwm[0];
 TIM1->CCR2=pwm[1];
 TIM8->CCR1=pwm[2];
 TIM15->CCR1=pwm[3];



  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);

//  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
//  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
//  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
//  HAL_TIMEx_PWMN_Start(&htim15, TIM_CHANNEL_1);

  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);

  PORT1_ON;
  PORT2_ON;
  PORT3_ON;
  PORT4_ON;

  HAL_ADCEx_Calibration_Start(&hadc2,5);
  HAL_ADCEx_Calibration_Start(&hadc1,4);
  HAL_ADCEx_Calibration_Start(&hadc3,2);

  flag=1;

  HAL_Delay(100);

    char *text = " ";
    char int_to_str[10];



     HAL_ADC_Start_DMA(&hadc2,hall_sens, 5);

     HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rxFrame, 16);
        __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

      LED_2_OFF;
  RX_2;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {



	  if(flag==1){
		  HAL_ADC_Stop_DMA(&hadc2);
		  HAL_ADC_Start(&hadc1);
		  HAL_ADC_PollForConversion(&hadc1, 100);
		  current[3]=0;
		  current[3]=HAL_ADC_GetValue(&hadc1);
		  HAL_ADC_Stop(&hadc1);

		 	  		  ADC_read_1();
		 	  		  ADC_read_12();
		 	  		 HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

		      data_reg[0]=hall_sens[0];
		 	  data_reg[1]=hall_sens[1];
		 	  data_reg[2]=hall_sens[2];
		 	  data_reg[3]=hall_sens[3];
		 	  data_reg[4]=pwm[0];
		 	  data_reg[5]=pwm[1];
		 	  data_reg[6]=pwm[2];
		 	  data_reg[7]=pwm[3];
		 	  data_reg[8]=rpm;
		 	  data_reg[9]=f;
		 	  data_reg[10]=limit;
		 	  data_reg[11]=SLAVE_ID;
		 	  data_reg[12]=current[3];
		 	  data_reg[13]=hall_sens[4];
		 	  data_reg[14]=(sTime.Hours<<8)|(sTime.Minutes);
		      data_reg[15]=sTime.Seconds;

            //    current_avg=(current[0]+current[1]+current[2]+current[3])/4;
             //   current_sum=current[0]+current[1]+current[2]+current[3];





                if(flt<=19){
                adc_filter[flt]=current[0];
                flt++;
                for(uint8_t n=0; n<flt+1; ){

                filter=filter+adc_filter[n];
                 n++;

                }

                filter=filter/flt;

                  if(flt==19){
                	 for(uint8_t n=1;n==flt;){

                		adc_filter[n-1]=adc_filter[n];  }

                     flt=0;
                  }

                }




		 	  	  if((lcd_status==0)&&(lcd_update==1)){lcd_init();HAL_Delay(100);lcd_clear();lcd_update=0;}
		 	  	  if((lcd_status==1)&&(lcd_update==1)){lcd_init();lcd_clear();lcd_update=0;}

		 	  	  if(lcd_status==1){
		 	  		 adc_count++;

		 	  if(page_flag==1){lcd_clear();page_flag=0;}
		 	  	  if((page_num==0)&&(adc_count==20)){


				 	  		sprintf(int_to_str, "RPM-%05d t/min     ",rpm);
				 	  		lcd_set_cursor(1, 0);
				 	  	    lcd_write_string(int_to_str);

				 	  	    sprintf(int_to_str, "PWM_CMD-%05d/%03d%% ",pwm_n,pwm_n/20);
				 	  	 	lcd_set_cursor(2, 0);
				 	  	 	lcd_write_string(int_to_str);

				 	  	    sprintf(int_to_str, "ADC_avg-%05d ",current_avg);
				 	  	 	lcd_set_cursor(3, 0);
				 	  	 	lcd_write_string(int_to_str);

				 	    	sprintf(int_to_str, "F=%05dHz ",f);
				 	  	 	lcd_set_cursor(4, 0);
				 	  	 	lcd_write_string(int_to_str);	  }

		 	  	  if((page_num==2)&&(adc_count==22)){

		 	  		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN); // RTC_FORMAT_BIN , RTC_FORMAT_BCD
		 	  	 HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

		 	  	                            sprintf(int_to_str, "START_TIME:%02d:%02d:%02d",hour,min,sec);
		 	  			 	  				lcd_set_cursor(1, 0);
		 	  			 	  			    lcd_write_string(int_to_str);

		 	  					 	  		sprintf(int_to_str, "TIME:%02d:%02d:%02d",sTime.Hours,sTime.Minutes,sTime.Seconds);
		 	  					 	  		lcd_set_cursor(2, 0);
		 	  					 	  	    lcd_write_string(int_to_str);


		 	  					 	  		sprintf(int_to_str, "ADC_filter:%02d",adc_filter[0]);
		 	  					 	  		lcd_set_cursor(3, 0);
		 	  					 	  	    lcd_write_string(int_to_str); }



		 	  	  if(page_num==1){
		 	  	                 	if(adc_count==18){

		 	  				 	  		sprintf(int_to_str, "PWM-%02d%%/%04d",pwm[0]/20,pwm[0]);
		 	  				 	  		lcd_set_cursor(1, 0);
		 	  				 	  	    lcd_write_string(int_to_str);

		 	  				 	     	sprintf(int_to_str, "PWM-%02d%%/%04d",pwm[1]/20,pwm[1]);
		 	  				 	  	    lcd_set_cursor(2, 0);
		 	  				 	  	    lcd_write_string(int_to_str);

		 	  				 	  	    sprintf(int_to_str, "PWM-%02d%%/%04d",pwm[2]/20,pwm[2]);
		 	  				 	  	    lcd_set_cursor(3, 0);
		 	  				 	  		lcd_write_string(int_to_str);

		 	  				 	    	sprintf(int_to_str, "PWM-%02d%%/%04d",pwm[3]/20,pwm[3]);
		 	  				 	  	    lcd_set_cursor(4, 0);
		 	  				 	  		lcd_write_string(int_to_str);

		 	  				 	  		}

		 	  		if(adc_count>=180){


		 	  			sprintf(int_to_str, "Adc-%03d",hall_sens[0]);
		 	  			lcd_set_cursor(1, 13);
		 	  		    lcd_write_string(int_to_str);

		 	  		    sprintf(int_to_str, "Adc-%03d",hall_sens[1]);
		 	  		 	lcd_set_cursor(2, 13);
		 	  		    lcd_write_string(int_to_str);

		 	  		    sprintf(int_to_str, "Adc-%03d",hall_sens[2]);
		 	  		    lcd_set_cursor(3, 13);
		 	  		    lcd_write_string(int_to_str);

		 	  	        sprintf(int_to_str, "Adc-%03d",hall_sens   [3]);
		 	  	 		lcd_set_cursor(4, 13);
		 	  	 		lcd_write_string(int_to_str);

		 	  	    adc_count=0;
		 	  		}


		 	  	  }
		 	  	  }








	     PWM_correction();
	  	 TIM1->CCR1=pwm[0];
	  	 TIM1->CCR2=pwm[1];
	  	 TIM8->CCR1=pwm[2];
	  	 TIM15->CCR1=pwm[3];
	  	 flag=0;
	  	 HAL_ADC_Start_DMA(&hadc2, (uint16_t*)hall_sens, 5);

	  }











    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_RTC
                              |RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_TIM8
                              |RCC_PERIPHCLK_ADC12|RCC_PERIPHCLK_ADC34;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Adc34ClockSelection = RCC_ADC34PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim8ClockSelection = RCC_TIM8CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 5;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 36;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 311;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x9;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JUNE;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 100;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 64000000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 12;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 2000;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 100;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 2000;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 100;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 230400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, LED_2_Pin|LED_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Port_2_Pin|Port_1_Pin|Port_3_Pin|Port_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Direct_GPIO_Port, Direct_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : START_Pin STOP_Pin */
  GPIO_InitStruct.Pin = START_Pin|STOP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_1_Pin */
  GPIO_InitStruct.Pin = LED_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_2_Pin LED_3_Pin */
  GPIO_InitStruct.Pin = LED_2_Pin|LED_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : INT_Pin */
  GPIO_InitStruct.Pin = INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Port_2_Pin Port_1_Pin Port_3_Pin Port_4_Pin */
  GPIO_InitStruct.Pin = Port_2_Pin|Port_1_Pin|Port_3_Pin|Port_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Direct_Pin */
  GPIO_InitStruct.Pin = Direct_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Direct_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
        {

        	 HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        	// HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
        	 HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
        	// HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
        	 HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
        	// HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
        	 HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);
        	// HAL_TIMEx_PWMN_Start(&htim15, TIM_CHANNEL_1);

        	 period = 0;

            TIM2->CNT = 0;
            period = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
            pulseWidth = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);

            rpm= 480000000/period;
            f=32000000/period;


            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
            HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
            HAL_TIMEx_PWMN_Start(&htim15, TIM_CHANNEL_1);

        }

        if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
             {

        	  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
        	//  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        	  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
        	//  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        	  HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_1);
        	//  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
        	  HAL_TIMEx_PWMN_Stop(&htim15, TIM_CHANNEL_1);
        	//  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);

        	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
        	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
        	  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);

                                                         }

    }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
   if(GPIO_Pin == GPIO_PIN_13) // если прерывание поступило от ножки PC13
   {
      // Переключение в противоположное состояние ножки
      // выхода для управления светодиодом (LED):
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
      if (GPIO_PIN_RESET == HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_13)){



            if((pwm_cmd>=0)&&(pwm_cmd<=19)&&(status==0)){

            	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN); // RTC_FORMAT_BIN , RTC_FORMAT_BCD
            	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

            	hour=sTime.Hours;
            	min=sTime.Minutes;
            	sec=sTime.Seconds;

            	status=1;
            }

        		   pwm_cmd=pwm_cmd+10;
        		   if(pwm_cmd>=1999){pwm_cmd=2000;}
        		   pwm_tgr=4;

        		   EXTI->PR = EXTI_PR_PR13;
        	   }




   }


   if(GPIO_Pin == GPIO_PIN_14) // если прерывание поступило от ножки PC14
     {

	   if (GPIO_PIN_SET == HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_14)){
        // Переключение в противоположное состояние ножки
        // выхода для управления светодиодом (LED):
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);}
	   if (GPIO_PIN_RESET == HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_14)){

		   if((pwm_cmd==0)&&(status==1)){
		              	status=0;

		              	hour=0;
		              	min=0;
		              	sec=0;
		              }


		   pwm_cmd=pwm_cmd-10;
		   if(pwm_cmd>=2100){pwm_cmd=0;}
		   if(pwm_cmd<=1){pwm_cmd=0;}
		   pwm_tgr=4;
		   EXTI->PR = EXTI_PR_PR14;
	   }


     }


   if(GPIO_Pin == GPIO_PIN_5) // если прерывание поступило от ножки PB_5
      {
	   if (GPIO_PIN_SET == HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_5)){
	             LED_3_OFF;

	   			// if(page_num>2){page_num=0;}
	   			// page_flag=0;

	             if((HAL_GetTick()-timing)>=3000){
	            	 if(lcd_status==0){lcd_update=1;}
	            	 lcd_status=!lcd_status;}
	   		         EXTI->PR = EXTI_PR_PR5;
      }

     if (GPIO_PIN_RESET == HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_5)){

    	         LED_3_ON;
    	 	     page_flag=1;
    	         page_num++;
    	         if(page_num>2){page_num=0;}
    	         timing=HAL_GetTick();
    	         EXTI->PR = EXTI_PR_PR5;
    	         timing=HAL_GetTick();

     }



}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)

{

RX_2;
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC2)
    {
        flag = 1;
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
