/* USER CODE BEGIN Header */
/**
  **************************
  * @file           : main.c
  * @brief          : Main program body
  **************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  **************************
  */

/***************************************
I2C1 	- MS5611
I2C2 	- MPU9250
USART6 	- GPS
PD6 	- APOGEE TETİKLEME
PD7 	- MAIN TETİKLEME
 ***************************************/

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "string.h"
#include "ms5611.h"
#include "mpu9255.h"
#include "lwgps/lwgps.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MS5611_I2C_ADDRESS_H 0xEE		/*! CSB pin is HIGH */
#define MS5611_I2C_ADDRESS_L 0xEC 		/*! CSB pin is LOW */
#define RAD_TO_DEG 57.295779513f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
float MS5611_Press, MS5611_Altitude, MS5611_Temp;
float baslangic_irtifa, irtifa, onceki_irtifa;
float angX, angY,angZ, accX, accY, accZ, gyroX, gyroY, gyroZ;
float enlem, boylam, gps_irtifa, yunuslama;
int rocket_state;
volatile int calisma_durumu = 0;
volatile int dinleme_durumu = 0;
bool ilk_irtifa = false;
char gonderilecek_veri[128];
MPU9255_t DataStruct;
MS5611_HandleTypeDef MS5611;
uint32_t lastTick = 0;

MPU9255_t DataStruct;
MS5611_HandleTypeDef MS5611;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_UART4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

lwgps_t gps;

uint8_t rx_buffer[128];
uint8_t rx_index = 0;
uint8_t rx_data = 0;
uint8_t test_buffer[5];
uint8_t test_index = 0;
uint8_t test_data = 0;
uint8_t sut_buffer[36];
uint8_t sut_index = 0;
uint8_t veri[36];

uint8_t b7=0,b6=0,b5=0,b4=0,b3=0,b2=0,b1=0,b0=0;
uint8_t sut_byte=0;
uint8_t sut_durum_bilgi[6];

	uint8_t adres_h = 0x00;
	uint8_t adres_l = 0x02;
	uint8_t kanal	= 0x12;


	void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
	{
	    if (huart == &huart6)
	    {
	        if (rx_data != '\n' && rx_index < sizeof(rx_buffer))
	            rx_buffer[rx_index++] = rx_data;
	        else {
	            lwgps_process(&gps, rx_buffer, rx_index + 1);
	            rx_index = 0;
	        }

	        // Sadece huart6 için tekrar alım ayarla
	        HAL_UART_Receive_IT(&huart6, &rx_data, 1);
	    }
	    else if (huart == &huart4)
	    {
	        if (test_index == 0 && sut_index == 0)
	        {
	            if (test_data == 0xAA) dinleme_durumu = 0;
	            else if (test_data == 0xAB) dinleme_durumu = 1;
	        }

	        if (dinleme_durumu == 0)
	        {
	            if (test_data != 0x0A)
	                test_buffer[test_index++] = test_data;
	            else
	            {
	                if (test_buffer[1] == 0x20) { calisma_durumu = 1; dinleme_durumu = 0; }
	                else if (test_buffer[1] == 0x22) { calisma_durumu = 2; dinleme_durumu = 1; }
	                else { calisma_durumu = 0; dinleme_durumu = 0; }
	                test_index = 0;
	            }
	        }
	        else if (dinleme_durumu == 1)
	        {
	            sut_buffer[sut_index] = test_data;
	            sut_index = (sut_index + 1) % 36;
	        }

	        // Sadece huart4 için tekrar alım ayarla
	        HAL_UART_Receive_IT(&huart4, &test_data, 1);
	    }
	}


	float Calculate_Altitude(float pressure_mbar) {
	    return 44330.0f * (1.0f - pow((pressure_mbar * 100.0f / 101325.0f), 1.0f / 5.255f));
	}

	float Calculate_Pitch(float accX, float accY, float accZ) {
	    return atan2f(accX, sqrtf(accY * accY + accZ * accZ)) * 180.0f / M_PI;
	}


	void floatToBigEndian(uint8_t *d,float v){
	    float r=((int)(v*100.0f))/100.0f;
	    union{float f;uint8_t b[4];}u;u.f=r;
	    d[0]=u.b[3];d[1]=u.b[2];d[2]=u.b[1];d[3]=u.b[0];
	}

	float bigEndianToFloat(uint8_t *d)
	{
	    union { float f; uint8_t b[4]; } u;
	    // Big endian gelen veriyi ters çeviriyoruz
	    u.b[0] = d[3];
	    u.b[1] = d[2];
	    u.b[2] = d[1];
	    u.b[3] = d[0];
	    return u.f;
	}


	void transmit_SIT_packet(float irtifa, float press, float accX, float accY, float accZ,
	                         float gyroX, float gyroY, float gyroZ) {

	    veri[0] = 0xAB;

	    floatToBigEndian(&veri[1],  irtifa);
	    floatToBigEndian(&veri[5],  press);
	    floatToBigEndian(&veri[9],  accX);
	    floatToBigEndian(&veri[13], accY);
	    floatToBigEndian(&veri[17], accZ);
	    floatToBigEndian(&veri[21], gyroX);
	    floatToBigEndian(&veri[25], gyroY);
	    floatToBigEndian(&veri[29], gyroZ);

	    uint8_t checksum = 0;
	    for (int i = 0; i <= 32; i++) {
	        checksum += veri[i];
	    }
	    checksum &= 0xFF;

	    veri[33] = checksum;
	    veri[34] = 0x0D;
	    veri[35] = 0x0A;

	    HAL_UART_Transmit(&huart4, veri, sizeof(veri), HAL_MAX_DELAY);
	}

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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  lastTick = HAL_GetTick();


HAL_GPIO_WritePin(GPIOD, LoRa_M0_Pin, GPIO_PIN_RESET); // M0 = 0
HAL_GPIO_WritePin(GPIOD, LoRa_M1_Pin, GPIO_PIN_RESET); // M1 = 0
HAL_Delay(50); // Modülün moda geçmesini bekle

lwgps_init(&gps);
HAL_UART_Receive_IT(&huart6, &rx_data, 1);
HAL_UART_Receive_IT(&huart4, &test_data, 1);


#ifdef MS5611_H_
MS5611.I2C_ADDRESS = MS5611_I2C_ADDRESS_H;
MS5611.i2c = &hi2c1;
MS5611_Init(&MS5611);
#endif

rocket_state = 0;

MS5611_Read_ActVal(&MS5611);
MS5611_FirstCalculateDatas(&MS5611);
MPU9255_Init(&hi2c2);

baslangic_irtifa = 44330 * (1 - pow((MS5611_Press * 100 / 101325), (1 / 5.255)));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
while (1)
{

	if (HAL_GetTick() - lastTick >= 100)
	{
	// calışma moduna uygun veri alımının belirlenmesi
	lastTick = HAL_GetTick();
	        if ((calisma_durumu == 0) || (calisma_durumu == 1))
	        {
	            MS5611_Read_ActVal(&MS5611);
	            MS5611_FirstCalculateDatas(&MS5611);
	            MPU9255_Read_All(&hi2c2,&DataStruct);

              float roll_rad = DataStruct.KalmanAngleX * (M_PI / 180.0f);
              float pitch_rad = DataStruct.KalmanAngleY * (M_PI / 180.0f);

              float mag_x_comp = DataStruct.Mx * cos(pitch_rad) + DataStruct.My * sin(pitch_rad) * sin(roll_rad) + DataStruct.Mz * sin(pitch_rad) * cos(roll_rad);
              float mag_y_comp = DataStruct.My * cos(roll_rad) - DataStruct.Mz * sin(roll_rad);

              float yaw_angle = atan2f(mag_y_comp, mag_x_comp) * RAD_TO_DEG;

	            angX=DataStruct.KalmanAngleX; angY=DataStruct.KalmanAngleY; angZ=yaw_angle;
	            accX=DataStruct.Ax; accY=DataStruct.Ay; accZ=DataStruct.Az;
	            gyroX=DataStruct.Gx; gyroY=DataStruct.Gy; gyroZ=DataStruct.Gz;
	            enlem=gps.latitude; boylam=gps.longitude; gps_irtifa=gps.altitude;

	            irtifa=Calculate_Altitude(MS5611_Press);
	            yunuslama=Calculate_Pitch(accX,accY,accZ);
	        }
	        else if (calisma_durumu == 2)
	                {


	        		  irtifa       = bigEndianToFloat(&sut_buffer[1]);
	        	    MS5611_Press = bigEndianToFloat(&sut_buffer[5]);
	        	    accX         = bigEndianToFloat(&sut_buffer[9]);
	        	    accY         = bigEndianToFloat(&sut_buffer[13]);
	        	    accZ         = bigEndianToFloat(&sut_buffer[17]);
	        	    gyroX        = bigEndianToFloat(&sut_buffer[21]);
	        	    gyroY        = bigEndianToFloat(&sut_buffer[25]);
	        	    gyroZ        = bigEndianToFloat(&sut_buffer[29]);

                      if(ilk_irtifa == false){
                        baslangic_irtifa =irtifa;
                        ilk_irtifa = true;
                      }

	                    yunuslama=Calculate_Pitch(accX,accY,accZ);

	                    if ((irtifa-baslangic_irtifa)>5){b0=1;}
	                    if ((b0==1)&&(accZ<0)){b1=1;}
	                    if (irtifa>2000){b2=1;}
	                    if (gyroY>50||gyroY<-60){b3=1;}
	                    if (((onceki_irtifa-irtifa)>0)&&(b3==1)){b4=1;}
	                    if ((b1==1)&&(b4==1)&&(b3 == 1)){b5=1;HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, SET);}
	                    if ((irtifa<600)&&(b5==1)){b6=1;}
	                    if ((b6==1)&&(irtifa<600 )&& (irtifa>400)){b7=1; HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, SET);}
	                    sut_byte=(b7<<7)|(b6<<6)|(b5<<5)|(b4<<4)|(b3<<3)|(b2<<2)|(b1<<1)|(b0<<0);
	                    sut_durum_bilgi[0]=0xAA; sut_durum_bilgi[1]=sut_byte; sut_durum_bilgi[2]=0;
	                    uint8_t chk=0; for(int i=1;i<3;i++) chk+=sut_durum_bilgi[i]; chk%=256;
	                    sut_durum_bilgi[3]=chk; sut_durum_bilgi[4]=0x0D; sut_durum_bilgi[5]=0x0A;
                      HAL_UART_Transmit(&huart4, &sut_durum_bilgi, sizeof(sut_durum_bilgi), HAL_MAX_DELAY);

                      onceki_irtifa = irtifa;

	                }

	if (calisma_durumu == 1) {
	    transmit_SIT_packet(irtifa, MS5611_Press, accX, accY, accZ, angX, angY, angZ);
	}

	  else if (calisma_durumu == 0) {
		  if ((irtifa - baslangic_irtifa > 10) && rocket_state == 0) {
			  rocket_state = 1;
		  }


		  if ((rocket_state == 1) && ((onceki_irtifa - irtifa) > 1) && (yunuslama > 60.0 || yunuslama < -60.0)) {
			  rocket_state = 2;
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, SET);

		  }


		  if ((rocket_state == 2) && (irtifa < 600 || irtifa > 400)) {
			  rocket_state = 3;
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, SET);

		  }


		  if ((rocket_state == 3) && (irtifa == onceki_irtifa)) {
			  rocket_state = 4;

		  }


		  snprintf(gonderilecek_veri, sizeof(gonderilecek_veri), "$AA,%.0f,%.3f,%.3f,%.0f,%.2f,%.1f,%.1f,%.2f,%.2f,%.1f,%.2f,%d"
		  , irtifa, enlem, boylam, gps_irtifa, gyroX, gyroY, gyroZ, accX, accY, accZ
		  , yunuslama, rocket_state);

		  HAL_UART_Transmit(&huart2, &adres_h, sizeof(adres_h), HAL_MAX_DELAY);
		  HAL_UART_Transmit(&huart2, &adres_l, sizeof(adres_l), HAL_MAX_DELAY);
		  HAL_UART_Transmit(&huart2, &kanal, sizeof(kanal), HAL_MAX_DELAY);
		  HAL_UART_Transmit(&huart2, (uint8_t*)gonderilecek_veri, strlen(gonderilecek_veri), HAL_MAX_DELAY);

		  onceki_irtifa = irtifa;
	  }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LoRa_Aux_GPIO_Port, LoRa_Aux_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LoRa_M0_Pin|LoRa_M1_Pin|PC817_D1_Pin|PC817_D2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUZZER_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LoRa_Aux_Pin */
  GPIO_InitStruct.Pin = LoRa_Aux_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LoRa_Aux_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LoRa_M0_Pin LoRa_M1_Pin PC817_D1_Pin PC817_D2_Pin */
  GPIO_InitStruct.Pin = LoRa_M0_Pin|LoRa_M1_Pin|PC817_D1_Pin|PC817_D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
#ifdef USE_FULL_ASSERT
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
