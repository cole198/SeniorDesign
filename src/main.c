/**
  ******************************************************************************
  * @file    main.c
  * @author  Nathan Cole (cole198@purdue.edu)
  * @version V1.0
  * @date    02/16/2023
  * @brief   Ultrasonic distance sensing with STM32
  * Generated code from STM32CubeIDE
  * User code adapted from https://www.micropeta.com/video42
  ******************************************************************************
*/


#include "stm32f0xx.h"


#define TRIG_PIN GPIO_PIN_13 //Trig pin of ultrasonic Sensor
#define TRIG_PORT GPIOC
#define ECHO_PIN GPIO_PIN_10 //Echo pin of ultrasonic sensor
#define ECHO_PORT GPIOC
#define LED_PIN GPIO_PIN_11

uint32_t Value1 = 0; //Time measurements to determine distance
uint32_t Value2 = 0;
float Distance  = 0;  // cm

TIM_HandleTypeDef htim2;
static void setup_tim2(void);
static void MX_GPIO_Init(void);

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


  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */

//  MX_TIM2_Init();
  MX_GPIO_Init(); //Setup GPIO pins
  setup_tim2(); //Setup timer
  htim2.Instance = TIM2;

  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
      HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
      htim2.Instance->CNT = 0;
      while(htim2.Instance->CNT < 10); //Wait for 10 us
      HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET); // pull trig pin LOW

      // wait for the echo pin to go high
      while (!(GPIOC->IDR & ECHO_PIN));
      Value1 = htim2.Instance->CNT;

      // wait for echo pin to go low
      while (GPIOC->IDR & ECHO_PIN);
      Value2 = htim2.Instance->CNT;

      Distance = (Value2-Value1)* 0.034/2; //time divided by speed of sounds / 2 to get distance

      if (Distance < 20) { // If distance less than 20 cm
        GPIOC->BSRR = 1 << 11; //Set LED (Will notify vibration motors)
      }
      else {
        GPIOC->BSRR = 1 << 27; //Reset LED
      }
      HAL_Delay(60); // Wait 60 ms before sensing distance again (recommended)
//      GPIOC->ODR ^= GPIO_PIN_13;
    /* USER CODE BEGIN 3 */
  }

  /* USER CODE END 3 */
}


void TIM2_IRQHandler() { //Simple interrupt handler to reset timer.
    TIM2->SR = 0;


}

void setup_tim2() //Setup a timer so each increment of CNT is 1 us
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 7;
    TIM2->ARR = 60000;
    TIM2->DIER |= TIM_DIER_UIE;
    NVIC->ISER[0] = 1<<15;
    TIM2->CR1 |= TIM_CR1_CEN;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */



/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  //OUT
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
  GPIOC->MODER &= ~(0xff<<16);
  GPIOC->MODER |= 1 << 26;
  GPIOC->MODER |= 1 << 22;

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */



