/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "stm32f3xx.h"
#include "stm32f3xx_it.h"

/* USER CODE BEGIN 0 */
#include "LiquidCrystal.h"

#define RATE_OF_7_SEG 5
extern int flag;
int select = 1;
extern int isOnFire;
int InitTemp;
int isTempRecorded = 0;
int temp = 0, light = 0;
int LEDEnabled = 0;
int LDRorVOL = 0; //0 = LDR. 1 = VOLUME

void showModeMenu(){
	clear();
	setCursor(1, 1);
	print("LDR");
	setCursor(0, 1);
	write(5);
	setCursor(1, 2);
	print("VOLUME");
}
void enableSeg(int i){
	switch (i){
		case 0:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 1);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 1);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 1);
			break;
		case 1:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 1);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 0);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 1);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 1);
			break;
		case 2: 
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 1);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 1);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 0);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 1);
			break;
		case 3:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 1);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 1);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 1);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 0);
			break;
	}
}

void printOn7Seg(char c){
	switch (c){
		case 'S':
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, 1);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 1);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, 1);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, 1);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, 1);
			break;
		case 'A':
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, 1);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, 1);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 1);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 1);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, 1);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, 1);
			break;
		case 'F':
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, 1);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 1);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, 1);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, 1);
			break;
		case 'E':
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, 1);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, 1);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 1);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, 1);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, 1);
			break;
		case 'I':
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, 1);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 1);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, 0);
			break;
		case ' ':
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, 0);
			break;
	}
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart2;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line0 interrupt.
*/
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */
	
	clear();
	if(!flag){
		setCursor(0, 1);
		write(4);
		setCursor(1, 1);	
		print("1.MODE");
		setCursor(1, 2);
		print("2.ON/OFF/AUTO");
		flag = 1;
		//printOn7Seg('S');
	}else if(flag == 1 && select == 1){
		showModeMenu();
		flag = 2;
	}else if(flag == 2){
		flag = 0;
	}
	
	
  /* USER CODE END EXTI0_IRQn 1 */
}

/**
* @brief This function handles ADC1 and ADC2 interrupts.
*/
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */
	temp = HAL_ADC_GetValue(&hadc1);
	light = HAL_ADC_GetValue(&hadc2);
  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  HAL_ADC_IRQHandler(&hadc2);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */
	HAL_ADC_Start_IT(&hadc1);
	HAL_ADC_Start_IT(&hadc2);	
	
  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
	//HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_12);
	if(!isOnFire){
		enableSeg(0);
		printOn7Seg('S');

		//HAL_Delay(RATE_OF_7_SEG);
		enableSeg(1);
		printOn7Seg('A');
		HAL_Delay(RATE_OF_7_SEG);
		enableSeg(2);
		printOn7Seg('F');
		HAL_Delay(RATE_OF_7_SEG);
		enableSeg(3);
		printOn7Seg('E');
		HAL_Delay(RATE_OF_7_SEG);
	}else{
		enableSeg(0);
		printOn7Seg('F');
		HAL_Delay(RATE_OF_7_SEG);
		enableSeg(1);
		printOn7Seg('I');
		HAL_Delay(RATE_OF_7_SEG);
		enableSeg(2);
		printOn7Seg('A');
		HAL_Delay(RATE_OF_7_SEG);
		enableSeg(3);
		printOn7Seg('E');
		HAL_Delay(RATE_OF_7_SEG);
		printOn7Seg(' ');
		HAL_Delay(RATE_OF_7_SEG);
	}
	
	
  /* USER CODE END TIM2_IRQn 1 */
}

/**
* @brief This function handles TIM4 global interrupt.
*/
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */
	if(!flag){
	
	setCursor(17, 1);
	write(0);
	setCursor(16, 1);
	write(1);
	setCursor(15, 1);
	print(":");
	setCursor(13, 1);
	print("C");
	setCursor(10, 1);
	char data[5];
	sprintf(data, "%d ", temp);
	print(data);
	if(!isTempRecorded){
		InitTemp = temp;
		isTempRecorded = 1;
	}else{
		if(temp > InitTemp){
			isOnFire = 1;
		}else{
			isOnFire = 0;
		}
	}
	
	setCursor(17, 2);
	write(2);
	setCursor(16, 2);
	write(3);
	setCursor(15, 2);
	print(":");
	setCursor(13, 2);
	print("%");
	setCursor(10, 2);
	char data2[5];
	sprintf(data2, "%d ", light);
	print(data2);
  /* USER CODE END TIM4_IRQn 1 */
}
	}

/**
* @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
*/
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
	extern unsigned char safe[1];
	
  /* USER CODE END USART2_IRQn 1 */
}

/**
* @brief This function handles EXTI line[15:10] interrupts.
*/
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
	
  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */
	HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8);
	if(flag != 0){
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)){
			setCursor(0, 1);
			print(" ");
			setCursor(0, 2);
			write(4);
			select = 1;
		}else{
			setCursor(0, 2);
			print(" ");
			setCursor(0, 1);
			write(4);
			select = 2;
		}
	}
  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
