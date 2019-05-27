/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

/* USER CODE BEGIN 0 */
#include "main.h"
#include "movement.h"
extern volatile float dist0,dist1,dist4;
volatile uint8_t attac,turning=0,fire_var=0;
extern volatile uint8_t avoidance_on;
extern volatile uint32_t count;
extern int fake_flag,fake_timer;
//extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;

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
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
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
  uint32_t counter;
	if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0)==GPIO_PIN_SET){
		__HAL_TIM_SET_COUNTER(&htim5,0);
		HAL_TIM_Base_Start(&htim5);
		//count=__HAL_TIM_GET_COUNTER(&htim3);
	}
	if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0)==GPIO_PIN_RESET){
		counter=__HAL_TIM_GET_COUNTER(&htim5);
		dist0= (float) counter/58;
		if(dist0<1.5){
			HAL_GPIO_WritePin(GPIOA,LD2_Pin,GPIO_PIN_SET);
		}
		else{
			HAL_GPIO_WritePin(GPIOA,LD2_Pin,GPIO_PIN_RESET);
		}
		HAL_NVIC_DisableIRQ(EXTI0_IRQn);

	}
  /* USER CODE END EXTI0_IRQn 1 */
}

/**
* @brief This function handles EXTI line1 interrupt.
*/
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI1_IRQn 1 */
  uint32_t counter;
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1)==GPIO_PIN_SET){
			__HAL_TIM_SET_COUNTER(&htim2,0);
			HAL_TIM_Base_Start(&htim2);
			//count=__HAL_TIM_GET_COUNTER(&htim3);
		}
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1)==GPIO_PIN_RESET){
			counter=__HAL_TIM_GET_COUNTER(&htim2);
			dist1= (float) counter/58;
			if(dist1<1.5){
				HAL_GPIO_WritePin(GPIOA,LD2_Pin,GPIO_PIN_SET);
			}
			else{
				HAL_GPIO_WritePin(GPIOA,LD2_Pin,GPIO_PIN_RESET);
			}
			HAL_NVIC_DisableIRQ(EXTI1_IRQn);

		}


  /* USER CODE END EXTI1_IRQn 1 */
}

/**
* @brief This function handles EXTI line2 interrupt.
*/
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */

  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */



  HAL_GPIO_TogglePin(GPIOA,LD2_Pin);


  /* USER CODE END EXTI2_IRQn 1 */
}

/**
* @brief This function handles EXTI line3 interrupt.
*/
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */

  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
  /* USER CODE BEGIN EXTI3_IRQn 1 */
//  if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_3)==GPIO_PIN_SET){
//
//	  //HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
//	  HAL_GPIO_WritePin(GPIOA,LD2_Pin,GPIO_PIN_SET);
//  }
//  else{
//	  //HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
//	  HAL_GPIO_WritePin(GPIOA,LD2_Pin,GPIO_PIN_RESET);
//  }
  //HAL_GPIO_TogglePin(GPIOA,LD2_Pin);
  fire_var=1;
  turning=1;
  avoidance_on=0;
  //turning=1;
  HAL_NVIC_DisableIRQ(EXTI3_IRQn);//photodiode main
  //HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
  //HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  //HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);


  /* USER CODE END EXTI3_IRQn 1 */
}

/**
* @brief This function handles EXTI line4 interrupt.
*/
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_IRQn 1 */
  uint32_t counter;
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4)==GPIO_PIN_SET){
		__HAL_TIM_SET_COUNTER(&htim4,0);
		HAL_TIM_Base_Start(&htim4);
		//count=__HAL_TIM_GET_COUNTER(&htim3);
	}
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4)==GPIO_PIN_RESET){
		counter=__HAL_TIM_GET_COUNTER(&htim4);
		dist4= (float) counter/58;
		if(dist4<1.5){
			HAL_GPIO_WritePin(GPIOA,LD2_Pin,GPIO_PIN_SET);
		}
		else{
			HAL_GPIO_WritePin(GPIOA,LD2_Pin,GPIO_PIN_RESET);
		}
		HAL_NVIC_DisableIRQ(EXTI4_IRQn);

	}


  /* USER CODE END EXTI4_IRQn 1 */
}

/**
* @brief This function handles EXTI line[9:5] interrupts.
*/
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */
  if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_5)==GPIO_PIN_SET ){
//
//	  HAL_GPIO_WritePin(GPIOA,LD2_Pin,GPIO_PIN_SET);
	  right();
	  turning=1;
	  avoidance_on=0;
  }
  else{
//	  HAL_GPIO_WritePin(GPIOA,LD2_Pin,GPIO_PIN_RESET);
	  left();
	  turning=1;
	  avoidance_on=0;
  }
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
  HAL_GPIO_TogglePin(GPIOA,LD2_Pin);
  turning=1;
  avoidance_on=0;

  //HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);//photodiode
//  turning=1;


  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
* @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
*/
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */
  //HAL_GPIO_TogglePin(GPIOA,LD2_Pin);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);//photodiode
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);//photodiode
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);//photodiode main
  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
* @brief This function handles EXTI line[15:10] interrupts.
*/
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */
  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_10)==GPIO_PIN_SET  || HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_11)==GPIO_PIN_SET){
//	  start();
//	  right();
	  HAL_GPIO_TogglePin(GPIOA,LD2_Pin);
  }

//  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_10)==GPIO_PIN_SET  || HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_11)==GPIO_PIN_SET){
//
//	  //HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
//	  HAL_GPIO_WritePin(GPIOA,LD2_Pin,GPIO_PIN_SET);
//  }
//  else{
//	  //HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
//	  HAL_GPIO_WritePin(GPIOA,LD2_Pin,GPIO_PIN_RESET);
//  }
  turning=1;
  HAL_GPIO_TogglePin(GPIOA,LD2_Pin);


  //HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);//photodiode
//  turning=1;

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
