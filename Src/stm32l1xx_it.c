/**
  ******************************************************************************
  * @file    stm32l1xx_it.c
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
#include "stm32l1xx_hal.h"
#include "stm32l1xx.h"
#include "stm32l1xx_it.h"

/* USER CODE BEGIN 0 */
#include "waves.h"

static uint32_t flancos=0;
static uint32_t milisec=0;
static uint16_t velocidad_prev=0;
static uint32_t flancos_prev=0;
static uint16_t num=0;
static uint8_t onda=0;
static uint8_t var=0;
static uint8_t contador=0;//para saber si el flanco es de subida o bajada en la PWM
extern uint16_t velocidad;
extern int16_t aceleracion;
extern uint16_t corriente;
extern uint8_t send;
extern uint16_t medida;
extern uint16_t pwm;
extern int16_t modo;
static int16_t boton;
static uint16_t t=0;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;

/******************************************************************************/
/*            Cortex-M3 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
	__disable_irq();
	milisec++;
	if (milisec%25==0)//Se hace el calculo de la corriente @ 20000Hz con TIM2.
	//También se hace el mustreo @ 40Hz (Nyquist) mediante el Systick ya que la velocidad varia a 20Hz(una onda comleta en 5 s)
	//Aunque en el Cube esta configurada la interrupcion solo por flanco de subida
	//detecta subida y bajada, debido seguramente a rebotes.
	{	
		velocidad=(flancos-flancos_prev)*20.944;//pi/6 radianes*40 (12 flancos) 
		aceleracion = velocidad - velocidad_prev;
		aceleracion= aceleracion*40;
		flancos_prev=flancos;
    velocidad_prev = velocidad;
	}
	__enable_irq();
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l1xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line0 interrupt.
*/
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */
	__disable_irq();
	EXTI->PR |= GPIO_PIN_0;  // borra flag (se borra escribiendo un 1)
	flancos++;
	__enable_irq();
  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
* @brief This function handles EXTI line4 interrupt.
*/
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */
	__disable_irq();
	EXTI->PR |= GPIO_PIN_4;  // borra flag (se borra escribiendo un 1)
	contador++;
	if (contador==1) 
	{
		pwm=1000;
	}
	else 
	{
		pwm=0;
		contador=0;
	}
	__enable_irq();
  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
* @brief This function handles ADC global interrupt.
*/
void ADC1_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_IRQn 0 */

  /* USER CODE END ADC1_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc);
  /* USER CODE BEGIN ADC1_IRQn 1 */

  /* USER CODE END ADC1_IRQn 1 */
}

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	__disable_irq();
	TIM2->SR &= 0xFFFFFFFE; //Borra flag
	corriente = (4090 - medida)*1.3455;
	if (t>10 && t<=20)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
			
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)==GPIO_PIN_SET){boton=1;}
			else
			{
				if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)==GPIO_PIN_SET){boton=4;}
				else
				{
					if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)==GPIO_PIN_SET){boton=7;}
					else
					{
						if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)==GPIO_PIN_SET){boton=10;};
					};
				};
			};
		}
	else if(t>20 && t<=30)
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
				
				if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)==GPIO_PIN_SET){boton=2;}
				else
				{
					if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)==GPIO_PIN_SET){boton=5;}
					else
					{
						if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)==GPIO_PIN_SET){boton=8;}
						else
						{
							if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)==GPIO_PIN_SET){boton=10;};
						};
					};
				};
			}
	else if (t>30 && t<=40)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
			
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)==GPIO_PIN_SET){boton=3;}
			else
			{
				if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)==GPIO_PIN_SET){boton=6;}
				else
				{
					if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)==GPIO_PIN_SET){boton=9;}
					else
					{
						if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)==GPIO_PIN_SET){boton=10;};
					};
				};
			};
		}
		
	if(boton<=9 && boton!=0)
		{
			onda=boton;
			modo=onda;
			boton=0;
			t=0;
		}
	else if (boton==0){modo=onda;}
	else 
		{
			onda=0;
			modo=onda;
		};
	
	if(t>=50)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
			t=0;
		};
	t++;
	send=1;
	__enable_irq();
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
* @brief This function handles TIM6 global interrupt.
*/
void TIM6_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_IRQn 0 */
	__disable_irq();
	TIM6->SR &= 0xFFFFFFFE; //Borra flag
	num=(var*40)+1000;
	if(var>=100)
	{
		var=0;
	};
	
	switch (onda)
	{
		case 0://Estado parado
			TIM3->CCR1=0;
			break;
		case 1://escalones(1V)
			TIM3->CCR1=1000;
			break;
		case 2://escalones(2V)
			TIM3->CCR1=2000;
			break;
		case 3://escalones(3V)
			TIM3->CCR1=3000;
			break;
		case 4://escalones(4V)
			TIM3->CCR1=4000;
			break;
		case 5://escalones(5V)
			TIM3->CCR1=5000;
			break;
		case 6://Diente de sierra
			TIM3->CCR1=num;
			break;
		case 7://Triangular
			if(var<50) TIM3->CCR1=(num*2)-1000;
			else TIM3->CCR1=11000-(num*2);
			break;
		case 8://Sinusoidal
			TIM3->CCR1=sinusoidal[var];
			break;
		case 9://Tiempo respuesta
			TIM3->CCR1=4000;
			break;
	}
	var++;
	__enable_irq();
  /* USER CODE END TIM6_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_IRQn 1 */

  /* USER CODE END TIM6_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
