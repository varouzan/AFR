/*
 * movement.c
 *
 *  Created on: Nov 11, 2018
 *      Author: varou
 */
#include "main.h"
#include "stm32f4xx_hal.h"

uint8_t reset=1;
int l=0,r=0;
extern volatile float dist0,dist1,dist4;
extern volatile uint8_t turning,fire_var,attac,avoidance_on;
extern int fake_flag,fake_timer;


void start(void){
	  set_pwm_ch1(3000);
	  set_pwm_ch2(3000);

}

void left(void){
	  start();
	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
	  //set_pwm_ch1(1000);


	  set_pwm_ch1(2000);
	  set_pwm_ch2(2000);
}

void right(void){
	  start();
	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
	  //set_pwm_ch2(1000);

	  set_pwm_ch1(2000);
	  set_pwm_ch2(2000);
}

void forward(void){
	  start();
	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
}

void backward(void){
	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
}

void stop(void){
	  set_pwm_ch1(0);
	  set_pwm_ch2(0);
}


void fire(void){
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);
}

void avoid(void){
		if(dist4<=2){
			  if(dist0>dist1 && reset){
				  left();
				  l++;
				  reset=0;
			  }
			  else if(reset){
				  right();
				  r++;
				  reset=0;
			  }
		  }
		  else if(dist0 <=1.5 && reset){
			  left();
			  if(reset){
				  l++;
				  reset=0;
			  }
		  }
		  else if(dist1 <=1.5 && reset){
			  right();
			  if(reset){
				  r++;
				  reset=0;
			  }
		  }
		  else if(dist4>4 && dist0>1.5 && dist1>1.5){
			  forward();
			  reset=1;
		  }
}


void avoid2(void){
	switch(turning){
	case 0:
		if(dist4<=2){
			  HAL_NVIC_DisableIRQ(EXTI2_IRQn);//photodiode
			  HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);//photodiode
			  if(dist0>dist1 && reset){
				  left();
				  l++;
				  reset=0;
			  }
			  else if(reset){
				  right();
				  r++;
				  reset=0;
			  }
		  }
		  else if(dist0 <=1.5 && reset){
			  left();
			  if(reset){
				  l++;
				  reset=0;
			  }
		  }
		  else if(dist1 <=1.5 && reset){
			  right();
			  if(reset){
				  r++;
				  reset=0;
			  }
		  }
		  else if(dist4>4 && dist0>2.5 && dist1>2.5){
			  HAL_NVIC_EnableIRQ(EXTI3_IRQn);//photodiode
			  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);//photodiode
			  forward();
			  reset=1;
		  }
		break;
	case 1:
		turning=1;
		break;
	default:
		avoid();
	}
}


void check_fire(void){

	if(fire_var){
		turning=0;
		attac=0;
		fire_var=0;
		avoidance_on=1;
		forward();
		stop();
		//HAL_Delay(500);
		fire();
		start();
		forward();
		//HAL_GPIO_TogglePin(GPIOA,LD2_Pin);
		HAL_NVIC_EnableIRQ(EXTI3_IRQn);
		HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	}


}



void check_fire2(void){
	if(fire_var){
		if(fake_flag){
			if(attac){
				turning=0;
				attac=0;
				fire_var=0;
				forward();
				stop();
				//HAL_Delay(500);
				fire();
				start();
				forward();
				HAL_GPIO_TogglePin(GPIOA,LD2_Pin);
				HAL_NVIC_EnableIRQ(EXTI3_IRQn);
				HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
			}
			else{
				start();
				forward();
			}
		}
	}

}
