#include "AS5048.h"

u8 flag_AS5048_outtime = 0;            //��ʱ��־λ

void AS5048_timer(){
	if (TIM_GetITStatus(AS5048_TIMERX, TIM_IT_Update) != RESET){  //���TIM3�����жϷ������
		  
		  flag_AS5048_outtime  = 1;        //��ʱ
		
	  	TIM_ClearITPendingBit(AS5048_TIMERX, TIM_IT_Update);  //���TIMx�����жϱ�־ 
	}
}

/**
 * @return[out]  ��������ʼ������---������ĳ������˿ڣ�����������GPIO����
 */
void AS5048_pwm_init(void){
	 
	GPIO_QuickInit(AS5048_HW_GPIOx, AS5048_GPIO_Pin_x, GPIO_Mode_IPU);//echo
		
	TIM_us_Init_org(AS5048_TIMERX, 40000, 12);	      
	TIM_Cmd(AS5048_TIMERX, DISABLE);  
	
	if(AS5048_TIMERX == TIM2) 	TIMER_CallbackInstall(HW_TIMER2, AS5048_timer);	 
	else if(AS5048_TIMERX == TIM3) 	TIMER_CallbackInstall(HW_TIMER3, AS5048_timer);	 
	else if(AS5048_TIMERX == TIM4) 	TIMER_CallbackInstall(HW_TIMER4, AS5048_timer);	 
	
}

/**
 * @return[out]  ���ز�õĳ������ľ���  ��λmm
 */
u16 get_AS5048_pwm_value(void){
	flag_AS5048_outtime = 0;
	
	TIM_SetCounter(AS5048_TIMERX,0);
	TIM_Cmd(AS5048_TIMERX, ENABLE); 
	
	while(ECHO && flag_AS5048_outtime == 0); 
	
	while(!ECHO && flag_AS5048_outtime == 0);
	TIM_SetCounter(AS5048_TIMERX,0);
	TIM_Cmd(AS5048_TIMERX, ENABLE); 
	
	while(ECHO && flag_AS5048_outtime == 0); 
	TIM_Cmd(AS5048_TIMERX,DISABLE);
	
	if(flag_AS5048_outtime == 1){
		flag_AS5048_outtime = 0;
		return 65535;
	}
	
	return TIM_GetCounter(AS5048_TIMERX);   //���ز�ľ���
}


