#include "main.h"

int main() {
	int count=0;
	int i =450;
	int state =0;
	int oldstate =1;
	int speedstate =0;
	ticks_init();
	servo_init(143,10000,450);
	//Initialization
	button_init();
		u32 tick_img =0;
	while (1) {
		
				oldstate = state;
				state =  read_button(GPIO_Pin_14);
																									//0 preseed 1 not preseed
				if(oldstate == 0 &&state ==1){
					if(speedstate==0)
						speedstate=1;
					else
						speedstate=0;
				}
				
			if(tick_img != get_real_ticks()){
					tick_img = get_real_ticks();	
				
			if(speedstate==0){
				if(read_button(GPIO_Pin_13)==0&&i<1050&&tick_img%10==0){
				i+=50;
				servo_control(1,i);
			}
				else if(read_button(GPIO_Pin_15)==0&&i>450&&tick_img%10==0){
				i-=50;
				servo_control(1,i);
			}
				
				if(speedstate==1){
				if(read_button(GPIO_Pin_13)==0&&i<1050&&tick_img%10==0){
				i+=10;
				servo_control(1,i);
			}
				else if(read_button(GPIO_Pin_15)==0&&i>450&&tick_img%10==0){
				i-=10;
				servo_control(1,i);
			}
			}
			
			
			
			
		}
	}
}
	}
		//Run stuff continuously here

