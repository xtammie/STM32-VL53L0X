#include "helper.h"
#include "stm32f0_discovery.h"
#include "singleRanging.h"

int main(int argc, char **argv)
{	
		VL53L0X_Error Status = VL53L0X_ERROR_NONE;
		VL53L0X_Dev_t MyDevice;
		VL53L0X_Dev_t *pMyDevice = &MyDevice;
		char buf [15] = "\0";

		STM_EVAL_LEDInit(LED3);
		STM_EVAL_LEDInit(LED4);
		
		USART_Setup();
		USART_Clearscreen();
		USART_Putstr("Initialization and single range measurement VL53L0X:\n\n");

		Status = initDevice(pMyDevice);
	
		if(Status == VL53L0X_ERROR_NONE){
			measure(pMyDevice, 5);
		}
				
		snprintf(buf, sizeof(buf), "inhoud: %d\n", calcVolume(0));
		USART_Putstr(buf);
		
		USART_Putstr("\nTHE END");
    
    return (0);
}
