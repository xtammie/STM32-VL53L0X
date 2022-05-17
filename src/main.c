#include "helper.h"
#include "singleRanging.h"

int main(int argc, char **argv)
{	
		VL53L0X_Error Status = VL53L0X_ERROR_NONE;
		VL53L0X_Dev_t MyDevice;
		VL53L0X_Dev_t *pMyDevice = &MyDevice;
		
		USART_Setup();
		USART_Clearscreen();
		USART_Putstr("Initialization and single range measurement VL53L0X:\n\n");

		Status = initDevice(pMyDevice);
	
		if(Status == VL53L0X_ERROR_NONE){
			measure(pMyDevice, 5);
		}
		
		USART_Putstr("\nTHE END");
    
    return (0);
}
