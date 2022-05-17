#include "singleRanging.h"
#include "stm32f0xx.h"
#include "helper.h"

uint8_t lastSensorValue;

VL53L0X_Error measure(VL53L0X_Dev_t *pMyDevice, uint8_t count){
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	int i;
	char buf[VL53L0X_MAX_STRING_LENGTH];
	VL53L0X_RangingMeasurementData_t    RangingMeasurementData;
	FixPoint1616_t LimitCheckCurrent;
	
	if(Status == VL53L0X_ERROR_NONE)
    {
        for(i=0;i<count;i++){
            USART_Putstr("Call of VL53L0X_PerformSingleRangingMeasurement\n");
            Status = VL53L0X_PerformSingleRangingMeasurement(pMyDevice, &RangingMeasurementData);

            VL53L0X_GetLimitCheckCurrent(pMyDevice, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, &LimitCheckCurrent);

						lastSensorValue = RangingMeasurementData.RangeMilliMeter;
					
            if (Status != VL53L0X_ERROR_NONE) break;
						snprintf(buf, sizeof(buf), "%d", RangingMeasurementData.RangeMilliMeter);
						USART_Putstr("Measured distance: ");
            USART_Putstr(buf);
						USART_Putstr("\n");
						Delay(SystemCoreClock/8);
        }
    }
    return Status;
}

VL53L0X_Error initDevice(VL53L0X_Dev_t *pMyDevice){
		VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    VL53L0X_Version_t                   Version;
    VL53L0X_Version_t                  *pVersion   = &Version;
    VL53L0X_DeviceInfo_t                DeviceInfo;
    uint8_t VhvSettings;
    uint8_t PhaseCal;
		uint32_t refSpadCount;
		uint8_t isApertureSpads;

    int32_t status_int;
	
		VL53L0X_set_gpio();//init STM32F0 I2C

		// Initialize Comms
    pMyDevice->I2cDevAddr      = 0x52;
    pMyDevice->comms_type      =  1;
    pMyDevice->comms_speed_khz =  100;
    pMyDevice->comms_speed_khz =  400;

    if(Status == VL53L0X_ERROR_NONE)
    {
        status_int = VL53L0X_GetVersion(pVersion);
        if (status_int != 0)
            Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    /*
     *  Verify the version of the VL53L0X API running in the firmware
     */

    if(Status == VL53L0X_ERROR_NONE)
    {
        USART_Putstr("Call of VL53L0X_DataInit\n");
        Status = VL53L0X_DataInit(pMyDevice); // Data initialization
    }

    if(Status == VL53L0X_ERROR_NONE)
    {
        Status = VL53L0X_GetDeviceInfo(pMyDevice, &DeviceInfo);
    }
		
		    if(Status == VL53L0X_ERROR_NONE)
    {
        USART_Putstr("Call of VL53L0X_StaticInit\n");
        Status = VL53L0X_StaticInit(pMyDevice); // Device Initialization	
    }
		
    
    if(Status == VL53L0X_ERROR_NONE)
    {
       USART_Putstr("Call of VL53L0X_PerformRefCalibration\n");
        Status = VL53L0X_PerformRefCalibration(pMyDevice, &VhvSettings, &PhaseCal); // Device Initialization
    }

    if(Status == VL53L0X_ERROR_NONE)
    {
        USART_Putstr("Call of VL53L0X_PerformRefSpadManagement\n");
        Status = VL53L0X_PerformRefSpadManagement(pMyDevice, &refSpadCount, &isApertureSpads); // Device Initialization
    }

    if(Status == VL53L0X_ERROR_NONE)
    {
        USART_Putstr("Call of VL53L0X_SetDeviceMode\n");
        Status = VL53L0X_SetDeviceMode(pMyDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
    }

    // Enable/Disable Sigma and Signal check
    if (Status == VL53L0X_ERROR_NONE) 
		{
        Status = VL53L0X_SetLimitCheckEnable(pMyDevice, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    }
    if (Status == VL53L0X_ERROR_NONE) 
		{
        Status = VL53L0X_SetLimitCheckEnable(pMyDevice, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    }

    if (Status == VL53L0X_ERROR_NONE) 
		{
        Status = VL53L0X_SetLimitCheckEnable(pMyDevice, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 1);
    }

    if (Status == VL53L0X_ERROR_NONE) 
		{
        Status = VL53L0X_SetLimitCheckValue(pMyDevice, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, (FixPoint1616_t)(1.5*0.023*65536));
    }
	
		return Status;
}