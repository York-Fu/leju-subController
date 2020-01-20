#include "vl53l0x_cali.h"
//#include "online.h"

_vl53l0x_adjust Vl53l0x_adjust; //校准数据24c02写缓存区(用于在校准模式校准数据写入24c02)
_vl53l0x_adjust Vl53l0x_data;   //校准数据24c02读缓存区（用于系统初始化时向24C02读取数据）

#define adjust_num 5//校准错误次数

//VL53L0X校准函数
//dev:设备I2C参数结构体
VL53L0X_Error vl53l0x_adjust(VL53L0X_Dev_t *dev)
{
	
	VL53L0X_DeviceError Status = VL53L0X_ERROR_NONE;
	uint32_t refSpadCount = 7;
	uint8_t  isApertureSpads = 0;
	uint32_t XTalkCalDistance = 100;
	uint32_t XTalkCompensationRateMegaCps;
	uint32_t CalDistanceMilliMeter = 100<<16;
	int32_t  OffsetMicroMeter = 30000;
	uint8_t VhvSettings = 23;
	uint8_t PhaseCal = 1;
	u8 i=0;
	
	VL53L0X_StaticInit(dev);//数值恢复默认,传感器处于空闲状态
	
	//SPADS校准----------------------------
	spads:
	delay_ms(10);
	INFO("The SPADS Calibration Start...\r\n");
	Status = VL53L0X_PerformRefSpadManagement(dev,&refSpadCount,&isApertureSpads);//执行参考Spad管理
	if(Status == VL53L0X_ERROR_NONE)
	{
	    INFO("refSpadCount = %d\r\n",refSpadCount);
	    Vl53l0x_adjust.refSpadCount = refSpadCount;
	    INFO("isApertureSpads = %d\r\n",isApertureSpads);	
	    Vl53l0x_adjust.isApertureSpads = isApertureSpads;
        INFO("The SPADS Calibration Finish...\r\n\r\n");		
	    i=0;
	}
	else
	{
	    i++;
	    if(i==adjust_num) return Status;
	    INFO("SPADS Calibration Error,Restart this step\r\n");
	    goto spads;
	}
	//设备参考校准---------------------------------------------------
	ref:
	delay_ms(10);
	INFO("The Ref Calibration Start...\r\n");
	Status = VL53L0X_PerformRefCalibration(dev,&VhvSettings,&PhaseCal);//Ref参考校准
	if(Status == VL53L0X_ERROR_NONE)
	{
		INFO("VhvSettings = %d\r\n",VhvSettings);
		Vl53l0x_adjust.VhvSettings = VhvSettings;
		INFO("PhaseCal = %d\r\n",PhaseCal);
		Vl53l0x_adjust.PhaseCal = PhaseCal;
		INFO("The Ref Calibration Finish...\r\n\r\n");
		i=0;
	}
	else
	{
		i++;
		if(i==adjust_num) return Status;
		INFO("Ref Calibration Error,Restart this step\r\n");
		goto ref;
	}
	//偏移校准------------------------------------------------
	offset:
	delay_ms(10);
	INFO("Offset Calibration:need a white target,in black space,and the distance keep 100mm!\r\n");
	INFO("The Offset Calibration Start...\r\n");
	Status = VL53L0X_PerformOffsetCalibration(dev,CalDistanceMilliMeter,&OffsetMicroMeter);//偏移校准
	if(Status == VL53L0X_ERROR_NONE)
	{
		INFO("CalDistanceMilliMeter = %d mm\r\n",CalDistanceMilliMeter);
		Vl53l0x_adjust.CalDistanceMilliMeter = CalDistanceMilliMeter;
		INFO("OffsetMicroMeter = %d mm\r\n",OffsetMicroMeter);	
		Vl53l0x_adjust.OffsetMicroMeter = OffsetMicroMeter;
		INFO("The Offset Calibration Finish...\r\n\r\n");
		i=0;
	}
	else
	{
		i++;
		if(i==adjust_num) return Status;
		INFO("Offset Calibration Error,Restart this step\r\n");
		goto offset;
	}
	//串扰校准-----------------------------------------------------
	xtalk:
	delay_ms(20);
	INFO("Cross Talk Calibration:need a grey target\r\n");
	INFO("The Cross Talk Calibration Start...\r\n");	
	Status = VL53L0X_PerformXTalkCalibration(dev,XTalkCalDistance,&XTalkCompensationRateMegaCps);//串扰校准
	if(Status == VL53L0X_ERROR_NONE)
	{
		INFO("XTalkCalDistance = %d mm\r\n",XTalkCalDistance);
		Vl53l0x_adjust.XTalkCalDistance = XTalkCalDistance;
		INFO("XTalkCompensationRateMegaCps = %d\r\n",XTalkCompensationRateMegaCps);	
		Vl53l0x_adjust.XTalkCompensationRateMegaCps = XTalkCompensationRateMegaCps;
		INFO("The Cross Talk Calibration Finish...\r\n\r\n");
		i=0;
	}
	else
	{
		i++;
		if(i==adjust_num) return Status;
		INFO("Cross Talk Calibration Error,Restart this step\r\n");
		goto xtalk;
	}
	INFO("All the Calibration has Finished!\r\n");
	INFO("Calibration is successful!!\r\n");

	Vl53l0x_adjust.adjustok = 0xAA;//校准成功
	//AT24CXX_Write(0,(u8*)&Vl53l0x_adjust,sizeof(_vl53l0x_adjust));//将校准数据保存到24c02
	memcpy(&Vl53l0x_data,&Vl53l0x_adjust,sizeof(_vl53l0x_adjust));//将校准数据复制到Vl53l0x_data结构体
	return Status;
}

//vl53l0x校准测试
//dev:设备I2C参数结构体
void vl53l0x_calibration_test(VL53L0X_Dev_t *dev)
{  

}
