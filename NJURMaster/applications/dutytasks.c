#include "main.h"

/**
  * @brief 1ms任务
  * @param 系统从开机到现在经过的毫秒数
  * @retval None
  */
static void Duty_1ms(u32 _time)
{
	//u32 loop_time = GetInnerLoop(Task_1ms_Time);
	DatatransferTask(_time);
	
}

/**
  * @brief 2ms任务
  * @param 系统从开机到现在经过的毫秒数
  * @retval None
  */
static void Duty_2ms(u32 _time)
{
	u32 loop_time = GetInnerLoop(Task_2ms_Time);
	MPU6500_Data_Prepare();
	IMUupdate(loop_time/2000000.0f,MPU6500_Gyro.x,MPU6500_Gyro.y,MPU6500_Gyro.z,
						MPU6500_Acc.x,MPU6500_Acc.y,MPU6500_Acc.z,
						&Roll,&Pitch,&Yaw);
	
}

/**
  * @brief 5ms任务
  * @param 系统从开机到现在经过的毫秒数
  * @retval None
  */
static void Duty_5ms(u32 _time)
{
	u32 loop_time = GetInnerLoop(Task_5ms_Time);
	CheckDog();
	ChassisControl(loop_time/1000000.0f);
	GimbalControl(loop_time/1000000.0f);
}

/**
  * @brief 10ms任务
  * @param 系统从开机到现在经过的毫秒数
  * @retval None
  */
static void Duty_10ms(u32 _time)
{
	u32 loop_time = GetInnerLoop(Task_10ms_Time);
	IST8310_Data_Prepare();
	FireControl(loop_time/1000000.0f);
	
}

/**
  * @brief 20ms任务
  * @param 系统从开机到现在经过的毫秒数
  * @retval None
  */
static void Duty_20ms(u32 _time)
{
	u32 loop_time = GetInnerLoop(Task_20ms_Time);

}

/**
  * @brief 50ms任务
  * @param 系统从开机到现在经过的毫秒数
  * @retval None
  */
static void Duty_50ms(u32 _time)
{

	//u32 loop_time = GetInnerLoop(Task_50ms_Time);
//	if (IsDeviceLost(DEVICE_INDEX_RC))
//	BOTH_LED_TOGGLE();
//	int j=0;
//	char buffer[200];
//	j = sprintf(buffer," Hello World !\nthis time is %d\nj=%d\n\n",Get_Time_Micros(),j);
//		Usart2_Send((u8*)buffer,j);
	if (IsDeviceLost(DEVICE_INDEX_TIMEOUT))
	{
		BOTH_LED_TOGGLE();
	}
}

/**
  * @brief 系统主任务循环
  * @param None
  * @retval None
  * @details 由定时器每毫秒准时调用，由这里进入各种task
  */
void Duty_loop(void)
{
	static u32 systime_ms=0;
	(GetInnerLoop(DutyLoop_Time));
	systime_ms++;
	Duty_1ms(systime_ms);
	if (systime_ms%2==0)Duty_2ms(systime_ms);
	if (systime_ms%5==0)Duty_5ms(systime_ms);
	if (systime_ms%10==0)Duty_10ms(systime_ms);
	if (systime_ms%20==0)Duty_20ms(systime_ms);
	if (systime_ms%50==0)Duty_50ms(systime_ms);
	if (GetInnerLoop(DutyLoop_Time)<1000)FeedDog(DEVICE_INDEX_TIMEOUT);
	
}
	
	
