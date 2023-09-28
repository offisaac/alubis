## 阿路比LPMS_BE2库使用指南

此驱动库使用串口通信实现与陀螺仪的数据传输，**串口通信波特率默认为230400**



### 基本使用操作

**一、**

创建类对象，分别输入与陀螺仪通信的串口号，和自定义的传感器id号

```c++
LPMS_BE2_Classdef LPMS1(1,1);
```

**二、**

在Service_Communication.cpp的串口接受任务中调用

LPMS_BE2_Get_Data（(uint8_t *)Usart_RxCOB.address）数据接收函数

```c++
void Task_UsartReceive(void *arg)
{
  /* Cache for Task */
  static USART_COB  Usart_RxCOB;
  /* Pre-Load for task */
  /* Infinite loop */
  for(;;)
  {
    /* Usart Receive Port*/
    if(xQueueReceive(USART_RxPort,&Usart_RxCOB,portMAX_DELAY) == pdPASS)
    {
      /* User Code Begin Here -------------------------------*/
      switch(Usart_RxCOB.port_num)
      {
		case 1:
		LPMS1.LPMS_BE2_Get_Data((uint8_t *)Usart_RxCOB.address);  // 接收串口数据
		  break;
        case 3:      
          break;
        case 4:
          break;
		case 5:
		  break;
		case 6:
		  break;
      }
      /* User Code End Here ---------------------------------*/
		  }
  }
}
```

**三、**

在Service_Device.cpp中创建任务实现陀螺仪的初始化和数据更新

```c++
extern LPMS_BE2_Classdef LPMS1;
TaskHandle_t LPMS_Handle;
void tskLPMS(void *arg);
void Service_Devices_Init(void)
{
  xTaskCreate(tskDjiMotor, 	"App.Motor",   Small_Stack_Size, NULL, PriorityAboveNormal, &DjiMotor_Handle);
	#if  USE_SRML_MPU6050
  xTaskCreate(tskIMU,				"App.IMU",	   Small_Stack_Size, NULL, PriorityNormal, &IMU_Handle);
	#endif
  xTaskCreate(tskDR16, 			"App.DR16",    Small_Stack_Size, NULL, PriorityAboveNormal, &DR16_Handle);
 xTaskCreate(tskLPMS,      "App.LPMS", Normal_Stack_Size, NULL, PriorityAboveNormal,&LPMS_Handle);
}

/**
* @brief <freertos> 阿路比陀螺仪数据读取任务
*/
void tskLPMS(void *arg)
{
 for(;;)
 { 
    if(LPMS1.is_init==false)
    {
      LPMS1.LPMS_BE2_Init();
    }
    else if(LPMS1.is_init==true)
    {
      LPMS1.LPMS_BE2_Data_Convert();
    }
   vTaskDelay(1);
 }
}

```

**PS：记得初始化串口时（System_Config.cpp)把串口中断回调函数重定向到SRML库中的函数，以及在串口发送任务中定义相关的串口指针（Service_Communication.cpp）**

```C++
Uart_Init(&huart1, Uart1_Rx_Buff, USART1_RX_BUFFER_SIZE, User_UART1_RxCpltCallback);
Uart_Init(&huart6, Uart6_Rx_Buff, USART2_RX_BUFFER_SIZE, User_UART6_RxCpltCallback);
```

```C++
void Task_UsartTransmit(void *arg)
{
  /* Cache for Task */
	UART_HandleTypeDef* pUart_x = NULL;
  static USART_COB  Usart_TxCOB;
  /* Pre-Load for task */
  /* Infinite loop */
  for(;;)
  {
    /* Usart Receive Port*/
    if(xQueueReceive(USART_TxPort,&Usart_TxCOB,portMAX_DELAY) == pdPASS)
    {
      /* User Code Begin Here -------------------------------*/
      switch(Usart_TxCOB.port_num)
      {
		case 1:
		  pUart_x = &huart1;
		  break;
        case 3:
          pUart_x = &huart3;
          break;
        case 4:
          pUart_x = &huart4;
          break;
        case 6:
          pUart_x = &huart6;
		  break;
		default:
		  pUart_x = NULL;
		  break;
      }
      /* User Code End Here ---------------------------------*/
		if(pUart_x != NULL)
			HAL_UART_Transmit_DMA(pUart_x,(uint8_t*)Usart_TxCOB.address,Usart_TxCOB.len);
    }
  }
}

```

做完这些，就能够成功读取陀螺仪所有数据了



**使用陀螺仪参数，请按以下方式调用**

```C++
LPMS1.get_data().calAccX;
```



### 配置陀螺仪

**一、选择读取部分陀螺仪数据**

在LPMS_BE2.h文件中，有用宏定义组成的数据读取开关

```C++
/* 数据选择开关 */
#define RAW_ACC 		1		// 原始的加速度数据
#define CAL_ACC 		1		// 校准的加速度数据
#define RAW_GYRO 		1		// 原始的陀螺仪数据
#define CALI_GYRO 		1		// 静止偏差校准后的陀螺仪数据
#define ALIG_GYRO 		1		// 坐标轴校准后的陀螺仪数据
#define ANGULAR_SPEED 	1 		// 角速度
#define QUETERNION 		1		// 四元数
#define EULER 			1		// 欧拉角
#define LINEAR_ACC 		1		// 线性加速度
#define TEMPERATURE 	1		// 温度
```

如果定义为1，则表示需要读取这一组数据，如果定义为0，则表示不需要读取，所有开关可自行选择

**二、配置陀螺仪参数**

LPMS_BE2.h文件中，可供配置的宏定义参数如下

```C++
/* 参数设置宏定义 */
//数据类型
#define DATA_32BIT 1
#define DATA_16BIT 0
//对应函数
void Data_Type_Config(uint8_t _data_type);//陀螺仪数据输出类型设置函数

//陀螺仪量程范围
#define GYR_RANGE_125DPS  125
#define GYR_RANGE_250DPS  250
#define GYR_RANGE_500DPS  500
#define GYR_RANGE_1000DPS 1000
#define GYR_RANGE_2000DPS 2000
//加速度量程范围
#define ACC_RANGE_2G 	2
#define ACC_RANGE_4G 	4
#define ACC_RANGE_8G 	8
#define ACC_RANGE_16G   16
//数据输出单位
#define DEGREE 0
#define RADIAN 1
//对应函数
void Data_Range_Config(uint16_t _gyr_range, uint8_t _acc_range, uint8_t _degrad_unit);	//陀螺仪数据量程设置函数

//滤波模式
#define GYR_ONLY 0
#define KALMAN 	 1
#define DCM 	 3
//使能自动校准
#define AUTO_CALI_ENABLE  1
#define AUTO_CALI_DISABLE 0
//调整坐标系模式
#define OBJECT 	 0
#define HEADING  1
#define ALIGMENT 2
//对应函数
void Filter_Mode_Config(uint8_t _filter_mode, uint8_t _auto_cali, uint8_t _offset_mode);//陀螺仪滤波修正设置函数

//设置数据流频率
#define FREQ_5HZ  	5
#define FREQ_10HZ 	10
#define FREQ_50HZ 	50
#define FREQ_100HZ 	10
#define FREQ_250HZ 	250
#define FREQ6_500HZ 500
//对应函数
void Stream_Freq_Config(uint16_t _stream_freq);	//陀螺仪数据流频率设置函数
```

修改参数函数可以在Service_Device.cpp里面的陀螺仪任务进入for循环之前调用，例

```
/**
* @brief <freertos> 阿路比陀螺仪数据读取任务
*/
void tskLPMS(void *arg)
{
 LPMS1.Data_Range_Config(GYR_RANGE_2000DPS,ACC_RANGE_4G,DEGREE);
 for(;;)
 { 
    if(LPMS1.is_init==false)
    {
      LPMS1.LPMS_BE2_Init();
    }
    else if(LPMS1.is_init==true)
    {
      LPMS1.LPMS_BE2_Data_Convert();
    }
   vTaskDelay(1);
 }
}

```

**PS：由于未知bug，当前陀螺仪设置数据流输出精度和设置输出单位时只能设为DATA_32BIT和RADIAN，其他方式只能通过上位机修改（建议使用默认设置）**