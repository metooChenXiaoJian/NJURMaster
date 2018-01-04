#include "main.h"



/*******串口1功能********************
   与遥控器接收机通信
	 查数据手册可知串口1的接收数据寄存器
	 与DMA2的流2通道4是对应的
	 另外采用双缓冲机制
*************************************/

/*******串口1变量********************/
static uint8_t _USART1_DMA_RX_BUF[2][BSP_USART1_DMA_RX_BUF_LEN];//DMA的两个缓冲区，缓冲区稍大于遥控器数据帧
/************************************/

/**
  * @brief 串口1初始化，DMA2初始化
  * @param BaudRate
  * @retval None
  * @details	BaudRate	115200
	*						使能DMA接收，外设到内存，使能串口IDLE空闲中断
	*						
	*						RX	PB7			
  */



void USART1_Configuration(uint32_t baud_rate)
{
    GPIO_InitTypeDef gpio;
	  USART_InitTypeDef usart;
	  NVIC_InitTypeDef nvic;
    DMA_InitTypeDef dma;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); 
    
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
    
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &gpio);//初始化PB7
    
    USART_DeInit(USART1);
    USART_StructInit(&usart);
    usart.USART_BaudRate = baud_rate;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_Even;
    usart.USART_Mode = USART_Mode_Rx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &usart);
    
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    
    DMA_DeInit(DMA2_Stream2);
    DMA_StructInit(&dma);
    dma.DMA_Channel = DMA_Channel_4;//串口1对应流2通道4
    dma.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);//外设基址为串口1
    dma.DMA_Memory0BaseAddr = (uint32_t)(&_USART1_DMA_RX_BUF[0][0]);//DMA缓冲区内存基址1
    dma.DMA_DIR = DMA_DIR_PeripheralToMemory;//外设到内存
    dma.DMA_BufferSize = sizeof(_USART1_DMA_RX_BUF)/2;//缓冲区大小为20
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址不变
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;//内存地址自增
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据单位为字节
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//缓冲区数据单位为字节
    dma.DMA_Mode = DMA_Mode_Circular;//循环模式
    dma.DMA_Priority = DMA_Priority_Medium;//中等优先级
    dma.DMA_FIFOMode = DMA_FIFOMode_Disable;//禁止队列
    //dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    //dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    //dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream2, &dma);
    
    //配置Memory1,Memory0是第一个使用的Memory
    DMA_DoubleBufferModeConfig(DMA2_Stream2, (uint32_t)&_USART1_DMA_RX_BUF[1][0], DMA_Memory_0);   //first used memory configuration
    DMA_DoubleBufferModeCmd(DMA2_Stream2, ENABLE);
    
    DMA_Cmd(DMA2_Stream2, ENABLE);
    
    nvic.NVIC_IRQChannel = USART1_IRQn;                          
		nvic.NVIC_IRQChannelPreemptionPriority = 0;   //抢占优先级最高
		nvic.NVIC_IRQChannelSubPriority = 0;		    //副优先级最高
		nvic.NVIC_IRQChannelCmd = ENABLE;			
		NVIC_Init(&nvic);	
    
		USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);        //使能串口1空闲中断
    USART_Cmd(USART1, ENABLE);
}

//串口1接收中断服务函数
void USART1_IRQHandler(void)
{
	static uint32_t this_time_rx_len = 0;

	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
	{
		//清除空闲中断
		(void)USART1->SR;
		(void)USART1->DR;

		//当前在Memory0
		if(DMA_GetCurrentMemoryTarget(DMA2_Stream2) == 0)
		{
			DMA_Cmd(DMA2_Stream2, DISABLE);
			this_time_rx_len = BSP_USART1_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA2_Stream2);
			DMA2_Stream2->NDTR = (uint16_t)BSP_USART1_DMA_RX_BUF_LEN; //将数据接收缓冲区的指针复位
			DMA2_Stream2->CR |= (uint32_t)(DMA_SxCR_CT);                  //将数据接收缓冲区设为Memory1
			DMA_Cmd(DMA2_Stream2, ENABLE);
      if(this_time_rx_len == RC_FRAME_LENGTH)
			{
				  RcProtocolAnalysis(_USART1_DMA_RX_BUF[0],RC_FRAME_LENGTH);
          FeedDog(DEVICE_INDEX_RC);
			}         
		}
		//当前在Memory1
		else 
		{
			DMA_Cmd(DMA2_Stream2, DISABLE);
			this_time_rx_len = BSP_USART1_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA2_Stream2);
			DMA2_Stream2->NDTR = (uint16_t)BSP_USART1_DMA_RX_BUF_LEN;     //将数据接收缓冲区的指针复位
			DMA2_Stream2->CR &= ~(uint32_t)(DMA_SxCR_CT);                 //将数据接收缓冲区设为Memory0
			DMA_Cmd(DMA2_Stream2, ENABLE);

      if(this_time_rx_len == RC_FRAME_LENGTH)
			{
          RcProtocolAnalysis(_USART1_DMA_RX_BUF[1],RC_FRAME_LENGTH);
          FeedDog(DEVICE_INDEX_RC);
			}

		}
	}       
}
void Rc_Init(void)
{
	USART1_Configuration(100000);//100k
}










