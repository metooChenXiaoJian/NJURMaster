#include "main.h"



/*******����1����********************
   ��ң�������ջ�ͨ��
	 �������ֲ��֪����1�Ľ������ݼĴ���
	 ��DMA2����2ͨ��4�Ƕ�Ӧ��
	 �������˫�������
*************************************/

/*******����1����********************/
static uint8_t _USART1_DMA_RX_BUF[2][BSP_USART1_DMA_RX_BUF_LEN];//DMA���������������������Դ���ң��������֡
/************************************/

/**
  * @brief ����1��ʼ����DMA2��ʼ��
  * @param BaudRate
  * @retval None
  * @details	BaudRate	115200
	*						ʹ��DMA���գ����赽�ڴ棬ʹ�ܴ���IDLE�����ж�
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
    GPIO_Init(GPIOB, &gpio);//��ʼ��PB7
    
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
    dma.DMA_Channel = DMA_Channel_4;//����1��Ӧ��2ͨ��4
    dma.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);//�����ַΪ����1
    dma.DMA_Memory0BaseAddr = (uint32_t)(&_USART1_DMA_RX_BUF[0][0]);//DMA�������ڴ��ַ1
    dma.DMA_DIR = DMA_DIR_PeripheralToMemory;//���赽�ڴ�
    dma.DMA_BufferSize = sizeof(_USART1_DMA_RX_BUF)/2;//��������СΪ20
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//�����ַ����
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;//�ڴ��ַ����
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//�������ݵ�λΪ�ֽ�
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//���������ݵ�λΪ�ֽ�
    dma.DMA_Mode = DMA_Mode_Circular;//ѭ��ģʽ
    dma.DMA_Priority = DMA_Priority_Medium;//�е����ȼ�
    dma.DMA_FIFOMode = DMA_FIFOMode_Disable;//��ֹ����
    //dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    //dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    //dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream2, &dma);
    
    //����Memory1,Memory0�ǵ�һ��ʹ�õ�Memory
    DMA_DoubleBufferModeConfig(DMA2_Stream2, (uint32_t)&_USART1_DMA_RX_BUF[1][0], DMA_Memory_0);   //first used memory configuration
    DMA_DoubleBufferModeCmd(DMA2_Stream2, ENABLE);
    
    DMA_Cmd(DMA2_Stream2, ENABLE);
    
    nvic.NVIC_IRQChannel = USART1_IRQn;                          
		nvic.NVIC_IRQChannelPreemptionPriority = 0;   //��ռ���ȼ����
		nvic.NVIC_IRQChannelSubPriority = 0;		    //�����ȼ����
		nvic.NVIC_IRQChannelCmd = ENABLE;			
		NVIC_Init(&nvic);	
    
		USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);        //ʹ�ܴ���1�����ж�
    USART_Cmd(USART1, ENABLE);
}

//����1�����жϷ�����
void USART1_IRQHandler(void)
{
	static uint32_t this_time_rx_len = 0;

	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
	{
		//��������ж�
		(void)USART1->SR;
		(void)USART1->DR;

		//��ǰ��Memory0
		if(DMA_GetCurrentMemoryTarget(DMA2_Stream2) == 0)
		{
			DMA_Cmd(DMA2_Stream2, DISABLE);
			this_time_rx_len = BSP_USART1_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA2_Stream2);
			DMA2_Stream2->NDTR = (uint16_t)BSP_USART1_DMA_RX_BUF_LEN; //�����ݽ��ջ�������ָ�븴λ
			DMA2_Stream2->CR |= (uint32_t)(DMA_SxCR_CT);                  //�����ݽ��ջ�������ΪMemory1
			DMA_Cmd(DMA2_Stream2, ENABLE);
      if(this_time_rx_len == RC_FRAME_LENGTH)
			{
				  RcProtocolAnalysis(_USART1_DMA_RX_BUF[0],RC_FRAME_LENGTH);
          FeedDog(DEVICE_INDEX_RC);
			}         
		}
		//��ǰ��Memory1
		else 
		{
			DMA_Cmd(DMA2_Stream2, DISABLE);
			this_time_rx_len = BSP_USART1_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA2_Stream2);
			DMA2_Stream2->NDTR = (uint16_t)BSP_USART1_DMA_RX_BUF_LEN;     //�����ݽ��ջ�������ָ�븴λ
			DMA2_Stream2->CR &= ~(uint32_t)(DMA_SxCR_CT);                 //�����ݽ��ջ�������ΪMemory0
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










