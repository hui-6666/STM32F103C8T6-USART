#include "stm32f10x.h"                  // Device header

/**
 ****************************************************************************************************
 * @file        USART.c
 * @author      �Ƽ�ʵ��Э��(JJH)
 * @version     V1.0
 * @date        2024-11-12
 * @brief       ������������
 * @license     ��
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:STM32F103C8T6
 * �޸�˵��
 * V1.0 20241112
 * ��һ�η���
 *
 ****************************************************************************************************
 */

extern uint8_t Serial1_RxData;		//���崮��1���յ����ݱ���
extern uint8_t Serial1_RxFlag;		//���崮��1���յı�־λ����

extern uint8_t Serial2_RxData;		//���崮��2���յ����ݱ���
extern uint8_t Serial2_RxFlag;		//���崮��2���յı�־λ����

extern uint8_t Serial3_RxData;		//���崮��3���յ����ݱ���
extern uint8_t Serial3_RxFlag;		//���崮��3���յı�־λ����




/*****************************************************************************
 * @name       :void USART_Init_Config(USART_TypeDef *USARTx, uint32_t baudrate, uint16_t stop_bits, uint16_t parity, FunctionalState remap)
 * @date       :2024-11-13 
 * @function   :�ú������ڳ�ʼ��������STM32΢�������ϵ�USART���衣 
 *              ������ʱ�����á�GPIO�������ã�������ѡ��������ӳ�䣩�� 
 *              USART�Ĳ����ʡ�����λ��ֹͣλ��У��λ���ã������ý����жϡ�
 *              ֧��USART1��USART2��USART3��
 * @parameters :
 *      		- USARTx: ָ��Ҫ��ʼ����USART�����ָ�롣����������ֵ֮һ��USART1��USART2 �� USART3��
 *      		- baudrate: USARTͨ�ŵĲ����ʣ����磬9600��115200�ȣ���
 *      		- stop_bits: ���ݴ����ֹͣλ��������������ֵ֮һ��
 *              	-> USART_StopBits_1��1��ֹͣλ
 *              	-> USART_StopBits_2��2��ֹͣλ
 *      		- parity: У��λ���á�����������ֵ֮һ��
 *                  -> USART_Parity_No����У��λ
 *                  -> USART_Parity_Even��żУ��
 *                  -> USART_Parity_Odd����У��
 *      		- remap: �������û����������ӳ��ı�־�� 
 *              	-> ENABLE������������ӳ�䣨��������USART1����
 *              	-> DISABLE������������ӳ�䣨Ĭ�ϣ���          
 * @retvalue   :�޷���ֵ��
 * @note       :�ú��������ý����жϣ�USART_IT_RXNE����������ÿ��USART��NVIC�ж����ȼ���USART1��USART2��USART3���� 
 *				������һ���������ֻ������һ���������ж����ȼ�
 *              ������ӳ���������USART1��������������ӳ��ʱ������1Ĭ�ϵ�TX��RX���Ż�ı�              
 * @example    :USART_Init_Config(USART1, 115200, USART_StopBits_1, USART_Parity_No, ENABLE);
 *      		����ʼ��USART1��������Ϊ115200��1��ֹͣλ����У��λ��������������ӳ�䡣
******************************************************************************/
void USART_Init_Config(USART_TypeDef *USARTx, uint32_t baudrate, uint16_t stop_bits, uint16_t parity, FunctionalState remap) 
{
    //����GPIO��USART��ʼ���ṹ��
    GPIO_InitTypeDef GPIO_InitStructure;  
    USART_InitTypeDef USART_InitStructure;

    //�����Ҫ����������ӳ��
    if(remap==ENABLE)
    {       
        // ʹ��AFIO���������I/O��ʱ�ӣ�����������ӳ��
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);    
    }

    // ���Ҫ����USART1��ʱ�Ӻ�����
    if(USARTx==USART1)
    {    
        // ʹ��USART1ʱ��
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);        
        // ���������������ӳ��
        if (remap == ENABLE) 
        {
            // ʹ��GPIOBʱ�ӣ�����������ӳ������
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
        }        
        // ʹ��GPIOAʱ�ӣ���������USART1������
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    } 
    // ���Ҫ����USART2��ʱ�Ӻ�����
    else if (USARTx == USART2) 
    {
        // ʹ��USART2ʱ��
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
        // ʹ��GPIOAʱ�ӣ���������USART2������
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    }
    // ���Ҫ����USART3��ʱ�Ӻ�����
    else if (USARTx == USART3) 
    {
        // ʹ��USART3ʱ��
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
        // ʹ��GPIOBʱ�ӣ���������USART3������
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    }

    // ���Ҫ����USART1������
    if (USARTx == USART1) 
    {
        // ���������������ӳ��
        if (remap == ENABLE) 
        {
            // ��ӳ��USART1������
            GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);
            
            // ����PB6ΪUSART1��TX���ţ������������ģʽ��
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_Init(GPIOB, &GPIO_InitStructure);                

            // ����PB7ΪUSART1��RX���ţ���������ģʽ��
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_Init(GPIOB, &GPIO_InitStructure);                    
        } 
        // ���û������������ӳ��
        else if(remap == DISABLE)
        {
            // ����PA9ΪUSART1��TX���ţ������������ģʽ��
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;  
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
            GPIO_Init(GPIOA, &GPIO_InitStructure);  

            // ����PA10ΪUSART1��RX���ţ���������ģʽ��
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;  
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_Init(GPIOA, &GPIO_InitStructure);  
        }
    } 
    // ���Ҫ����USART2������
    else if (USARTx == USART2) 
    {
        // ����PA2ΪUSART2��TX���ţ������������ģʽ��
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;  
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
        GPIO_Init(GPIOA, &GPIO_InitStructure);  

        // ����PA3ΪUSART2��RX���ţ���������ģʽ��
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;  
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA, &GPIO_InitStructure);  
    }
    // ���Ҫ����USART3������
    else if (USARTx == USART3) 
    {
        // ����PB10ΪUSART3��TX���ţ������������ģʽ��
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;  
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
        GPIO_Init(GPIOB, &GPIO_InitStructure);  

        // ����PB11ΪUSART3��RX���ţ���������ģʽ��
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;  
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOB, &GPIO_InitStructure);  
    }

    // ���Ҫ����USART�Ĳ����������ʡ�����λ��ֹͣλ��У��λ��
    USART_InitStructure.USART_BaudRate 				= baudrate;                         //���ò�����
    USART_InitStructure.USART_WordLength 			= USART_WordLength_8b;  			//��������λ�ֽڳ���Ϊ8bit
    USART_InitStructure.USART_StopBits 				= stop_bits;  						//����ֹͣλ
    USART_InitStructure.USART_Parity 				= parity;  							//����У��λ
    USART_InitStructure.USART_HardwareFlowControl 	= USART_HardwareFlowControl_None;   //�ر�Ӳ��������
    USART_InitStructure.USART_Mode 					= USART_Mode_Tx | USART_Mode_Rx;  	//����������Ϊ�շ�ģʽ

	// ���Ҫ����USART1���ж�
	if(USARTx == USART1)
	{
		/*�ж��������*/
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);			//�������ڽ������ݵ��ж�
		
		/*NVIC�жϷ���*/
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);			//����NVICΪ����2
		
		/*NVIC����*/
		NVIC_InitTypeDef NVIC_InitStructure;					//����ṹ�����
		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		//ѡ������NVIC��USART1��
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//ָ��NVIC��·ʹ��
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//ָ��NVIC��·����ռ���ȼ�Ϊ0
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//ָ��NVIC��·����Ӧ���ȼ�Ϊ1
		NVIC_Init(&NVIC_InitStructure);							//���ṹ���������NVIC_Init������NVIC����

	}
	// ���Ҫ����USART2���ж�
	if(USARTx == USART2)
	{
		/*�ж��������*/
		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);			//�������ڽ������ݵ��ж�
		
		/*NVIC�жϷ���*/
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);			//����NVICΪ����2
		
		/*NVIC����*/
		NVIC_InitTypeDef NVIC_InitStructure;					//����ṹ�����
		NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;		//ѡ������NVIC��USART2��
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//ָ��NVIC��·ʹ��
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//ָ��NVIC��·����ռ���ȼ�Ϊ0
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//ָ��NVIC��·����Ӧ���ȼ�Ϊ1
		NVIC_Init(&NVIC_InitStructure);							//���ṹ���������NVIC_Init������NVIC����

	}
	// ���Ҫ����USART3���ж�
	if(USARTx == USART3)
	{
		/*�ж��������*/
		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);			//�������ڽ������ݵ��ж�
		
		/*NVIC�жϷ���*/
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);			//����NVICΪ����2
		
		/*NVIC����*/
		NVIC_InitTypeDef NVIC_InitStructure;					//����ṹ�����
		NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;		//ѡ������NVIC��USART3��
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//ָ��NVIC��·ʹ��
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//ָ��NVIC��·����ռ���ȼ�Ϊ1
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//ָ��NVIC��·����Ӧ���ȼ�Ϊ1
		NVIC_Init(&NVIC_InitStructure);							//���ṹ���������NVIC_Init������NVIC����

	}	
	
    // ��ʼ��USARTx
    USART_Init(USARTx, &USART_InitStructure);  

    // ʹ��USARTx
    USART_Cmd(USARTx, ENABLE);  
}

/**
  * ��    ��������1����һ���ֽ�
  * ��    ����Byte Ҫ���͵�һ���ֽ�
  * �� �� ֵ����
  */
void Serial1_SendByte(uint8_t Byte)
{
	USART_SendData(USART1, Byte);		//���ֽ�����д�����ݼĴ�����д���USART�Զ�����ʱ����
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);	//�ȴ��������
	/*�´�д�����ݼĴ������Զ����������ɱ�־λ���ʴ�ѭ�������������־λ*/
}

/**
  * ��    ��������1����һ������
  * ��    ����Array Ҫ����������׵�ַ
  * ��    ����Length Ҫ��������ĳ���
  * �� �� ֵ����
  */
void Serial1_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)		//��������
	{
		Serial1_SendByte(Array[i]);		//���ε���Serial_SendByte����ÿ���ֽ�����
	}
}

/**
  * ��    ��������1����һ���ַ���
  * ��    ����String Ҫ�����ַ������׵�ַ
  * �� �� ֵ����
  */
void Serial1_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)//�����ַ����飨�ַ������������ַ���������־λ��ֹͣ
	{
		Serial1_SendByte(String[i]);		//���ε���Serial_SendByte����ÿ���ֽ�����
	}
}

/**
  * ��    ��������2����һ���ֽ�
  * ��    ����Byte Ҫ���͵�һ���ֽ�
  * �� �� ֵ����
  */
void Serial2_SendByte(uint8_t Byte)
{
    USART_SendData(USART2, Byte);          // ���ֽ�����д�����ݼĴ�����д���USART�Զ�����ʱ����
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);  // �ȴ��������
    /*�´�д�����ݼĴ������Զ����������ɱ�־λ���ʴ�ѭ�������������־λ*/
}

/**
  * ��    ��������2����һ������
  * ��    ����Array Ҫ����������׵�ַ
  * ��    ����Length Ҫ��������ĳ���
  * �� �� ֵ����
  */
void Serial2_SendArray(uint8_t *Array, uint16_t Length)
{
    uint16_t i;
    for (i = 0; i < Length; i++)          // ��������
    {
        Serial2_SendByte(Array[i]);        // ���ε���Serial2_SendByte����ÿ���ֽ�����
    }
}

/**
  * ��    ��������2����һ���ַ���
  * ��    ����String Ҫ�����ַ������׵�ַ
  * �� �� ֵ����
  */
void Serial2_SendString(char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i++)   // �����ַ����飨�ַ������������ַ���������־λ��ֹͣ
    {
        Serial2_SendByte(String[i]);       // ���ε���Serial2_SendByte����ÿ���ֽ�����
    }
}

/**
  * ��    ��������3����һ���ֽ�
  * ��    ����Byte Ҫ���͵�һ���ֽ�
  * �� �� ֵ����
  */
void Serial3_SendByte(uint8_t Byte)
{
    USART_SendData(USART3, Byte);          // ���ֽ�����д�����ݼĴ�����д���USART�Զ�����ʱ����
    while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);  // �ȴ��������
    /*�´�д�����ݼĴ������Զ����������ɱ�־λ���ʴ�ѭ�������������־λ*/
}

/**
  * ��    ��������3����һ������
  * ��    ����Array Ҫ����������׵�ַ
  * ��    ����Length Ҫ��������ĳ���
  * �� �� ֵ����
  */
void Serial3_SendArray(uint8_t *Array, uint16_t Length)
{
    uint16_t i;
    for (i = 0; i < Length; i++)          // ��������
    {
        Serial3_SendByte(Array[i]);        // ���ε���Serial3_SendByte����ÿ���ֽ�����
    }
}

/**
  * ��    ��������3����һ���ַ���
  * ��    ����String Ҫ�����ַ������׵�ַ
  * �� �� ֵ����
  */
void Serial3_SendString(char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i++)   // �����ַ����飨�ַ������������ַ���������־λ��ֹͣ
    {
        Serial3_SendByte(String[i]);       // ���ε���Serial3_SendByte����ÿ���ֽ�����
    }
}

void USART1_IRQHandler(void)
{
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)		//�ж��Ƿ���USART1�Ľ����¼��������ж�
	{
		Serial1_RxData = USART_ReceiveData(USART1);				//��ȡ���ݼĴ���������ڽ��յ����ݱ���
		Serial1_RxFlag = 1;										//�ý��ձ�־λ����Ϊ1
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);			//���USART1��RXNE��־λ
																//��ȡ���ݼĴ������Զ�����˱�־λ
																//����Ѿ���ȡ�����ݼĴ�����Ҳ���Բ�ִ�д˴���
	}
}

void USART2_IRQHandler(void)
{
	if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)		//�ж��Ƿ���USART2�Ľ����¼��������ж�
	{
		Serial2_RxData = USART_ReceiveData(USART2);				//��ȡ���ݼĴ���������ڽ��յ����ݱ���
		Serial2_RxFlag = 1;										//�ý��ձ�־λ����Ϊ1
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);			//���USART1��RXNE��־λ
																//��ȡ���ݼĴ������Զ�����˱�־λ
																//����Ѿ���ȡ�����ݼĴ�����Ҳ���Բ�ִ�д˴���
	}
}

void USART3_IRQHandler(void)
{
	if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET)		//�ж��Ƿ���USART3�Ľ����¼��������ж�
	{
		Serial3_RxData = USART_ReceiveData(USART3);				//��ȡ���ݼĴ���������ڽ��յ����ݱ���
		Serial3_RxFlag = 1;										//�ý��ձ�־λ����Ϊ1
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);			//���USART1��RXNE��־λ
																//��ȡ���ݼĴ������Զ�����˱�־λ
																//����Ѿ���ȡ�����ݼĴ�����Ҳ���Բ�ִ�д˴���
	}
}	
	
