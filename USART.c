#include "stm32f10x.h"                  // Device header

/**
 ****************************************************************************************************
 * @file        USART.c
 * @author      科技实践协会(JJH)
 * @version     V1.0
 * @date        2024-11-12
 * @brief       串口驱动代码
 * @license     无
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:STM32F103C8T6
 * 修改说明
 * V1.0 20241112
 * 第一次发布
 *
 ****************************************************************************************************
 */

extern uint8_t Serial1_RxData;		//定义串口1接收的数据变量
extern uint8_t Serial1_RxFlag;		//定义串口1接收的标志位变量

extern uint8_t Serial2_RxData;		//定义串口2接收的数据变量
extern uint8_t Serial2_RxFlag;		//定义串口2接收的标志位变量

extern uint8_t Serial3_RxData;		//定义串口3接收的数据变量
extern uint8_t Serial3_RxFlag;		//定义串口3接收的标志位变量




/*****************************************************************************
 * @name       :void USART_Init_Config(USART_TypeDef *USARTx, uint32_t baudrate, uint16_t stop_bits, uint16_t parity, FunctionalState remap)
 * @date       :2024-11-13 
 * @function   :该函数用于初始化并配置STM32微控制器上的USART外设。 
 *              它处理时钟设置、GPIO引脚配置（包括可选的引脚重映射）、 
 *              USART的波特率、数据位、停止位、校验位配置，并启用接收中断。
 *              支持USART1、USART2和USART3。
 * @parameters :
 *      		- USARTx: 指向要初始化的USART外设的指针。可以是以下值之一：USART1、USART2 或 USART3。
 *      		- baudrate: USART通信的波特率（例如，9600、115200等）。
 *      		- stop_bits: 数据传输的停止位数。可以是以下值之一：
 *              	-> USART_StopBits_1：1个停止位
 *              	-> USART_StopBits_2：2个停止位
 *      		- parity: 校验位配置。可以是以下值之一：
 *                  -> USART_Parity_No：无校验位
 *                  -> USART_Parity_Even：偶校验
 *                  -> USART_Parity_Odd：奇校验
 *      		- remap: 用于启用或禁用引脚重映射的标志。 
 *              	-> ENABLE：启用引脚重映射（仅适用于USART1）。
 *              	-> DISABLE：禁用引脚重映射（默认）。          
 * @retvalue   :无返回值。
 * @note       :该函数会启用接收中断（USART_IT_RXNE），并配置每个USART的NVIC中断优先级（USART1、USART2、USART3）。 
 *				但调用一次这个函数只能配置一个函数的中断优先级
 *              引脚重映射仅适用于USART1。当启用引脚重映射时，串口1默认的TX和RX引脚会改变              
 * @example    :USART_Init_Config(USART1, 115200, USART_StopBits_1, USART_Parity_No, ENABLE);
 *      		这会初始化USART1，波特率为115200，1个停止位，无校验位，并启用引脚重映射。
******************************************************************************/
void USART_Init_Config(USART_TypeDef *USARTx, uint32_t baudrate, uint16_t stop_bits, uint16_t parity, FunctionalState remap) 
{
    //声明GPIO和USART初始化结构体
    GPIO_InitTypeDef GPIO_InitStructure;  
    USART_InitTypeDef USART_InitStructure;

    //如果需要进行引脚重映射
    if(remap==ENABLE)
    {       
        // 使能AFIO（替代功能I/O）时钟，用于引脚重映射
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);    
    }

    // 如果要配置USART1的时钟和引脚
    if(USARTx==USART1)
    {    
        // 使能USART1时钟
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);        
        // 如果启用了引脚重映射
        if (remap == ENABLE) 
        {
            // 使能GPIOB时钟，用于配置重映射引脚
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
        }        
        // 使能GPIOA时钟，用于配置USART1的引脚
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    } 
    // 如果要配置USART2的时钟和引脚
    else if (USARTx == USART2) 
    {
        // 使能USART2时钟
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
        // 使能GPIOA时钟，用于配置USART2的引脚
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    }
    // 如果要配置USART3的时钟和引脚
    else if (USARTx == USART3) 
    {
        // 使能USART3时钟
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
        // 使能GPIOB时钟，用于配置USART3的引脚
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    }

    // 如果要配置USART1的引脚
    if (USARTx == USART1) 
    {
        // 如果启用了引脚重映射
        if (remap == ENABLE) 
        {
            // 重映射USART1的引脚
            GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);
            
            // 配置PB6为USART1的TX引脚（复用推挽输出模式）
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_Init(GPIOB, &GPIO_InitStructure);                

            // 配置PB7为USART1的RX引脚（上拉输入模式）
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_Init(GPIOB, &GPIO_InitStructure);                    
        } 
        // 如果没有启用引脚重映射
        else if(remap == DISABLE)
        {
            // 配置PA9为USART1的TX引脚（复用推挽输出模式）
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;  
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
            GPIO_Init(GPIOA, &GPIO_InitStructure);  

            // 配置PA10为USART1的RX引脚（上拉输入模式）
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;  
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_Init(GPIOA, &GPIO_InitStructure);  
        }
    } 
    // 如果要配置USART2的引脚
    else if (USARTx == USART2) 
    {
        // 配置PA2为USART2的TX引脚（复用推挽输出模式）
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;  
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
        GPIO_Init(GPIOA, &GPIO_InitStructure);  

        // 配置PA3为USART2的RX引脚（上拉输入模式）
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;  
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA, &GPIO_InitStructure);  
    }
    // 如果要配置USART3的引脚
    else if (USARTx == USART3) 
    {
        // 配置PB10为USART3的TX引脚（复用推挽输出模式）
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;  
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
        GPIO_Init(GPIOB, &GPIO_InitStructure);  

        // 配置PB11为USART3的RX引脚（上拉输入模式）
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;  
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOB, &GPIO_InitStructure);  
    }

    // 如果要配置USART的参数：波特率、数据位、停止位、校验位等
    USART_InitStructure.USART_BaudRate 				= baudrate;                         //设置波特率
    USART_InitStructure.USART_WordLength 			= USART_WordLength_8b;  			//设置数据位字节长度为8bit
    USART_InitStructure.USART_StopBits 				= stop_bits;  						//设置停止位
    USART_InitStructure.USART_Parity 				= parity;  							//设置校验位
    USART_InitStructure.USART_HardwareFlowControl 	= USART_HardwareFlowControl_None;   //关闭硬件流控制
    USART_InitStructure.USART_Mode 					= USART_Mode_Tx | USART_Mode_Rx;  	//将串口配置为收发模式

	// 如果要配置USART1的中断
	if(USARTx == USART1)
	{
		/*中断输出配置*/
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);			//开启串口接收数据的中断
		
		/*NVIC中断分组*/
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);			//配置NVIC为分组2
		
		/*NVIC配置*/
		NVIC_InitTypeDef NVIC_InitStructure;					//定义结构体变量
		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		//选择配置NVIC的USART1线
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//指定NVIC线路使能
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//指定NVIC线路的抢占优先级为0
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//指定NVIC线路的响应优先级为1
		NVIC_Init(&NVIC_InitStructure);							//将结构体变量交给NVIC_Init，配置NVIC外设

	}
	// 如果要配置USART2的中断
	if(USARTx == USART2)
	{
		/*中断输出配置*/
		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);			//开启串口接收数据的中断
		
		/*NVIC中断分组*/
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);			//配置NVIC为分组2
		
		/*NVIC配置*/
		NVIC_InitTypeDef NVIC_InitStructure;					//定义结构体变量
		NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;		//选择配置NVIC的USART2线
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//指定NVIC线路使能
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//指定NVIC线路的抢占优先级为0
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//指定NVIC线路的响应优先级为1
		NVIC_Init(&NVIC_InitStructure);							//将结构体变量交给NVIC_Init，配置NVIC外设

	}
	// 如果要配置USART3的中断
	if(USARTx == USART3)
	{
		/*中断输出配置*/
		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);			//开启串口接收数据的中断
		
		/*NVIC中断分组*/
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);			//配置NVIC为分组2
		
		/*NVIC配置*/
		NVIC_InitTypeDef NVIC_InitStructure;					//定义结构体变量
		NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;		//选择配置NVIC的USART3线
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//指定NVIC线路使能
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//指定NVIC线路的抢占优先级为1
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//指定NVIC线路的响应优先级为1
		NVIC_Init(&NVIC_InitStructure);							//将结构体变量交给NVIC_Init，配置NVIC外设

	}	
	
    // 初始化USARTx
    USART_Init(USARTx, &USART_InitStructure);  

    // 使能USARTx
    USART_Cmd(USARTx, ENABLE);  
}

/**
  * 函    数：串口1发送一个字节
  * 参    数：Byte 要发送的一个字节
  * 返 回 值：无
  */
void Serial1_SendByte(uint8_t Byte)
{
	USART_SendData(USART1, Byte);		//将字节数据写入数据寄存器，写入后USART自动生成时序波形
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);	//等待发送完成
	/*下次写入数据寄存器会自动清除发送完成标志位，故此循环后，无需清除标志位*/
}

/**
  * 函    数：串口1发送一个数组
  * 参    数：Array 要发送数组的首地址
  * 参    数：Length 要发送数组的长度
  * 返 回 值：无
  */
void Serial1_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)		//遍历数组
	{
		Serial1_SendByte(Array[i]);		//依次调用Serial_SendByte发送每个字节数据
	}
}

/**
  * 函    数：串口1发送一个字符串
  * 参    数：String 要发送字符串的首地址
  * 返 回 值：无
  */
void Serial1_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)//遍历字符数组（字符串），遇到字符串结束标志位后停止
	{
		Serial1_SendByte(String[i]);		//依次调用Serial_SendByte发送每个字节数据
	}
}

/**
  * 函    数：串口2发送一个字节
  * 参    数：Byte 要发送的一个字节
  * 返 回 值：无
  */
void Serial2_SendByte(uint8_t Byte)
{
    USART_SendData(USART2, Byte);          // 将字节数据写入数据寄存器，写入后USART自动生成时序波形
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);  // 等待发送完成
    /*下次写入数据寄存器会自动清除发送完成标志位，故此循环后，无需清除标志位*/
}

/**
  * 函    数：串口2发送一个数组
  * 参    数：Array 要发送数组的首地址
  * 参    数：Length 要发送数组的长度
  * 返 回 值：无
  */
void Serial2_SendArray(uint8_t *Array, uint16_t Length)
{
    uint16_t i;
    for (i = 0; i < Length; i++)          // 遍历数组
    {
        Serial2_SendByte(Array[i]);        // 依次调用Serial2_SendByte发送每个字节数据
    }
}

/**
  * 函    数：串口2发送一个字符串
  * 参    数：String 要发送字符串的首地址
  * 返 回 值：无
  */
void Serial2_SendString(char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i++)   // 遍历字符数组（字符串），遇到字符串结束标志位后停止
    {
        Serial2_SendByte(String[i]);       // 依次调用Serial2_SendByte发送每个字节数据
    }
}

/**
  * 函    数：串口3发送一个字节
  * 参    数：Byte 要发送的一个字节
  * 返 回 值：无
  */
void Serial3_SendByte(uint8_t Byte)
{
    USART_SendData(USART3, Byte);          // 将字节数据写入数据寄存器，写入后USART自动生成时序波形
    while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);  // 等待发送完成
    /*下次写入数据寄存器会自动清除发送完成标志位，故此循环后，无需清除标志位*/
}

/**
  * 函    数：串口3发送一个数组
  * 参    数：Array 要发送数组的首地址
  * 参    数：Length 要发送数组的长度
  * 返 回 值：无
  */
void Serial3_SendArray(uint8_t *Array, uint16_t Length)
{
    uint16_t i;
    for (i = 0; i < Length; i++)          // 遍历数组
    {
        Serial3_SendByte(Array[i]);        // 依次调用Serial3_SendByte发送每个字节数据
    }
}

/**
  * 函    数：串口3发送一个字符串
  * 参    数：String 要发送字符串的首地址
  * 返 回 值：无
  */
void Serial3_SendString(char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i++)   // 遍历字符数组（字符串），遇到字符串结束标志位后停止
    {
        Serial3_SendByte(String[i]);       // 依次调用Serial3_SendByte发送每个字节数据
    }
}

void USART1_IRQHandler(void)
{
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)		//判断是否是USART1的接收事件触发的中断
	{
		Serial1_RxData = USART_ReceiveData(USART1);				//读取数据寄存器，存放在接收的数据变量
		Serial1_RxFlag = 1;										//置接收标志位变量为1
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);			//清除USART1的RXNE标志位
																//读取数据寄存器会自动清除此标志位
																//如果已经读取了数据寄存器，也可以不执行此代码
	}
}

void USART2_IRQHandler(void)
{
	if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)		//判断是否是USART2的接收事件触发的中断
	{
		Serial2_RxData = USART_ReceiveData(USART2);				//读取数据寄存器，存放在接收的数据变量
		Serial2_RxFlag = 1;										//置接收标志位变量为1
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);			//清除USART1的RXNE标志位
																//读取数据寄存器会自动清除此标志位
																//如果已经读取了数据寄存器，也可以不执行此代码
	}
}

void USART3_IRQHandler(void)
{
	if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET)		//判断是否是USART3的接收事件触发的中断
	{
		Serial3_RxData = USART_ReceiveData(USART3);				//读取数据寄存器，存放在接收的数据变量
		Serial3_RxFlag = 1;										//置接收标志位变量为1
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);			//清除USART1的RXNE标志位
																//读取数据寄存器会自动清除此标志位
																//如果已经读取了数据寄存器，也可以不执行此代码
	}
}	
	
