
#include "usart.h"


//本段代码参考了正点原子，微调
/******************************************************************************/
//加入以下代码，支持printf函数，而不需要选择MicroLIB
#if 1
#pragma import(__use_no_semihosting)
//标准库需要的支持函数
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;
//定义_sys_exit避免使用半主机模式
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数
int fputc(int ch, FILE *f)
{      
	while((USART2->SR&0X40)==0);
    USART2->DR = (u8) ch;      
	return ch;
}
#endif 
	
/******************************************************************************/
unsigned char USART_RX_BUF[USART_REC_LEN];     //接收缓冲，usart.h中定义长度
//接收状态
//bit15  接收完成标志
//bit14  接收到0x0D
//bit13~0  接收的字节数
unsigned short USART_RX_STA=0;       //接收状态标志
/******************************************************************************/
void uart_init(unsigned long bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    //|RCC_APB2Periph_AFIO
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//使能GPIOA时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能USART2时钟
 
    //PA2  TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    //复用推挽
    GPIO_Init(GPIOA, &GPIO_InitStructure);
 
    //PA3  RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);  
 
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2,ENABLE);//复位串口2
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2,DISABLE);//停止复位
 
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    //设置NVIC中断分组2:2位抢占优先级，2位响应优先级   0-3;
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; //使能串口2中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //先占优先级2级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; //从优先级2级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //使能外部中断通道
    NVIC_Init(&NVIC_InitStructure); //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
 
    USART_InitStructure.USART_BaudRate = bound;//波特率设置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8位数据长度
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;///奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//收发模式
 
    USART_Init(USART2, &USART_InitStructure); ; //初始化串口
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启中断接收
    USART_Cmd(USART2, ENABLE);                    //使能串口 

}
/******************************************************************************/
float temp;

uint8_t RxBuffer[1];//串口接收缓冲
uint16_t RxLine = 0;//指令长度
uint8_t DataBuff[200];//指令内容



float Get_Data(void)
{
    uint8_t data_Start_Num = 0; // 记录数据位开始的地方
    uint8_t data_End_Num = 0; // 记录数据位结束的地方
    uint8_t data_Num = 0; // 记录数据位数
    uint8_t minus_Flag = 0; // 判断是不是负数
    float data_return = 0; // 解析得到的数据
    for(uint8_t i=0;i<200;i++) // 查找等号和问号的位置
    {
        if(DataBuff[i] == '=') data_Start_Num = i + 1; // +1是直接定位到数据起始位
        if(DataBuff[i] == '?')
        {
            data_End_Num = i - 1;
            break;
        }
    }
    if(DataBuff[data_Start_Num] == '-') // 如果是负数
    {
        data_Start_Num += 1; // 后移一位到数据位
        minus_Flag = 1; // 负数flag
    }
    data_Num = data_End_Num - data_Start_Num + 1;
    if(data_Num == 4) // 数据共4位
    {
        data_return = (DataBuff[data_Start_Num]-48)  + (DataBuff[data_Start_Num+2]-48)*0.1f +
                (DataBuff[data_Start_Num+3]-48)*0.01f;
    }
    else if(data_Num == 5) // 数据共5位
    {
        data_return = (DataBuff[data_Start_Num]-48)*10 + (DataBuff[data_Start_Num+1]-48) + (DataBuff[data_Start_Num+3]-48)*0.1f +
                (DataBuff[data_Start_Num+4]-48)*0.01f;
    }
    else if(data_Num == 6) // 数据共6位
    {
        data_return = (DataBuff[data_Start_Num]-48)*100 + (DataBuff[data_Start_Num+1]-48)*10 + (DataBuff[data_Start_Num+2]-48) +
                (DataBuff[data_Start_Num+4]-48)*0.1f + (DataBuff[data_Start_Num+5]-48)*0.01f;
    }
    if(minus_Flag == 1)  data_return = -data_return;

    return data_return;
}
/*
 * 根据串口信息进行PID调参
 */

int mode;//模式选择
void PID_Adjust(uint8_t Motor_n)
{
 
    float data_Get = Get_Data(); // 存放接收到的数据
    if(Motor_n == 1)//左边电机
    {
        if(DataBuff[0]=='P' && DataBuff[1]=='1') // 位置环P
            PID_Pos.P = data_Get;
        else if(DataBuff[0]=='I' && DataBuff[1]=='1') // 位置环I
            PID_Pos.I = data_Get;
        else if(DataBuff[0]=='D' && DataBuff[1]=='1') // 位置环D
            PID_Pos.D = data_Get;
        else if(DataBuff[0]=='L' && DataBuff[1]=='1') // 目标位置
            position = data_Get;
        else if(DataBuff[0]=='P' && DataBuff[1]=='2') // 速度环P
            PID_Vel.P = data_Get/10.0;
        else if(DataBuff[0]=='I' && DataBuff[1]=='2') // 速度环I
            PID_Vel.I = data_Get/10.0;
        else if(DataBuff[0]=='D' && DataBuff[1]=='2') // 速度环D
            PID_Vel.D = data_Get/10.0;
        else if(DataBuff[0]=='P' && DataBuff[1]=='3') // 电流环P
            PID_Cur.P = data_Get;
        else if(DataBuff[0]=='I' && DataBuff[1]=='3') // 电流环I
            PID_Cur.I = data_Get;
        else if(DataBuff[0]=='D' && DataBuff[1]=='3') // 电流环D
            PID_Cur.D = data_Get;            
        else if(DataBuff[0]=='L' && DataBuff[1]=='2') //目标速度
            velocity = data_Get;
        else if(DataBuff[0]=='M' && DataBuff[1]=='2') //位置限制
            PID_Pos.limit = data_Get;
        else if(DataBuff[0]=='C' && DataBuff[1]=='2') //目标电流
             current = data_Get;
        else if(DataBuff[0]=='C' && DataBuff[1]=='1') //电流限制
             PID_Cur.limit = data_Get;     
        else if(DataBuff[0]=='S' && DataBuff[1]=='1') //速度限制
             PID_Vel.limit = data_Get;           
        else if(DataBuff[0]=='M' && DataBuff[1]=='1') //模式选择
             mode = data_Get;             
    }
}

void USART2_IRQHandler(void) 
{
    if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
	{
        RxBuffer[0] =USART_ReceiveData(USART2);
        RxLine++;                      //每接收到一个数据，进入回调数据长度加1
        DataBuff[RxLine-1]=RxBuffer[0];  //把每次接收到的数据保存到缓存数组
        if(RxBuffer[0]==0x3F)            //接收结束标志位，这个数据可以自定义，根据实际需求，这里只做示例使用，不一定是0x21
        {
            PID_Adjust(1);//数据解析和参数赋值函数
            memset(DataBuff,0,sizeof(DataBuff));  //清空缓存数组
            RxLine=0;  //清空接收长度
            
        }
        RxBuffer[0]=0;
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
	}

}

