#include "AS5600.h"

#define AS5600_CPR 4096
#define  AS5600_Address  0x36
#define  RAW_Angle_Hi    0x0C 



void As5600_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//GPIOB
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	

	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0;     //I2C_OAR1
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 100000;    //400KHz
	I2C_Init(I2C2, &I2C_InitStructure);
	I2C_Cmd(I2C2, ENABLE);

}


unsigned short I2C_getRawCount(I2C_TypeDef* I2Cx)
{
	uint32_t Timeout;
	unsigned short temp;
	unsigned char dh,dl;	
	
	Timeout = 0xFFFF;
	/* Send START condition */
	I2Cx->CR1 |= 0x0100;//CR1_START_Set;
	/* Wait until SB flag is set: EV5 */
	while ((I2Cx->SR1&0x0001) != 0x0001)
	{
		if (Timeout-- == 0)return 0;
	}
	/* Send the slave address, Reset the address bit0 for write*/
	I2Cx->DR = AS5600_Address<<1;
	Timeout = 0xFFFF;
	/* Wait until ADDR is set: EV6 */
	while ((I2Cx->SR1 &0x0002) != 0x0002)
	{
		if (Timeout-- == 0)return 0;
	}
	/* Clear ADDR flag by reading SR2 register */
	temp = I2Cx->SR2;
	/* Write the first data in DR register (EV8_1) */
	I2Cx->DR = RAW_Angle_Hi;
	/* EV8_2: Wait until BTF is set before programming the STOP */
	while ((I2Cx->SR1 & 0x00004) != 0x000004);
	
	/////////////////////////////////////////////////////////////////////////
	/* Set POS bit */
	I2Cx->CR1 |= 0x0800;//CR1_POS_Set;
	Timeout = 0xFFFF;
	/* Send START condition */
	I2Cx->CR1 |= 0x0100;//CR1_START_Set;
	/* Wait until SB flag is set: EV5 */
	while ((I2Cx->SR1&0x0001) != 0x0001)
	{
		if (Timeout-- == 0)return 0;
	}
	Timeout = 0xFFFF;
	/* Send slave address */
	I2Cx->DR = (AS5600_Address<<1)+1;

	/* Wait until ADDR is set: EV6 */
	while ((I2Cx->SR1&0x0002) != 0x0002)
	{
		if (Timeout-- == 0)return 0;
	}
	/* EV6_1: The acknowledge disable should be done just after EV6,
	that is after ADDR is cleared, so disable all active IRQs around ADDR clearing and 
	ACK clearing */
	__disable_irq();
	/* Clear ADDR by reading SR2 register  */
	temp = I2Cx->SR2;
	/* Clear ACK */
	I2Cx->CR1 &= 0xFBFF;//CR1_ACK_Reset;
	/*Re-enable IRQs */
	__enable_irq();
	/* Wait until BTF is set */
	while ((I2Cx->SR1 & 0x00004) != 0x000004);
	/* Disable IRQs around STOP programming and data reading because of the limitation ?*/
	__disable_irq();
	/* Program the STOP */
	I2C_GenerateSTOP(I2Cx, ENABLE);
	/* Read first data */
	dh = I2Cx->DR;
	/* Re-enable IRQs */
	__enable_irq();
	/**/
	/* Read second data */
	dl = I2Cx->DR;
	/* Make sure that the STOP bit is cleared by Hardware before CR1 write access */
	while ((I2Cx->CR1&0x200) == 0x200);
	/* Enable Acknowledgement to be ready for another reception */
	I2Cx->CR1  |= 0x0400;//CR1_ACK_Set;
	/* Clear POS bit */
	I2Cx->CR1  &= 0xF7FF;//CR1_POS_Reset;
	
	temp++;  //useless,otherwise warning
	return ((dh<<8)+dl);
}
ENCODER_TypeDef E1;

float Get_Angel_L(ENCODER_TypeDef *CODERx)
{
    float angle_data=0,d_angle=0;
	
    CODERx->cpr=AS5600_CPR;
	angle_data =  I2C_getRawCount(I2C2);
	

	d_angle = angle_data - CODERx->angle_data_prev;

 	if(fabs(d_angle) > (0.8*CODERx->cpr) ) CODERx->full_rotation_offset += d_angle > 0 ? -_2PI : _2PI; 

	CODERx->angle_data_prev = angle_data;

	return  (CODERx->full_rotation_offset + ( angle_data / (float)CODERx->cpr) * _2PI) ;
}

float Get_Angel()
{
	return Get_Angel_L(&E1);
}


ENCODER_TypeDef E2;
float Get_Velocity_L(ENCODER_TypeDef* CODERx)
{
	unsigned long now_us;
	float Ts, angle_c, vel;

	// calculate sample time
	now_us = SysTick->VAL; //_micros();
	if(now_us<CODERx->velocity_calc_timestamp)Ts = (float)(CODERx->velocity_calc_timestamp - now_us)/9*1e-6;
	else
		Ts = (float)(0xFFFFFF - now_us + CODERx->velocity_calc_timestamp)/9*1e-6;
	// quick fix for strange cases (micros overflow)
	if(Ts == 0 || Ts > 0.5) Ts = 1e-3;

	// current angle
	angle_c = Get_Angel(CODERx);
	// velocity calculation
	vel = (angle_c - CODERx->angle_prev)/Ts;

	// save variables for future pass
	CODERx->angle_prev = angle_c;
	CODERx->velocity_calc_timestamp = now_us;
	
	return vel;
}

float Get_Velocity2()
{
	return Get_Velocity_L(&E2);
}

float Get_Velocity()
{
	float vel_ori=Get_Velocity_L(&E2);
  	float vel_flit=LowPassFilter_operator(&LPF_velocity,vel_ori);
  	return vel_flit;   
}

float Get_Angel_Notrack()
{

    return  I2C_getRawCount(I2C2)*0.08789*PI/180;
    
}




void As5600_WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT)
{
	uint32_t Timeout;
	Timeout = 10000;
	while (I2C_CheckEvent(I2Cx, I2C_EVENT) != SUCCESS)
	{
		Timeout --;
		if (Timeout == 0)
		{
			break;
		}
	}
}

void As5600_Write_One(uint8_t address,uint8_t data)
{
	I2C_GenerateSTART(I2C2, ENABLE);
	As5600_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);
	
	I2C_SendData(I2C2,AS5600_WRITE_ADDRESS);
	As5600_WaitEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
	
	I2C_SendData(I2C2, address);
	As5600_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTING);
	
	I2C_SendData(I2C2, data);
	As5600_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
	
	I2C_GenerateSTOP(I2C2, ENABLE);
}

uint8_t As5600_Read_One(uint8_t address)
{
    uint8_t data;
    I2C_GenerateSTART(I2C2,ENABLE);
    As5600_WaitEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT);

    I2C_SendData(I2C2,AS5600_WRITE_ADDRESS);
    As5600_WaitEvent(I2C2,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);

    I2C_SendData(I2C2,address);
    As5600_WaitEvent(I2C2,I2C_EVENT_MASTER_BYTE_RECEIVED);

    I2C_GenerateSTART(I2C2,ENABLE);
    As5600_WaitEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT);

    I2C_SendData(I2C2,AS5600_READ_ADDRESS);
	As5600_WaitEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);

	I2C_AcknowledgeConfig(I2C2, DISABLE);
	I2C_GenerateSTOP(I2C2, ENABLE);
	
	As5600_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);
	data = I2C_ReceiveData(I2C2);
	
	I2C_AcknowledgeConfig(I2C2, ENABLE);
    
    return data;
}

double As5600_Read_Two()
{
    uint8_t high, low;
    uint16_t readValue = 0;
    high = As5600_Read_One(AS5600_RAW_ANGLE_REGISTER_HIGH);
    low = As5600_Read_One(AS5600_RAW_ANGLE_REGISTER_LOW);
    return high<<8|low; //原始数据
    // int _bit_resolution=12;
    // int _bits_used_msb=11-7;
    // float cpr = pow(2, _bit_resolution);
    // int lsb_used = _bit_resolution - _bits_used_msb;

    // uint8_t lsb_mask = (uint8_t)( (2 << lsb_used) - 1 );
    // uint8_t msb_mask = (uint8_t)( (2 << _bits_used_msb) - 1 );
    
    // readValue = ( low &  lsb_mask );
    // readValue += ( ( high & msb_mask ) << lsb_used );
    // return (readValue/ (float)cpr) * _2PI; 


}

