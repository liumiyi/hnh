#include "mpuiic.h"
#include "main.h"

 //MPU IIC 延时函数
static void MPU_IIC_Delay(void)
{
	//粗略延时吧
	volatile uint8_t CNT=168;
	for(int i =0;i<CNT*10;i++)
	{
	  __NOP(); //168Mh主频，1s执行168M条指令,1us执行168条指令  延时2us，执行2*CNT条空指令
	}
	
}

/**
  * 函    数：I2C写SCL引脚电平
  * 参    数：BitValue 协议层传入的当前需要写入SCL的电平，范围0~1
  * 返 回 值：无
  * 注意事项：此函数需要用户实现内容，当BitValue为0时，需要置SCL为低电平，当BitValue为1时，需要置SCL为高电平
  */
void I2C_W_SCL(uint8_t BitValue)
{
	HAL_GPIO_WritePin(mpu_i2c_scl_port, mpu_i2c_scl_pin,BitValue);		//根据BitValue，设置SCL引脚的电平
	MPU_IIC_Delay();												//延时10us，防止时序频率超过要求
}

/**
  * 函    数：I2C写SDA引脚电平
  * 参    数：BitValue 协议层传入的当前需要写入SDA的电平，范围0~0xFF
  * 返 回 值：无
  * 注意事项：此函数需要用户实现内容，当BitValue为0时，需要置SDA为低电平，当BitValue非0时，需要置SDA为高电平
  */
void I2C_W_SDA(uint8_t BitValue)
{
	HAL_GPIO_WritePin(mpu_i2c_sda_port, mpu_i2c_sda_pin,BitValue);		//根据BitValue，设置SCL引脚的电平
	MPU_IIC_Delay();														//延时10us，防止时序频率超过要求
}

/**
  * 函    数：I2C读SDA引脚电平
  * 参    数：无
  * 返 回 值：协议层需要得到的当前SDA的电平，范围0~1
  * 注意事项：此函数需要用户实现内容，当前SDA为低电平时，返回0，当前SDA为高电平时，返回1
  */
uint8_t I2C_R_SDA(void)
{
	uint8_t BitValue;
	BitValue = HAL_GPIO_ReadPin(mpu_i2c_sda_port, mpu_i2c_sda_pin);		//读取SDA电平
	MPU_IIC_Delay();											 //延时10us，防止时序频率超过要求
	return BitValue;											//返回SDA电平
}

//初始化IIC(HAL库完成)
void MPU_IIC_Init(void)
{		  	
} 

/*----------------------------------------------协议层-------------------------------------------------------*/
//产生IIC起始信号
void MPU_IIC_Start(void)
{
	
	I2C_W_SDA(1);							//释放SDA，确保SDA为高电平
	I2C_W_SCL(1);							//释放SCL，确保SCL为高电平
	I2C_W_SDA(0);							//在SCL高电平期间，拉低SDA，产生起始信号
	I2C_W_SCL(0);							//起始后把SCL也拉低，即为了占用总线，也为了方便总线时序的拼接
	
}	  
//产生IIC停止信号
void MPU_IIC_Stop(void)
{
	I2C_W_SDA(0);							//拉低SDA，确保SDA为低电平(先拉低，后面才能产生上升沿信号)
	I2C_W_SCL(1);							//释放SCL，使SCL呈现高电平
	I2C_W_SDA(1);							//在SCL高电平期间，释放SDA，产生终止信号					   	
}

//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t MPU_IIC_Wait_Ack(void)
{
	uint8_t AckBit;							//定义应答位变量
	I2C_W_SDA(1);							//接收前，主机先确保释放SDA，避免干扰从机的数据发送
	I2C_W_SCL(1);							//释放SCL，主机机在SCL高电平期间读取SDA
	AckBit = I2C_R_SDA();					//将应答位存储到变量里
	I2C_W_SCL(0);							//拉低SCL，开始下一个时序模块
	return AckBit;							  //返回定义应答位变量
} 

//产生ACK应答
void MPU_IIC_Ack(void)
{
	I2C_W_SDA(0);					//主机把应答位数据放到SDA线
	I2C_W_SCL(1);							//释放SCL，从机在SCL高电平期间，读取应答位
	I2C_W_SCL(0);							//拉低SCL，开始下一个时序模块

}
//不产生ACK应答		    
void MPU_IIC_NAck(void)
{
	I2C_W_SDA(1);					     //主机把应答位数据放到SDA线
	I2C_W_SCL(1);							//释放SCL，从机在SCL高电平期间，读取应答位
	I2C_W_SCL(0);							//拉低SCL，开始下一个时序模块

}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void MPU_IIC_Send_Byte(uint8_t Byte)
{                        
	uint8_t i;
	for (i = 0; i < 8; i ++)				//循环8次，主机依次发送数据的每一位
	{
		I2C_W_SDA(Byte & (0x80 >> i));	//使用掩码的方式取出Byte的指定一位数据并写入到SDA线
		I2C_W_SCL(1);						//释放SCL，从机在SCL高电平期间读取SDA
		I2C_W_SCL(0);						//拉低SCL，主机开始发送下一位数据
	} 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
uint8_t MPU_IIC_Read_Byte(uint8_t ack)
{

	
	uint8_t i, Byte = 0x00;					//定义接收的数据，并赋初值0x00，此处必须赋初值0x00，后面会用到
	I2C_W_SDA(1);							//接收前，主机先确保释放SDA（相当于切换为输入模式），避免干扰从机的数据发送
	for (i = 0; i < 8; i ++)				//循环8次，主机依次接收数据的每一位
	{
		I2C_W_SCL(1);						//释放SCL，主机机在SCL高电平期间读取SDA
		if (I2C_R_SDA() == 1){Byte |= (0x80 >> i);}	//读取SDA数据，并存储到Byte变量
														//当SDA为1时，置变量指定位为1，当SDA为0时，不做处理，指定位为默认的初值0
		I2C_W_SCL(0);						//拉低SCL，从机在SCL低电平期间写入SDA
	}
	
    if (!ack)
        MPU_IIC_NAck();//发送nACK
    else
        MPU_IIC_Ack(); //发送ACK
		
	return Byte;							//返回接收到的一个字节数据	
}
