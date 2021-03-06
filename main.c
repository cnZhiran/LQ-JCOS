#include <STC15F2K60S2.H>
#include <intrins.h>
#include <string.h>
#include <stdio.h>
#include <onewire.h>
#include <iic.h>
#include <ds1302.h>

#ifndef u8
#define u8 unsigned char
#endif

#ifndef u16
#define u16 unsigned int
#endif

#ifndef u32
#define u32 unsigned long
#endif

#define temp_mod 0
#define len_mod 1
#define vol_mod 2
#define bright_mod 3
#define eep_mod 4
#define freq_mod 5

#define len_read_IT		//len_read_once一次性读距离，len_read_IT发读分离法读距离

#define trans(x) ((x&0x7f)>>4)*10.0+(x&0x0f) //对DS1302的值进行进制转换的函数

sbit Trig = P1^0;
sbit Echo = P1^1;

u8 code font[10]={0xc0,0xf9,0xa4,0xb0,0x99,0x92,0x82,0xf8,0x80,0x90};
u8 code time_write_addr[8]={0x80,0x82,0x84,0x86,0x88,0x8a,0x8c,0x8e};
u8 code time_read_addr[8]={0x81,0x83,0x85,0x87,0x89,0x8b,0x8d,0x8f};
u8 code y4=0x80,y5=0xa0,y6=0xc0,y7=0xe0;

u8 bdata led=0,out=0;
bit smg_sign=0,temp_sign=0,len_flag=0,vol_flag=0,bright_flag=0,break_flag=0,echo_flag=0,tx_flag=0,rx_flag=0,time_sign=0;
u8 idata time[8]={0x00,0x00,0x12,0x01,0x01,0x03,0x20,0x00},dis[8],tx_buf[16]="init_well\r\n",rx_buf[16]="\0",freq_T=0,freq_H=0;
u8 idata key_flag=0,key_sign=0,tx_pot=0,rx_pot=0,cnt=0,write_flag=0,write_sign=0,read_flag=0,read_sign=0;
u16 idata L1_timing=0,L1_mode=800;
u16 idata key_count=0,temp_timing=25,vol_timing=13,len_timing=0,bright_timing=38,delay_timing=0,write_timing=50,freq_timing=31,count_timing=0,time_timing=18;
u16 mod_flag=len_mod,read_mod=len_mod,*write_addr,*read_addr,freq_sign=0;
u16 length=0,temp=-2000,len=20,vol=250,bright=250,vout=250;
u32 freq=10000;

sbit l1=led^0;
sbit l2=led^1;
sbit l3=led^2;
sbit l4=led^3;
sbit l5=led^4;
sbit l6=led^5;
sbit l7=led^6;
sbit l8=led^7;
sbit relay=out^4;
sbit buzz=out^6;

void mod_init();
void mod_ctrl();
void read_temp();
#ifdef len_read_once
void read_len();
#elif defined len_read_IT 
void send_len();
void read_len();
#endif
void read_vol();
void read_bright();
void write_vol();
void scankey();
void send_str();
void uart_reply();
void eep_write();
void eep_read();
void freq_read();
void time_init();
void time_read();

void dis_smg();
void dis_led();
void dis_out();

/*************************************************
*函数：mod_init()系统模式初始化函数
*功能：系统模式初始化		  
*************************************************/
void mod_init(){
	u8 i;
	
	switch(mod_flag){
	case temp_mod:
		dis[0]=0xc6;
		dis[1]=0xff;
		dis[2]=0xff;
		dis[3]=0xff;
		if(temp>=0x8000){
			dis[3]=font[-temp/10000%10];
			dis[4]=font[-temp/1000%10];
			dis[5]=font[-temp/100%10]&0x7f;
			dis[6]=font[-temp/10%10];
			dis[7]=font[-temp%10];
			for(i=3;dis[i]==font[0];i++) dis[i]=0xff;
			dis[i-1]=0xbf;
		}else{
			dis[3]=font[temp/10000%10];
			dis[4]=font[temp/1000%10];
			dis[5]=font[temp/100%10]&0x7f;
			dis[6]=font[temp/10%10];
			dis[7]=font[temp%10];
			for(i=3;dis[i]==font[0];i++) dis[i]=0xff;
		}
		if(write_flag == 0&&write_sign == 0) write_addr = &temp;
		return;
	case len_mod:
		dis[0]=0xc7;
		dis[1]=0xff;
		dis[2]=0xff;
		if(length==0){
			dis[3]=font[9];
			dis[4]=font[9];
			dis[5]=font[9];
			dis[6]=font[9]&0x7f;
			dis[7]=font[9];
		}else{
			dis[3]=font[len/10000];
			dis[4]=font[len/1000%10];
			dis[5]=font[len/100%10];
			dis[6]=font[len/10%10]&0x7f;
			dis[7]=font[len%10];
			for(i=3;dis[i]==font[0];i++) dis[i]=0xff;
		}
		if(write_flag == 0&&write_sign == 0) write_addr = &len;
		return;
	case vol_mod:
		dis[0]=0xc1;
		dis[1]=0xff;
		dis[2]=0xff;
		dis[3]=0xff;
		dis[4]=0xff;
		dis[5]=font[vol/100%10]&0x7f;
		dis[6]=font[vol/10%10];
		dis[7]=font[vol%10];
		if(write_flag == 0&&write_sign == 0) write_addr = &vol;
		return;
	case bright_mod:
		dis[0]=0x83;
		dis[1]=0xff;
		dis[2]=0xff;
		dis[3]=0xff;
		dis[4]=0xff;
		dis[5]=font[bright/100%10]&0x7f;
		dis[6]=font[bright/10%10];
		dis[7]=font[bright%10];
		if(write_flag == 0&&write_sign == 0) write_addr = &bright;
		return;
	case freq_mod:
		dis[0]=0x8e;
		dis[1]=font[freq/1000000%10];
		dis[2]=font[freq/100000%10];
		dis[3]=font[freq/10000%10];
		dis[4]=font[freq/1000%10];
		dis[5]=font[freq/100%10];
		dis[6]=font[freq/10%10];
		dis[7]=font[freq%10];
		for(i=1;dis[i]==font[0]&&i<8;i++) dis[i]=0xff;
		if(write_flag == 0&&write_sign == 0) write_addr = (u16 *)&freq;
		return;
	}
}
/*************************************************
*函数：Sysclk_init()系统计时初始化函数
*功能：系统计时初始化
*硬件：使用T2定时器，规定1ms溢出中断一次
*************************************************/
void Sysclk_init(){
	AUXR |= 0x04;		//定时器2时钟1T模式
	T2L = 0x20;			//设置定时初值
	T2H = 0xD1;			//设置定时初值
	IE2 |= 0x04;		//开定时器2中断
	EA = 1;
	AUXR |= 0x10;		//定时器2开始计时
}
/*************************************************
*函数：PCA_init()PCA定时器初始化函数
*功能：PCA定时器初始化			 
*硬件：使用PCA定时器，规定溢出和P11下降沿中断
*************************************************/
void PCA_init(){
	P_SW1 &= 0xcf;		//(P1.2/ECI, P1.1/CCP0, P1.0/CCP1, P3.7/CCP2)
	CCON = 0;                       //初始化PCA控制寄存器
                                  //PCA定时器停止
                                  //清除CF标志
                                  //清除模块中断标志
  CL = 0;                         //复位阵列寄存器
  CH = 0;
  CMOD = 0x01;                    //设置PCA时钟源,允许溢出中断
  CCAPM0 = 0x11;                  //PCA模块0为下降沿触发,开启中断。
  EA = 1;
}
/*************************************************
*函数：Uart_init()串口初始化函数
*功能：串口初始化 
*硬件：使用T1定时器，波特率为4800
*************************************************/
void Uart_init(void)		//4800bps@12.000MHz
{
	SCON = 0x50;		//8位数据,可变波特率
	AUXR |= 0x40;		//定时器1时钟为Fosc,即1T
	AUXR &= 0xFE;		//串口1选择定时器1为波特率发生器
	TMOD &= 0x0F;		//设定定时器1为16位自动重装方式
	TL1 = 0x8F;			//设定定时初值
	TH1 = 0xFD;			//设定定时初值
	ET1 = 0;			//禁止定时器1中断
	ES = 1;
	EA = 1;
	TR1 = 1;			//启动定时器1

	send_str();
}
/*************************************************
*函数：T0init()T0计数器初始化函数
*功能：计数器初始化
*************************************************/
void T0init(void)		//100微秒@12.000MHz
{
	AUXR &= 0x7F;		//定时器时钟12T模式
	TMOD |= 0x04;		//设置定时器模式
	TL0 = 0x00;		//设置定时初值
	TH0 = 0x00;		//设置定时初值
	TF0 = 0;		//清除TF0标志
	ET0 = 1;
	EA = 1;
	TR0 = 1;		//定时器0开始计时
}
/*************************************************
*函数：time_init()DS1302初始化函数
*功能：时间芯片初始化
*************************************************/
void time_init(){
	u8 i=8;
	while(i--) Write_Ds1302_Byte(time_write_addr[i],time[i]);
}
/*************************************************
*函数：delay_us()微秒级延时函数
*功能：微秒级延时服务
*备注：尽可能的使用STC-ISP的延时计算器，提高延时精度
*************************************************/
void delay100us()		//@12.000MHz
{
	unsigned char i, j;

	i = 2;
	j = 39;
	do
	{
		while (--j);
	} while (--i);
}
void delay12us()		//@12.000MHz
{
	unsigned char i;

	_nop_();
	_nop_();
	i = 33;
	while (--i);
}
/*************************************************
*函数：init()初始化函数
*功能：系统进入的初始化服务
*************************************************/
void init(){
	mod_init();
	Trig = 0;
	Echo = 1;
	PCA_init();
	Sysclk_init();
	T0init();
	Uart_init();
	time_init();
}
/*************************************************
*函数：loop()快速响应函数
*功能：快速I/O设备的驱动服务，模式变换服务
*备注：要求函数进行一次的时长要尽可能的短，这样不会影响其他函数的延时等待函数。
*************************************************/
void loop(){
	scankey();
	mod_ctrl();
	dis_led();
	dis_out();
	write_vol();
}
/*************************************************
*函数：soft_IT()中断捕获和处理函数
*功能：中断捕获和处理服务
*************************************************/
void soft_IT(){
	
	if(smg_sign)dis_smg();
	if(temp_sign) read_temp();
	if(vol_flag) read_vol();
	if(bright_flag) read_bright();
	#ifdef len_read_once
	if(len_flag) read_len();
	#elif defined len_read_IT
	if(len_flag) send_len();
	if(echo_flag||break_flag) read_len();
	#endif
	if(write_sign) eep_write();
	if(read_sign) eep_read();
	if(time_sign) time_read();
	if(freq_sign) freq_read();
	if(rx_flag) uart_reply();
}
/*************************************************
*函数：mod_ctrl()模式变换函数
*功能：模式变换服务
*************************************************/
void mod_ctrl(){
	if(read_flag == 0){
		if(key_sign==4){
			mod_flag=len_mod;
			mod_init();
		}else if(key_sign==5){
			mod_flag=temp_mod;
			mod_init();
		}else if(key_sign==8){
			mod_flag=vol_mod;
			mod_init();
		}else if(key_sign==9){
			mod_flag=bright_mod;
			mod_init();
		}else if(key_sign==13){
			read_addr = write_addr;
			read_mod = mod_flag;
			write_flag = 10;
			l6 = 1;
			mod_init();
		}else if(key_sign==12){
			mod_flag=freq_mod;
			mod_init();
		}else if(key_sign==23){
			if(write_flag == 0 && write_sign == 0){
				mod_flag = read_mod;
				mod_init();
				mod_flag = eep_mod;
				l7 = 1;
				read_flag = 9;
				read_sign = 10;
			}
		}
	}else{
		if(key_sign==13){
			read_sign = read_flag--;
			if(read_flag == 0) read_flag = 10;
			mod_init();
		}else if(key_sign==23){
			l7 = 0;
			mod_flag = read_mod;
			read_flag = 0;
			mod_init();
		}
	}
	key_sign = 0;
}
/*************************************************
*函数：read_temp()读温度函数
*功能：读取温度
*************************************************/
void read_temp(){
	int tp;
	u8 tl,th,i;

	//while(init_ds18b20())loop();
	init_ds18b20();
	Write_DS18B20(0xCC);
	Write_DS18B20(0x44);
	init_ds18b20();
	//while(init_ds18b20())loop();
	Write_DS18B20(0xCC);
	Write_DS18B20(0xBE);
	tl=Read_DS18B20();
	th=Read_DS18B20();
	tp=(th<<8)|tl;
	temp=(tp>>2)*25;

	if(mod_flag == temp_mod){
		if(temp&0x8000){
			dis[3]=font[-temp/10000%10];
			dis[4]=font[-temp/1000%10];
			dis[5]=font[-temp/100%10]&0x7f;
			dis[6]=font[-temp/10%10];
			dis[7]=font[-temp%10];
			for(i=3;dis[i]==font[0];i++) dis[i]=0xff;
			dis[i-1]=0xbf;
		}else{
			dis[3]=font[temp/10000%10];
			dis[4]=font[temp/1000%10];
			dis[5]=font[temp/100%10]&0x7f;
			dis[6]=font[temp/10%10];
			dis[7]=font[temp%10];
			for(i=3;dis[i]==font[0];i++) dis[i]=0xff;
		}
	}
	temp_sign = 0;
}	
/*************************************************
*函数：read_len()一次性读距离函数
*功能：读取距离
*************************************************/
#ifdef len_read_once
void read_len(){
	u8 i=8;
	
	break_flag = 0;
	echo_flag = 0;
	len_flag = 0;
	//发送
	while(i--){
		Trig = 1;
		delay12us();
		Trig = 0;
		delay12us();
	}
	CL = 0;
	CH = 0;
	CCF0 = 0;
	CF = 0;
	CR = 1;                 //PCA定时器开始工作
	CCAPM0 |= 0x01;					//开启中断
	while(echo_flag==0&&break_flag==0)loop();
	//接收
	if(break_flag){
		len=9999;
		if(mod_flag==len_mod){
			dis[3]=font[9];
			dis[4]=font[9];
			dis[5]=font[9];
			dis[6]=font[9]&0x7f;
			dis[7]=font[9];
		}
	}else if(echo_flag){
		len=length*0.17;
		if(mod_flag==len_mod){
			dis[3]=font[len/10000];
			dis[4]=font[len/1000%10];
			dis[5]=font[len/100%10];
			dis[6]=font[len/10%10]&0x7f;
			dis[7]=font[len%10];
			for(i=3;dis[i]==font[0];i++) dis[i]=0xff;
		}
	}
}	
/*************************************************
*函数：send_len()+read_len()发读分离法读距离函数
*功能：读取距离
*************************************************/
#elif defined len_read_IT
//发送
void send_len(){
	u8 i=8;
	
	break_flag = 0;
	echo_flag = 0;
	len_flag = 0;
	while(i--){
		Trig = 1;
		delay12us();
		Trig = 0;
		delay12us();
	}
	CL = 0;									//计时器清零
	CH = 0;
	CCF0 = 0;								//清标志
	CF = 0;
	CR = 1;                 //PCA定时器开始工作
	CCAPM0 |= 0x01;					//开启中断
}

//接收
void read_len(){
	u8 i;
	
	if(echo_flag){
		len=length *0.017 *10;	//保留1位小数
		if(mod_flag==len_mod){
			dis[3]=font[len/10000];
			dis[4]=font[len/1000%10];
			dis[5]=font[len/100%10];
			dis[6]=font[len/10%10]&0x7f;
			dis[7]=font[len%10];
			for(i=3;dis[i]==font[0];i++) dis[i]=0xff;
		}
	}else if(break_flag){
		len=9999;
		if(mod_flag==len_mod){
			dis[3]=font[9];
			dis[4]=font[9];
			dis[5]=font[9];
			dis[6]=font[9]&0x7f;
			dis[7]=font[9];
		}
	}
	break_flag = 0;
	echo_flag = 0;
}
#endif
/*************************************************
*函数：read_vol()读电位器函数
*功能：读取电位器电压
*************************************************/
void read_vol(){
	l3=1;
	vol_flag = 0;
	IIC_Start();
	IIC_SendByte(0x90);
	IIC_WaitAck();
	IIC_SendByte(0x43);
	IIC_WaitAck();
	IIC_Start();
	IIC_SendByte(0x91);
	IIC_WaitAck();
	IIC_RecByte();
	IIC_SendAck(0);
	vol=IIC_RecByte();
	vol=vol*(500.0/255);
	IIC_SendAck(1);
	IIC_Stop();

	if(mod_flag==vol_mod){
		dis[5]=font[vol/100%10]&0x7f;
		dis[6]=font[vol/10%10];
		dis[7]=font[vol%10];
	}
	if(vol>300){
		buzz=1;
	}else{
		buzz=0;
	}
	l3=0;
}
/*************************************************
*函数：write_vol()读电位器函数
*功能：读取电位器电压
*************************************************/
void write_vol() {
	u8 d = vout*(256/500.0);		//(256/500)约为0.5，用vout/2可避免浮点运算，但有误差 

	IIC_Start();
	IIC_SendByte(0x90);		//器件地址，写操作
	IIC_WaitAck();
	IIC_SendByte(0x40);		//设置允许模拟输出，输入通道随意
	IIC_WaitAck();
	IIC_SendByte(d);		//设置输出值
	IIC_WaitAck();
	IIC_Stop();
}
/*************************************************
*函数：read_bright()读亮度函数
*功能：读取光敏电阻电压
*************************************************/
void read_bright(){
	u8 d;

	l4=1;
	bright_flag = 0;
	IIC_Start();
	IIC_SendByte(0x90);
	IIC_WaitAck();
	IIC_SendByte(0x41);
	IIC_WaitAck();
	IIC_Start();
	IIC_SendByte(0x91);
	IIC_WaitAck();
	IIC_RecByte();
	IIC_SendAck(0);
	d=IIC_RecByte();
	bright=d*(500.0/255);
	IIC_SendAck(1);
	IIC_Stop();

	if(mod_flag==bright_mod){
		dis[5]=font[bright/100%10]&0x7f;
		dis[6]=font[bright/10%10];
		dis[7]=font[bright%10];
	}
	if(bright>300){
		relay=1;
	}else{
		relay=0;
	}
	l4=0;
} 
/*************************************************
*函数：send_str()发送字符串函数
*功能：向串口发送字符串
*************************************************/
void send_str(){
	tx_flag = 1;
	tx_pot = 0;
	SBUF = tx_buf[tx_pot++];		//写数据到UART数据寄存器
}
/*************************************************
*函数：eep_write()EEPROM写函数
*功能：向EEPROM写字节
*************************************************/
void eep_write(){
	IIC_Start();
	IIC_SendByte(0xa0);
	IIC_WaitAck();
	IIC_SendByte(write_sign-1<<1);
	IIC_WaitAck();
	IIC_SendByte(((u8 *)write_addr)[0]);
	IIC_WaitAck();
	IIC_SendByte(((u8 *)write_addr)[1]);
	IIC_WaitAck();
	IIC_Stop();
	write_sign = 0;
	if(write_flag == 0){
		l6 = 0;
		if(read_mod != mod_flag)mod_init();
	}
}
/*************************************************
*函数：eep_read()EEPROM读函数
*功能：向EEPROM读字节
*************************************************/
void eep_read(){
	IIC_Start();
	IIC_SendByte(0xa0);
	IIC_WaitAck();
	IIC_SendByte(read_sign-1<<1);
	IIC_WaitAck();
	IIC_Start();
	IIC_SendByte(0xa1);
	IIC_WaitAck();
	((u8 *)read_addr)[0] = IIC_RecByte();
	IIC_SendAck(0);
	((u8 *)read_addr)[1] = IIC_RecByte();
	IIC_Stop();
	mod_flag = read_mod;
	mod_init();
	mod_flag = eep_mod;
	read_sign = 0;
}
/*************************************************
*函数：time_read()读时间函数
*功能：读取DS1302
*************************************************/
void time_read(){
	u8 i=7;
	while(i--) time[i]=Read_Ds1302_Byte(time_read_addr[i]);
	time_sign = 0;
}
/*************************************************
*函数：uart_reply()串口响应函数
*功能：串口响应接收字符串
*************************************************/
void freq_read(){
	u8 i;
	
	freq = (u32)freq_sign<<3;
	if(mod_flag == freq_mod){
		dis[1]=font[freq/1000000%10];
		dis[2]=font[freq/100000%10];
		dis[3]=font[freq/10000%10];
		dis[4]=font[freq/1000%10];
		dis[5]=font[freq/100%10];
		dis[6]=font[freq/10%10];
		dis[7]=font[freq%10];
		for(i=1;dis[i]==font[0]&&i<8;i++) dis[i]=0xff;
	}
	freq_sign = 0;
	freq_H = 0;
}
/*************************************************
*函数：uart_reply()串口响应函数
*功能：串口响应接收字符串
*************************************************/
void uart_reply(){
	l5=1;
	rx_flag = 0;
	if(strcmp(rx_buf,"temp\r\n")==0){
		while(tx_flag) loop();
		sprintf(tx_buf,"temp:%.2f'C\r\n",temp/100.0);
		send_str();
	}else if(strcmp(rx_buf,"len\r\n")==0){
		while(tx_flag) loop();
		sprintf(tx_buf,"len:%.1fcm\r\n",len/10.0);
		send_str();
	}else if(strcmp(rx_buf,"vol\r\n")==0){
		while(tx_flag) loop();
		sprintf(tx_buf,"vol:%.2fV\r\n",vol/100.0);
		send_str();
	}else if(strcmp(rx_buf,"bright\r\n")==0){
		while(tx_flag) loop();
		sprintf(tx_buf,"bright:%.2fV\r\n",bright/100.0);
		send_str();
	}else if(strcmp(rx_buf,"time\r\n")==0){
		while(tx_flag) loop();
		sprintf(tx_buf,"time:%.0f:%.0f:%.0f\r\n",trans(time[2]),trans(time[1]),trans(time[0]));
		send_str();
	}else if(strcmp(rx_buf,"date\r\n")==0){
		while(tx_flag) loop();
		sprintf(tx_buf,"date:%.0f-%.0f-%.0f\r\n",trans(time[6]),trans(time[4]),trans(time[3]));
		send_str();
	}
	l5=0;
}
/*************************************************
*函数：scankey()扫描按键函数
*功能：扫描按键
*************************************************/
void scankey(){
	u8 key;

	P4=0xff;P3=0xff;P3&=0xf3;
	if(P35==0|P42==0|P44==0){
		if(key_count==0){
			key_count = 1;
			key = (u8)P35<<5|(u8)P42<<6|(u8)P44<<7;
			P3=0xff;P35=0;P4=0x00;
			//delay12us();
			key |= P3 &0x0c;
			switch(key){
				case 0x64:key_flag=4;break;
				case 0x68:key_flag=5;break;
				case 0xa4:key_flag=8;break;
				case 0xa8:key_flag=9;break;
				case 0xc4:key_flag=12;break;
				case 0xc8:key_flag=13;break;
			}
		}
		return;
	}
	if(key_count>1000){
		key_count = 0;
		key_sign = key_flag +10;
	}else if(key_count>2){
		key_count=0;
		key_sign = key_flag;
	}else{
		key_count=0;
	}
}
/*************************************************
*函数：dis_smg()数码管显示函数
*功能：驱动显示数码管
*************************************************/
void dis_smg(){
	u8 i;

	for(i=0;i<8;i++){
		P2&=0x1f;
		P0=1<<i;
		P2|=y6;
		_nop_();
		P2&=0x1f;
		P0=dis[i];
		P2|=y7;
		delay100us();
		P0=0xff;
	}
	smg_sign = 0;
}
/*************************************************
*函数：dis_led()LED显示函数
*功能：驱动显示LED
*************************************************/
void dis_led(){
	P2&=0x1f;
	P0=~led;
	P2|=y4;
	P2&=0x1f;
}
/*************************************************
*函数：dis_out()OUT显示函数
*功能：驱动显示OUT
*************************************************/
void dis_out(){
	P2&=0x1f;
	P0=out;
	P2|=y5;
	P2&=0x1f;
}
/*************************************************
*函数：main()系统进入函数
*功能：系统进入初始化服务，系统进行服务
*************************************************/
void main(){
	//初始化
	init();
	while(1){
		//快速I/O操作
		loop();
		//中断检查与处理
		soft_IT();
	}	
}
/*************************************************
*函数：T0_it()T0中断函数
*功能：设置T0计数溢出的情况
*************************************************/
void T0_it() interrupt 1 using 1
{
	freq_T++;
}

/*************************************************
*函数：Uart()串口中断处理函数
*功能：软件中断标志的定时置位服务，毫秒级的计时计数服务
*硬件：使用T2定时器，规定1ms溢出中断一次
*************************************************/
void Uart() interrupt 4	using 2
{
    if (RI){
		RI = 0;                 //清除RI位
		rx_buf[rx_pot] = SBUF;//存串口数据
		if(rx_buf[rx_pot]=='?'){
			rx_pot = 0;
		}else if(rx_buf[rx_pot]=='\n'){
			rx_buf[++rx_pot] ='\0';
			rx_flag = 1;
			rx_pot = 0; 	
		}else{
			if(++rx_pot>=15) rx_pot = 0;
		}
    }
    if (TI){
		TI = 0;                 //清除TI位
		if(tx_buf[tx_pot]){
			SBUF = tx_buf[tx_pot];                 //写数据到UART数据寄存器
			if(++tx_pot>=15) tx_pot=0;
		}else{
			tx_pot = 0;
			tx_flag = 0;
		}
	}
}
/*************************************************
*函数：PCA_isr()PCA定时器中断处理函数
*功能：脉冲时长计数服务
*硬件：使用PCA定时器，规定溢出和P11下降沿中断
*输出：count记录从定时器打开到下降沿之前的时长计数
*************************************************/
void PCA_isr() interrupt 7 using 3
{	
	if (CCF0){
		CCF0 = 0;
		length = (CCAP0H<<8)|CCAP0L;  //保存本次的捕获值
		echo_flag = 1;
		CR = 0;												//PCA定时器停止工作
		CCAPM0 &= 0xfe;								//关闭中断
	}
	if (CF){
		CF = 0;
		break_flag = 1;
		CR = 0;												//PCA定时器停止工作
		CCAPM0 &= 0xfe;								//关闭中断
	}
}
/*************************************************
*函数：Sysclk_IT()系统定时中断处理函数
*功能：软件中断标志的定时置位服务，毫秒级的计时计数服务
*硬件：使用T2定时器，规定1ms溢出中断一次
*************************************************/
void Sysclk_IT() interrupt 12 using 3
{
	//毫秒级延时服务
	if(delay_timing){
		delay_timing--;
	}
	//数码管定时刷新
	smg_sign = 1;
	//18B20定时读取
	if(temp_timing){
		temp_timing--;
	}else{
		temp_timing=50;
		temp_sign=1;
	}
	//超声波定时读取
	if(len_timing){
		len_timing--;
	}else{
		len_timing=100;
		len_flag=1;
	}
	//电位器定时读取
	if(vol_timing){
		vol_timing--;
	}else{
		vol_timing=50;
		vol_flag=1;
	}
	//光敏电阻定时读取
	if(bright_timing){
		bright_timing--;
	}else{
		bright_timing=50;
		bright_flag=1;
	}
	//EEPROM写时钟
	if(write_timing){
		write_timing--;
	}else{
		write_timing=100;
		if(write_flag){
			write_sign = write_flag--;
		}
	}
	if(freq_timing){
		freq_timing--;
	}else{
		freq_timing=500;
		count_timing=125;
		TH0 = 0x00;
		TL0 = 0x00;
		TR0 = 1;
	}
	if(count_timing){
		count_timing--;
	}else{
		TR0 = 0;
		((u8 *)&freq_sign)[0] = TH0;
		((u8 *)&freq_sign)[1] = TL0;
		freq_H = freq_T;
		freq_T = 0;
	}
	if(time_timing){
		time_timing--;
	}else{
		time_timing = 50;
		time_sign = 1;
	}
	if(L1_timing){
		L1_timing--;
	}else if(L1_mode){
		L1_timing = L1_mode;
		l1=~l1;
	}
	//按键时长计数
	if(key_count){
		if(++key_count==0)key_count=1001;
	}
}