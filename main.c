#include <STC15F2K60S2.H>
#include <intrins.h>
#include <onewire.h>

#ifndef u8
#define u8 unsigned char
#endif

#ifndef u16
#define u16 unsigned int
#endif

#ifndef u32
#define u32 unsigned long
#endif

sbit Trig = P1^0;
sbit Echo = P1^1;

u8 code font[10]={0xc0,0xf9,0xa4,0xb0,0x99,0x92,0x82,0xf8,0x80,0x90};
u8 code y4=0x80,y5=0xa0,y6=0xc0,y7=0xe0;

bit temp_flag=0,len_flag=0,break_flag=0,echo_flag=0,tx_flag=0;
bit temp_mod=0;len_mod=1;
u8 dis[8]={0},tx_buf[16]="\0",rx_buf[16]="\0";
u8 key_flag=0,tx_pot=0,rx_pot=0;
u16 temp_timing=0,len_timing=250;
u16 count=0,len=20,key_count=0;
int temp=20;

void mod_init();
void mod_ctrl();
void read_temp();
void read_len();
u8 scankey();
void dis_smg();

void Sysclk_init(){
	AUXR |= 0x04;		//定时器2时钟1T模式
	T2L = 0x20;			//设置定时初值
	T2H = 0xD1;			//设置定时初值
    IE2 |= 0x04;		//开定时器2中断
    EA = 1;
	AUXR |= 0x10;		//定时器2开始计时
}

void PCA_init(){
	P_SW1 &= 0xcf;		//(P1.2/ECI, P1.1/CCP0, P1.0/CCP1, P3.7/CCP2)
    CCON = 0;                       //初始化PCA控制寄存器
                                    //PCA定时器停止
                                    //清除CF标志
                                    //清除模块中断标志
    CL = 0;                         //复位PCA寄存器
    CH = 0;
    CCAP0L = 0;
    CCAP0H = 0;
    CMOD = 0x01;                    //设置PCA时钟源为系统时钟/12,且使能PCA计时溢出中断
    CCAPM0 = 0x10;					//PCA模块0为16位捕获模式(下降沿捕获,可测从低电平开始的整个周期)
	
    EA = 1;

}

void UartInit(void)		//4800bps@12.000MHz
{
	SCON = 0x50;		//8位数据,可变波特率
	AUXR |= 0x40;		//定时器1时钟为Fosc,即1T
	AUXR &= 0xFE;		//串口1选择定时器1为波特率发生器
	TMOD &= 0x0F;		//设定定时器1为16位自动重装方式
	TL1 = 0x8F;			//设定定时初值
	TH1 = 0xFD;			//设定定时初值
	ET1 = 0;			//禁止定时器1中断
	TR1 = 1;			//启动定时器1
}

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

void init(){
	mod_init();
	Trig = 0;
	Echo = 1;
	PCA_init();
	Sysclk_init();
}

void loop(){
	mod_ctrl();
	dis_smg();
}

void soft_IT(){
	
	if(temp_flag) read_temp();
	if(len_flag) read_len();
}

void mod_init(){
	if(temp_mod){
		dis[0]=0xc6;
		dis[1]=0xff;
		dis[2]=0xff;
		dis[3]=0xff;
		return;
	}
	if(len_mod){
		dis[0]=0xc7;
		dis[1]=0xff;
		dis[2]=0xff;
		return;
	}
}

void mod_ctrl(){
	u8 key;

	key=scankey();
	if(temp_mod){
		if(key==12){
			temp_mod=0;
			len_mod=1;
			dis[0]=0xc7;
			dis[1]=0xff;
			dis[2]=0xff;
			if(count==0){
				dis[3]=font[9];
				dis[4]=font[9];
				dis[5]=font[9];
				dis[6]=font[9]&0x7f;
				dis[7]=font[9];
			}else{
				len=count*0.17;
				dis[3]=font[len/10000];
				dis[4]=font[len/1000%10];
				dis[5]=font[len/100%10];
				dis[6]=font[len/10%10]&0x7f;
				dis[7]=font[len%10];
			}
			return;
		}
	}
	if(len_mod){
		if(key==12){	   
			len_mod=0;
			temp_mod=1;
			dis[0]=0xc6;
			dis[1]=0xff;
			dis[2]=0xff;
			dis[3]=0xff;
			dis[4]=font[temp/1000%10];
			dis[5]=font[temp/100%10]&0x7f;
			dis[6]=font[temp/10%10];
			dis[7]=font[temp%10];
			return;
		}
	}
}

void read_temp(){
	int tp;
	u8 tl,th;

	while(init_ds18b20())loop();
	Write_DS18B20(0xCC);
	Write_DS18B20(0x44);
	while(init_ds18b20())loop();
	Write_DS18B20(0xCC);
	Write_DS18B20(0xBE);
	tl=Read_DS18B20();
	th=Read_DS18B20();
	tp=(th<<8)|tl;
	temp=tp*6.25;

	if(temp_mod){
		dis[4]=font[temp/1000%10];
		dis[5]=font[temp/100%10]&0x7f;
		dis[6]=font[temp/10%10];
		dis[7]=font[temp%10];
	}
	temp_flag=0;
}
void read_len(){
	u8 i=8;
			  
	//发送
	while(i--){
		Trig = 1;
		delay12us();
		Trig = 0;
		delay12us();
	}
	//接收
    CR = 1;                         //PCA定时器开始工作
    CCF0 = 0;
	CCAPM0 |= 0x01;					//开启中断
	while(echo_flag!=0&&break_flag!=0)loop();
	if(len_mod){
		if(break_flag){
			dis[3]=font[9];
			dis[4]=font[9];
			dis[5]=font[9];
			dis[6]=font[9]&0x7f;
			dis[7]=font[9];
		}else{
			len=count*0.17;
			dis[3]=font[len/10000];
			dis[4]=font[len/1000%10];
			dis[5]=font[len/100%10];
			dis[6]=font[len/10%10]&0x7f;
			dis[7]=font[len%10];
		}
	}
	break_flag = 0;
	echo_flag = 0;
	len_flag = 0;
}

u8 scankey(){
	u8 key;

	P3=0xff;P3&=0xf3;
	if(P34==0|P35==0){
		delay100us();
		if(P34==0|P35==0){
			key = P3 &0x30;
			if(key_count==0){
				key_count = 1;
				P3=0xff;P3&=0xcf;
				delay12us();
				key |= P3 &0x0c;
				switch(key){
					case 0x14:key_flag=12;return 0;
					case 0x18:key_flag=13;return 0;
					case 0x24:key_flag=16;return 0;
					case 0x28:key_flag=17;return 0;
				}
			}
			return 0;
		}	
	}
	if(key_count){
		if(key_count<1000){
			key_count=0;
			return key_flag;
		}else{
			key_count=0;
			return key_flag+10;
		}
	}
	return 0;
}

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
}

void main(){
	init();
	while(1){
		loop();
		soft_IT();
	}	
}

void Uart() interrupt 4	using 2
{
    if (RI)
    {
        RI = 0;                 //清除RI位
        rx_buf[rx_pot] = SBUF;//存串口数据
		if(rx_buf[rx_pot]=='\n'){
			rx_pot=0;
		}else{
			rx_pot++;
		}
    }
    if (TI)
    {
        TI = 0;                 //清除TI位
        if(tx_buf[tx_pot]){
    		SBUF = tx_buf[tx_pot++];                 //写数据到UART数据寄存器
		}else{
			tx_flag = 0;
		}
    }
}

void PCA_isr() interrupt 7 using 3
{	
    if (CF)
    {
        CF = 0;
        break_flag=1;
    }
    if (CCF0)
    {
        CCF0 = 0;
			echo_flag = 1;
        	count=(CCAP0H<<8)|CCAP0L;	//保存本次的捕获值
			CCAPM0 &= 0xfe;				//关闭中断
	    	CR = 0;						//PCA定时器停止工作
    		CL = 0;                     //复位PCA寄存器
    		CH = 0;
    		CCAP0L = 0;
    		CCAP0H = 0;
		
    }
}

void Sysclk_IT() interrupt 12 using 3
{
	//18B20定时读取
	if(temp_timing){
		temp_timing--;
	}else{
		temp_timing=500;
		temp_flag=1;
	}
	//超声波定时读取
	if(len_timing){
		len_timing--;
	}else{
		len_timing=1000;
		len_flag=1;
	}
	//按键时长计数
	if(key_count){
		if(++key_count==0)key_count=1000;
	}
}