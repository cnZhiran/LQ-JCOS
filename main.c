#include <STC15F2K60S2.H>
#include <intrins.h>
#include <string.h>
#include <stdio.h>
#include <onewire.h>
#include <iic.h>

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

#define len_read_IT		//len_read_onceһ���Զ����룬len_read_IT�������뷨������


sbit Trig = P1^0;
sbit Echo = P1^1;

u8 code font[10]={0xc0,0xf9,0xa4,0xb0,0x99,0x92,0x82,0xf8,0x80,0x90};
u8 code y4=0x80,y5=0xa0,y6=0xc0,y7=0xe0;

u8 bdata led=0,out=0;
bit temp_flag=0,len_flag=0,vol_flag=0,bright_flag=0,break_flag=0,echo_flag=0,tx_flag=0,rx_flag=0;
u8 idata dis[8]={0},tx_buf[16]="init_well\r\n",rx_buf[16]="\0";
u8 idata key_flag=0,key_sign=0,tx_pot=0,rx_pot=0,cnt=0,write_flag=0,write_sign=0,read_flag=0,read_sign=0;
u16 idata key_count=0,temp_timing=250,vol_timing=125,len_timing=0,bright_timing=375,delay_timing=0,write_timing=500;
u16 mod_flag=len_mod,read_mod=len_mod,*write_addr,*read_addr;
u16 length=0,temp=-2000,len=20,vol=250,bright=250;

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
void scankey();
void send_str();
void uart_reply();
void eep_write();
void eep_read();

void dis_smg();
void dis_led();
void dis_out();

/*************************************************
*������mod_init()ϵͳģʽ��ʼ������
*���ܣ�ϵͳģʽ��ʼ��
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
	}
}
/*************************************************
*������Sysclk_init()ϵͳ��ʱ��ʼ������
*���ܣ�ϵͳ��ʱ��ʼ��
*Ӳ����ʹ��T2��ʱ�����涨1ms����ж�һ��
*************************************************/
void Sysclk_init(){
	AUXR |= 0x04;		//��ʱ��2ʱ��1Tģʽ
	T2L = 0x20;			//���ö�ʱ��ֵ
	T2H = 0xD1;			//���ö�ʱ��ֵ
	IE2 |= 0x04;		//����ʱ��2�ж�
	EA = 1;
	AUXR |= 0x10;		//��ʱ��2��ʼ��ʱ
}
/*************************************************
*������PCA_init()PCA��ʱ����ʼ������
*���ܣ�PCA��ʱ����ʼ��			 
*Ӳ����ʹ��PCA��ʱ�����涨�����P11�½����ж�
*************************************************/
void PCA_init(){
	P_SW1 &= 0xcf;		//(P1.2/ECI, P1.1/CCP0, P1.0/CCP1, P3.7/CCP2)
	CCON = 0;                       //��ʼ��PCA���ƼĴ���
                                  //PCA��ʱ��ֹͣ
                                  //���CF��־
                                  //���ģ���жϱ�־
  CL = 0;                         //��λPCA�Ĵ���
  CH = 0;
  CMOD = 0x01;                    //����PCAʱ��Դ,��������ж�
  CCAPM0 = 0x11;                  //PCAģ��0Ϊ�½��ش���,�����жϡ�
  EA = 1;
}
/*************************************************
*������Uart_init()���ڳ�ʼ������
*���ܣ����ڳ�ʼ�� 
*Ӳ����ʹ��T1��ʱ����������Ϊ4800
*************************************************/
void Uart_init(void)		//4800bps@12.000MHz
{
	SCON = 0x50;		//8λ����,�ɱ䲨����
	AUXR |= 0x40;		//��ʱ��1ʱ��ΪFosc,��1T
	AUXR &= 0xFE;		//����1ѡ��ʱ��1Ϊ�����ʷ�����
	TMOD &= 0x0F;		//�趨��ʱ��1Ϊ16λ�Զ���װ��ʽ
	TL1 = 0x8F;			//�趨��ʱ��ֵ
	TH1 = 0xFD;			//�趨��ʱ��ֵ
	ET1 = 0;			//��ֹ��ʱ��1�ж�
	ES = 1;
	EA = 1;
	TR1 = 1;			//������ʱ��1

	send_str();
}
/*************************************************
*������delay_us()΢�뼶��ʱ����
*���ܣ�΢�뼶��ʱ����
*��ע�������ܵ�ʹ��STC-ISP����ʱ�������������ʱ����
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
*������init()��ʼ������
*���ܣ�ϵͳ����ĳ�ʼ������
*************************************************/
void init(){
	mod_init();
	Trig = 0;
	Echo = 1;
	PCA_init();
	Sysclk_init();
	Uart_init();
}
/*************************************************
*������loop()������Ӧ����
*���ܣ�����I/O�豸����������ģʽ�任����
*��ע��Ҫ��������һ�ε�ʱ��Ҫ�����ܵĶ̣���������Ӱ��������������ʱ�ȴ�������
*************************************************/
void loop(){
	scankey();
	mod_ctrl();
	dis_smg();
	dis_led();
	dis_out();
}
/*************************************************
*������soft_IT()�жϲ���ʹ�����
*���ܣ��жϲ���ʹ������
*************************************************/
void soft_IT(){
	
	if(temp_flag) read_temp();
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
	if(rx_flag) uart_reply();
}
/*************************************************
*������mod_ctrl()ģʽ�任����
*���ܣ�ģʽ�任����
*************************************************/
void mod_ctrl(){
	if(read_flag == 0){
		if(key_sign==4){
			mod_flag=len_mod;
		}else if(key_sign==5){
			mod_flag=temp_mod;
		}else if(key_sign==8){
			mod_flag=vol_mod;
		}else if(key_sign==9){
			mod_flag=bright_mod;
		}else if(key_sign==13){
			read_addr = write_addr;
			read_mod = mod_flag;
			write_flag = 10;
			l6 = 1;
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
		}else if(key_sign==23){
			l7 = 0;
			mod_flag = read_mod;
			read_flag = 0;
		}
	}
	key_sign = 0;
	mod_init();
}
/*************************************************
*������read_temp()���¶Ⱥ���
*���ܣ���ȡ�¶�
*************************************************/
void read_temp(){
	int tp;
	u8 tl,th;

	l1=1;
	temp_flag=0;
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

	if(mod_flag==temp_mod){
		dis[4]=font[temp/1000%10];
		dis[5]=font[temp/100%10]&0x7f;
		dis[6]=font[temp/10%10];
		dis[7]=font[temp%10];
	}
	l1=0;

}	
/*************************************************
*������read_len()һ���Զ����뺯��
*���ܣ���ȡ����
*************************************************/
#ifdef len_read_once
void read_len(){
	u8 i=8;
	
	break_flag = 0;
	echo_flag = 0;
	len_flag = 0;
	//����
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
	CR = 1;                 //PCA��ʱ����ʼ����
	CCAPM0 |= 0x01;					//�����ж�
	while(echo_flag==0&&break_flag==0)loop();
	//����
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
*������send_len()+read_len()�������뷨�����뺯��
*���ܣ���ȡ����
*************************************************/
#elif defined len_read_IT
//����
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
	CL = 0;									//��ʱ������
	CH = 0;
	CCF0 = 0;								//���־
	CF = 0;
	CR = 1;                 //PCA��ʱ����ʼ����
	CCAPM0 |= 0x01;					//�����ж�
}

//����
void read_len(){
	u8 i;
	
	if(echo_flag){
		len=length*0.17;
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
}
#endif
/*************************************************
*������read_vol()����λ������
*���ܣ���ȡ��λ����ѹ
*************************************************/
void read_vol(){
	l3=1;
	vol_flag = 0;
	IIC_Start();
	IIC_SendByte(0x90);
	IIC_WaitAck();
	IIC_SendByte(0x03);
	IIC_WaitAck();
	IIC_Start();
	IIC_SendByte(0x91);
	IIC_WaitAck();
	IIC_RecByte();
	IIC_SendAck(0);
	vol=IIC_RecByte();
	vol=vol*500.0/255;
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
*������read_bright()�����Ⱥ���
*���ܣ���ȡ���������ѹ
*************************************************/
void read_bright(){
	l4=1;
	bright_flag = 0;
	IIC_Start();
	IIC_SendByte(0x90);
	IIC_WaitAck();
	IIC_SendByte(0x01);
	IIC_WaitAck();
	IIC_Start();
	IIC_SendByte(0x91);
	IIC_WaitAck();
	IIC_RecByte();
	IIC_SendAck(0);
	bright=IIC_RecByte();
	bright=bright*500.0/255;
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
*������send_str()�����ַ�������
*���ܣ��򴮿ڷ����ַ���
*************************************************/
void send_str(){
	tx_flag = 1;
	tx_pot = 0;
	SBUF = tx_buf[tx_pot++];		//д���ݵ�UART���ݼĴ���
}
/*************************************************
*������eep_write()EEPROMд����
*���ܣ���EEPROMд�ֽ�
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
*������eep_write()EEPROMд����
*���ܣ���EEPROMд�ֽ�
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
*������uart_reply()������Ӧ����
*���ܣ�������Ӧ�����ַ���
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
	}
	l5=0;
}
/*************************************************
*������scankey()ɨ�谴������
*���ܣ�ɨ�谴��
*************************************************/
void scankey(){
	u8 key;

	P4=0xff;P3=0xff;P3&=0xf3;
	if(P34==0|P35==0|P42==0|P44==0){
		if(key_count==0){
			key_count = 1;
			key = P3 &0x30;key|=(u8)P42<<6;key|=(u8)P44<<7;
			P3=0xff;P3&=0xcf;P4=0x00;
			delay12us();
			key |= P3 &0x0c;
			switch(key){
				case 0x74:key_flag=4;break;
				case 0x78:key_flag=5;break;
				case 0xb4:key_flag=8;break;
				case 0xb8:key_flag=9;break;
				case 0xd4:key_flag=12;break;
				case 0xd8:key_flag=13;break;
				case 0xe4:key_flag=16;break;
				case 0xe8:key_flag=17;break;
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
*������dis_smg()�������ʾ����
*���ܣ�������ʾ�����
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
}
/*************************************************
*������dis_led()LED��ʾ����
*���ܣ�������ʾLED
*************************************************/
void dis_led(){
	P2&=0x1f;
	P0=~led;
	P2|=y4;
	P2&=0x1f;
}
/*************************************************
*������dis_out()OUT��ʾ����
*���ܣ�������ʾOUT
*************************************************/
void dis_out(){
	P2&=0x1f;
	P0=out;
	P2|=y5;
	P2&=0x1f;
}
/*************************************************
*������main()ϵͳ���뺯��
*���ܣ�ϵͳ�����ʼ������ϵͳ���з���
*************************************************/
void main(){
	//��ʼ��
	init();
	while(1){
		//����I/O����
		loop();
		//�жϼ���봦��
		soft_IT();
	}	
}
/*************************************************
*������Uart()�����жϴ�����
*���ܣ�����жϱ�־�Ķ�ʱ��λ���񣬺��뼶�ļ�ʱ��������
*Ӳ����ʹ��T2��ʱ�����涨1ms����ж�һ��
*************************************************/
void Uart() interrupt 4	using 2
{
    if (RI){
			RI = 0;                 //���RIλ
			rx_buf[rx_pot] = SBUF;//�洮������
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
			TI = 0;                 //���TIλ
			if(tx_buf[tx_pot]){
    		SBUF = tx_buf[tx_pot];                 //д���ݵ�UART���ݼĴ���
			if(++tx_pot>=15) tx_pot=0;
		}else{
			tx_pot = 0;
			tx_flag = 0;
		}
	}
}
/*************************************************
*������PCA_isr()PCA��ʱ���жϴ�����
*���ܣ�����ʱ����������
*Ӳ����ʹ��PCA��ʱ�����涨�����P11�½����ж�
*�����count��¼�Ӷ�ʱ���򿪵��½���֮ǰ��ʱ������
*************************************************/
void PCA_isr() interrupt 7 using 3
{	
	if (CCF0){
		CCF0 = 0;
		length = (CCAP0H<<8)|CCAP0L;  //���汾�εĲ���ֵ
		echo_flag = 1;
		CR = 0;												//PCA��ʱ��ֹͣ����
		CCAPM0 &= 0xfe;								//�ر��ж�
	}
	if (CF){
		CF = 0;
		break_flag = 1;
		CR = 0;												//PCA��ʱ��ֹͣ����
		CCAPM0 &= 0xfe;								//�ر��ж�
	}
}
/*************************************************
*������Sysclk_IT()ϵͳ��ʱ�жϴ�����
*���ܣ�����жϱ�־�Ķ�ʱ��λ���񣬺��뼶�ļ�ʱ��������
*Ӳ����ʹ��T2��ʱ�����涨1ms����ж�һ��
*************************************************/
void Sysclk_IT() interrupt 12 using 3
{
	//���뼶��ʱ����
	if(delay_timing){
		delay_timing--;
	}
	//18B20��ʱ��ȡ
	if(temp_timing){
		temp_timing--;
	}else{
		temp_timing=500;
		temp_flag=1;
	}
	//��������ʱ��ȡ
	if(len_timing){
		len_timing--;
	}else{
		len_timing=1000;
		len_flag=1;
	}
	//��λ����ʱ��ȡ
	if(vol_timing){
		vol_timing--;
	}else{
		vol_timing=500;
		vol_flag=1;
	}
	//�������趨ʱ��ȡ
	if(bright_timing){
		bright_timing--;
	}else{
		bright_timing=500;
		bright_flag=1;
	}
	//EEPROMдʱ��
	if(write_timing){
		write_timing--;
	}else{
		write_timing=1000;
		if(write_flag){
			write_sign = write_flag--;
		}
	}
	//����ʱ������
	if(key_count){
		if(++key_count==0)key_count=1001;
	}
}