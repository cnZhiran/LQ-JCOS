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
	AUXR |= 0x04;		//��ʱ��2ʱ��1Tģʽ
	T2L = 0x20;			//���ö�ʱ��ֵ
	T2H = 0xD1;			//���ö�ʱ��ֵ
    IE2 |= 0x04;		//����ʱ��2�ж�
    EA = 1;
	AUXR |= 0x10;		//��ʱ��2��ʼ��ʱ
}

void PCA_init(){
	P_SW1 &= 0xcf;		//(P1.2/ECI, P1.1/CCP0, P1.0/CCP1, P3.7/CCP2)
    CCON = 0;                       //��ʼ��PCA���ƼĴ���
                                    //PCA��ʱ��ֹͣ
                                    //���CF��־
                                    //���ģ���жϱ�־
    CL = 0;                         //��λPCA�Ĵ���
    CH = 0;
    CCAP0L = 0;
    CCAP0H = 0;
    CMOD = 0x01;                    //����PCAʱ��ԴΪϵͳʱ��/12,��ʹ��PCA��ʱ����ж�
    CCAPM0 = 0x10;					//PCAģ��0Ϊ16λ����ģʽ(�½��ز���,�ɲ�ӵ͵�ƽ��ʼ����������)
	
    EA = 1;

}

void UartInit(void)		//4800bps@12.000MHz
{
	SCON = 0x50;		//8λ����,�ɱ䲨����
	AUXR |= 0x40;		//��ʱ��1ʱ��ΪFosc,��1T
	AUXR &= 0xFE;		//����1ѡ��ʱ��1Ϊ�����ʷ�����
	TMOD &= 0x0F;		//�趨��ʱ��1Ϊ16λ�Զ���װ��ʽ
	TL1 = 0x8F;			//�趨��ʱ��ֵ
	TH1 = 0xFD;			//�趨��ʱ��ֵ
	ET1 = 0;			//��ֹ��ʱ��1�ж�
	TR1 = 1;			//������ʱ��1
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
			  
	//����
	while(i--){
		Trig = 1;
		delay12us();
		Trig = 0;
		delay12us();
	}
	//����
    CR = 1;                         //PCA��ʱ����ʼ����
    CCF0 = 0;
	CCAPM0 |= 0x01;					//�����ж�
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
        RI = 0;                 //���RIλ
        rx_buf[rx_pot] = SBUF;//�洮������
		if(rx_buf[rx_pot]=='\n'){
			rx_pot=0;
		}else{
			rx_pot++;
		}
    }
    if (TI)
    {
        TI = 0;                 //���TIλ
        if(tx_buf[tx_pot]){
    		SBUF = tx_buf[tx_pot++];                 //д���ݵ�UART���ݼĴ���
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
        	count=(CCAP0H<<8)|CCAP0L;	//���汾�εĲ���ֵ
			CCAPM0 &= 0xfe;				//�ر��ж�
	    	CR = 0;						//PCA��ʱ��ֹͣ����
    		CL = 0;                     //��λPCA�Ĵ���
    		CH = 0;
    		CCAP0L = 0;
    		CCAP0H = 0;
		
    }
}

void Sysclk_IT() interrupt 12 using 3
{
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
	//����ʱ������
	if(key_count){
		if(++key_count==0)key_count=1000;
	}
}