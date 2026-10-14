#include "gps.h"  
#include "sys.h"
#include "string.h"	
#include "oled.h"
#include "delay.h"
#include <stdio.h>
#include "acost_benchmark.h"  /* ACoST single-point benchmark */

//���ڷ��ͻ����� 	
__align(8) uint8_t USART2_TX_BUF[USART_MAX_SEND_LEN]; 	//���ͻ���,���USART_MAX_SEND_LEN�ֽ� 
uint8_t USART2_RX_BUF[USART_MAX_RECV_LEN]; 				    //���ջ���,���USART_MAX_RECV_LEN���ֽ�.
extern void USART_BaudRate_Init(uint32_t Data);

//���ڷ���һ���ֽ�
static void Com_SendData(uint8_t data)
{
	
	USART_SendData(USART2, data);//��������
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);

}

void HAL_UART_Transmit(USART_TypeDef* USARTx,  uint8_t *pData, uint16_t Size)
{
	uint16_t i;
	for(i=0;i<Size;i++)
	{
	Com_SendData(*(pData+i));
	}
}

static uint8_t* NMEA_FindSentence(uint8_t *buf, const char *type)
{
	const char *talkers[] = {"GP", "GN", "GL", "GA", "GB", "BD"};
	char key[8];
	uint8_t *p;
	uint8_t i;

	for (i = 0; i < (sizeof(talkers) / sizeof(talkers[0])); i++)
	{
		key[0] = '$';
		key[1] = talkers[i][0];
		key[2] = talkers[i][1];
		key[3] = type[0];
		key[4] = type[1];
		key[5] = type[2];
		key[6] = '\0';
		p = (uint8_t*)strstr((const char *)buf, key);
		if (p)
		{
			return p;
		}
	}

	return 0;
}

static uint8_t NMEA_HexToNibble(uint8_t ch)
{
	if (ch >= '0' && ch <= '9') return (uint8_t)(ch - '0');
	if (ch >= 'A' && ch <= 'F') return (uint8_t)(ch - 'A' + 10);
	if (ch >= 'a' && ch <= 'f') return (uint8_t)(ch - 'a' + 10);
	return 0xFF;
}

uint8_t NMEA_Comma_Pos(uint8_t *buf,uint8_t cx)
{	 		    
	uint8_t *p=buf;
	while(cx)
	{		 
		if(*buf=='*'||*buf<' '||*buf>'z')return 0XFF;//����'*'���߷Ƿ��ַ�,�򲻴��ڵ�cx������
		if(*buf==',')cx--;
		buf++;
	}
	return buf-p;	 
}
//m^n����
//����ֵ:m^n�η�.
uint32_t NMEA_Pow(uint8_t m,uint8_t n)
{
	uint32_t result=1;	 
	while(n--)result*=m;    
	return result;
}
//strת��Ϊ����,��','����'*'����
//buf:���ִ洢��
//dx:С����λ��,���ظ����ú���
//����ֵ:ת��������?
int NMEA_Str2num(uint8_t *buf,uint8_t*dx)
{
	while (*buf == ' ' || *buf == '\t')
	{
		buf++;
	}

	uint8_t *p=buf;
	uint32_t ires=0,fres=0;
	uint8_t ilen=0,flen=0,i;
	uint8_t mask=0;
	int res;
	while(1) //�õ�������С���ĳ���
	{
		if(*p=='-'){mask|=0X02;p++;}//�Ǹ���
		if(*p==','||(*p=='*'))break;//����������
		if(*p=='.'){mask|=0X01;p++;}//����С������
		else if(*p>'9'||(*p<'0'))	//�зǷ��ַ�
		{	
			ilen=0;
			flen=0;
			break;
		}	
		if(mask&0X01)flen++;
		else ilen++;
		p++;
	}
	if(mask&0X02)buf++;	//ȥ������
	for(i=0;i<ilen;i++)	//�õ�������������
	{  
		ires+=NMEA_Pow(10,ilen-1-i)*(buf[i]-'0');
	}
	if(flen>5)flen=5;	//����?λС��
	*dx=flen;	 		//С����λ��
	for(i=0;i<flen;i++)	//�õ�С����������
	{  
		fres+=NMEA_Pow(10,flen-1-i)*(buf[ilen+1+i]-'0');
	} 
	res=ires*NMEA_Pow(10,flen)+fres;
	if(mask&0X02)res=-res;		   
	return res;
}	  							 
//����GPGSV��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void NMEA_GPGSV_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p,*p1,dx;
	uint8_t len,i,j,slx=0;
	uint8_t posx;   	 
	p=buf;
	p1=NMEA_FindSentence(p,"GSV");
	if(p1==0)return;
	posx=NMEA_Comma_Pos(p1,1);
	if(posx==0XFF)return;
	len=NMEA_Str2num(p1+posx,&dx);							//�õ�GPGSV������
	posx=NMEA_Comma_Pos(p1,3); 					//�õ��ɼ���������
	if(posx!=0XFF)gpsx->svnum=NMEA_Str2num(p1+posx,&dx);
	for(i=0;i<len;i++)
	{	 
		p1=NMEA_FindSentence(p,"GSV");
		if(p1==0)break;
		for(j=0;j<4;j++)
		{	  
			posx=NMEA_Comma_Pos(p1,4+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].num=NMEA_Str2num(p1+posx,&dx);	//�õ����Ǳ��?
			else break; 
			posx=NMEA_Comma_Pos(p1,5+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].eledeg=NMEA_Str2num(p1+posx,&dx);//�õ��������� 
			else break;
			posx=NMEA_Comma_Pos(p1,6+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].azideg=NMEA_Str2num(p1+posx,&dx);//�õ����Ƿ�λ��
			else break; 
			posx=NMEA_Comma_Pos(p1,7+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].sn=NMEA_Str2num(p1+posx,&dx);	//�õ����������?
			else break;
			slx++;	   
		}   
 		p=p1+1;//�л�����һ��GPGSV��Ϣ
	}   
}
//����GPGGA��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void NMEA_GPGGA_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p1,dx;			 
	uint8_t posx;    
	p1=NMEA_FindSentence(buf,"GGA");
	if(p1==0)return;
	posx=NMEA_Comma_Pos(p1,6);								//�õ�GPS״̬
	if(posx!=0XFF)gpsx->gpssta=NMEA_Str2num(p1+posx,&dx);	
	posx=NMEA_Comma_Pos(p1,7);								//�õ����ڶ�λ��������
	if(posx!=0XFF)gpsx->posslnum=NMEA_Str2num(p1+posx,&dx); 
	posx=NMEA_Comma_Pos(p1,9);								//�õ����θ߶�
	if(posx!=0XFF)gpsx->altitude=NMEA_Str2num(p1+posx,&dx);  
}
//����GPGSA��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void NMEA_GPGSA_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p1,dx;			 
	uint8_t posx; 
	uint8_t i;   
	p1=NMEA_FindSentence(buf,"GSA");
	if(p1==0)return;
	posx=NMEA_Comma_Pos(p1,2);								//�õ���λ����
	if(posx!=0XFF)gpsx->fixmode=NMEA_Str2num(p1+posx,&dx);	
	for(i=0;i<12;i++)										//�õ���λ���Ǳ��?
	{
		posx=NMEA_Comma_Pos(p1,3+i);					 
		if(posx!=0XFF)gpsx->possl[i]=NMEA_Str2num(p1+posx,&dx);
		else break; 
	}				  
	posx=NMEA_Comma_Pos(p1,15);								//�õ�PDOPλ�þ�������
	if(posx!=0XFF)gpsx->pdop=NMEA_Str2num(p1+posx,&dx);  
	posx=NMEA_Comma_Pos(p1,16);								//�õ�HDOPλ�þ�������
	if(posx!=0XFF)gpsx->hdop=NMEA_Str2num(p1+posx,&dx);  
	posx=NMEA_Comma_Pos(p1,17);								//�õ�VDOPλ�þ�������
	if(posx!=0XFF)gpsx->vdop=NMEA_Str2num(p1+posx,&dx);  
}
//����GPRMC��У��ͣ�����?˵��У��ɹ�?

uint16_t NMEA_GPRMC_CheckSum(uint8_t *buf)
{
	uint8_t checksum = 0;
	uint8_t hi, lo;
	uint8_t *p;

	if (buf == 0 || *buf != '$')
	{
		return 0;
	}

	p = buf + 1;
	while (*p && *p != '*' && *p != '\r' && *p != '\n')
	{
		checksum ^= *p;
		p++;
	}

	if (*p != '*' || *(p + 1) == 0 || *(p + 2) == 0)
	{
		return 0;
	}

	hi = NMEA_HexToNibble(*(p + 1));
	lo = NMEA_HexToNibble(*(p + 2));
	if (hi == 0xFF || lo == 0xFF)
	{
		return 0;
	}

	return (checksum == (uint8_t)((hi << 4) | lo)) ? 1 : 0;
}

//����GPRMC��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void NMEA_GPRMC_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p1,dx;			 
	uint8_t posx;     
	uint32_t temp;	   
	float rs;  
	p1=NMEA_FindSentence(buf,"RMC");
	if(p1==0)return;
	if(NMEA_GPRMC_CheckSum(p1)==0)//У�鲻ͨ���������н�������ʾ
	{
		return;
	}
	posx=NMEA_Comma_Pos(p1,1);								//�õ�UTCʱ��
	if(posx!=0XFF)
	{
		uint32_t raw=(uint32_t)NMEA_Str2num(p1+posx,&dx);
		uint32_t divisor=NMEA_Pow(10,dx);
		temp=raw/divisor;
		gpsx->utc.hour=temp/10000;
		gpsx->utc.min=(temp/100)%100;
		gpsx->utc.sec=temp%100;
		/* Parse milliseconds from fractional seconds (dx decimal digits) */
		uint32_t frac=raw%divisor;
		gpsx->utc.msec=(uint16_t)(dx<=3 ? frac*NMEA_Pow(10,3-dx) : frac/NMEA_Pow(10,dx-3));
	}	
	posx=NMEA_Comma_Pos(p1,3);								//�õ�γ��
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);		 	 
		gpsx->latitude=temp/NMEA_Pow(10,dx+2);	//�õ���
		rs=temp%NMEA_Pow(10,dx+2);				//�õ�'		 
		gpsx->latitude=gpsx->latitude*NMEA_Pow(10,5)+(rs*NMEA_Pow(10,5-dx))/60;//ת��Ϊ�� 
	}
	posx=NMEA_Comma_Pos(p1,4);								//��γ���Ǳ�γ 
	if(posx!=0XFF)gpsx->nshemi=*(p1+posx);					 
 	posx=NMEA_Comma_Pos(p1,5);								//�õ�����
	if(posx!=0XFF)
	{												  
		temp=NMEA_Str2num(p1+posx,&dx);		 	 
		gpsx->longitude=temp/NMEA_Pow(10,dx+2);	//�õ���
		rs=temp%NMEA_Pow(10,dx+2);				//�õ�'		 
		gpsx->longitude=gpsx->longitude*NMEA_Pow(10,5)+(rs*NMEA_Pow(10,5-dx))/60;//ת��Ϊ�� 
	}
	posx=NMEA_Comma_Pos(p1,6);								//������������
	if(posx!=0XFF)gpsx->ewhemi=*(p1+posx);		 
	posx=NMEA_Comma_Pos(p1,9);								//�õ�UTC����
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);		 				//�õ�UTC����
		gpsx->utc.date=temp/10000;
		gpsx->utc.month=(temp/100)%100;
		gpsx->utc.year=2000+temp%100;	 	 
	} 
}
//����GPVTG��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void NMEA_GPVTG_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p1,dx;			 
	uint8_t posx;    
	p1=NMEA_FindSentence(buf,"VTG");
	if(p1==0)return;
	posx=NMEA_Comma_Pos(p1,7);								//�õ���������
	if(posx!=0XFF)
	{
		gpsx->speed=NMEA_Str2num(p1+posx,&dx);
		if(dx<3)gpsx->speed*=NMEA_Pow(10,3-dx);	 	 		//ȷ������1000��
	}
}  
//��ȡNMEA-0183��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void GPS_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	NMEA_GPGGA_Analysis(gpsx,buf);//GPGGA������Ԥ��
	NMEA_GPGSA_Analysis(gpsx,buf);//GPGSA��GN GSA������λģʽ
	NMEA_GPRMC_Analysis(gpsx,buf);//GPRMC��������γ�ȣ�ʱ�����?
	NMEA_GPVTG_Analysis(gpsx,buf);//GPVTG������Ԥ��
}

//GPSУ��ͼ���?
//buf:���ݻ������׵�ַ
//len:���ݳ���
//cka,ckb:����У����.
void Ublox_CheckSum(uint8_t *buf,uint16_t len,uint8_t* cka,uint8_t*ckb)
{
	uint16_t i;
	*cka=0;*ckb=0;
	for(i=0;i<len;i++)
	{
		*cka=*cka+buf[i];
		*ckb=*ckb+*cka;
	}
}
extern volatile uint16_t RX_len;//�����ֽڼ���

/////////////////////////////////////////UBLOX ���ô���/////////////////////////////////////
//���CFG����ִ�����?
//����ֵ:0,ACK�ɹ�
//       1,���ճ�ʱ����
//       2,û���ҵ�ͬ���ַ�
//       3,���յ�NACKӦ��
uint8_t Ublox_Cfg_Ack_Check(void)
{			 
	uint16_t len=0,i;
	uint8_t rval=0;
	while(RX_len==0 && len<100)//�ȴ����յ�Ӧ��   
	{
		len++;
		delay_ms(5);
	}		 
	if(RX_len)
	{
		len=RX_len;
		for(i=0;i<len;i++)if(USART2_RX_BUF[i]==0XB5)break;//����ͬ���ַ� 0XB5
		if(i==len)rval=2;						//û���ҵ�ͬ���ַ�
		else if(USART2_RX_BUF[i+3]==0X00)rval=3;//���յ�NACKӦ��
		else rval=0;	   						//���յ�ACKӦ��
	}else rval=1;
	return rval;  
}
//���ñ���
//����ǰ���ñ������ⲿEEPROM����
//����ֵ:0,ִ�гɹ�;1,ִ��ʧ��.
uint8_t Ublox_Cfg_Cfg_Save(void)
{
	uint8_t i;
	_ublox_cfg_cfg *cfg_cfg=(_ublox_cfg_cfg *)USART2_TX_BUF;
	cfg_cfg->header=0X62B5;		//cfg header
	cfg_cfg->id=0X0906;			//cfg cfg id
	cfg_cfg->dlength=13;		//����������Ϊ13���ֽ�.		 
	cfg_cfg->clearmask=0;		//��������?
	cfg_cfg->savemask=0XFFFF; 	//��������Ϊ0XFFFF
	cfg_cfg->loadmask=0; 		//��������Ϊ0 
	cfg_cfg->devicemask=4; 		//������EEPROM����		 
	Ublox_CheckSum((uint8_t*)(&cfg_cfg->id),sizeof(_ublox_cfg_cfg)-4,&cfg_cfg->cka,&cfg_cfg->ckb);	
	HAL_UART_Transmit(USART2, (uint8_t *)&USART2_TX_BUF, sizeof(_ublox_cfg_cfg));
	
	for(i=0;i<6;i++)if(Ublox_Cfg_Ack_Check()==0)break;		//EEPROMд����Ҫ�ȽϾ�ʱ��,���������ж϶��?
	return i==6?1:0;
}
//����NMEA�����Ϣ���?
//msgid:Ҫ������NMEA��Ϣ��Ŀ,���������Ĳ�����
//      00,GPGGA;01,GPGLL;02,GPGSA;
//		03,GPGSV;04,GPRMC;05,GPVTG;
//		06,GPGRS;07,GPGST;08,GPZDA;
//		09,GPGBS;0A,GPDTM;0D,GPGNS;
//uart1set:0,����ر�?1,�������?	  
//����ֵ:0,ִ�гɹ�;����,ִ��ʧ��.
uint8_t Ublox_Cfg_Msg(uint8_t msgid,uint8_t uart1set)
{
	_ublox_cfg_msg *cfg_msg=(_ublox_cfg_msg *)USART2_TX_BUF;
	cfg_msg->header=0X62B5;		//cfg header
	cfg_msg->id=0X0106;			//cfg msg id
	cfg_msg->dlength=8;			//����������Ϊ8���ֽ�.	
	cfg_msg->msgclass=0XF0;  	//NMEA��Ϣ
	cfg_msg->msgid=msgid; 		//Ҫ������NMEA��Ϣ��Ŀ
	cfg_msg->iicset=1; 			//Ĭ�Ͽ���
	cfg_msg->uart1set=uart1set; //��������
	cfg_msg->uart2set=1; 	 	//Ĭ�Ͽ���
	cfg_msg->usbset=1; 			//Ĭ�Ͽ���
	cfg_msg->spiset=1; 			//Ĭ�Ͽ���
	cfg_msg->ncset=1; 			//Ĭ�Ͽ���	  
	Ublox_CheckSum((uint8_t*)(&cfg_msg->id),sizeof(_ublox_cfg_msg)-4,&cfg_msg->cka,&cfg_msg->ckb);
	HAL_UART_Transmit(USART2, (uint8_t *)&USART2_TX_BUF, sizeof(_ublox_cfg_msg));
	
	return Ublox_Cfg_Ack_Check();
}
//����NMEA�����Ϣ���?
//baudrate:������,4800/9600/19200/38400/57600/115200/230400	  
//����ֵ:0,ִ�гɹ�;����,ִ��ʧ��(���ﲻ�᷵��0��)
uint8_t Ublox_Cfg_Prt(uint32_t baudrate)
{
	_ublox_cfg_prt *cfg_prt=(_ublox_cfg_prt *)USART2_TX_BUF;
	cfg_prt->header=0X62B5;		//cfg header
	cfg_prt->id=0X0006;			//cfg prt id
	cfg_prt->dlength=20;		//����������Ϊ20���ֽ�.	
	cfg_prt->portid=1;			//��������1
	cfg_prt->reserved=0;	 	//�����ֽ�,����Ϊ0
	cfg_prt->txready=0;	 		//TX Ready����Ϊ0
	cfg_prt->mode=0X08D0; 		//8λ,1��ֹͣλ,��У��λ
	cfg_prt->baudrate=baudrate; //����������
	cfg_prt->inprotomask=0X0007;//0+1+2
	cfg_prt->outprotomask=0X0007;//0+1+2
 	cfg_prt->reserved4=0; 		//�����ֽ�,����Ϊ0
 	cfg_prt->reserved5=0; 		//�����ֽ�,����Ϊ0 
	Ublox_CheckSum((uint8_t*)(&cfg_prt->id),sizeof(_ublox_cfg_prt)-4,&cfg_prt->cka,&cfg_prt->ckb);
	HAL_UART_Transmit(USART2, (uint8_t *)&USART2_TX_BUF, sizeof(_ublox_cfg_prt));
	delay_ms(200);				//�ȴ��������?
	USART_BaudRate_Init(baudrate);
	return Ublox_Cfg_Ack_Check();//���ﲻ�ᷴ��0,��ΪUBLOX��������Ӧ���ڴ������³�ʼ����ʱ���Ѿ���������.
} 
//����UBLOX NEO-6��ʱ���������?
//interval:������(us)
//length:�������?us)
//status:��������:1,�ߵ�ƽ��Ч;0,�ر�;-1,�͵�ƽ��Ч.
//����ֵ:0,���ͳɹ�;����,����ʧ��.
uint8_t Ublox_Cfg_Tp(uint32_t interval,uint32_t length,signed char status)
{
	_ublox_cfg_tp *cfg_tp=(_ublox_cfg_tp *)USART2_TX_BUF;
	cfg_tp->header=0X62B5;		//cfg header
	cfg_tp->id=0X0706;			//cfg tp id
	cfg_tp->dlength=20;			//����������Ϊ20���ֽ�.
	cfg_tp->interval=interval;	//������,us
	cfg_tp->length=length;		//�������?us
	cfg_tp->status=status;	   	//ʱ����������
	cfg_tp->timeref=0;			//�ο�UTC ʱ��
	cfg_tp->flags=0;			//flagsΪ0
	cfg_tp->reserved=0;		 	//����λΪ0
	cfg_tp->antdelay=820;    	//������ʱΪ820ns
	cfg_tp->rfdelay=0;    		//RF��ʱΪ0ns
	cfg_tp->userdelay=0;    	//�û���ʱΪ0ns
	Ublox_CheckSum((uint8_t*)(&cfg_tp->id),sizeof(_ublox_cfg_tp)-4,&cfg_tp->cka,&cfg_tp->ckb);
	HAL_UART_Transmit(USART2, (uint8_t *)&USART2_TX_BUF, sizeof(_ublox_cfg_tp));
	return Ublox_Cfg_Ack_Check();
}
//����UBLOX NEO-6�ĸ�������	    
//measrate:����ʱ��������λΪms�����ٲ���С��200ms��5Hz��
//reftime:�ο�ʱ�䣬0=UTC Time��1=GPS Time��һ������Ϊ1��
//����ֵ:0,���ͳɹ�;����,����ʧ��.
uint8_t Ublox_Cfg_Rate(uint16_t measrate,uint8_t reftime)
{
	_ublox_cfg_rate *cfg_rate=(_ublox_cfg_rate *)USART2_TX_BUF;
 	if(measrate<200)return 1;	//С��200ms��ֱ���˳�
 	cfg_rate->header=0X62B5;	//cfg header
	cfg_rate->id=0X0806;	 	//cfg rate id
	cfg_rate->dlength=6;	 	//����������Ϊ6���ֽ�.
	cfg_rate->measrate=measrate;//������,us
	cfg_rate->navrate=1;		//�������ʣ����ڣ����̶�Ϊ1
	cfg_rate->timeref=reftime; 	//�ο�ʱ��ΪGPSʱ��
	Ublox_CheckSum((uint8_t*)(&cfg_rate->id),sizeof(_ublox_cfg_rate)-4,&cfg_rate->cka,&cfg_rate->ckb);
	HAL_UART_Transmit(USART2, (uint8_t *)&USART2_TX_BUF, sizeof(_ublox_cfg_rate));
	return Ublox_Cfg_Ack_Check();
}
nmea_msg gpsx; 	
__align(4) uint8_t dtbuf[50];
// Show parsed GPS location and time on OLED.
void Gps_Msg_Show(void)
{
 	float tp;
	tp=gpsx.longitude;	   
	sprintf((char *)dtbuf,":%.5f %1c   ",tp/=100000,gpsx.ewhemi);
	OLED_ShowCHinese(0,2,5);//��
	OLED_ShowCHinese(16,2,7);//��
	OLED_ShowString(32,2,dtbuf);
	tp=gpsx.latitude;	   
	sprintf((char *)dtbuf,":%.5f %1c   ",tp/=100000,gpsx.nshemi);
	OLED_ShowCHinese(0,4,6);//γ
	OLED_ShowCHinese(16,4,7);//��
	OLED_ShowString(32,4,dtbuf); 	 
	 				    
	/* Replace time display with ACoST avg E2E latency */
	{
		uint32_t avg_x10 = AcostBench_GetAvgX10us();
		uint32_t cnt     = AcostBench_GetCount();
		if (cnt > 0) {
			sprintf((char *)dtbuf, ":%lu.%luus N=%lu   ",
			        (unsigned long)(avg_x10 / 10),
			        (unsigned long)(avg_x10 % 10),
			        (unsigned long)cnt);
		} else {
			sprintf((char *)dtbuf, ":---     N=0    ");
		}
		OLED_ShowCHinese(0,6,8);  /* �?*/
		OLED_ShowString(16,6,dtbuf);
	}
  
}	 

void GpsDataRead(void)
{
		uint16_t i,rxlen;
		if( RX_len)
		{
			rxlen=RX_len;
			RX_len=0;
			for(i=0;i<rxlen;i++)USART2_TX_BUF[i]=USART2_RX_BUF[i];	   
			USART2_TX_BUF[i]=0;
			GPS_Analysis(&gpsx,(uint8_t*)USART2_TX_BUF);
			Gps_Msg_Show();

			/* --- ACoST benchmark: feed every valid GPS fix --- */
			if (gpsx.gpssta >= 1 && gpsx.latitude != 0 && gpsx.longitude != 0)
			{
				/* Convert raw integer (units of 1e-5 degree) to decimal degrees */
				double lon_deg = (double)gpsx.longitude / 100000.0;
				double lat_deg = (double)gpsx.latitude  / 100000.0;
				if (gpsx.ewhemi == 'W') lon_deg = -lon_deg;
				if (gpsx.nshemi == 'S') lat_deg = -lat_deg;
				/* Milliseconds since UTC midnight (including sub-second for 5/10 Hz) */
				uint64_t ts_ms = (uint64_t)gpsx.utc.hour * 3600000UL
				               + (uint64_t)gpsx.utc.min  *   60000UL
				               + (uint64_t)gpsx.utc.sec  *    1000UL
				               + (uint64_t)gpsx.utc.msec;
				AcostBench_Feed(lon_deg, lat_deg, ts_ms);
			}
		}
}





