//#include <STC15W4K48S4.h>	//STC15W4K48S4 ��Ƭ��ͷ�ļ�
#include "LT8910.h"			//����2.4Gģ��
uchar RegH;
uchar RegL;

/*LT8910�˿ڶ���*/
sbit RST_LT8910 = P1 ^ 2; //��RST_n Ϊ��ʱ��������оƬ���Ĵ������ظ���λֵ
sbit SPI_ENA_LT8910 = P1 ^ 1; //ʹ��SPI �źţ��͵�ƽ��Ч��Ҳ����ʹоƬ����sleep mode
sbit SPI_CLK_LT8910 = P1 ^ 5; //SPI/I2C ʱ�������
sbit MOSI_LT8910 = P1 ^ 3; //data �����
sbit MISO_LT8910 = P1 ^ 4; //data �����
sbit PKT_LT8910 = P3 ^ 2; //����/����״̬��־λ ��ͨ������Ϊ�߻����Ч

/**
 * us����ʱ����
 *
 * @author KingBoy (2018/5/20)
 *
 * @param n us
 */
void Delay_us(uint n)
{
	for (; n > 0; n--);
}
/**
 * д�Ĵ�����spiʹ�ܣ����ͣ���data���ɸ�λ����λ���͵�ַ������
 * ��1�ֽڵ�ַ��1�ֽڸ�8λ���ݣ�1�ֽڵ�8λ���ݣ�spiʹ�ܹرգ����ߣ�
 * �½��ز���
 *
 * @author KingBoy (2018/5/20)
 *
 * @param addr д��ĵ�ַ
 * @param H ���ֽ�
 * @param L ���ֽ�
 */
void SPI_WriteReg(uchar addr, uchar H, uchar L)
{
	int i;
	SPI_ENA_LT8910 = 0;
	for (i = 0; i < 8; ++i)
	{
		MOSI_LT8910 = addr & 0x80;
		SPI_CLK_LT8910 = 1;
		SPI_CLK_LT8910 = 0;
		addr = addr << 1;
	}
	for (i = 0; i < 8; ++i)
	{
		MOSI_LT8910 = H & 0x80;
		SPI_CLK_LT8910 = 1;
		SPI_CLK_LT8910 = 0;
		H = H << 1;
	}
	for (i = 0; i < 8; ++i)
	{
		MOSI_LT8910 = L & 0x80;
		SPI_CLK_LT8910 = 1;
		SPI_CLK_LT8910 = 0;
		L = L << 1;
	}
	SPI_ENA_LT8910 = 1;
}
/**
 * ���Ĵ�����spiʹ�ܣ����ͣ���data���ɸ�λ����λ���͵�ַ
 * �ȶ����ֽڣ��ɵ�����8λ���ٶ����ֽڣ��ɵ�����8λ��spiʹ�ܹرգ����ߣ�
 * �½��ز���
 *
 * @author KingBoy (2018/5/20)
 *
 * @param addr Ҫ��ȡ�ĵ�ַ
 */
void SPI_ReadReg(uchar addr)
{
	int i;
	SPI_ENA_LT8910 = 0;
	addr += 0x80;
	for (i = 0; i < 8; ++i)
	{
		MOSI_LT8910 = addr & 0x80;
		SPI_CLK_LT8910 = 1;
		SPI_CLK_LT8910 = 0;
		addr = addr << 1;
	}
	for (i = 0; i < 8; ++i)
	{
		SPI_CLK_LT8910 = 1;
		SPI_CLK_LT8910 = 0;
		RegH = RegH << 1;
		RegH |= MISO_LT8910;
	}
	for (i = 0; i < 8; ++i)
	{
		SPI_CLK_LT8910 = 1;
		SPI_CLK_LT8910 = 0;
		RegL = RegL << 1;
		RegL |= MISO_LT8910;
	}
	SPI_ENA_LT8910 = 1;
}
/**
 * LT8910��ʼ��
 *
 * @author KingBoy (2018/5/20)
 *
 * @param void
 */
void LT8910_Init(void)
{
	RST_LT8910 = 0;       //��λ ����
	Delay_us(500); //��ʱ 500΢��
	RST_LT8910 = 1;       //��λ ����
	Delay_us(500); //��ʱ 500΢��
	PKT_LT8910 = 1;       //�շ� ��־λ ����
	SPI_WriteReg(0, 0x6f, 0xef);  //
	SPI_WriteReg(1, 0x56, 0x81);  //
	SPI_WriteReg(2, 0x66, 0x17);  //
	SPI_WriteReg(4, 0x9c, 0xc9);  //
	SPI_WriteReg(5, 0x66, 0x37);  //
	SPI_WriteReg(7, 0x00, 0x00);  //�趨RF Ƶ��������Ƶ��Ϊ��f=2402+ RF_PLL_CH_NO
	SPI_WriteReg(8, 0x6c, 0x90);  //
	SPI_WriteReg(9, 0x48, 0x00);  //PA ������� -12.2dbm
	SPI_WriteReg(10, 0x7f, 0xfd);  //��������
	SPI_WriteReg(11, 0x00, 0x08);  //RSSI
	SPI_WriteReg(12, 0x00, 0x00);  //У׼VCO
	SPI_WriteReg(13, 0x48, 0xbd);  //
	SPI_WriteReg(22, 0x00, 0xff);  //
	SPI_WriteReg(23, 0x80, 0x05);  //
	SPI_WriteReg(24, 0x00, 0x67);  //
	SPI_WriteReg(25, 0x16, 0x59);  //
	SPI_WriteReg(26, 0x19, 0xe0);  //
	SPI_WriteReg(27, 0x13, 0x00);  //
	SPI_WriteReg(28, 0x18, 0x00);  //
	SPI_WriteReg(32, 0x58, 0x00);  //LEN(��Ҫ����)
	SPI_WriteReg(33, 0x3f, 0xc7);  //
	SPI_WriteReg(34, 0x20, 0x00);  //
	SPI_WriteReg(35, 0x0a, 0x00);  //�ط�����Ϊ9�� һ������10��
	SPI_WriteReg(36, 0x00, 0x01);  //SYNC_WORD[15:0]
	SPI_WriteReg(37, 0x00, 0x01);  //SYNC_WORD[31:16]
	SPI_WriteReg(38, 0x00, 0x01);  //SYNC_WORD[47:32]
	SPI_WriteReg(39, 0x00, 0x01);  //SYNC_WORD[63:48]
	SPI_WriteReg(40, 0x44, 0x02);  //FIFO ��ֵ
	SPI_WriteReg(41, 0xb8, 0x00);  //CRC=ON;scramble=OFF;AUTO_ACK=ON
	SPI_WriteReg(42, 0xfd, 0xff);  //RSSI ɨ����ŵ�����
	SPI_WriteReg(43, 0x00, 0x0f);  //��ʼɨ��RSSI
								   //SPI_WriteReg(44, 0x10, 0x00);  //
								   //SPI_WriteReg(45, 0x05, 0x52);  //
}
/**
 * 2.4G����ģ��LT8910����Ϊ����ģʽ
 * ���˻�
 *
 * @author KingBoy (2018/5/20)
 *
 * @param void
 */
void RX_model(void)
{
	SPI_WriteReg(52, 0x80, 0x80); //���RX FIFO,TX FIFO
	SPI_WriteReg(7, 0x00, 0x80 + 0x20); //RX״̬��freq2434
	Delay_us(200);
}

/**
 * ��������
 *
 * @author KingBoy (2018/5/20)
 *
 * @param rx_buf ���ջ���
 * @param Num ���ݸ���(ushort?)
 */
void ReceiveRC_Data(uchar *rx_buf, uchar Num)
{
	uchar i = 0;
	SPI_ReadReg(50);
	if (RegH == Num && RegL == 0)
	{
		while (i < Num)
		{
			SPI_ReadReg(50);
			rx_buf[i++] = RegH;
			rx_buf[i++] = RegL;
		}
	}
}
/**
 * ͨѶУ�麯��
 *
 * @author KingBoy (2018/10/8)
 *
 * @param buf ������У�������
 * @param len ������У������鳤��
 * @param mac ��Ҫ�ȶԵ�У��ֵ
 *
 * @return int 0 Ϊһ�£���0Ϊ�����У����
 */
int MAC_calc(unsigned char * buff,int len, unsigned char mac)
{
	int i = 0;
	unsigned char mac_calc = 0;
	for (i = 0;i<len;i++)
	{
		mac_calc^=(*buff++);
	}
	if (mac == mac_calc)
	{
		return 0;
	}
	else
	{
		if (mac_calc !=0)
		{
			return ((int)mac_calc);
		}
		else
		{
			return -1;
		}
	}
}
