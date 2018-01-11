
#include <DAVE.h>
#include <math.h>
#include "variables.h"

int XMC_Init();
void XMC_Check();
void wyslij(uint8_t);
void wyslij_noga(struct Noga);
void interpreter_wiadomosci();
void SYS1(void);
int init = 0;
uint16_t  skokz = 0;
void skok()
{
	  if(skokz > 1)
		  stanowisko.stopa.y= 250;
	  else
		  stanowisko.stopa.y =110;
}
int main(void)
{
  DAVE_STATUS_t status;
  status = DAVE_Init();
 while(init == 0);
  Init();
  delay(2500);
  XMC_Check();
  XMC_Init();
  XMC_Check();


  while(1)
  {

	  FK_Circle();
	  //skok();
	  update();
	  Spring();
	  delay(2000);

  }
}

int XMC_Init()
{
	init = 1;
	for(uint8_t i = 0; i < 8; i++)
	{
		txData[1] = 0x10 + i;
		txData[2] = 0xA1;
		txData[3] = EOF;
		wyslij(4);
		delay(300);
	}
	return 0;
}
void XMC_Check()
{
	for(uint8_t i = 0; i < 8; i++)
	{
		DIGITAL_IO_SetOutputHigh(&LED1);
		txData[1] = 0x10 + i;
		txData[2] = 0xA3;
		txData[3] = EOF;
		wyslij(4);
		delay(300);
		DIGITAL_IO_SetOutputLow((&LED1));
	}
}
void eorx()
{

	if(rxByte == SOF)
	{
		rxData[0] = rxByte;
		iterator_wiadomosci = 1;
	}
	else if (rxByte == EOF)
	{
		rxData[iterator_wiadomosci +1] = EOF;
		iterator_wiadomosci++;
		interpreter_wiadomosci();
	}
	else if ( iterator_wiadomosci < 9)
	{
		rxData[iterator_wiadomosci] =rxByte;
		iterator_wiadomosci++;
	}
	UART_Receive(&RS, &rxByte,1);
}
void eotx()
{

}
void interpreter_wiadomosci()
{

	if(rxData[0] == SOF && rxData[1] == adress)
	{
		uint8_t numer_silnika;
		numer_silnika = rxData[2] - 0x10;
		if(rxData[3] == INIT)
			stanowisko.silnik_start[numer_silnika] = rxData[4];
		else if(rxData[3] == CHECK)
		{
			stanowisko.silnik_start[numer_silnika] = rxData[4];
			stanowisko.silnik_numer[numer_silnika] = rxData[5];
		}
		else
		{
			stanowisko.i_net[numer_silnika] =rxData[3] << 8 | rxData[4];
			stanowisko.kat_abs[numer_silnika] = rxData[5] << 24 | rxData[6] << 16 | rxData[7] << 8 | rxData[8];
		}
	}

}

