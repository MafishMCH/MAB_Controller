/*
 * communications.h
 *
 *  Created on: 11 Jan 2018
 *      Author: Mafish
 */

int XMC_Init();
void XMC_Check();
void eorx();
void Message_interpreter();
void Send(uint8_t);
void Send_Leg(struct Leg *n);

uint8_t  XMC_Init(uint8_t n)
{
	if(n == 10)
		for(uint8_t i = 0; i < 8; i++)
		{
			txData[1] = 0x10 + i;
			txData[2] = 0xA1;
			txData[3] = EOF;
			Send(4);
			delay(300);
		}
	else
		txData[1] = 0x10 + n;
		txData[2] = 0xA1;
		txData[3] = EOF;
		Send(4);
		delay(300);

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
		Send(4);
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
		Message_interpreter();
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
void Message_interpreter()
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
void Send_Leg(struct Leg *n)
{
	  txData[1] = n->adresy[0];
	  txData[2] = n->poz_zad[0] >> 8;
	  txData[3] = n->poz_zad[0];
	  txData[4] = EOF;
	  Send(5);
	  delay(500);
	  txData[1] = n->adresy[1];
	  txData[2] = n->poz_zad[1] >> 8;
	  txData[3] = n->poz_zad[1];
	  Send(5);
	  delay(500);
}
void Send(uint8_t size )
{
	UART_Transmit(&RS, txData, size);
	while(UART_IsTxBusy(&RS));
	UART_Receive(&RS, &rxByte, 1);
}
