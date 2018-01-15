/*
 * communications.h
 *
 *  Created on: 11 Jan 2018
 *      Author: Mafish
 */

uint8_t  XMC_Init(uint8_t);			//initialize driver (n). n=10 for all drivers
void XMC_Check();						//check state all drivers
void eorx();									//end of receive interrupt
void Message_interpreter();		//read last message
void Send(uint8_t);						// Send (n) bytes of data via half-duplex uart
void Send_Leg(struct Leg *n);	//Send regular command string to both of legs drivers

uint8_t  XMC_Init(uint8_t n)	//initialize driver (n). n=10 for all drivers
{
	if(n == 10)							//if n = 10 then init all drivers
		for(uint8_t i = 0; i < 8; i++)
		{
			txData[1] = 0x10 + i;
			txData[2] = 0xA1;
			txData[3] = EOF;
			Send(4);
			delay(1100);
		}
	else										//init single driver (n)
		txData[1] = 0x10 + n;
		txData[2] = 0xA1;
		txData[3] = EOF;
		Send(4);
		delay(1100);

	return 0;
}
void XMC_Check()	//check state all drivers
{
	for(uint8_t i = 0; i < 8; i++)		//iterate to check state of all drivers
	{
		DIGITAL_IO_SetOutputHigh(&LED1);
		txData[1] = 0x10 + i;
		txData[2] = 0xA3;
		txData[3] = EOF;
		Send(4);
		delay(1100);
		DIGITAL_IO_SetOutputLow((&LED1));
	}
}
void eorx()		//end of receive interrupt
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
void Message_interpreter()		//read last message
{

	if(rxData[0] == SOF && rxData[1] == adress)
	{
		uint8_t numer_silnika;
		numer_silnika = rxData[2] - 0x10;
		if(rxData[3] == INIT)
			motors[numer_silnika].is_go = rxData[4];
		else if(rxData[3] == CHECK)
		{
			motors[numer_silnika].is_go = rxData[4];
			motors[numer_silnika].motor_n = rxData[5];
		}
		else
		{
			motors[numer_silnika].i_net =rxData[3] << 8 | rxData[4];
			motors[numer_silnika].ang_abs = rxData[5] << 24 | rxData[6] << 16 | rxData[7] << 8 | rxData[8];
		}
	}

}
void Send_Leg(struct Leg *n)	//Send regular command string to both of legs drivers
{
	motors[n->motor_L].poz_zad = motors[n->motor_L].teta * INT16_MAX / pi;
	motors[n->motor_L].poz_zad = motors[n->motor_L].teta * INT16_MAX / pi;
	 txData[1] = motors[n->motor_L].adress;
	 txData[2] = motors[n->motor_L].poz_zad >> 8;
	 txData[3] = motors[n->motor_L].poz_zad;
	 txData[4] = motors[n->motor_L].ks >> 8;
	 txData[5] = motors[n->motor_L].ks;
	 txData[6] = motors[n->motor_L].kd >>8;
	 txData[7] = motors[n->motor_L].kd;
	 txData[8] = EOF;
	 Send(9);
	 delay(1100);
	 txData[1] = motors[n->motor_R].adress;
	 txData[2] = motors[n->motor_R].poz_zad >> 8;
	 txData[3] = motors[n->motor_R].poz_zad;
	 txData[4] = motors[n->motor_R].ks >> 8;
	 txData[5] = motors[n->motor_R].ks;
	 txData[6] = motors[n->motor_R].kd >>8;
	 txData[7] = motors[n->motor_R].kd;
	 txData[8] = EOF;
	 Send(9);
	 delay(1100);
}
void Send(uint8_t size )	// Send (n) bytes of data via half-duplex uart
{

	UART_Transmit(&RS, txData, size);
	while(UART_IsTxBusy(&RS));
	UART_Receive(&RS, &rxByte, 1);
}
