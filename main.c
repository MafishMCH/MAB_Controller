
#include <DAVE.h>
#include <math.h>
#include "variables.h"
#include "kinematics.h"
#include "communications.h"

uint8_t Motors_go_nogo();

int main(void)
{
  DAVE_STATUS_t status;
  status = DAVE_Init();
  while(status == DAVE_STATUS_FAILURE )
  {
	  delay(100000);
	   DIGITAL_IO_ToggleOutput(&LED1);
  }

  while(init == 0);
  Init();
  delay(2500);
  XMC_Check();
  XMC_Init();
  XMC_Check();


  while(1)
  {
	  update(&stanowisko);
	  Spring(&stanowisko);
	  Send_Leg(&stanowisko);
	  t+= dt;
	  delay(2000);
  }
}

uint8_t Motors_go_nogo()
{
	for(uint8_t i =0; i < 2; i++)
	{
		if(stanowisko.silnik_start[i] == 0)
			motors_go = 0;
	}
	if(motors_go == 1)
		return 1;
	else
		return 0;
}
