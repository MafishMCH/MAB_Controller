
#include <DAVE.h>
#include <math.h>
#include "variables.h"
#include "kinematics.h"
#include "communications.h"
#include "dynamics.h"

uint8_t Motors_go_nogo();

int main(void)
{
  DAVE_STATUS_t status;
  status = DAVE_Init();
  while(status == DAVE_STATUS_FAILURE )
  {
	  delay(50000);
	   DIGITAL_IO_ToggleOutput(&LED1);
  }
  delay(250000);
  Init();
 /*
  while(init == 0)							//wait for input from PC
  {
	  delay(500000);
	   DIGITAL_IO_ToggleOutput(&LED1);
  }
  */

  XMC_Init(10);

/*
  while(motors_go == 0)				//Procedure to check if all motors are go
  {
	  XMC_Check();
	  if(Motors_go_nogo() == 0)
		  XMC_Init(10);
  }
*/
  while(1)
  {
	  Ik(&stanowisko);
	  Update(&stanowisko);
	  Send_Leg(&stanowisko);

	 stanowisko.torque[0] = -TorqueFromInet(stanowisko.i_net[0]);
	 stanowisko.torque[1] = TorqueFromInet(stanowisko.i_net[1]);
	 Fk(&stanowisko);

	  t+= dt;

	  DIGITAL_IO_ToggleOutput(&LED1);
	  delay(30000);
  }
}

uint8_t Motors_go_nogo()			//Chceck if all motor are initialized properly and ready to be driven TODO przerobic zeby oblugiwalo wszystkie nogi a nie stanowisko
{
	for(uint8_t i =0; i < 2; i++)
	{
		if(stanowisko.motor_go[i] == 0)
			motors_go = 0;
	}
	if(motors_go == 1)
		return 1;
	else
		return 0;
}
