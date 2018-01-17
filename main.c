
#include <DAVE.h>
#include <math.h>
#include "variables.h"
#include "communications.h"
#include "kinematics.h"

//#include "dynamics.h"

int main(void)
{
  DAVE_STATUS_t status;
  status = DAVE_Init();
  while(status == DAVE_STATUS_FAILURE )
  {
	  delay(50000);
	   DIGITAL_IO_ToggleOutput(&LED1);
  }

  //#####################
  PWM_CCU4_Init(&PWM_CCU4_0);
  PWM_CCU4_Start(&PWM_CCU4_0);
  PWM_CCU4_SetDutyCycle(&PWM_CCU4_0, 1500);

  delay(250000);
  Init();


  while(init == 0)							//wait for input from PC
  {
	  delay(500000);
	   DIGITAL_IO_ToggleOutput(&LED1);
  }

  XMC_Init(10);

  //motors_go();

  while(1)
  {
	  DIGITAL_IO_ToggleOutput(&LED1);
	  for(uint8_t i = 1; i < 8;i++)
	  {
		  motors[i].ks = motors[0].ks;
		  motors[i].kd = motors[0].kd;
	  }
	  CPG();
	  for(uint8_t i =0; i < 4; i++)
	  {
		  /*Legs[i].foot.y = Legs[0].foot.y;
		  if( i == 2 || i == 3)
			  Legs[i].foot.x = -Legs[0].foot.x;
		  else
			  Legs[i].foot.x = Legs[0].foot.x;
		  Ik(&Legs[i]);
		  Update(&Legs[i]);
		  Send_Leg(&Legs[i]);
		  delay(3000);
		  */

	  }

	// stanowisko.torque[0] = TorqueFromInet(stanowisko.i_net[0]);
	// stanowisko.torque[1] = TorqueFromInet(stanowisko.i_net[1]);
	// t = Z_score_filter(stanowisko.eFY_buffer) * 25;
	 //Reibert_Hopper(&stanowisko);
	 //Fk(&stanowisko);
	  t+= dt;

	  DIGITAL_IO_ToggleOutput(&LED1);

  }
}
