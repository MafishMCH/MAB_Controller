/*
 * kinematics.h
 *
 *  Created on: 11 Jan 2018
 *      Author: Mafish
 */

void Ik(struct Leg *n);		//Inverse kinematics
void Fk(struct Leg *n);	//Forward kinematics
void Update(struct Leg *n);	//Update floating numbers from raw data from motor drivers
void Trajectory(struct Leg *n);		//calculate trajectory for selected leg

void Ik(struct Leg *n)		//Inverse kinematics for (Leg)
{
	float x = n->foot.x;
	float y = n->foot.y;
	float La = sqrtf(((d/2)-x)*((d/2)-x)+y*y);
	float Lb = sqrtf(((d/2)+x)*((d/2)+x)+y*y);
	float fi1 = (l1*l1+La*La-l2*l2)/(2*l1*La);
	fi1 = acosf(fi1);
	float fi2 = (l1*l1+Lb*Lb-l2*l2)/(2*l1*Lb);
	fi2 = acosf(fi2);
	float psi1 = asinf(((d/2)-x)/La);
	float psi2 = asinf(((d/2)+x)/Lb);

	motors[n->motor_L]. teta = fi1 - psi1;
	motors[n->motor_R]. teta = fi2 - psi2;
}
void Fk(struct Leg *n)		//Forward kinematics for (Leg)
{
	float a = d+l1*(sinf(motors[n->motor_L].ang_abs_rad)+sinf(motors[n->motor_R].ang_abs_rad));
	float b = l1*(cosf(motors[n->motor_R].ang_abs_rad)-cosf(motors[n->motor_L].ang_abs_rad));
	float p = sqrtf(a*a+b*b);
	float fi = atan2f(b,a);
	float theta = asinf(p/(2*l2));
	float gama = (pi/2)-theta;
	float psi = gama+fi;

	n->real_foot.x = -(d/2) - l1*sinf(motors[n->motor_L].ang_abs_rad) + l2*cosf(psi);	//Forward kinematics
	n->real_foot.y = l1*cosf(motors[n->motor_L].ang_abs_rad) + l2*sinf(psi);

	float A = a*cosf(motors[n->motor_L].ang_abs_rad)+b*sinf(motors[n->motor_L].ang_abs_rad);
	float B = sqrtf((a*a+b*b)*(4*l2*l2-p*p));
	float C = a*sinf(motors[n->motor_L].ang_abs_rad)-b*cosf(motors[n->motor_L].ang_abs_rad);
	float D = a*a+b*b;
	float E = a*cosf(motors[n->motor_R].ang_abs_rad)-b*sinf(motors[n->motor_R].ang_abs_rad);
	float F = a*sinf(motors[n->motor_R].ang_abs_rad)+b*cosf(motors[n->motor_R].ang_abs_rad);
  	n->J[0][0] = -l1*l2*sinf(psi)*(-(A/B)+(C/D))-l1*cosf(motors[n->motor_L].ang_abs_rad);
  	n->J[0][1] = -l1*cosf(motors[n->motor_L].ang_abs_rad)+l1*l2*cosf(psi)*(-(A/B)+(C/D));
  	n->J[1][0] = -l1*l2*sinf(psi)*(-(E/B)+(F/D));
  	n->J[1][1] = l1*l2*sinf(psi)*(-(E/B)+(F/D));

  	float mianownik = n->J[0][0]*n->J[1][1] - n->J[0][1]*n->J[1][0];
  	float H = n->J[1][1]*motors[n->motor_L].torque - n->J[0][1]*motors[n->motor_R].torque;
  	float I = n->J[0][0] *motors[n->motor_R].torque - n->J[1][0] * motors[n->motor_L].torque;


  	n->real_speed.x = n->J[0][0] * motors[n->motor_L].predkosc_silnika + n->J[0][1] * motors[n->motor_R].predkosc_silnika;
  	n->real_speed.y = n-> J[1][0] * motors[n->motor_L].predkosc_silnika + n->J[1][1] * motors[n->motor_R].predkosc_silnika;

  	for(uint8_t i = 4; i >0; i--)								//moving eFy_buffer to make space for new value
  		n->eFY_buffer[i] = n->eFY_buffer[i-1];

  	n->eF.x = H/(mianownik*5);				//mnozenie *5 poniewaz tak wyszlo z pomiarów nie wiadomo czemu ale jest dobrze

  	n->eF.y = I/mianownik;

  	n->eFY_buffer[0] = n->eF.y;							//filling buffer with newest data
}
void Update(struct Leg *n)		//Update floating numbers from raw data from motor drivers
{
	for(uint8_t i = 0; i < 8; i++)
	{
		motors[i].ang_abs_poprzedni = motors[i].ang_abs_rad;
		motors[i].ang_abs_poprzedni = motors[i].ang_abs_rad;
		motors[i].ang_abs_rad = (float)motors[i].ang_abs * pi / 32767.0f;;
		motors[i].ang_abs_rad = (float)motors[i].ang_abs  * pi / 32767.0f;;
		motors[i].predkosc_silnika =(motors[i].predkosc_silnika * 0.4f) + ((motors[i].ang_abs_rad - motors[i].ang_abs_poprzedni) / dt * 0.6f);
		motors[i].predkosc_silnika =(motors[i].predkosc_silnika * 0.4f) + ((motors[i].ang_abs_rad - motors[i].ang_abs_poprzedni) / dt * 0.6f);
	}
}
void Trajectory(struct Leg *n)		//calculate trajectory for selected leg
{
	//circle
	n->foot.x = sinf(t) * 80.0f;
	n->foot.y =180.0f + cosf(t) * 80.0f;
	/*if(n->skoki == 1)
		n->foot.y = 280;
	else
	{
		if(n->foot.y > 100)
			n->foot.y -= 2;
	}*/
}
