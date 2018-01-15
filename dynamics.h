/*
 * dynamics.h
 *
 *  Created on: 13 Jan 2018
 *      Author: Mafish
 */

float TorqueFromInet(uint16_t i_net);
uint16_t  VrefFromTorque(float torque);
uint8_t Z_score_filter(float buffer[]);
uint8_t hopper = 0;
float TorqueFromInet(uint16_t i_net)		//Input : I_net[mA] , returns estimated Torque in [Nmm]
{
	float result = 0;
	if(i_net < 22000)
		result = 0.05445f * i_net + 51.66269f;
	else
		result = 0.02978f * i_net + 588.051f;
	return result;
}
uint16_t VrefFromTorque(float torque)		//input Torque [Nmm], returns corresponding V_ref
{
	if(torque < 0)
		torque = -torque;
	if(torque < 1400)
		torque = 2.7541f * torque + 125.34;
	else
		torque =4.8671f * torque - 2698.687f;
	return (uint16_t)torque;
}
uint8_t Z_score_filter(float buffer[5])		//input buffer with collected data, newest data point -- returns 1 if peak was detected, 0 when no
{
	float result = 0;
	float threshhold = 5;
	for(uint8_t i = 0; i < 4; i++)
		result += buffer[i];
	result /= 4;

	if(buffer[0] > result + threshhold)
		return 1;
	else
		return 0;
}
void Reibert_Hopper(struct Leg *n)
{
	float min_y = 110;
	Fk(n);
	//uint8_t touched = Z_score_filter(n->eFY_buffer);		//TODO t jest tylko poglądowe i równowazne z touched , usunac stad t i dac touched
	if(t > 20 || hopper != 0)
	{
		hopper = 1;

		if(n->foot.y < min_y)		//jump
		{
			hopper = 2;
			n->foot.y = 280;
			Ik(n);
			n->ks[0] = 5000;
			n->kd[0] = 0;
			n->ks[1] = 5000;
			n->kd[1] = 0;
		}
		else										//landing
		{
			hopper = 1;
			float dy = n->real_foot.y - n->r0;
			float fy = (n->ksr*dy) - (n->kdr*n->real_speed.y);
			float fx = 0;
			float t1 = n->J[0][0] * fx + n->J[0][1] * fy;
			float t2 = n->J[1][0] * fx + n->J[1][1] * fy;
			float dTeta = n->r0_angle - n->ang_abs_rad[0];
			dTeta = t1 / dTeta;
			//n->ks[0] = dTeta;

			dTeta = n->r0_angle - n->ang_abs_rad[1];
			dTeta = t2 / dTeta;
			//n->ks[1] = dTeta;
			n->teta[0] = n->r0_angle;
			n->teta[1] = n->r0_angle;
		}
	}
	else			//flight
	{
		hopper = 0;
		n->foot.y = 250;
		Ik(n);
		n->ks[0] = 200;
		n->ks[1] = 200;
		n->kd[0] = 1500;
		n->kd[0] = 1500;
	}
}
