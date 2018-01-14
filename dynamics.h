/*
 * dynamics.h
 *
 *  Created on: 13 Jan 2018
 *      Author: Mafish
 */

float TorqueFromInet(uint16_t i_net);
uint16_t  VrefFromTorque(float torque);

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
