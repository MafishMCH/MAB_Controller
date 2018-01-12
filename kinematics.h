/*
 * kinematics.h
 *
 *  Created on: 11 Jan 2018
 *      Author: Mafish
 */

void Ik(struct Leg *n);		//Inverse kinematics
void Fk(struct Leg *n);	//Forward kinematics
void Update(struct Leg *n);	//Update floating numbers from raw data from motor drivers

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

	n->teta[0] = fi1 - psi1;
	n->teta[1] = fi2 - psi2;
}
void Fk(struct Leg *n)		//Forward kinematics for (Leg)
{
	float a = d + l1*(sinf(n->teta[1]) + sinf(n->teta[0])) ;
	float b = l1 * (cosf(n->teta[1]) - cosf(n->teta[0]));
	float p = sqrtf(a*a + b*b);
	float fi = atan2f(b,a);
	float theta = asinf(p / 2*l2);
	float gamma = pi / 2 - theta;
	float psi = gamma + fi;
	struct vec2 xy;
	xy.x = d/2 - l1*sin(n->teta[0]) + l2*cosf(psi);
	xy.y = l1*cosf(n->teta[0]) + l2* cosf(psi);
	n->foot = xy;
}
void Update(struct Leg *n)		//Update floating numbers from raw data from motor drivers
{
	n->ang_abs_poprzedni[0] = n->ang_abs_rad[0];
	n->ang_abs_poprzedni[1] = n->ang_abs_rad[1];
	n->ang_abs_rad[0] = (float)n->ang_abs[0] * pi / 32767.0f;;
	n->ang_abs_rad[1] = (float)n->ang_abs[1]  * pi / 32767.0f;;
	n->predkosc_silnika[0] =(n->predkosc_silnika[0] * 0.4f) + ((n->ang_abs_rad[0] - n->ang_abs_poprzedni[0]) / dt * 0.6f);
	n->predkosc_silnika[1] =(n->predkosc_silnika[0] * 0.4f) + ((n->ang_abs_rad[1] - n->ang_abs_poprzedni[1]) / dt * 0.6f);
	n->poz_zad[0] = n->teta[0] * INT16_MAX / pi;
	n->poz_zad[1] = n->teta[1] * INT16_MAX / pi;
}
