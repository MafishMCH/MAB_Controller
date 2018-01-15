#define SOF 0xCC			//start of message
#define EOF 0xDD			//end of message
#define INIT 0xA1			//init message
#define CHECK 0xA3		//go no go message

// kinematics vars
float pi = 3.14159f;
float l1 =100;					//length of thight
float l2 =200;					//length of tibia
float d =89;					//distance between motors
float t = 0.0f;					//time of execution
float dt = -0.15;				//delta t

//communication vars
uint8_t rxData[10];								//buffer for incoming messages
uint8_t txData[10];								//buffer for outgoing messages
uint8_t adress =0xEE;						//adress of Controller device
uint8_t rxByte = 0;								//temprorary incoming byte holder
uint8_t iterator_wiadomosci = 0;	//message iterator

//other vars
uint8_t is_delay = 1;							//is on active delay
uint8_t init = 0;									//is initiated

struct Motor
{
	uint8_t motor_n;						//driver numbers
	uint8_t is_go;								//is motor go
	uint8_t adress;							//adresses
	int16_t i_net;								//summarized current
	int32_t ang_abs;						//absolute angle of the motor
	float ang_abs_poprzedni;		//last absolute angle of the motor
	float predkosc_silnika;			//motor speed
	float ang_abs_rad;					//current motor angle in radians
	uint16_t poz_zad;							//commanded motor angle as 16bit unsigned integer
	uint16_t ks;										//virtal spring strength
	uint16_t kd;									//virtual damper strength
	float torque;								//measured Torque on the motors
	int32_t teta_int;							//commanded motor angle as 32bit integer
	float teta;									//commanded motor angle in radaians
};
struct Motor motors[8];

struct vec2										//floating point 2d vector
{
	float x;
	float y;
};
struct vec3										//floating point 3d vector
{
	float x;
	float y;
	float z;
};
struct Leg											//structure holding leg parameters
{
	uint8_t motor_L;
	uint8_t motor_R;
	struct vec2 foot;							//commanded foot position in milimeters in x-y reference frame
	struct vec2 real_foot;					//measured foot position in milimeters in x-y reference frame
	struct vec2 real_speed;				//measured speed of the foot in x-y referance frame
	struct vec2 eF;								//estimated Forces on the foot
	float eFY_buffer[5];						// buffer for filtering estimated Y forces
	float r0;											//base virtual spring length for Reiberts Hopper
	float r0_angle;								//motor  angle in radians to achive r0 position for  Reiberts Hopper
	float ksr;											//virtual spring stiffness cooefficient for Reiberts Hopper
	float kdr;										//virtual spring damping cooefficient for Reiberts Hopper
	float J[2][2];									//Jackobian
};
struct Leg Legs[4];


void delay(uint32_t us)					//active delay
{
	us *= 100;
	is_delay = 1;
	TIMER_SetTimeInterval(&DELAY, us);
	TIMER_Start(&DELAY);
	while(is_delay);
	TIMER_Stop(&DELAY);
}

void Init()			//initialization function for test stand
{
	for(uint8_t i = 0; i < 4; i++ )
	{
			motors[i].i_net = 0;
			motors[i].ang_abs = 0;
			motors[i].ang_abs_poprzedni = 0;
			motors[i].predkosc_silnika = 0;
			motors[i].is_go  =0;
			motors[i].motor_n = 0;
			motors[i].ks = 180;
			motors[i].kd = 800;
	}
	struct vec2 zero;	zero.x = 0;	zero.y = 0;

	uint8_t iterator = 0;
	for(uint8_t i = 0; i < 4; i++ )
	{
		Legs[i].motor_L = iterator;
		iterator++;
		Legs[i].motor_R = iterator;
		iterator++;
		Legs[i].eF = zero;
		Legs[i].kdr = 0;
		Legs[i].ksr = 0;
		Legs[i].r0 = 0;
		Legs[i].r0_angle = 0;
		Legs[i].real_foot = zero;
		Legs[i].foot.y = 150;
		Legs[i].foot.x = 40;
		Legs[i].real_speed = zero;
	}
	txData[0] = SOF;
}
void TIMER_IRQ()				//delay interrupt
{
	is_delay = 0;
}
