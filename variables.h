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
float dt = 0.02;				//delta t

//communication vars
uint8_t rxData[10];								//buffer for incoming messages
uint8_t txData[10];								//buffer for outgoing messages
uint8_t adress =0xEE;						//adress of Controller device
uint8_t rxByte = 0;								//temprorary incoming byte holder
uint8_t iterator_wiadomosci = 0;	//message iterator

//other vars
uint8_t is_delay = 1;							//is on active delay
uint8_t init = 0;									//is initiated
uint8_t motors_go = 0;						//are all motors go

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
	uint8_t motor_n[2];						//driver numbers
	uint8_t motor_go[2];					//is motor go
	uint8_t adresy[2];							//adresses
	int16_t i_net[2];								//summarized current
	int32_t ang_abs[2];						//absolute angle of the motor
	float ang_abs_poprzedni[2];		//last absolute angle of the motor
	float predkosc_silnika[2];			//motor speed
	int16_t ks;										//virtal spring strength
	int16_t kd;										//virtual damper strength
	int32_t teta_int[2];							//commanded motor angle as 32bit integer
	int16_t poz_zad[2];						//commanded motor angle as 16bit integer
	struct vec2 foot;							//commanded foot position in milimeters in x-y reference frame
	float teta[2];									//commanded motor angle in radaians
	float ang_abs_rad[2];					//current motor angle in radians
};
struct Leg stanowisko;					//struct for test stand , a single leg


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
	txData[0] = SOF;
	stanowisko.adresy[0] = 0x10;
	stanowisko.adresy[1] = 0x11;
	for(uint8_t i = 0 ; i < 2; i++)
	{
		stanowisko.i_net[i] = 0;
		stanowisko.ang_abs[i] = 0;
		stanowisko.ang_abs_poprzedni[i] = 0;
		stanowisko.predkosc_silnika[i] = 0;
		stanowisko.motor_go[i]  =0;
		stanowisko.motor_n[i] = 0;
	}
	stanowisko.ks = 5050;
	stanowisko.kd = 8;
	stanowisko.foot.x = 0;
	stanowisko.foot.y = 200;
}
void TIMER_IRQ()				//delay interrupt
{
	is_delay = 0;
}
