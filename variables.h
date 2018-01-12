#define SOF 0xCC
#define EOF 0xDD
#define INIT 0xA1
#define CHECK 0xA3


// kinematics vars
float pi = 3.14159f;
float l1 =100;
float l2 =200;
float d =89;
float t = 0.0f;
float dt = 0.02;

//communication vars
uint8_t rxData[10];
uint8_t txData[10];
uint8_t adress =0xEE;
uint8_t rxByte = 0;
uint8_t iterator_wiadomosci = 0;

//other vars
uint8_t is_delay = 1;
uint8_t init = 0;
uint8_t motors_go = 0;

struct vec2
{
	float x;
	float y;
};
struct vec3
{
	float x;
	float y;
	float z;
};
struct Leg
{
	uint8_t silnik_numer[2];
	uint8_t silnik_start[2];
	uint8_t adresy[2];
	int16_t i_net[2];
	int32_t kat_abs[2];
	float kat_abs_poprzedni[2];
	float predkosc_silnika[2];
	int16_t ks;
	int16_t kd;
	int16_t dzielnik;
	int16_t Iq_zadane[2];
	int32_t teta_int[2];
	int16_t poz_zad[2];
	struct vec2 stopa;
	float teta[2];
	float r;
	float r_poprzednie;
	float kat_abs_rad[2];
};
struct Leg stanowisko;


void delay(uint32_t us)
{
	us *= 100;
	is_delay = 1;
	TIMER_SetTimeInterval(&DELAY, us);
	TIMER_Start(&DELAY);
	while(is_delay);
	TIMER_Stop(&DELAY);
}

void Init()
{
	txData[0] = SOF;
	stanowisko.adresy[0] = 0x10;
	stanowisko.adresy[1] = 0x11;
	for(uint8_t i = 0 ; i < 2; i++)
	{
		stanowisko.i_net[i] = 0;
		stanowisko.kat_abs[i] = 0;
		stanowisko.kat_abs_poprzedni[i] = 0;
		stanowisko.predkosc_silnika[i] = 0;
		stanowisko.silnik_start[i]  =0;
		stanowisko.silnik_numer[i] = 0;
		stanowisko.Iq_zadane[i] = 0;
	}
	stanowisko.ks = 5050;
	stanowisko.kd = 8;
	stanowisko.dzielnik = 100;
	stanowisko.stopa.x = 0;
	stanowisko.stopa.y = 200;
}
void TIMER_IRQ()
{
	is_delay = 0;
}
