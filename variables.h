#define SOF 0xCC
#define EOF 0xDD
#define INIT 0xA1
#define CHECK 0xA3
// kinematyka
float pi = 3.14159f;
float l1 =100;
float l2 =200;
float d =89;
float t = 0.0f;
float dt = 0.02;



//komunikacja
uint8_t rxData[10];
uint8_t txData[10];
uint8_t adress =0xEE;
uint8_t rxByte = 0;
uint8_t iterator_wiadomosci = 0;

uint8_t is_delay = 1;

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
struct Noga
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
struct Noga stanowisko;

void wyslij(uint8_t size )
{
	UART_Transmit(&RS, txData, size);
	while(UART_IsTxBusy(&RS));
	UART_Receive(&RS, &rxByte, 1);

}
void delay(uint32_t us)
{
	us *= 100;
	is_delay = 1;
	TIMER_SetTimeInterval(&DELAY, us);
	TIMER_Start(&DELAY);
	while(is_delay);
	TIMER_Stop(&DELAY);
}
void wyslij_noga(struct Noga n)
{
	  txData[1] = n.adresy[0];
	  txData[2] = n.poz_zad[0] >> 8;
	  txData[3] = n.poz_zad[0];
	  txData[4] = EOF;
	  wyslij(5);
	  delay(500);
	  txData[1] = n.adresy[1];
	  txData[2] = n.poz_zad[1] >> 8;
	  txData[3] = n.poz_zad[1];
	  wyslij(5);
	  delay(500);
}
void Ik(struct Noga *n)
{
	float x = n->stopa.x;
	float y = n->stopa.y;
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
void update()
{
	stanowisko.kat_abs_poprzedni[0] = stanowisko.kat_abs_rad[0];
	stanowisko.kat_abs_poprzedni[1] = stanowisko.kat_abs_rad[1];
	stanowisko.kat_abs_rad[0] = (float)stanowisko.kat_abs[0] * pi / 32767.0f;;
	stanowisko.kat_abs_rad[1] = (float)stanowisko.kat_abs[1]  * pi / 32767.0f;;
	stanowisko.predkosc_silnika[0] =(stanowisko.predkosc_silnika[0] * 0.4f) + ((stanowisko.kat_abs_rad[0] - stanowisko.kat_abs_poprzedni[0]) / dt * 0.6f);
	stanowisko.predkosc_silnika[1] =(stanowisko.predkosc_silnika[0] * 0.4f) + ((stanowisko.kat_abs_rad[1] - stanowisko.kat_abs_poprzedni[1]) / dt * 0.6f);
	t+= dt;
	Ik(&stanowisko);
}
void FK_Circle()
{
	//stanowisko.stopa.x =cosf(t) * 80;
	stanowisko.stopa.y =210 + sinf(t) * 55;

}
void Spring()
{
	stanowisko.poz_zad[0] = stanowisko.teta[0] * INT16_MAX / pi;
	stanowisko.poz_zad[1] = stanowisko.teta[1] * INT16_MAX / pi;

	wyslij_noga(stanowisko);
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
	Ik(&stanowisko);
}
void Raibert_Hopper(struct Noga n)
{
	n.r_poprzednie = n.r;
	n.r = n.stopa.x * n.stopa.x + n.stopa.y * n.stopa.y;
	n.r = sqrtf(n.r);
}
void Fk(struct Noga n)
{
	float a = d + l1*(sinf(n.teta[1]) + sinf(n.teta[0])) ;
	float b = l1 * (cosf(n.teta[1]) - cosf(n.teta[0]));
	float p = sqrtf(a*a + b*b);
	float fi = atan2f(b,a);
	float theta = asinf(p / 2*l2);
	float gamma = pi / 2 - theta;
	float psi = gamma + fi;
	struct vec2 xy;
	xy.x = d/2 - l1*sin(n.teta[0]) + l2*cosf(psi);
	xy.y = l1*cosf(n.teta[0]) + l2* cosf(psi);
	stanowisko.stopa = xy;
}

void Trajektoria()
{
	//x = 30*cosf(t);
	//y = 130 - 30*sinf(t);
}

void TIMER_IRQ()
{
	is_delay = 0;
}
