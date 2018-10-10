/*


 * main.c


 *
 *  Created on: Aug 2018
 *      Author: Lesniowski P. :: Tarnow
 *      lesniowski.pawel@wp.pl
 *
 *      atmega, internal clock
 */


//--------------- BIBLIOTEKI ------------------------
#include 
#include 
#include 
#include
#include
#include
#include
#include 



#define CA1 (1<<PB1) 
#define CA3 (1<<PB3)
#define CA5 (1<<PC2)

#define CA2 (1<<PB2) 
#define CA4 (1<<PC1)
#define CA6 (1<<PC3)

#define ring (1<<PB7)

#define ustaw cyf1=10; cyf2=10; cyf3=10; cyf4=10; cyf5=10; cyf6=10; 
#define data PC5 
#define clk PC4  
#define clk_0 PORTC &= ~(1<<clk) 
#define clk_1 PORTC |= (1<<clk)  
#define data_0 PORTC &= ~(1<<data) 
#define data_1 PORTC |= (1<<data)  

#define home (1<<PD4)                  

#define limit (1<<PD2)                 

#define prawo (1<<PD3)                 
#define lewo (1<<PD0)				   
#define kasuj (1<<PB7)                 

#define impulse (1<<PD5)               
#define NULL 0						  
#define DEFAULT 0

#define SPEED     


void init_send(void); 					
void led_init(void);  					
void sendbajt (uint8_t bajt); 			
void timer1_init(void); 				

void engine(bool dir);				
uint8_t set_start(uint8_t button);		
uint8_t stop(uint8_t button2);			
uint8_t left(uint8_t lewy);      		
uint8_t right(uint8_t prawy);			

void krok_01_lewo(void);				
void krok_1_lewo(void);					
void krok_01_prawo(void);				
void krok_1_prawo(void);				

void counter(void);						
void limit2(void);						
void limit3(void);						
void limit4(void);						
void limit5(void);						
void limit6(void);						

void sound(void);						
void sound2(void);						

void SuperDebounce(uint8_t * key_state, volatile uint8_t *KPIN, 
				   uint8_t key_mask, uint16_t rep_time, uint16_t rep_wait,
				   void (*push_proc)(void), void (*rep_proc)(void) );



volatile uint16_t Timer1, Timer2;	


//volatile uint8_t s1_flag;	
//volatile uint8_t sekundy;	
//volatile uint8_t zegar;
static uint8_t wysw_on;     

volatile uint8_t s1_flag;	
//volatile uint8_t sekundy;	

bool kierunek_lewo = 0;  
bool kierunek_prawo = 1;	

bool flaga;  		
bool flaga2;		
bool flaga3;		


uint8_t block;				
uint16_t max = 2400;			
uint8_t min = 240;			
uint16_t i;					
uint8_t k1, k2;				
uint16_t liczba;

bool block_add;				
bool block_add2;			
bool block_sub;				
bool block_sub2;			

int16_t cyf1, cyf2, cyf3, cyf4, cyf5, cyf6; 

uint8_t cyfryn[11] = {17, 125, 35, 41, 77, 137, 129, 29, 1, 9, 239}; 
uint8_t cyfryp[11] = {136, 235, 76, 73, 43, 25, 24, 139, 8, 9, 127}; 



int main(void)
{
	
	led_init();			
	init_send();		
	timer1_init();		


	DDRD &= ~(home);		
	PORTD |= home;		
	DDRD &= ~(limit);		
	PORTD |= limit;		

	DDRD |= impulse;	
	DDRB |= ring;		

	DDRD &= ~(lewo) | ~(prawo);	
	PORTD |= (lewo) | (prawo);	

	DDRB |= (1<<PB6);           
	wysw_on=1;					
	//sekundy=1;					
	flaga=0;					
	block=1;					
	flaga2=0;					
	flaga3=0;
	s1_flag=1;
	liczba=0;
	cyf6=0; cyf5=0; cyf4=0; cyf3=0; cyf2=0; cyf1=0; 

	sei();                                          
	// ****** pêtla g³ówna programu  *********
	while(1)
	{

		counter();				
		if( set_start(home))  {  }
		else if( flaga!=0)  { engine(); SPEED; sound2(); } 

		else if (right(prawo)) { block=1; } 		

		else { _delay_ms(1); flaga=0; flaga2=0; PORTB &= ~(ring); /*cyf2=0; cyf3=0; cyf4=0; cyf5=0; cyf6=0; */ } 

		SuperDebounce(&k1, &PIND, lewo, DEFAULT, DEFAULT, krok_01_lewo, krok_1_lewo );			
		SuperDebounce(&k2, &PIND, prawo, DEFAULT, DEFAULT, krok_01_prawo, krok_1_prawo );		

	}
}

// **************** DEFINICJE FUNKCJI ******************************

void init_send(void) {
	DDRC |= (1<<data) | (1<<clk); //ustaw piny jako wyjœciowe
}
// -----------------------------------------------------------------
void led_init(void)
{

	DDRB |= (CA1) | (CA3); //piny ustaw jako wyjœciowe -> cyfryn
	DDRC |= (CA5);

	DDRC |= (CA4) | (CA6); //piny ustaw jako wyjœciowe -> cyfryp
	DDRB |= (CA2) | (CA3);

	PORTB |= (CA1); PORTB |= (CA3); PORTC |= (CA5); PORTC |= (CA4) | (CA6); PORTB |= (CA2); 
}
// -----------------------------------------------------------------
void sendbajt (uint8_t bajt) {  			
	uint8_t cnt = 0x80;						

	clk_0;									
	while(cnt) {							
		if(bajt & cnt) data_1;				
		else data_0;						
		clk_1;								
		clk_0;								
		cnt>>=1;							
	}

	_delay_us(10);							
}
// -----------------------------------------------------------------
void timer1_init() {
	//timer
	TCCR1B |= (1<<WGM12); //CTC
	TCCR1B |= (1<<CS11) | (1<<CS10); 

	OCR1A = 125; 
	TIMSK |= (1<<OCIE1A);
}
// -----------------------------------------------------------------
ISR (TIMER1_COMPA_vect) { 

	static uint8_t licznik=0;	

	// ****** OBS£UGA MULTIPLEKSERA DLA WYŒWIETLACZY, W£¥CZA PO KOLEI TYLKO JEDEN, OD PRAWEJ STRONY ZACZYNAJ¥C  ******
	if (wysw_on==1) { PORTB &= ~(CA1); PORTB |= (CA3); PORTC |= (CA5); PORTC |= (CA4) | (CA6); PORTB |= (CA2); sendbajt(cyfryn[cyf1]); } 
	else if (wysw_on==2) { PORTC &= ~(CA6);  PORTB |= (CA1); PORTB |= (CA3); PORTC |= (CA5); PORTC |= (CA4); PORTB |= (CA2); sendbajt(cyfryp[cyf2]-8); } 
	else if(wysw_on==4) { PORTC |= (CA6); * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  PORTB |= (CA2); sendbajt(cyfryn[cyf3]); }
	else if(wysw_on==8) { PORTC |= (CA6); * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * PORTB |= (CA2); sendbajt(cyfryp[cyf4]); }
	else if(wysw_on==16) { PORTC |= (CA4); * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * PORTC |= (CA5); sendbajt(cyfryn[cyf5]); }
	else if(wysw_on==32) { PORTB |= (CA3); * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * PORTC |= (CA6); sendbajt(cyfryp[cyf6]); }

		wysw_on <<= 1;					
		if (wysw_on>32) wysw_on = 1;		

		uint16_t n;
		n = Timer1;		
		if (n) Timer1 = --n;
		n = Timer2;		
		if (n) Timer2 = --n;

		if(licznik>50) {	
		s1_flag++;
		licznik=0;
		//sekundy++;	
		//if(sekundy>250)  { sekundy = 0; }
		if(s1_flag>10) s1_flag=1;
				}
		++licznik;

}
// -----------------------------------------------------------------
uint8_t set_start(uint8_t button)			
{
	if( ! (PIND & button ))					
	{
		_delay_ms(5);				
		if( !(PIND & button) ) flaga=1;  return 1;   	

	}

	return 0;	
}
// -----------------------------------------------------------------
uint8_t stop(uint8_t button2)			
{
	if( ! (PIND & button2 ))					
	{
		_delay_us(100);				
		if( !(PIND & button2) ) flaga3=1;  block=0; cyf6=0; cyf5=0; cyf4=0; cyf3=0; cyf2=0; cyf1=0; flaga=0; return 1;   	

	}

	return 0;	
}
// -----------------------------------------------------------------
uint8_t left(uint8_t lewy)			
{
	if( ! (PIND & lewy ))					
	{
		_delay_ms(5);				
		if( !(PIND & lewy) ) flaga2=1; flaga3=0; return 1;   	

	}
	return 0;	// jeœli nie wciœniêty klawisz, zakoñcz funkcjê, rezultat = 0
}
// -----------------------------------------------------------------
uint8_t right(uint8_t prawy)		
{
	if( ! (PIND & prawy ))					
	{
		_delay_ms(5);				// czas drgañ styków
		if( !(PIND & prawy) ) flaga2=1; flaga3=1;  return 1;   	// flaga2=1; jeœli wciœniêty?  zakoñcz funkcjê - rezultat = 1

	}
	return 0;	
}
// -----------------------------------------------------------------
void krok_1_lewo(void) {		
	if(block_add!=true) {		
	do {
	for(i=1; i<=max; i++) { engine(); SPEED; sound2(); } i=0; right(prawo); cyf2++;  counter(); limit2(); limit3(); limit4(); } while(!flaga3); 
	}
	flaga2=0;				
	flaga3=0;
	block_sub=false;		
}
// -----------------------------------------------------------------
void krok_01_lewo(void) {		
	if(block_add2!=true) {		
	for(i=1; i<=min; i++) { engine(); SPEED; sound(); } i=0;
	 cyf1++; counter();}
	flaga2=0;				
	block_sub2=false;		
}
// -----------------------------------------------------------------
void krok_1_prawo(void) {		
	if(block_sub!=true) {
	do {
	for(i=1; i<=max; i++) { engine(); SPEED; sound2();} i=0; left(lewo); cyf2--; counter();  limit6(); } while(flaga3); 
	}
	block=1;				
	flaga2=0;

	block_add=false;
}
// -----------------------------------------------------------------
void krok_01_prawo(void) {		
	if(block_sub2!=true) {
	for(i=1; i<=min; i++) { engine(); SPEED; sound();} i=0;
	cyf1--; counter();}
	block=1;				
	flaga2=0;
	block_add2=false;
}
// -----------------------------------------------------------------

