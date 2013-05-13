// Lab 3 for ECE 4760, Spring 2012, Cornell University
// Written by William Myers (wgm37) and Guo Jie Chin (gc348), Feb 26, 2012
// parameter
// Port Description
// B.3 - Waveform Output
// D.[0-7] - Keypad
// state enumeration of parameters:
// tau_fall - 1
// tau_rise - 2
// fm_tau - 3
// depth - 4
// freq - 5


#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>
#include <math.h>
#include "uart.h"
#include "lcd_lib.h"

// Button press macros
#define button_count 16
#define maxkeys 16
#define PORTDIR DDRA
#define PORTDATA PORTA
#define PORTIN PINA

// Button Push Defines
#define NoPush 1 
#define MaybePush 2
#define Pushed 3
#define MaybeNoPush 4

#define voice_count 2	// total voices
#define max_amp 32767	// initializer for differential exponential equations

// Random Number
#define bit30 0x4000
#define bit27 0x0800

// Note values (hz)
#define note0 262		// C4
#define note1 294		// D4
#define note2 330		// E4
#define note3 392		// G4
#define note4 440		// A4
#define note5 523		// C5
#define note6 587		// D5
#define note7 659		// E5

// button macros
#define togglev 4
#define togglep 8
#define add 12


//flash storage for LCD
const int8_t LCD_voice1[] PROGMEM = "Voice 1:    \0";
const int8_t LCD_voice2[] PROGMEM = "Voice 2:    \0";
const int8_t LCD_tau_fall[] PROGMEM = "Tau Fall:        \0";
const int8_t LCD_tau_rise[] PROGMEM = "Tau Rise:        \0";
const int8_t LCD_fm_tau_fall[] PROGMEM = "FM Tau:          \0";
const int8_t LCD_depth[] PROGMEM = "Depth:           \0";
const int8_t LCD_fm_increment[] PROGMEM = "FM Freq:         \0";
const int8_t LCD_matrix[] PROGMEM = "Matrix:         \0";

//Global const
const unsigned char note_matrix[4][8][8] = {{{0,128,0,0,0,0,0,0},   		// Scale
											{0,0,128,0,0,0,0,0},
											{0,0,0,128,0,0,0,0},
											{0,0,0,0,128,0,0,0},
											{0,0,0,0,0,128,0,0},
											{0,0,0,0,0,0,128,0},
											{0,0,0,0,0,0,0,128},
											{128,0,0,0,0,0,0,0}},
									
											{{64,32,16,8,4,2,1,1},			// s = .5
											{26,51,26,13,6,3,2,1},
											{12,24,46,24,12,6,3,1},
											{6,11,23,45,23,11,6,3},
											{3,6,11,23,45,23,11,6},
											{1,3,6,12,24,46,24,12},
											{1,2,3,6,13,26,51,26},
											{1,1,2,4,8,16,32,64}},
									
											{{23,20,18,16,15,13,12,11},		// s = .9
											{19,21,19,17,15,14,12,11},
											{16,18,20,18,16,15,13,12},
											{14,16,18,19,18,16,14,13},
											{13,14,16,18,19,18,16,14},
											{12,13,15,16,18,20,18,16},
											{11,12,14,15,17,19,21,19},
											{11,12,13,15,16,18,20,23}},
									
											{{0,0,0,0,0,0,0,128},
											{128,0,0,0,0,0,0,0},
											{0,128,0,0,0,0,0,0},
											{0,0,128,0,0,0,0,0},
											{0,0,0,128,0,0,0,0},
											{0,0,0,0,128,0,0,0},
											{0,0,0,0,0,128,0,0},
											{0,0,0,0,0,0,128,0}}};
									
									
// Program Functions
void initialize(void);
void init_lcd(void);
void check_button(int i, unsigned char button_phys); 
unsigned char get_button(void);
void update_LCD(void);

char random_gen(void);
void update_notes(void);
void select_voice(void);

// Generally nice variables to have around
int8_t lcd_buffer[17];
unsigned char PushFlag[button_count] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};		// message indicating a button push for 2 buttons
unsigned char PushState[button_count] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};		// state machine for 2 buttons
unsigned int i;										// used for button checking
unsigned char butnum;								// decoded button number
unsigned char button_phys;							// storage for physical button press
unsigned char key; 									// raw keyscan
unsigned char init;									// useful for state initialization
unsigned char voice_state;							// select which voice we're editing
unsigned char param_state;							// select which param we're editing
unsigned char current_voice;							// which voice we're currently editing
unsigned char current_param;						// which param we're currently selecting

// Project Specific Variables
signed char sineTable[256];							// DDS Sine Table
volatile unsigned char needs_next;					// flag to generate next frequencies
volatile unsigned char n;								// voice indexing
volatile unsigned char in;							// interrupt n
unsigned char notes[voice_count];			// what notes are currently being played on each voice (0-7)
unsigned char voice_note[voice_count];		// which note matrix to use with each voice <-- User Defined -->
volatile unsigned char summed_wave;					// wave summing variable
volatile unsigned char tau_fall[voice_count];			// time constant for main fall			<-- User Defined -->
volatile unsigned char tau_rise[voice_count];			// time constant for main rise			<-- User Defined -->
volatile unsigned char fm_tau_fall[voice_count];		// time constant for fm fall				<-- User Defined -->
volatile unsigned char depth[voice_count];			// depth of fm modulation 				<-- User Defined -->
volatile unsigned int accumulator[voice_count];		// phase accumulator for main sine wave
volatile unsigned int fm_accumulator[voice_count];	// phase accumulator for fm sine wave
volatile unsigned int increment[voice_count];			// phase increment for main sine 
volatile unsigned int next_increment[voice_count];	// next increments to load
volatile unsigned int fm_increment[voice_count];		// phase increment for fm sine 			<-- User Defined -->
unsigned int freq_table[8];							// preloaded frequencies
volatile unsigned int amp[voice_count];				// amplitude of rise/fall envelopes for main wave 
volatile unsigned int fm_amp[voice_count];			// amplitude of fall envelope for main wave 
volatile unsigned int amp_fall[voice_count];			// exponential fall value 
volatile unsigned int amp_rise_temp[voice_count];		// exponential rise value (e^-t/T)
volatile unsigned int amp_rise[voice_count];			// final exponential rise value (1-e^-t/t)
volatile unsigned int time;							// used to recalculate exponentials
volatile signed char fm[voice_count];				// fm sine value
volatile signed char output[voice_count];			// output sine value


unsigned char offset;								// mucking with probabities
unsigned char key_press;
unsigned char got_push;								// temp char for string stuff

// Random Number Generation
char bit0, bit1, p;									// some chars for generation
unsigned long noise_gen;								// must seed this value!

// Key pad scan table
unsigned char keytbl[16]={0xee, 0xed, 0xeb, 0xe7, 
						  0xde, 0xdd, 0xdb, 0xd7, 
						  0xbe, 0xbd, 0xbb, 0xb7, 
						  0x7e, 0x7d, 0x7b, 0x77};

FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

//timer 1 compare ISR - fires every 125us
ISR (TIMER1_COMPA_vect)
{   


	TCNT2 = 0; TCCR2B = 2;

	summed_wave = 0;
	for(in = 0; in < voice_count; in++){
		
		// every 256 entries, recalculate exponentials!
		if ((time & 0x00ff) == 0){
			amp_fall[in] = amp_fall[in] - (amp_fall[in] >> tau_fall[in]);	// calculate exponential
			amp_rise_temp[in] = amp_rise_temp[in] - (amp_rise_temp[in] >> tau_rise[in]);	// calculate exponential 
			fm_amp[in] = fm_amp[in] - (fm_amp[in] >> fm_tau_fall[in]);			// FM ampltide calc
		}
		amp_rise[in] = max_amp - amp_rise_temp[in];						// calculate acutal rise time	
		amp[in] = (amp_rise[in] >> 8) * (amp_fall[in] >> 8);				// and find final amplitude	

		// FM DDS
		fm_accumulator[in] = fm_accumulator[in] + fm_increment[in];			// step the FM phase
		fm[in] = sineTable[(char)(fm_accumulator[in] >> 8)];				// get the sine value!
		
		// Main DDS
		accumulator[in] = accumulator[in] + (increment[in] + (fm[in] * (fm_amp[in] >> depth[in])));	// calculate the big sine value
		
		// Sum and scale the output voices
		summed_wave = summed_wave + (((amp[in] >> 8) * (int)sineTable[(char)(accumulator[in] >> 8)]) >> 7);			// divide output by 2 (voices) and sum!
	}
	
	OCR0A = 128 + summed_wave;	
	time++;

	TCCR2B = 0;

}


int main(void)
{
    initialize();		// setup general port and variable stuff once
	update_LCD();
	while(1){
		//fprintf(stdout,"%d\n\r",TCNT2);
		if((time & 0x07FF) == 0){						// check for button presses...
			key_press = get_button();					// check for any buttons
			for(i = 0; i < button_count; i++) check_button(i, key_press);		// update state machine
			if(got_push == 1){	//if we got any push...
				got_push = 0;	// reset the push flag
				select_voice();	// drop into the state machine
				update_LCD();	// and only update the LCD when we get a button press
			}
		}

		if(time > 16384){					// every second re-initialize all the sounds!
			for(n = 0; n < voice_count; n++){
				amp_fall[n] = max_amp;				// reset envelope
				amp_rise_temp[n] = max_amp;			// reset envelope
				amp_rise[n] = 0;					// reset envelope
				amp[n] = 0;
				fm_amp[n] = max_amp;				// reset enevelope
				increment[n] = next_increment[n];	// load frequency
				fm_increment[n] = increment[n] * 2;	// load harmonic
				needs_next = 1;						// tell main to load the next frequencies!
				time = 0;
			}
		}	
	    
	    random_gen();													// always cycle the number generator
	    
	    if(needs_next == 1){											// calculate note transition ahead of time!
	    	update_notes();
	    	needs_next = 0;
	    }
	
	}
	
}


unsigned char get_button(void){

	//get lower nibble
  	PORTDIR = 0x0f;
  	PORTDATA = 0xf0; 
  	_delay_us(5);
  	key = PORTIN;
  	  
  	//get upper nibble
  	PORTDIR = 0xf0;
  	PORTDATA = 0x0f; 
  	_delay_us(5);
  	key = key | PORTIN;
  	  
  	//find matching keycode in keytbl
  	if (key != 0xff){ 
  	 	for (butnum=0; butnum<maxkeys; butnum++){  
  	  		if (keytbl[butnum]==key)  break;   
  	  	}

  	  	if (butnum==maxkeys) butnum=0;
  	  	else butnum++;	   //adjust by one to make range 1-16
  	} 
  	else butnum=0;

	return butnum;

}


void check_button(int i, unsigned char button_phys){

  	switch (PushState[i]){
    		case NoPush: 
    		    if (button_phys == i + 1) PushState[i] = MaybePush;
    		    else {PushState[i] = NoPush;}
    		    break;
    		case MaybePush:
        		if (button_phys == i + 1){
				got_push = 1;
           		PushState[i] = Pushed;   
           		PushFlag[i] = 1;
        		}
        		else PushState[i] = NoPush;
        		break;
     	case Pushed:  
        		if (button_phys == i + 1) PushState[i] = Pushed; 
        		else PushState[i] = MaybeNoPush;
        		break;
     	case MaybeNoPush:
        		if (button_phys == i + 1) PushState[i] = Pushed; 
        		else {
           		PushState[i] = NoPush;
           		PushFlag[i] = 0;
        		}  
        		break;
  	}		
}

char random_gen(void){
	bit0 = (noise_gen & bit27) > 0;
	bit1 = (noise_gen & bit30) > 0;
	noise_gen <<= 1;
	noise_gen += (bit0 ^ bit1);
	return (char)(noise_gen & 0x7f);			// returns a random number from 0-127
}

void update_notes(void){

	for(n = 0; n < voice_count; n++){			// one calc for each note
		p = random_gen();						// new rand
		offset = 0;								// reset offset
		for(i = 0; i < 8; i++){					// iterate over markov columns
			offset = offset + note_matrix[voice_note[n]][notes[n]][i];	// compound offset
			if(p < offset){						// if the number is  within the correct range...
				notes[n] = i;					// set that as the next note!
				next_increment[n] = freq_table[i];	// precalculate frequency
			break;
			}
		}
	}


}

void select_voice(){

	switch(voice_state){
	    	case 1:			// selecting voice 1...
				if(PushFlag[togglev] == 1){voice_state = 2; PushFlag[togglev] = 0;}  	// toggle
		
	    	break;
	    	case 2:			// selecting voice 2...
				if(PushFlag[togglev] == 1){voice_state = 1; PushFlag[togglev] = 0;}  	// toggle
				
	    	break;
	}

	switch(param_state){
	    case 1:			// selecting tau_fall
			if(PushFlag[togglep] == 1){param_state = 2; PushFlag[togglep] = 0;}  	// toggle

			if(PushFlag[add] == 1){tau_fall[voice_state-1] = (tau_fall[voice_state-1] + 1)%10; PushFlag[add] = 0;}
		
	    break;
		case 2:			// selecting tau_rise
			if(PushFlag[togglep] == 1){param_state = 3; PushFlag[togglep] = 0;}  	// toggle

			if(PushFlag[add] == 1){tau_rise[voice_state-1] = (tau_rise[voice_state-1] + 1)%10; PushFlag[add] = 0;}
	    break;
		case 3:			// selecting fm_tau_fall
			if(PushFlag[togglep] == 1){param_state = 4; PushFlag[togglep] = 0;}  	// toggle

			if(PushFlag[add] == 1){fm_tau_fall[voice_state-1] = (fm_tau_fall[voice_state-1] + 1)%10; PushFlag[add] = 0;}
	    break;
		case 4:			// selecting depth
			if(PushFlag[togglep] == 1){param_state = 5; PushFlag[togglep] = 0;}  	// toggle

			if(PushFlag[add] == 1){depth[voice_state-1] = (depth[voice_state-1] + 1)%16; PushFlag[add] = 0;}
	    break;
		case 5:			// selecting matrix
			if(PushFlag[togglep] == 1){param_state = 1; PushFlag[togglep] = 0;}  	// toggle
	
			if(PushFlag[add] == 1){voice_note[voice_state-1] = (voice_note[voice_state-1] + 1)%4; PushFlag[add] = 0;}
	    break;
	    		
	}
	
}

void update_LCD(){

	switch(voice_state){
	    	case 1:			// selecting voice 1...
				CopyStringtoLCD(LCD_voice1,0,0);										// Voice 1			
	    	break;
	    	case 2:			// selecting voice 2...
				CopyStringtoLCD(LCD_voice2,0,0);										// Voice 2					
	    	break;
	}

	switch(param_state){
	    case 1:			// selecting tau_fall
			CopyStringtoLCD(LCD_tau_fall,0,1);	

			sprintf(lcd_buffer,"%u", tau_fall[voice_state-1]);									// Current Value
			LCDGotoXY(13,1);
			LCDstring(lcd_buffer, strlen(lcd_buffer));
		
	    break;
		case 2:			// selecting tau_rise
			CopyStringtoLCD(LCD_tau_rise,0,1);	

			sprintf(lcd_buffer,"%u", tau_rise[voice_state-1]);									// Current Value
			LCDGotoXY(13,1);
			LCDstring(lcd_buffer, strlen(lcd_buffer));
		
	    break;
		case 3:			// selecting fm_tau_fall
			CopyStringtoLCD(LCD_fm_tau_fall,0,1);	

			sprintf(lcd_buffer,"%u", fm_tau_fall[voice_state-1]);									// Current Value
			LCDGotoXY(13,1);
			LCDstring(lcd_buffer, strlen(lcd_buffer));
		
	    break;
		case 4:			// selecting depth
			CopyStringtoLCD(LCD_depth,0,1);	

			sprintf(lcd_buffer,"%u", depth[voice_state-1]);									// Current Value
			LCDGotoXY(13,1);
			LCDstring(lcd_buffer, strlen(lcd_buffer));
		
	    break;
		case 5:			// selecting matrix
			CopyStringtoLCD(LCD_matrix,0,1);
			
			sprintf(lcd_buffer,"%u", voice_note[voice_state-1]);									// Current Value
			LCDGotoXY(13,1);
			LCDstring(lcd_buffer, strlen(lcd_buffer));	
	    break;
	    		
	}
	
}

void init_lcd(void){
	LCDinit();
	LCDcursorOFF();
	LCDclr();
}
void initialize(void)
{
    DDRA = 0x00;								// set as input for buttons
    DDRB = (1<<PINB3);						// waveform output


	// set up timer 1 for audio synthesis (8000Hz)
  	TCCR2B= 0x01; 							// full speed
  	TCCR2A= 0x00;
    
    // set up timer 1 for audio synthesis (8000Hz)
  	OCR1A = 999;  							// set the comparere to 2000 time ticks
  	TIMSK1= (1 << OCIE1A);					// turn on timer 1 cmp match ISR 
  	TCCR1B= 0x09; 							// full speed
  	TCCR1A= 0x00;		  					// turn on clear-on-match
  	
  	// set up timer 0 for PWM output
   	TCCR0B = 1 ;  			  	 			// timer 0 runs at full rate
   	TIMSK0 = 0 ;								// turn off timer 0 overflow ISR
   	TCCR0A = (1<<COM0A0) | (1<<COM0A1) | (1<<WGM00) | (1<<WGM01) ;	// turn on fast PWM, OC0A output, full clock, toggle OC0A, 16us per PWM cycle 
   	OCR0A = 128 ; 							// set PWM to half full scale
           	
    // init some values
    time = 0;								// timer for exponential calculation
    init = 0;
    noise_gen = 0x19fd;						// seed for rand();
    voice_state = 1;							// editing voice 1..
    param_state = 1;							// editing param 1..
	got_push = 0;   

    // init the site table
   for (i=0; i<256; i++){
   		sineTable[i] = (char)(127.0 * sin(6.283*((float)i)/256.0)) ;
   }  

   freq_table[0] = (int) 4.096 * note0;
   freq_table[1] = (int) 4.096 * note1;
   freq_table[2] = (int) 4.096 * note2;
   freq_table[3] = (int) 4.096 * note3;
   freq_table[4] = (int) 4.096 * note4;
   freq_table[5] = (int) 4.096 * note5;
   freq_table[6] = (int) 4.096 * note6;
   freq_table[7] = (int) 4.096 * note7;
   
   // set up voices
   for(n = 0; n < voice_count; n++){
   		amp_fall[n] = max_amp;				// reset envelope
		amp_rise_temp[n] = max_amp;				// reset envelope
		amp_rise[n] = 0;						// reset envelope
		fm_amp[n] = max_amp;					// reset enevelope
		accumulator[n] = 0;						// reset phase
		fm_accumulator[n] = 0;					// reset phase
		increment[n] = (int) 4.196 * note0;		// load frequency
		notes[n] = 0;
		tau_fall[n] = 3;		
		tau_rise[n]	= 2;
		fm_tau_fall[n] = 6;	
		depth[n] = 14;
		fm_increment[n] = (int) 4.196 * note0 * 2;
		voice_note[n] = 1;
		needs_next = 1;						// tell main to load the next frequencies!
	}
		
    // init the UART -- uart_init() is in uart.c
    uart_init();
    stdout = stdin = stderr = &uart_str;
    fprintf(stdout,"Starting Lab 3...\n\r");
 
    
    init_lcd();			// setup some LCD stuff once
    TCNT1 = 0;			// get ready for some interrupts!
      
    //crank up the ISRs
    sei();

}
