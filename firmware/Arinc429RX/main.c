/*

  Arinc 429 Sniffer
  Danjovic, 2013-2017
  
  V1.1 - 23/05/2017
  - Corrected Timeout for 320us as in Arinc 429 low speed standard

  V1.2 - 27/11/2017
  - Added measurement of line load
  
*/

#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <ctype.h>

// definitions
#define EEPROM_CONFIG_ADDRESS 1
#define USART_BAUDRATE 57600
#define MAX_BUFFER 128
#define LABELS_PER_LINE 8
#define true 1
#define false 0
#define SNIFF 0
#define TRACK 1
#define FORWARD 2
#define T4BITS  320   // 320us Time for 4 bits at 12500kbps
//#define DEBUG

// macros
#define UBRR_VALUE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)
#define OCR0_VALUE (F_CPU / (2 * 64 * 1000 ))-1;                // 1ms, prescaler 64
#define __verbose config_mode & 0x80
#define TIMEOUT_INIT_VALUE  96 // 256 - (F_CPU * T4BITS )/( 32 * 1000000UL );


#define TOUT_TIMER_STOP() TCCR2B=0   //  stop timer 2


//
// Function prototypes
//
uint8_t kbhit(void);
void USART0Init(void);
int  USART0SendByte(char u8Data, FILE *stream);
int  USART0ReceiveByte(FILE *stream);
void InitTimers_and_Interrupts(void);
void InitIO(void);
uint8_t Track_label( uint8_t label);
uint8_t Acquire_labels(uint8_t display_option);
void start_captpure(void);
void Print_Message(void);
uint8_t Input_label(void);
uint8_t bitwise_reverse ( uint8_t label);
inline void Timeout_Timer_restart (void);
uint8_t Measure_Load(void);

//
//  Data Types
//
typedef union {
  uint32_t full;
  struct {
    uint8_t byte0;
	uint8_t byte1;
	uint8_t byte2;
	uint8_t byte3;
  } bytes;
} quadribyte;

typedef struct {
  quadribyte word_arinc;
  uint16_t    timestamp;
} buffer_entry;

//
//  Global Variables
//

static volatile uint8_t continha=0;


static volatile buffer_entry arinc_buffer[MAX_BUFFER];  // buffer circular
static volatile buffer_entry* p_buff;                   // ponteiros para estruturas dentro do buffer
static volatile buffer_entry* p_getb;
static volatile uint8_t position_get;
static volatile uint8_t position_put;                   // posicao dentro do buffer circular
//static volatile uint32_t arinc_word=0;                // palavra Arinc
static volatile uint8_t label_to_track;
static volatile uint16_t tick_counter=0;
static volatile uint8_t  bit_counter=0;
static volatile uint8_t timeout_arinc_bits=1;
static uint8_t config_mode; // bit 7 - verbose
                            // bits 1,0 Numeric mode:
							//          00 - binary
							//          01 - BCD
							//          10 - hexadecimal
							//          01 - octal


//set stream pointer
FILE usart0_str = FDEV_SETUP_STREAM(USART0SendByte, USART0ReceiveByte, _FDEV_SETUP_RW);

//
// Interrupts
//
ISR (TIMER0_COMPA_vect) { // count each 1ms
   tick_counter++;
 }

ISR (TIMER2_OVF_vect) { // overflows when no bit is received after 512us.
   TOUT_TIMER_STOP();
//   TCCR2B = 0; // stop counter 2
   bit_counter=0;
}



ISR (INT0_vect){  // Shift in a 'zero' bit
    uint8_t saved_pos;
#ifdef DEBUG
    PORTB |= (1<<PINB0);
#endif
	Timeout_Timer_restart(); // Restart timeout timer, clear any pending interrupt
//    TCNT2=0;  // clear timeout counter
//    TIFR2 |=(1<<TOV2); // clear any pending interrupt
//    TCCR2B = ((1 << CS20) | (1 << CS21)); // start Counter 2

    //continha++;

    p_buff->word_arinc.full>>=1;
    bit_counter++;
    if (bit_counter==32) {  // new word has just arrived
        PORTB |= (1<<PINB4);
        bit_counter=0;
        p_buff->timestamp=tick_counter; // insert timestamp
        saved_pos=position_put;  // salva posição atual dentro do buffer
        position_put++; //  tenta avançar posição no contador
        if (position_put>=MAX_BUFFER) position_put=0; // volta ao inicio, se necessário
        if (position_put==position_get) {            // se deu a volta no buffer
            position_put=saved_pos;                  // restaura posição original
        } else {                                     // caso contrario atualiza ponteiro
            if (position_put!=0)                        // se posicao (put) nao é zero
                p_buff++;                            // simplesmente avança o ponteiro da estrutura
            else                                     // caso contrario
                p_buff=&arinc_buffer[0];             // aponta para o início do buffer
        }
#ifdef DEBUG		
        PORTB &=~(1<<PINB4);
#endif		
    }
#ifdef DEBUG	
    PORTB &=~(1<<PINB0);
#endif
}

ISR (INT1_vect){  // Shift in a 'one' bit
    uint8_t saved_pos;
#ifdef DEBUG
    PORTB |= (1<<PINB1);
#endif

	Timeout_Timer_restart(); // Restart timeout timer, clear any pending interrupt
//    TCNT2=0;  // clear timeout counter
//    TIFR2 |=(1<<TOV2); // clear any pending interrupt
//    TCCR2B = ((1 << CS20) | (1 << CS21)); // start Counter 2

    p_buff->word_arinc.full>>=1;
    p_buff->word_arinc.bytes.byte3|=0x80; // insert bit 'one'

    bit_counter++;

    if (bit_counter==32) {  // new word has just arrived
        PORTB |= (1<<PINB4);
        bit_counter=0;
        p_buff->timestamp=tick_counter; // insert timestamp
        saved_pos=position_put;  // salva posição atual dentro do buffer
        position_put++; //  tenta avançar posição no contador
        if (position_put>=MAX_BUFFER) position_put=0; // volta ao inicio, se necessário
        if (position_put==position_get) {            // se deu a volta no buffer
            position_put=saved_pos;                  // restaura posição original
        } else {                                     // caso contrario atualiza ponteiro
            if (position_put!=0)                        // se posicao (put) diferente de zero (true)
                p_buff++;                            // simplesmente avança o ponteiro da estrutura
            else                                     // caso contrario
                p_buff=&arinc_buffer[0];             // aponta para o início do buffer
        }
#ifdef DEBUG		
        PORTB &=~(1<<PINB4);
#endif		
    }
#ifdef DEBUG	
    PORTB &=~(1<<PINB1);
#endif	
}

//
//  Main Program
//
int main (void)
{
  /* Variables */
  uint8_t KeyStroke;
  uint8_t k;

  /* Initialize variables */
  position_get=0;
  position_put=0;

  p_buff=&arinc_buffer[0];
  p_getb=&arinc_buffer[0];

  for(k=0;k<MAX_BUFFER;k++) {  // Clear buffer
    arinc_buffer[k].word_arinc.full=0xffffffff;
    arinc_buffer[k].timestamp=0;
  }

  /* Initialize hardware */

  USART0Init();                 //Initialize USART0
  stdin=stdout=&usart0_str;     // assign our stream to standart I/O streams
  InitIO();                     // Init I/O pins
  InitTimers_and_Interrupts();  // set 1ms tick counter and 512us timeout counter

  printf_P(PSTR("\nArinc 429 sniffer (c)2013 by Daniel Jose Viana"));

  /* Main Loop */
  while(1)
    {
      cli(); //Stop_interrupts, etc
	  config_mode = eeprom_read_byte ((const uint8_t *)EEPROM_CONFIG_ADDRESS);

	  if (__verbose) {
 //         printf_P(PSTR("\nArinc 429 sniffer (c)2013 by Daniel Jose Viana"));
          printf_P(PSTR("\nCommand? (P=help)"));
	  }
      //scan standard stream (USART)
	  KeyStroke=getc(&usart0_str);

	  switch (tolower(KeyStroke))
	  {

        /* Output */
		case 'p': printf_P(PSTR("\n\nArinc 429 sniffer V1.2 (c)2013-2017 by Daniel Jose Viana"));
		          printf_P(PSTR("\nOutput:\nV - Set Verbose Mode\nQ - Set Quiet Mode"));
				  printf_P(PSTR("\n\nData bits view format:\nB - Binary\nH - Hexadecimal"));
		          printf_P(PSTR("\n\nOperation:\nS - Show labels of incoming Messages\nT - Track Messages from a supplied label\nA - Show all Messages\nL - Measure line load"));
                  break;

		case 'v': printf_P(PSTR("\nVerbose Mode"));
		          config_mode |= 0x80; // set bit 7
                  eeprom_write_byte ((uint8_t *)EEPROM_CONFIG_ADDRESS, config_mode);
		          break;

        /* View */
		case 'q': if (__verbose) printf_P(PSTR("\nQuiet Mode"));
		          config_mode &= 0x3f; // clear bit 7
                  eeprom_write_byte ((uint8_t *)EEPROM_CONFIG_ADDRESS, config_mode);
				  break;

        case 'b': if (__verbose) printf_P(PSTR("\nBinary Mode"));
		          config_mode &= 0xfc; // clear bits 0 and 1
                  eeprom_write_byte ((uint8_t *)EEPROM_CONFIG_ADDRESS, config_mode);
				  break;

        case 'h': if (__verbose) printf_P(PSTR("\nHexadecimal Mode"));
		          config_mode &= 0xfe; // clear bit 0
		          config_mode |= 0x02; // Set   bit 1
                  eeprom_write_byte ((uint8_t *)EEPROM_CONFIG_ADDRESS, config_mode);
				  break;

        /* Operation */

        case 's': Acquire_labels(SNIFF);
				  break;

        case 't': if (! Input_label())
				     break;
                  Acquire_labels(TRACK);
 				  break;

        case 'a': Acquire_labels(FORWARD);
 				  break;
				  
        case 'l': Measure_Load();
 				  break;
				  

        default:  //printf_P(PSTR("\n"));
                  break;
	  } //switch
    } // while(1)
}



void USART0Init(void)
{
  // Set baud rate
   UBRR0H = (uint8_t)(UBRR_VALUE>>8);
   UBRR0L = (uint8_t)UBRR_VALUE;
   // Set frame format to 8 data bits, no parity, 1 stop bit
   UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);
   //enable transmission and reception
   UCSR0B |= (1<<RXEN0)|(1<<TXEN0);
}

int USART0SendByte(char u8Data, FILE *stream)
{
   if(u8Data == '\n')
   {
      USART0SendByte('\r', 0);
   }
   //wait while previous byte is completed
   while(!(UCSR0A&(1<<UDRE0))){};
   // Transmit data
   UDR0 = u8Data;
   return 0;
}

int USART0ReceiveByte(FILE *stream)
{
   uint8_t u8Data;
   // Wait for byte to be received
   while(!(UCSR0A&(1<<RXC0))){};
   u8Data=UDR0;
   //echo input data
//   USART0SendByte(u8Data,stream);
   // Return received data
   return u8Data;

}

uint8_t kbhit (void)
{
    if (UCSR0A&(1<<RXC0))
        return true;
    else
        return false;
}


void InitIO(void){
    // Init port D

    DDRD   = ~(1<<PIND2 | 1<<PIND3); // set pins as inputs
    DDRD  |=  (1<<PIND4 | 1<<PIND5); // set pins as outputs
    PORTD &= ~(1<<PIND4 | 1<<PIND5); // clear pins 4 and 5

#ifdef DEBUG
    // Debug-diagnostico-porta B
    DDRB  = (1<<PINB0 | 1<<PINB1 | 1<<PINB4 );
    PORTB = 0;
#endif	
}


void InitTimers_and_Interrupts(void){

    cli();
    EICRA=(1<<ISC00 | 1<<ISC01 | 1<<ISC10 | 1<<ISC11 ); // set interrupts on rising edges of INT0 and INT1
    EIMSK=( 1<<INT0 | 1<<INT1 ); // set interrupts for INT0 and INT1



    // Timer 0
    TCCR0A = ((1 << WGM01));               // CTC Mode
    TCCR0B = ((1 << CS00) | (1 << CS01)); // Prescaler = 64
    OCR0A  = 0xf9;
    TIMSK0 |= (1 << OCIE0A); // Enable Timer 1 CTC interrupt

    // Timer 2
    TCCR2A = 0;              // Normal mode
	TOUT_TIMER_STOP();
    //TCCR2B = 0; // contador parad0
    TCNT2  = TIMEOUT_INIT_VALUE; // zera contador
//    TCCR2B = ((1 << CS20) | (1 << CS21)); // Prescaler = 32 (overflows at 512us)
    TIMSK2 = (1 << TOIE2); // Enable Tiemr 2 Overflow interrupt

}

uint8_t Input_label(void)
{
  char buffer[3] = {0,0,0};  // buffer para os 3 digitos do label
  char c;                    // caractere obtido da serial
  uint8_t j=3;               //
  uint16_t L=0;              // Label (em decimal)

  if (__verbose) printf_P(PSTR("\nLabel to track? "));

  // get valid data from serial
  while (j>0)
  {
     c=getc(&usart0_str);
	 if (c>='0' && c<='7') {// caractere entre 0 e 7 ?
	   if (__verbose) putc(c,&usart0_str);  // Ecoa o caractere

	   buffer[0]=buffer[1];  // sim, gira o buffer
	   buffer[1]=buffer[2];
	   buffer[2]=c-'0';      // transforma ASCII em número de 0 a 7
	   j--;

	 } else {
	   if ( (c==' ') || (c==27) ) // encerra se pressionado espaço ou escape
	      return 0;
	 }

  }

  // convert received data into a label. Shall be less than 377.
  L=((uint16_t)buffer[2]   ) +    //last  character x 8^0
    ((uint16_t)buffer[1]<<3) +    //midle character x 8^1
	((uint16_t)buffer[0]<<6);       //first character x 8^2

  if (L>255) return 0; // greatest label is Octal 377 which corresponds to 0xFF or 255
  label_to_track=(uint8_t)L;
  return 1;                 // all ok.
}

uint8_t bitwise_reverse ( uint8_t label)
{
  uint8_t i,r_label=0;  // Label reciproco  bits 12345678->87654321
  uint8_t mask=0x01;

  //bitwise reverse label
  for (i=0;i<8;i++) {
    r_label<<=1;
	if (label & mask) r_label|=0x01;
	mask<<=1;
  }
  return r_label;
}

//
// Aguarda a entrada de mensagens no buffer. Sai caso espaço ou escape sejam pressionados
// Esta rotina é comum às opções de "sniff" e "track"
//
uint8_t Acquire_labels(uint8_t display_option){

    uint8_t line_break=0;
    uint8_t c;

    if (__verbose) printf_P(PSTR("\nAcquiring Messages..."));

    position_get=0;
    position_put=0;
    p_getb=&arinc_buffer[0];
    p_buff=&arinc_buffer[0];

    sei();  // enable interrupts

    for(;;) {

        // Se alguma tecla pressionada, checa se é para sair
        if (kbhit()){
            c=getchar();
            if (c==27 || c==' ') {
                    if (__verbose) printf_P(PSTR("\nexiting..."));
                    cli();
                    return 0;
            }
        }  //if kbhit

      if (position_get!=position_put){   // Aguarda a entrada de uma nova mensagem no buffer
         // p_getb é global  // p_getb=&arinc_buffer[position_get];
          switch (display_option) {
              case SNIFF:  if (line_break == 0) putc('\n',&usart0_str);
                           line_break++;
                           if (line_break==LABELS_PER_LINE) line_break=0;
                           printf("%03o ",bitwise_reverse(p_getb->word_arinc.bytes.byte0));
                           break;

              case TRACK:  if (p_getb->word_arinc.bytes.byte0==bitwise_reverse(label_to_track)){
                               Print_Message();  // se chegou uma mensagem com o label desejado, imprime os dados
                           }
                           break;

              case FORWARD: Print_Message();  // se chegou uma mensagem com o label desejado, imprime os dados
                           break;
          } // switch

          // avança posição no buffer
          position_get++;
          if (position_get>=MAX_BUFFER) {
                position_get=0;
                p_getb=&arinc_buffer[0];   // reseta ponteiro
           } else {
                p_getb++; // avança ponteiro
          }
      } // if

    } // for(;;)
  }




//
// Imprime as mensagens que correspondem ao label desejado. Calcula o intervalo de acordo com o timestamp
//
void Print_Message(void)
{
   uint8_t i;//,j;
//   char s[5];             // string
   uint32_t databits;
   uint16_t time_now,delta_t;
   static uint16_t time_was=0;

     // calculate timestamp
     time_now=p_getb->timestamp;

     if (time_now > time_was)                // side effect: if time_was==time_now, intervall shall
        delta_t = time_now-time_was;         // be calculated as 0xffff which is good for our program
     else                                    // since intervals over 5000 will be considered as invalid
        delta_t=0xffff-(time_was-time_now);

     time_was=time_now;   // update time tracking variable

     // print Label
     putc('\n',&usart0_str);
	 if (__verbose) printf_P(PSTR("Label:"));
     i=p_getb->word_arinc.bytes.byte0;    // p_getb é global. byte0 é o Label na ordem reversa

     printf("%03o ",bitwise_reverse (i));

	 // print SDI
     i=p_getb->word_arinc.bytes.byte1;  // sdi= bits 0,1

	 if (__verbose) printf_P(PSTR("SDI:"));

	 if (i&0x02) putc('1',&usart0_str); else putc('0',&usart0_str);
	 if (i&0x01) putc('1',&usart0_str); else putc('0',&usart0_str);

     putc(' ',&usart0_str);

	 // print SSM
	 if (__verbose) printf_P(PSTR("SSM:"));

     i=p_getb->word_arinc.bytes.byte3;  // sdi= bits 0,1
	 if (i&0x40) putc('1',&usart0_str); else putc('0',&usart0_str);
	 if (i&0x20) putc('1',&usart0_str); else putc('0',&usart0_str);

     putc(' ',&usart0_str);

	 // print Data
     databits=(p_getb->word_arinc.full>>=10) & 0x7ffff;

	 if (__verbose) printf_P(PSTR("Data:"));

     switch (config_mode & 0x03) {
	   case 0: // binario
	          for (i=0;i<19;i++){
			    if (databits & 0x40000) putc('1',&usart0_str); else putc('0',&usart0_str);
				databits<<=1;
			  }
	          if (__verbose)  printf_P(PSTR(" Bin"));
			  break;


	   case 2: // Hexadecimal
              printf("%05lX",databits);
	          if (__verbose)  printf_P(PSTR(" Hexa"));

			  break;

	 }


	 // print Parity
	 if (__verbose) printf_P(PSTR("Parity:"));

	 if (i&0x80) putc('1',&usart0_str); else putc('0',&usart0_str);

	 putc(' ',&usart0_str);

	 // print Timestamp
	 if (__verbose) printf_P(PSTR("rate:"));

	 if (delta_t <=5000)
        printf("%04u",delta_t);
     else
        printf_P(PSTR("XXXX"));

	 if (__verbose) printf_P(PSTR("ms"));
	 putc(' ',&usart0_str);


}


//
//  Restart Timeout timer
//
inline void Timeout_Timer_restart (void) {
    TCNT2=TIMEOUT_INIT_VALUE;             // restore initial value for timeout timer/counters
    TIFR2 |=(1<<TOV2);                    // clear any pending interrupt
    TCCR2B = (1 << CS22);                 // restart Timer/counter 2
}

//
// Mede a carga do barramento. Sai caso espaço ou escape sejam pressionados
//
uint8_t Measure_Load(void){

	uint16_t ON_Time  = 0;  // Marcadores de ciclo ativo
	uint16_t OFF_Time = 0;
	double Load_percent = 0;
	uint8_t c;

	if (__verbose) printf_P(PSTR("\nMeasuring Traffic Load..."));
	
	// Inicializa Timer 0 para contagem de 100us
	TCCR0A = ((1 << WGM01) ); // CTC Mode
	TCCR0B = ((1 << CS01)  ); // Prescaler = 8
	OCR0A  = 199;             // overflow em 100us (16000000/8/200) 
	EIFR   = (1<<INTF1) | (1<<INTF0) ;   // limpa flags 
	TCNT0  = 0 ;


	for(;;) {

		// Se alguma tecla pressionada, checa se é para sair
		if (kbhit()){
			c=getchar();
			if (c==27 || c==' ') {
				if (__verbose) printf_P(PSTR("\nexiting..."));
				// Restaura estado do Timer 0
				TCCR0B = ((1 << CS00) | (1 << CS01)); // Prescaler = 64
				OCR0A  = 0xf9;


				return 0;
			}
		}  //if kbhit

		//  Mede  
		if (EIFR) { // Chegou algum bit durante o periodo de amostragem ?
			// Sim
			EIFR = (1<<INTF1) | (1<<INTF0) ;   // limpa flags
			ON_Time++;
			
		} else {     
			// Nao
			OFF_Time++;			
		}		
		
		// Exibe resultado
		if ( (ON_Time + OFF_Time) == 10000 ) {  // 1 segundo em intervalos de 100us
			printf_P(PSTR("\nLoad: "));
			if (__verbose) Load_percent = (double)ON_Time / 100 ;  // ON*100/10000
			printf("%.2f%%", Load_percent);
			//if (__verbose) printf_P(PSTR("%%"));
			ON_Time  = 0;  //        Reinicia valores
			OFF_Time = 0;
			TCNT0 = 0;
		}
		
		// Aguarda ate completar 100us
		while (!(TIFR0 & (1<<TOV0)) ) ;
		TIFR0 |= (1<<TOV0);             // Limpa flag

	} // for(;;)
}

