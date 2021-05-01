/***
           _         _              _  _  ____   ___    _______  __
          / \   _ __(_)_ __   ___  | || ||___ \ / _ \  |_   _\ \/ /
         / _ \ | '__| | '_ \ / __| | || |_ __) | (_) |   | |  \  /
        / ___ \| |  | | | | | (__  |__   _/ __/ \__, |   | |  /  \
       /_/   \_\_|  |_|_| |_|\___|    |_||_____|  /_/    |_| /_/\_\

       Danjovic 24-30/04/2021 - danjovic@hotmail.com
       Released under GNU General Public License, version 3
       

*/



#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>


//       _      __ _      _    /\/|
//    __| |___ / _(_)_ _ (_)__|/\/  ___ ___
//   / _` / -_)  _| | ' \| / _/ _ \/ -_|_-<
//   \__,_\___|_| |_|_||_|_\__\___/\___/__/
//                          )_)

#define DEBUG 1

#define SLOPE 0 // sinais na porta B
#define TXA   1
#define TXB   2

#define SLOPE 0 // sinais na porta D
#define RXA   3
#define RXB   2

#define BUFFER_SIZE  64
#define MAX_LABELS   64


#define writeOne()   PORTB |= (1<<TXA)
#define writeZero()  PORTB |= (1<<TXB)
#define writeClock() PORTB &= ~( (1<<TXA) | (1<<TXB) )

#if (DEBUG>0)
// Sinais de Debug
// Simula uma porta SPI transmitindo 32 bits, para facilitar visualização no analisador logico
//#define MOSI     1  // Sinal "positivo" -> igual a mensagem sendo enviada 
//#define MISO     2  // Sinal "negativo" -> sinal inverso (complemento) da mensagem sendo enviada
#define SPICLK   3
#define SPIEN    4
#define DEBUGPIN 5


#define debugPulse() do { PORTB |= (1<<DEBUGPIN); PORTB &= ~(1<<DEBUGPIN); } while (false)
#define spiclk() do { PORTB |= (1<<SPICLK); PORTB &= ~(1<<SPICLK); } while (false)
#define spicslo()  PORTB &= ~(1<<SPIEN)
#define spicshi()  PORTB |=  (1<<SPIEN)
#endif

//           _             _          _         _
//    ___ __| |_ _ _    __| |___   __| |__ _ __| |___ ___
//   / -_|_-<  _| '_|  / _` / -_) / _` / _` / _` / _ (_-<
//   \___/__/\__|_|(_) \__,_\___| \__,_\__,_\__,_\___/__/
//

enum BufferStatus {BUFFER_OK, BUFFER_EMPTY, BUFFER_FULL};

typedef struct  {  // Data Queue circular buffer
  uint32_t data[BUFFER_SIZE];
  uint8_t newest_index;
  uint8_t oldest_index;
} Buffer;

typedef struct  {
  unsigned parity :  1;
  unsigned ssm    :  2;
  unsigned long data   : 19;
  unsigned sdi    :  2;
  unsigned label  :  8; // em ordem reversa (rawdata)
} arinc429Word;

typedef union  {
  uint32_t rawdata;
  arinc429Word fields;
} arinc429Message ;

typedef struct  {
  arinc429Message messageData;
  uint16_t interval;  // reload value
  uint16_t countdown;
} arinc429entity;

//                 _   __         _
//   __ ____ _ _ _(_)_/_/__ _____(_)___
//   \ V / _` | '_| / _` \ V / -_) (_-<
//    \_/\__,_|_| |_\__,_|\_/\___|_/__/
//

arinc429entity schedulerList[MAX_LABELS];
Buffer tx_buffer;

static volatile uint8_t interMessageGap  = 3; // em tempos de meio bit
static volatile uint16_t fase = 64;
volatile uint32_t messageRegister = 0x12345678;


//                 _    __ _   _
//    _ __ _ _ ___| |_ /_/| |_(_)_ __  ___ ___
//   | '_ \ '_/ _ \  _/ _ \  _| | '_ \/ _ (_-<
//   | .__/_| \___/\__\___/\__|_| .__/\___/__/
//   |_|                        |_|

enum BufferStatus bufferRead(volatile /*struct*/ Buffer *buffer, volatile uint32_t *message);
enum BufferStatus bufferWrite(volatile /*struct*/ Buffer *buffer, volatile uint32_t message);
uint8_t reverseBitOrder (uint8_t label);
bool calculateParity(uint32_t rawdata);
uint32_t binaryToBCD (uint32_t binaryValue);
void clearSchedulerList (void);
void loop(void);
void setup(void);


//    _     _                             /\/|
//   (_)_ _| |_ ___ _ _ _ _ _  _ _ __  __|/\/  ___ ___
//   | | ' \  _/ -_) '_| '_| || | '_ \/ _/ _ \/ -_|_-<
//   |_|_||_\__\___|_| |_|  \_,_| .__/\__\___/\___/__/
//                              |_|    )_)

ISR (TIMER0_COMPA_vect ) {  // ativada a cada meio tempo de bit

#if (DEBUG>0)
  debugPulse();
  if (fase == 1) spicslo();
#endif

  // cada mensagem é transmitida em 64 meios tempos de bit, contados a partir de 1.
  // tempos ímpares correspondem aos bits e tempos pares correspondem ao clock

  if (fase <= 64 ) {  // se ainda nao chegou ao fim da mensagem transmite bit ou clock

    if ( fase & 1) {  // transmite bit
      if (messageRegister & 0x00000001)
        writeOne();
      else
        writeZero();

      // proximo bit
      messageRegister >>= 1;

#if (DEBUG>0)
      spiclk();
#endif

    } else { // transmite clock
      writeClock();
    } // if bit ou clock

  } else { // após o fim da mensagem transmite o intervalo ("gap") entre as mensagens
    writeClock();
  }

  // proximo meio bit (fase).
  fase++;
  
  if (fase > (64 + interMessageGap )) {
    // ao final da mensagem mais gap, procura nova mensagem na fila
    if ( bufferRead ( &tx_buffer, &messageRegister ) != BUFFER_EMPTY ) {
      fase = 1;  // se buffer nao estava vazio, se prepara para a proxima mensagem
      #if (DEBUG>0)
          debugPulse();
          debugPulse();
      #endif
    }
#if (DEBUG>0)
    spicshi();
#endif
  }

#if (DEBUG>0)
    debugPulse();  // segundo pulso de debug, permite medir tempo de processamento da ISR 
#endif
  TIFR0 |= (1 << OCF0A);  // limpa flag da ISR
}





  //               _       ____
  //    _ __  __ _(_)_ _  / /\ \
  //   | '  \/ _` | | ' \| |  | |
  //   |_|_|_\__,_|_|_||_| |  | |
  //                      \_\/_/

  int main (void) {
  setup();
  for (;;) {
    loop();
  }


  }






//    _                ____
//   | |___  ___ _ __ / /\ \ 
//   | / _ \/ _ \ '_ \ |  | |
//   |_\___/\___/ .__/ |  | |
//              |_|   \_\/_/


void loop(void) {

 // uint32_t received_message;
 // enum BufferStatus status;

  // scheduler
  // espera por flag timer 2 (1ms por iteracao)
  while ( ! ( TIFR2 & (1 << OCF2A) ) ) {

    // local para verificar dados presentes na porta serial. Se houver, processar interface de usuário
    //
    //
  };

  TIFR2 |= (1 << OCF2A);   // limpa flag timer 2


  // varre a lista dos labels (interval<5 significa label desativado)
  //      decrementa contagem
  //      se chegou a zero envia para fila de transmissao


  for (uint8_t i = 0; i < MAX_LABELS; i++) {

    if (schedulerList[i].interval >= 5) {                       // label ativo?
      if (--schedulerList[i].countdown == 0) {                  // sim, hora de transmitir?
        schedulerList[i].countdown = schedulerList[i].interval; // sim, restaura contador
        /*status = */ bufferWrite(&tx_buffer, schedulerList[i].messageData.rawdata ); // envia mensagem para lista
      }
    }
  } // for

} // loop()



//            _              ____
//    ___ ___| |_ _  _ _ __ / /\ \ 
//   (_-</ -_)  _| || | '_ \ |  | |
//   /__/\___|\__|\_,_| .__/ |  | |
//                    |_|   \_\/_/


void setup(void) {

 // limpa lista do scheduler
  clearSchedulerList();

   // pinos de saída 
  DDRB = (1 << SLOPE) | (1 << TXA) | (1 << TXB);
#if (DEBUG>0)
  DDRB |= ( (1 <<SPICLK ) | (1 << SPIEN ) | (1 << DEBUGPIN ) );
#endif

  DDRB = 0xff;
  PORTB = 0;

  // programa ISR timer 0 a 40us
  TCCR0A = (1 << WGM01);
  TCCR0B = (1 << CS01); // prescaler 8
  OCR0A = 79;          // 40us = 25KHz
  TCNT0 = 0;
  TIFR0 |= (1 << OCF0A);
  TIMSK0 = (1 << OCIE0A);


  // programa ISR timer 2 a 1ms
  TCCR2A = (1 << WGM21);
  TCCR2B = (1 << CS22); // prescaler 64
  TCNT2 = 0;
  OCR2A = 249;          // 1ms
  TIFR2 |= (1 << OCF2A);

  sei(); // habilita interrupcoes

  

  // Exemplos de inicialização dos dados da lista do scheduler

  // com dados crus
  //  0x  9    1    8    c    4    4    0    d
  //    1001.0001.1000.1100.0100.0100.0000.1101
  //    1.00.10.0011.00011.00010001.00.000.011.01  data
  //    P SSM 2    3     3       17 SDI  0   6  2  23/03 + 17mseconds label 260
  schedulerList[0].messageData.rawdata = 0x918c440d;
  schedulerList[0].interval = 100;
  schedulerList[0].countdown = 100;


  // com campos de bits
  schedulerList[1].messageData.fields.label = reverseBitOrder (0201); // 0 precedendo decimal-> octal (0201 = 0x81)
  schedulerList[1].messageData.fields.ssm = 0;
  schedulerList[1].messageData.fields.sdi = 0;
  schedulerList[1].messageData.fields.data = binaryToBCD(25786);
  schedulerList[1].messageData.fields.parity = calculateParity (schedulerList[1].messageData.rawdata);

  schedulerList[1].interval = 20;
  schedulerList[1].countdown = 20;


  // com campos de bits
  schedulerList[3].messageData.fields.label = reverseBitOrder (0342); // 0 precedendo decimal-> octal (0201 = 0x81)
  schedulerList[3].messageData.fields.ssm = 0;
  schedulerList[3].messageData.fields.sdi = 0;
  schedulerList[3].messageData.fields.data = binaryToBCD(12345);
  schedulerList[3].messageData.fields.parity = calculateParity (schedulerList[1].messageData.rawdata);

  schedulerList[3].interval = 50;
  schedulerList[3].countdown = 50;

  // Fim dos exemplos

}


//     __              /\/|
//    / _|_  _ _ _  __|/\/  ___ ___
//   |  _| || | ' \/ _/ _ \/ -_|_-<
//   |_|  \_,_|_||_\__\___/\___/__/
//                  )_)


// Push - sobe uma mensagem (rawdata) para o buffer circular de transmissão
// Esta função é usada somente pelo scheduler da função principal
enum BufferStatus bufferWrite(volatile /*struct*/ Buffer *buffer, volatile uint32_t message) {

  uint8_t next_index = (((buffer->newest_index) + 1) % BUFFER_SIZE);

  if (next_index == buffer->oldest_index) {
    return BUFFER_FULL;
  }

  buffer->data[buffer->newest_index] = message;
  buffer->newest_index = next_index;
  return BUFFER_OK;
}


// Pop - retira uma mensagem (rawdata) do buffer circular
// esta função é usada somente pela IRQ de transmissão
inline enum BufferStatus bufferRead(volatile /*struct*/ Buffer *buffer, volatile uint32_t *message) {

  if (buffer->newest_index == buffer->oldest_index) {
    return BUFFER_EMPTY;
  }

  *message = buffer->data[buffer->oldest_index];
  buffer->oldest_index = ((buffer->oldest_index + 1) % BUFFER_SIZE);
  return BUFFER_OK;
}


// reverte ordem dos bits de um byte, para facilitar preenchimento
uint8_t reverseBitOrder (uint8_t label) {
  uint8_t i = 0;
  uint8_t reverse = 0;

  for (i = 0; i < 8; i++) {
    reverse <<= 1;
    if (label & 0x01) {

      reverse |= 0x01;
    }
    label >>= 1;
  }
  return reverse;
}


// Retorna a paridade de uma mensagem. Avalia apenas os 31 bits menos significativos
// considerando a seguinte ordem:  MSBit: P(1)-SSM(2)-Data(19)-SDI(2)-Label(8) : LSBit
bool calculateParity(uint32_t rawdata) {

  uint8_t parity = 1;
  uint8_t i;

  for (i = 0; i < 31; i++) { // varre 31 bits (todos exceto o de paridade)
    if (rawdata & 0x00000001 ) {
      parity++;
    }
    rawdata >>= 1;
  }
  return (bool) (parity & 1);
}


// Converte um inteiro sem sinal de 32 bits para BCD. Satura em 79999
uint32_t binaryToBCD (uint32_t binaryValue) {
  uint32_t bcdResult = 0;
  uint32_t digit = 0;
  uint8_t position = 0;

  if (binaryValue < 80000 ) {
    while (binaryValue > 0) {
      digit = binaryValue % 10;
      bcdResult += (digit << position);
      position += 4;
      binaryValue = binaryValue / 10;
    }
  } else {
    bcdResult = 0x79999; // Satura
  }
  return bcdResult;
}

// Converte um inteiro sem sinal de 32 bits para BCD. Satura em 79999
void clearSchedulerList (void) {
  for (uint8_t i = 0; i < MAX_LABELS; i++) {
    schedulerList[i].messageData.rawdata = 0;
    schedulerList[i].interval = 0;
    schedulerList[i].countdown = 0;
  }


}
 

/*
  bit  |        1          |        2          |        3         |    4
       |  -----            |  -----            |  -----           |  -----
       | /     \           | /     \           | /     \          | /     \
       |/       \          |/       \          |/       \         |/       \
       -         -----------         -----------         ----------         ---------...
       |\       /|         |\       /|         |\       /|        |\       /|
       | \     / |         | \     / |         | \     / |        | \     / |
       |  -----  |         |  -----  |         |  -----  |        |  -----  |
       |         |         |         |         |         |        |         |
       |         |         |         |         |         |        |         |
  fase      1         2         3         4         5        6        7         8
          40us       40us



  bit      31         |        32         |    Intermessage GAP        |        1          |
      ----            |  -----            |                            |  -----            |
          \           | /     \           |                            | /     \           |
           \          |/       \          |          0-255             |/       \          |
            -----------         -------------------------------   ...  -         -----------...
           /|         |\       /|         |         |        |         |\       /|         |
          / |         | \     / |         |         |        |         | \     / |         |
      ----  |         |  -----  |         |         |        |         |  -----  |         |
            |         |         |         |         |        |         |         |         |
            |         |         |         |         |        |         |         |         |
  fase60        62        63        64        64        65                 1         2


*/
