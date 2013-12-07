#include <p30f6014A.h>

#include "e_randb.h"
#include "ir_comm.h"

#include "motor_led/advance_one_timer/e_agenda.h"

#include "stdlib.h" // rand()

// period = 500 ms (* 28.8 KHz) -> 14400
#define SEND_TIMER_PERIOD 14400
// max backoff = 100 ms
#define MAX_SEND_TIMER_BACKOFF 2880

// Holds pre-computed random values for communication protocol. Size must be power of 2
#define RANDOM_BACKOFF_SIZE 64
static int random_backoff[RANDOM_BACKOFF_SIZE];
static int rand_ind = 0;

int time_counter = 0;

static union comm_value tx_buffer;

// Calculates parity of the high-order 15 bits of @value.
static unsigned char parity(unsigned int value)
{
	unsigned char result = 0;
	unsigned char i;
	for (i=1; i < 16; i++) {
		result ^= (0x1 & (value >> i));
	}
	return result;
}

static void init_timer3(void)
{
    T3CON = 0x0;            // init config register
    TMR3 = 0;               // clear timer counter
    PR3 = SEND_TIMER_PERIOD;
    T3CONbits.TCKPS = 0x3;  // prescaler = 256
                            // @ clock 7.3728 MHz -> tick @ 28.8 KHz
    T3CONbits.TCS = 0;      // internal clock source
    IFS0bits.T3IF = 0;      // clear interrupt flag
    IEC0bits.T3IE = 1;      // enable timer interrupt
    T3CONbits.TON = 1;      // start timer
}

void __attribute__((interrupt, auto_psv))
_T3Interrupt(void) {
	IFS0bits.T3IF = 0;

	/* Add a pre-computed random delay to next message send */
	PR3 = SEND_TIMER_PERIOD + random_backoff[rand_ind];
	rand_ind = (rand_ind + 1) & (RANDOM_BACKOFF_SIZE - 1);

	/* Clear timer value */
	TMR3 = 0;
	time_counter++;

	e_randb_send_all_data(tx_buffer.value);
}

void comm_init(unsigned char seed, unsigned char ID)
{
	int i;

	e_init_randb(I2C); // IR ring in I2C mode
	e_randb_set_range(0); // Transmit IR with full power (0xff is min power)
	e_randb_store_light_conditions(); // Calibrate background IR levels

	/* Pre-compute random values for communication backoff */
	srand(seed + TMR1);
	for (i = 0; i < RANDOM_BACKOFF_SIZE; i++)
		random_backoff[i] = rand() % MAX_SEND_TIMER_BACKOFF;

	tx_buffer.bits.ID = ID;
		
	init_timer3(); // Start broadcasting the contents of tx_buffer
		
	e_start_agendas_processing();
}

/* Wait until receiving a non-corrupted packet */
void comm_rx(finalDataRegister *data)
{
  finalDataRegister buff;
	union comm_value rx;
	int corrupted = TRUE;	
	while (corrupted) {
	
		/* Block until receiving */
		while (!e_randb_reception(&buff)) ; //NADA

		/* Check parity bit */
		rx.value = buff.data;
		corrupted = (parity(rx.value) != rx.bits.checksum);
		char msg[80];
		sprintf(msg,"%i\r\n",corrupted);
		btcomSendString(msg);
	}
	// Copy result in output buffer
	*data = buff;
}

/* Store a packet for broadcasting */
void comm_store_tx(unsigned int data)
{
	tx_buffer.bits.data = data;
	tx_buffer.bits.checksum = parity(tx_buffer.value); 
}
