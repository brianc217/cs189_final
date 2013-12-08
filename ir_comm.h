union comm_value {
	struct {
		unsigned char checksum:1; // Parity bit
		unsigned int data:11;	// Data that you get to play with.
		unsigned char ID:4;		// Robot ID
	} bits;
	unsigned int value;
};

extern int time_counter; 

/* Initiazlie IR ring and protocol state */ 
void comm_init(unsigned char seed, unsigned char ID);

/* Wait until receiving a non-corrupted packet */
void comm_rx(finalDataRegister *data);

/* Store a packet for async broadcasting */
void comm_store_tx(unsigned int data);
