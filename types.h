#ifndef TYPES_H_
#define TYPES_H_

/* Data structure to TCP data and data length */
typedef struct
{
	uint8_t timestamp;
    uint8_t data[27];
}tcp_data_packet_t;




#endif /* TYPES_H_ */
