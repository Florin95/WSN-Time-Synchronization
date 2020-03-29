#ifndef TYPES_H_
#define TYPES_H_

/* Data structure to TCP data and data length */
typedef struct
{
	uint8_t sync;
    uint8_t data[27];
    uint32_t timestamp;
}tcp_data_packet_t;




#endif /* TYPES_H_ */
