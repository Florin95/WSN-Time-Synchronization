#ifndef TYPES_H_
#define TYPES_H_

/* Data structure to TCP data and data length */
typedef struct
{
	uint32_t sync_word;
	uint32_t sync_s;
	uint32_t sync_f;
	uint8_t device_id;
    uint8_t data[27];
}tcp_data_packet_t;


#endif /* TYPES_H_ */
