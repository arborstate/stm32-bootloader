#ifndef _XMODEM_H_
#define _XMODEM_H_

#include <stdint.h>

struct xmodem_state {
	uint8_t idx;
	uint8_t seq;
	int crc;
	char packet[128];
	size_t packetpos;
};

char xmodem_ingest(struct xmodem_state *s, char c);
char xmodem_cancel(struct xmodem_state *s);

#endif /* _XMODEM_H_ */
