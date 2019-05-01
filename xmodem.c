
#include <stdlib.h>

#include "stm32f334x8.h"
#include "system.h"

void
usart_send(USART_TypeDef *usart, char c)
{
	do {} while (!(usart->ISR & USART_ISR_TXE));
	usart->TDR = c;
}

extern volatile uint32_t tick;


#define STATE_WAITING	0

struct _xmodem_state {
	USART_TypeDef *usart;
	uint8_t idx;
	uint8_t seq;
	int crc;
	char packet[128];
	size_t packetpos;
	char buf[4096];
	size_t bufpos;
};

char xmodem_ingest(struct _xmodem_state *s, char c);

int
xmodem_receive(USART_TypeDef *usart)
{
	struct _xmodem_state state;
	uint32_t timeout;

#define _RESET_TIMEOUT() timeout = tick + (3 * 1000)
	char resp = 'C';
	int count = 0;

	state.usart = usart;
	state.seq = 1;
	state.idx = 0;
	state.bufpos = 0;

	// Force a timeout.
	timeout = 0;

	while (1) {
		if (usart->ISR & USART_ISR_RXNE) {
			_RESET_TIMEOUT();
			resp = xmodem_ingest(&state, state.usart->RDR);

			if (resp != 0) {
				usart_send(usart, resp);
			}
		}

		if (tick >= timeout) {
			_RESET_TIMEOUT();
			if (resp != 0) {
				usart_send(usart, resp);
			}
			count += 1;

			if (count >= 3) {
				resp = 'C';
				state.seq = 1;
				state.idx = 0;
				count = 0;
			}
		}
	}
#undef _RESET_TIMEOUT
}

char
xmodem_ingest(struct _xmodem_state *s, char c)
{
	char ret = 0;

	switch (s->idx) {
	case 0:
		switch (c) {
		case 0x1:
			// SOH
			s->crc = 0;
			s->packetpos = 0;
			break;
		case 0x4:
			// End of Transmission
			ret = 0x06;
			goto complete;
			break;
		case 0x17:
			// End of Transmission Block
			// ACK
			ret = 0x06;
			goto complete;
			break;
		case 0x18:
			// Cancel
			ret = 'C';
			goto complete;
			break;
		default:
			// NACK
			goto nack;
		}
		break;
	case 1:
		// Sequence
		if (c != s->seq) {
			// NACK
			goto nack;
		}
		break;
	case 2:
		// Sequence Recip
		if (c != 0xFF - (s->seq)) {
			// NACK
			goto nack;
		}

		break;
	case 131:
		// CRC
		if (c != ((s->crc >> 8) & 0xFF)) {
			// NACK
			goto nack;
		}

		break;
	case 132:
		if (c != (s->crc & 0xFF)) {
			// NACK
			goto nack;
		}

		s->seq += 1;

		// Save this packet.
		for (int i = 0; i < s->packetpos; i++) {
			s->buf[s->bufpos] = s->packet[i];
			s->bufpos += 1;
		}
		// ACK
		ret = 0x06;
		goto next;
		break;
	default:
		// Payload
		s->crc ^= ((int)c) << 8;

		for (int i = 0; i < 8; i++) {
			if  (s->crc & 0x8000) {
				s->crc = (s->crc << 1) ^ 0x1021;
			} else {
				s->crc = s->crc << 1;
			}
		}

		s->packet[s->packetpos] = c;
		s->packetpos += 1;
		break;
	}

	s->idx += 1;
	s->idx %= 133;

	return 0;
nack:
	ret = 0x15;
	goto next;
complete:
	s->seq = 1;
	s->bufpos = 0;
next:
	s->idx = 0;
	return ret;
}
