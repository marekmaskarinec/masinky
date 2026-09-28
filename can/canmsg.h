#ifndef _CANMSG_H
#define _CANMSG_H

#include <stdint.h>
#include <stdlib.h>

// Message IDs are in range 0-7ff. Lower number has larger priority.
enum canid
{
	CANID_SCAN = 0x101,
	CANID_SCAN_RESP = 0x102,
};

struct canmsg
{
	enum canid id;
	size_t msize;
	uint32_t esig;
	union
	{};
};

#endif
