#ifndef RFEASYLINKRX_H
#define RFEASYLINKRX_H

#define BUFFER_SIZE 28 // 4 pacotes : RFEASYLINKTXPAYLOAD_LENGTH/(count sending variables) = 29/4 [my_id,timestamp][id,rssi,2xtimestamp] : 7 medidas por pacote sobra 1 byte

/* Board Header files */
#include "Board.h"

/* Create the TaskManager */
void TaskManager_init(PIN_Handle ledPinHandle);

#endif /* RFEASYLINKRX_H */
