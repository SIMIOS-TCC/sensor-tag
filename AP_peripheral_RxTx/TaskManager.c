/***** Includes *****/
/* Standard C Libraries */
#include <string.h>
#include <stdlib.h>

/* Application Header files */ 
#include "rfEasyLinkRx.h"
#include "rfEasyLinkTx.h"

/* Board Header files */
#include "Board.h"

/***** Function definitions *****/
void TaskManager_init(void) {

}

void setConcentrator(PIN_Handle pinHandle){
    txTask_init(pinHandle);
    printf("Mudando pra concentrator\n");
}

void setNode(PIN_Handle pinHandle) {
    rxTask_init(pinHandle);
    printf("Mudando pra node\n");
}



