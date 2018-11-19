/*
 * Copyright (c) 2015-2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== rfEasyLinkRx.c ========
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>
#include <stdint.h>
#include <stddef.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/drivers/UART.h>

/* TI-RTOS Header files */
#include <ti/drivers/PIN.h>

/* Board Header files */
#include "Board.h"

/* EasyLink API Header files */
#include "easylink/EasyLink.h"

/***** Defines *****/

/* Undefine to remove address filter and async mode */
#define RFEASYLINKRX_ASYNC
#define RFEASYLINKRX_ADDR_FILTER

#define RFEASYLINKEX_TASK_STACK_SIZE 1024
#define RFEASYLINKEX_TASK_PRIORITY   3
#define UART_TASK_PRIORITY   2

#define RFEASYLINKTXPAYLOAD_LENGTH      30
#define QT_PACKETS 4

#define UART_STACK_SIZE 20

uint8_t uart_stack_counter = 0;
char uartStack[UART_STACK_SIZE];

/* Pin driver handle */
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;

/* UART params */
UART_Handle uart;
UART_Params uartParams;

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
PIN_Config pinTable[] = {
    Board_PIN_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_PIN_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#if defined __CC1352R1_LAUNCHXL_BOARD_H__
    Board_DIO30_RFSW | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#endif
    PIN_TERMINATE
};

/***** Variable declarations *****/
static Task_Params rxTaskParams;
Task_Struct rxTask;    /* not static so you can see in ROV */
static uint8_t rxTaskStack[RFEASYLINKEX_TASK_STACK_SIZE];

static Task_Params uartTaskParams;
Task_Struct uartTask;
static uint8_t uartTaskStack[RFEASYLINKEX_TASK_STACK_SIZE];

/* The RX Output struct contains statistics about the RX operation of the radio */
PIN_Handle pinHandle;

#ifdef RFEASYLINKRX_ASYNC
static Semaphore_Handle rxDoneSem;
#endif

/***** Function definitions *****/
static void uartFnx(UArg arg0, UArg arg1)
{
    int i = 0;
    while(1) {
        i++;
//        if(uart_stack_counter == UART_STACK_SIZE) {
//            uart_stack_counter = 0;
//            UART_write(uart, uartStack, UART_STACK_SIZE);
//            UART_write(uart, ".", 1);
//        }
    }
}

#ifdef RFEASYLINKRX_ASYNC
void rxDoneCb(EasyLink_RxPacket * rxPacket, EasyLink_Status status)
{
    if (status == EasyLink_Status_Success)
    {
        if(((int*)rxPacket->dstAddr)[0] == 0xBB) {
//            uartStack[uart_stack_counter++] = (char*)rxPacket->payload[0];
//            uartStack[uart_stack_counter++] = ";";
//
//            uint8_t i;
//            uint8_t j;
//            for(i = 2; i < RFEASYLINKTXPAYLOAD_LENGTH - 3; i++) {
//                for(j = 0; j < 4; j++) { // 4 bytes/measure
//                    uartStack[uart_stack_counter++] = (char*)rxPacket->payload[i++]; //simio id
//                    uartStack[uart_stack_counter++] = ";";
//                    uartStack[uart_stack_counter++] = (char*)rxPacket->payload[i++]; //rssi
//                    uartStack[uart_stack_counter++] = ";";
//                    uartStack[uart_stack_counter++] = (char*)rxPacket->payload[i++]; //delta time byte 1
//                    uartStack[uart_stack_counter++] = (char*)rxPacket->payload[i++]; //delta time byte 2
//                }
//            }
        }

        /* Toggle LED2 to indicate RX */
        PIN_setOutputValue(pinHandle, Board_PIN_LED2,!PIN_getOutputValue(Board_PIN_LED2));
    }
    else if(status == EasyLink_Status_Aborted)
    {
        /* Toggle LED1 to indicate command aborted */
        PIN_setOutputValue(pinHandle, Board_PIN_LED1,!PIN_getOutputValue(Board_PIN_LED1));
    }
    else
    {
        /* Toggle LED1 and LED2 to indicate error */
        PIN_setOutputValue(pinHandle, Board_PIN_LED1,!PIN_getOutputValue(Board_PIN_LED1));
        PIN_setOutputValue(pinHandle, Board_PIN_LED2,!PIN_getOutputValue(Board_PIN_LED2));
    }

    Semaphore_post(rxDoneSem);
}
#endif

static void rfEasyLinkRxFnx(UArg arg0, UArg arg1)
{
#ifndef RFEASYLINKRX_ASYNC
    EasyLink_RxPacket rxPacket = {0};
#endif

#ifdef RFEASYLINKRX_ASYNC
    /* Create a semaphore for Async*/
    Semaphore_Params params;
    Error_Block eb;

    /* Init params */
    Semaphore_Params_init(&params);
    Error_init(&eb);

    /* Create semaphore instance */
    rxDoneSem = Semaphore_create(0, &params, &eb);
    if(rxDoneSem == NULL)
    {
        System_abort("Semaphore creation failed");
    }

#endif //RFEASYLINKRX_ASYNC

	EasyLink_Params easyLink_params;
    EasyLink_Params_init(&easyLink_params);
	
	easyLink_params.ui32ModType = EasyLink_Phy_Custom; 

    /* Initialize EasyLink */
    if(EasyLink_init(&easyLink_params) != EasyLink_Status_Success)
    {
        System_abort("EasyLink_init failed");
    }

    /*
     * If you wish to use a frequency other than the default, use
     * the following API:
     * EasyLink_setFrequency(868000000);
     */

#ifdef RFEASYLINKRX_ADDR_FILTER
	/* 
     * The address filter is set to match on a single byte (0xAA) but 
     * EasyLink_enableRxAddrFilter will copy 
     * EASYLINK_MAX_ADDR_SIZE * EASYLINK_MAX_ADDR_FILTERS
     * bytes to the address filter bank
     */
    uint8_t addrFilter[EASYLINK_MAX_ADDR_SIZE * EASYLINK_MAX_ADDR_FILTERS] = {0xaa};
    EasyLink_enableRxAddrFilter(addrFilter, 1, 1);
#endif //RFEASYLINKRX_ADDR_FILTER

    while(1) {
#ifdef RFEASYLINKRX_ASYNC
        EasyLink_receiveAsync(rxDoneCb, 0);

        /* Wait 300ms for Rx */
        if(Semaphore_pend(rxDoneSem, (300000 / Clock_tickPeriod)) == FALSE)
        {
            /* RX timed out abort */
            if(EasyLink_abort() == EasyLink_Status_Success)
            {
               /* Wait for the abort */
               Semaphore_pend(rxDoneSem, BIOS_WAIT_FOREVER);
            }
        }
#else
        rxPacket.absTime = 0;
        EasyLink_Status result = EasyLink_receive(&rxPacket);

        if (result == EasyLink_Status_Success)
        {
            /* Toggle LED2 to indicate RX */
            PIN_setOutputValue(pinHandle, Board_PIN_LED2,!PIN_getOutputValue(Board_PIN_LED2));
        }
        else
        {
            /* Toggle LED1 and LED2 to indicate error */
            PIN_setOutputValue(pinHandle, Board_PIN_LED1,!PIN_getOutputValue(Board_PIN_LED1));
            PIN_setOutputValue(pinHandle, Board_PIN_LED2,!PIN_getOutputValue(Board_PIN_LED2));
        }
#endif //RX_ASYNC
    }
}

void rxTask_init(PIN_Handle ledPinHandle) {
    pinHandle = ledPinHandle;

    Task_Params_init(&rxTaskParams);
    rxTaskParams.stackSize = RFEASYLINKEX_TASK_STACK_SIZE;
    rxTaskParams.priority = RFEASYLINKEX_TASK_PRIORITY;
    rxTaskParams.stack = &rxTaskStack;
    rxTaskParams.arg0 = (UInt)1000000;

    Task_construct(&rxTask, rfEasyLinkRxFnx, &rxTaskParams, NULL);
}

void uartTask_init() {
    Task_Params_init(&uartTaskParams);
    rxTaskParams.stackSize = RFEASYLINKEX_TASK_STACK_SIZE;
    rxTaskParams.priority = UART_TASK_PRIORITY;
    rxTaskParams.stack = &uartTaskStack;
    rxTaskParams.arg0 = (UInt)1000000;

    Task_construct(&uartTask, uartFnx, &uartTaskParams, NULL);
}

/*
 *  ======== main ========
 */
int main(void)
{
    /* Call driver init functions */
    Board_initGeneral();

    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, pinTable);
	Assert_isTrue(ledPinHandle != NULL, NULL); 

    /* Clear LED pins */
    PIN_setOutputValue(ledPinHandle, Board_PIN_LED1, 0);
    PIN_setOutputValue(ledPinHandle, Board_PIN_LED2, 0);

    /* Call driver init functions */
    UART_init();

    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 115200;

    uart = UART_open(Board_UART0, &uartParams);

    while (uart == NULL) {
        uart = UART_open(Board_UART0, &uartParams);
    }

    uartTask_init();
    rxTask_init(ledPinHandle);

    /* Start BIOS */
    BIOS_start();

    return (0);
}
