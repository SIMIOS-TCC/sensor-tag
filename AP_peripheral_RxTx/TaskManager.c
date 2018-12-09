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
/* Custom includes */
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>   // for the typedefs (redundant, actually)
#include <inttypes.h> // for the macros
#include <time.h>
#include <math.h>

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>

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
#define RX_RFEASYLINKEX_TASK_PRIORITY   2
#define TX_RFEASYLINKEX_TASK_PRIORITY 3

#define RFEASYLINKTX_BURST_SIZE         10
#define RFEASYLINKTXPAYLOAD_LENGTH      30

#define ELECTROMAGNETIC_CTE 2.95
#define RSSI_1M 55
#define BUFFER_SIZE 28 // 4 pacotes : RFEASYLINKTXPAYLOAD_LENGTH/(count sending variables) = 29/4 [my_id,timestamp][id,rssi,2xtimestamp] : 7 medidas por pacote sobra 1 byte
#define QT_MEASURES 10
#define MEM_STACK_SIZE 10
#define QT_PACKETS 4
#define TIME_DELAY 1

#define MY_ID 1

/***** Variable declarations *****/
static Task_Params txTaskParams;
Task_Struct txTask;    /* not static so you can see in ROV */
static uint8_t txTaskStack[RFEASYLINKEX_TASK_STACK_SIZE];

static Task_Params rxTaskParams;
Task_Struct rxTask;    /* not static so you can see in ROV */
static uint8_t rxTaskStack[RFEASYLINKEX_TASK_STACK_SIZE];

//Storing data variables
struct Measure
{
        uint8_t id;
        int8_t rssi[QT_MEASURES];
        uint8_t counter;
        int local_time;
};

struct Measure memStack[MEM_STACK_SIZE][BUFFER_SIZE];
int mem_stack_dumper_counter = 0;
int mem_stack_counter = 0;
int mem_stack_filler_counter = 0;

void addMeasureRssi(struct Measure* m, int8_t rssi) {
    m->rssi[m->counter++] = rssi;
}

int8_t getAverageRssi(struct Measure* m) {
    int sum = 0;

    uint8_t i;
    for(i = 0; i < m->counter; i++) {
        sum += m->rssi[i];
    }

    return sum/(m->counter);
}

int findMeasureByIdTimestamp(uint8_t id, int local_time) {
    int i;
    for(i = 0; i < mem_stack_filler_counter; i++) {
        struct Measure m = memStack[mem_stack_counter][i];
        if(m.counter < BUFFER_SIZE &&
                m.id == id &&
                m.local_time >= local_time - TIME_DELAY) {
            return i;
        }
    }

    return BUFFER_SIZE;
}

/* The RX Output struct contains statistics about the RX operation of the radio */
PIN_Handle pinHandle;

#ifdef RFEASYLINKRX_ASYNC
static Semaphore_Handle doneSemaphore;
#endif

/***** Function definitions *****/
//float getRelativeDistance(uint8_t rssi) {
//    //RSSI = -10*n*log10(d) + A
//    return pow(10, ((RSSI_1M - rssi)/(-10*ELECTROMAGNETIC_CTE)));
//}

#ifdef RFEASYLINKRX_ASYNC
void rxDoneCb(EasyLink_RxPacket * rxPacket, EasyLink_Status status)
{
    if (status == EasyLink_Status_Success)
    {
        if(((int*)rxPacket->dstAddr)[0] == 0xAA) {

            uint8_t id = rxPacket->payload[0];
            int8_t rssi = (-1)*rxPacket->rssi;

            time_t t = time(NULL);
            struct tm tm = *localtime(&t);
            int local_time = ( ( ( ( (tm.tm_year - 70)*12 + tm.tm_mon )*30 + (tm.tm_mday - 1) )*24 + tm.tm_hour )*60 + tm.tm_min )*60 + tm.tm_sec;

            int measureId = findMeasureByIdTimestamp(id, local_time);
            if(measureId < BUFFER_SIZE) {
                addMeasureRssi(&(memStack[mem_stack_counter][measureId]), rssi);
            } else {
                struct Measure m;
                m.id = id;
                m.counter = 0;
                addMeasureRssi(&m, rssi);
                m.local_time = local_time;
                memStack[mem_stack_counter][mem_stack_filler_counter++] = m;
            }
        }

        //Checks if memory stack is full and updates counters
        if(mem_stack_filler_counter == BUFFER_SIZE) {
            mem_stack_filler_counter = 0;

            mem_stack_counter++;
            if(mem_stack_counter == MEM_STACK_SIZE) {
                mem_stack_counter = 0;
            }
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

    Semaphore_post(doneSemaphore);
}
#endif

void txDoneCb(EasyLink_Status status)
{
    if (status == EasyLink_Status_Success)
    {
        mem_stack_dumper_counter++;
        if(mem_stack_dumper_counter == MEM_STACK_SIZE) {
            mem_stack_dumper_counter = 0;
        }

        /* Toggle LED1 to indicate TX */
        PIN_setOutputValue(pinHandle, Board_PIN_LED1,!PIN_getOutputValue(Board_PIN_LED1));
    }
    else if(status == EasyLink_Status_Aborted)
    {
        /* Toggle LED2 to indicate command aborted */
        PIN_setOutputValue(pinHandle, Board_PIN_LED2,!PIN_getOutputValue(Board_PIN_LED2));
    }
    else
    {
        /* Toggle LED1 and LED2 to indicate error */
        PIN_setOutputValue(pinHandle, Board_PIN_LED1,!PIN_getOutputValue(Board_PIN_LED1));
        PIN_setOutputValue(pinHandle, Board_PIN_LED2,!PIN_getOutputValue(Board_PIN_LED2));
    }

    Semaphore_post(doneSemaphore);
}

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
    doneSemaphore = Semaphore_create(0, &params, &eb);
    if(doneSemaphore == NULL)
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
        if(Semaphore_pend(doneSemaphore, (300000 / Clock_tickPeriod)) == FALSE)
        {
            /* RX timed out abort */
            if(EasyLink_abort() == EasyLink_Status_Success)
            {
               /* Wait for the abort */
               Semaphore_pend(doneSemaphore, BIOS_WAIT_FOREVER);
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

static void rfEasyLinkTxFnx(UArg arg0, UArg arg1)
{
    uint8_t txBurstSize = 0;
    uint32_t absTime;

#ifdef RFEASYLINKTX_ASYNC
    /* Create a semaphore for Async */
    Semaphore_Params params;
    Error_Block eb;

    /* Init params */
    Semaphore_Params_init(&params);
    Error_init(&eb);

    /* Create semaphore instance */
    doneSemaphore = Semaphore_create(0, &params, &eb);
    if(doneSemaphore == NULL)
    {
        System_abort("Semaphore creation failed");
    }

#endif //TX_ASYNC

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

#if (defined __CC1352P1_LAUNCHXL_BOARD_H__)
    /* Set output power to 20dBm */
    EasyLink_Status pwrStatus = EasyLink_setRfPower(20);
#else
    /* Set output power to 12dBm */
    EasyLink_Status pwrStatus = EasyLink_setRfPower(12);
#endif
    if(pwrStatus != EasyLink_Status_Success)
    {
        // There was a problem setting the transmission power
        while(1);
    }

    while(1) {

        if(mem_stack_dumper_counter == mem_stack_counter) {
            continue;
        }

        EasyLink_TxPacket txPacket =  { {0}, 0, 0, {0} };

        /* Create packet with buffer on payload */
        txPacket.payload[0] = (uint8_t)(MY_ID);

        uint8_t i = 1;
        uint8_t data_counter = 0;
        while(i < RFEASYLINKTXPAYLOAD_LENGTH - 3) {
            struct Measure m = memStack[mem_stack_dumper_counter][data_counter];

            time_t t = time(NULL);
            struct tm tm = *localtime(&t);
            uint16_t deltaTime = (uint16_t)((( ( ( ( (tm.tm_year - 70)*12 + tm.tm_mon )*30 +
                    (tm.tm_mday - 1) )*24 + tm.tm_hour )*60 + tm.tm_min )*60 + tm.tm_sec)
                    - m.local_time);
            uint8_t deltaTimeFirstByte = (uint8_t)(deltaTime/256);
            uint8_t deltaTimeSecondByte = (uint8_t)(deltaTime - deltaTimeFirstByte*256);

            txPacket.payload[i++] = m.id;
            txPacket.payload[i++] = getAverageRssi(&m);
            txPacket.payload[i++] = deltaTimeFirstByte;
            txPacket.payload[i++] = deltaTimeSecondByte;

            data_counter++;
        }

        txPacket.len = RFEASYLINKTXPAYLOAD_LENGTH;
        txPacket.dstAddr[0] = 0xbb;

        /* Add a Tx delay for > 500ms, so that the abort kicks in and brakes the burst */
        if(EasyLink_getAbsTime(&absTime) != EasyLink_Status_Success)
        {
            // Problem getting absolute time
        }
        if(txBurstSize++ >= RFEASYLINKTX_BURST_SIZE)
        {
          /* Set Tx absolute time to current time + 1s */
          txPacket.absTime = absTime + EasyLink_ms_To_RadioTime(1000);
          txBurstSize = 0;
        }
        /* Else set the next packet in burst to Tx in 100ms */
        else
        {
          /* Set Tx absolute time to current time + 100ms */
          txPacket.absTime = absTime + EasyLink_ms_To_RadioTime(100);
        }

#ifdef RFEASYLINKTX_ASYNC
        EasyLink_transmitAsync(&txPacket, txDoneCb);
        /* Wait 300ms for Tx to complete */
        if(Semaphore_pend(doneSemaphore, (300000 / Clock_tickPeriod)) == FALSE)
        {
            /* TX timed out, abort */
            if(EasyLink_abort() == EasyLink_Status_Success)
            {
                /*
                 * Abort will cause the txDoneCb to be called and the doneSemaphore
                 * to be released, so we must consume the doneSemaphore
                 */
               Semaphore_pend(doneSemaphore, BIOS_WAIT_FOREVER);
            }
        }
#else
        EasyLink_Status result = EasyLink_transmit(&txPacket);

        if (result == EasyLink_Status_Success)
        {
            /* Toggle LED1 to indicate TX */
            PIN_setOutputValue(pinHandle, Board_PIN_LED1,!PIN_getOutputValue(Board_PIN_LED1));
        }
        else
        {
            /* Toggle LED1 and LED2 to indicate error */
            PIN_setOutputValue(pinHandle, Board_PIN_LED1,!PIN_getOutputValue(Board_PIN_LED1));
            PIN_setOutputValue(pinHandle, Board_PIN_LED2,!PIN_getOutputValue(Board_PIN_LED2));
        }
#endif //RFEASYLINKTX_ASYNC
    }
}

void rxTask_init(PIN_Handle ledPinHandle) {
    pinHandle = ledPinHandle;

    Task_Params_init(&rxTaskParams);
    rxTaskParams.stackSize = RFEASYLINKEX_TASK_STACK_SIZE;
    rxTaskParams.priority = RX_RFEASYLINKEX_TASK_PRIORITY;
    rxTaskParams.stack = &rxTaskStack;
    rxTaskParams.arg0 = (UInt)1000000;

    Task_construct(&rxTask, rfEasyLinkRxFnx, &rxTaskParams, NULL);
}

void txTask_init(PIN_Handle ledPinHandle) {
    pinHandle = ledPinHandle;

    Task_Params_init(&txTaskParams);
    txTaskParams.stackSize = RFEASYLINKEX_TASK_STACK_SIZE;
    txTaskParams.priority = TX_RFEASYLINKEX_TASK_PRIORITY;
    txTaskParams.stack = &txTaskStack;
    txTaskParams.arg0 = (UInt)1000000;

    Task_construct(&txTask, rfEasyLinkTxFnx, &txTaskParams, NULL);
}

void TaskManager_init(PIN_Handle ledPinHandle) {
    rxTask_init(ledPinHandle);
    txTask_init(ledPinHandle);
}
