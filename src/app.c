/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It
    implements the logic of the application's state machine and it may call
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include "timers.h"
#include "queue.h"
#include "msgQAPI.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

#define NUM_TIMERS 1
#define TICKS_TO_WAIT 0
#define QUEUE_ITEMS 5

unsigned int TIMER_ID = 0;

char name[] = {'B',  'r', 'a', 'e', 'd', 'o', 'n', ' ', 'D',
               'i', 'c', 'k', 'e', 'r', 's', 'o', 'n', ' '};

unsigned int counter = 0;

QueueHandle_t xQueue1;
TimerHandle_t xTimers[ NUM_TIMERS ];

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;
MSG_DATA msgData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

void vTimerCallback( TimerHandle_t time )
{
    if ( xQueue1 )
    {
        MsgQSendChar(name[counter]);
                  
        if (counter != 17)
            counter++;
        else
            counter = 0;
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void MsgQSendInt ( unsigned int val )

  Remarks:
    Sends an int to the message queue
 */
void MsgQSendInt( unsigned int val )
{
    if (xQueue1)
    {
        msgData.type = INT;
        memset(msgData.data, val, sizeof(val));
        xQueueSend ( xQueue1,
                    (void *) &msgData,
                    (TickType_t) TICKS_TO_WAIT );
    }
}

/*******************************************************************************
  Function:
    void MsgQSendChar ( unsigned char val )

  Remarks:
    Sends a char to the message queue
 */
void MsgQSendChar( unsigned char val )
{
    if (xQueue1)
    {
        msgData.type = CHAR;
        memset(msgData.data, val, sizeof(val));
        
        xQueueSend ( xQueue1,
                    (void *) &msgData,
                    (TickType_t) TICKS_TO_WAIT );
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    
    xTimers[NUM_TIMERS - 1] = xTimerCreate (    "Timer1",
                                                50/portTICK_PERIOD_MS,
                                                pdTRUE,
                                                ( void * ) TIMER_ID,
                                                vTimerCallback
                                           );
    xTimerStart(xTimers[NUM_TIMERS - 1], 0);
    
    xQueue1 = xQueueCreate( QUEUE_ITEMS, sizeof( MSG_DATA ) );
    
    PLIB_PORTS_DirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_E, 
                                                  0X00ff);

    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    MSG_DATA msgRecieved;
    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {

            while (1)
            {
                if (xQueue1)
                {
                    if (xQueueReceive ( xQueue1, &(msgRecieved), (TickType_t) TICKS_TO_WAIT))
                    {
                        if (msgRecieved.type == CHAR)
                            PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_E, (char) msgRecieved.data);
                    }
                }
            }

            break;
        }

        /* TODO: implement your application state machine.*/

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}


/*******************************************************************************
 End of File
 */
