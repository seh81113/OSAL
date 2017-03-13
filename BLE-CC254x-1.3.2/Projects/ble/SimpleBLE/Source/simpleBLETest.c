/**************************************************************************************************
  Filename:       SimpleBLETest.c
  Revised:        $Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
  Revision:       $Revision: 23333 $

  Description:    This file contains the Simple BLE Peripheral sample application
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2010 - 2013 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"
#include "simpleBLETest.h"
#include "oled.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */


/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8 SimpleBLETest_TaskID;   // Task ID for internal task/event processing
static uint8 AJY_TaskID;   //定义一个静态变量


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLETest_Init
 *
 * @brief   Initialization function for the Simple BLE Peripheral App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SimpleBLETest_Init( uint8 task_id )
{ //保存任务id到全局变量   
  SimpleBLETest_TaskID = task_id;       
 // HalLcdWriteString ( "SimpleBLETest", HAL_LCD_LINE_1);
  // Setup a delayed profile startup  
  /*
  设置一个任务， 这么做的目的是按照多任务处理的方法来做
  SimpleBLETest_ProcessEvent 就是处理 SBP_START_DEVICE_EVT
  */
  
  osal_set_event(SimpleBLETest_TaskID,SBP_START_DEVICE_EVT);
}


 void AJY_Init(uint8 task_id)
{
AJY_TaskID=task_id;
    I2CCFG = 0;             
    I2CWC |= 0x83;
    I2CIO |=0x0;
    OLED_Init();	
    OLED_Clear(); 
RegisterForKeys(AJY_TaskID);//170303   按键注册
osal_set_event(AJY_TaskID, AJYP_START_OLED_IIC);
}
 	
/*********************************************************************
 * @fn      SimpleBLETest_ProcessEvent
 *
 * @brief   Simple BLE Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
// 这个是我们的应用程序的事件处理函数 

uint16 SimpleBLETest_ProcessEvent( uint8 task_id, uint16 events )
{

  VOID task_id; // OSAL required parameter that isn't used in this function

  // SYS_EVENT_MSG 这是系统事件比如按键事件蓝牙读写事件处理，都会置这个事件
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( SimpleBLETest_TaskID )) != NULL )
    {
      simpleBLECentral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // 这个是我们应用程序自定义的事件，SBP_START_DEVICE_EVT 的值被定义为 0x0001， 
  // 实际上我们可以定义 16个事件， 第一的时候是以位来定义的
  // 这个 SBP_PERIODIC_EVT 就是在SimpleBLETest_Init初始化函数最后一行设置的事件
  if ( events & SBP_START_DEVICE_EVT )
  {
    HalLedSet(HAL_LED_1, HAL_LED_MODE_ON);      // 点亮led1               

    // 返回这个， 告诉osal，这个实践你已经处理了
    return ( events ^ SBP_START_DEVICE_EVT );   
  }

  // Discard unknown events
  return 0;
}

uint16 AJY_ProcessEvent(uint8 task_id,uint16 events)
{
//void task_id;

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( AJY_TaskID )) != NULL )
    {
      simpleBLECentral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }



if (events&AJYP_START_OLED_IIC)
        {
        OLED_ShowCHinese(40,2,0);//显示℃
        return (events ^ AJYP_START_OLED_IIC);
}
            
return 0;
}

            //simpleBLECentral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );
 void simpleBLECentral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
    case KEY_CHANGE:
      simpleBLECentral_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;

    case GATT_MSG_EVENT:
      //simpleBLECentralProcessGATTMsg( (gattMsgEvent_t *) pMsg );
      break;
  }
}




static void simpleBLECentral_HandleKeys( uint8 shift, uint8 keys )
{
  (void)shift;  // Intentionally unreferenced parameter


  // smartRF开发板上的S1 对应我们源码上的HAL_KEY_SW_6
    if ( keys & HAL_KEY_POWER )
  {
       OLED_Clear();
  }
 
}



/*********************************************************************
*********************************************************************/



