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
  PROVIDED �AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
static uint8 AJY_TaskID;   //����һ����̬����


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
{ //��������id��ȫ�ֱ���   
  SimpleBLETest_TaskID = task_id;       
 // HalLcdWriteString ( "SimpleBLETest", HAL_LCD_LINE_1);
  // Setup a delayed profile startup  
  /*
  ����һ������ ��ô����Ŀ���ǰ��ն�������ķ�������
  SimpleBLETest_ProcessEvent ���Ǵ��� SBP_START_DEVICE_EVT
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
RegisterForKeys(AJY_TaskID);//170303   ����ע��
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
// ��������ǵ�Ӧ�ó�����¼������� 

uint16 SimpleBLETest_ProcessEvent( uint8 task_id, uint16 events )
{

  VOID task_id; // OSAL required parameter that isn't used in this function

  // SYS_EVENT_MSG ����ϵͳ�¼����簴���¼�������д�¼���������������¼�
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

  // ���������Ӧ�ó����Զ�����¼���SBP_START_DEVICE_EVT ��ֵ������Ϊ 0x0001�� 
  // ʵ�������ǿ��Զ��� 16���¼��� ��һ��ʱ������λ�������
  // ��� SBP_PERIODIC_EVT ������SimpleBLETest_Init��ʼ���������һ�����õ��¼�
  if ( events & SBP_START_DEVICE_EVT )
  {
    HalLedSet(HAL_LED_1, HAL_LED_MODE_ON);      // ����led1               

    // ��������� ����osal�����ʵ�����Ѿ�������
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
        OLED_ShowCHinese(40,2,0);//��ʾ��
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


  // smartRF�������ϵ�S1 ��Ӧ����Դ���ϵ�HAL_KEY_SW_6
    if ( keys & HAL_KEY_POWER )
  {
       OLED_Clear();
  }
 
}



/*********************************************************************
*********************************************************************/



