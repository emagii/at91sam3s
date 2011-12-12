#ifndef _SAM_GUI_PORTING_
#define _SAM_GUI_PORTING_

#include "source/common/sam_gui_errors.h"

#include <stdint.h>

#define SAM_PORTING_NONE               0
#define SAM_PORTING_FREERTOS           1
#define SAM_PORTING_EMBOS              2
#define SAM_PORTING_NUTTX              3

#ifndef SAM_PORTING
	#error SAM_PORTING must be defined...
#  define SAM_PORTING SAM_PORTING_NONE
//#  define SAM_PORTING SAM_PORTING_FREERTOS
#endif

#if !defined SAM_PORTING
	#error SAM_PORTING must be defined
#endif

/**
 * Definitions for FreeRTOs
 */
#if SAM_PORTING == SAM_PORTING_NONE
#  define SAMGUI_TaskHandle uint32_t
#  define SAMGUI_QueueHandle uint32_t
#  define SAMGUI_SemaphoreHandle uint32_t
#endif // SAM_PORTING == SAM_PORTING_NONE

/**
 * Definitions for FreeRTOs
 */
#if SAM_PORTING == SAM_PORTING_FREERTOS
// FreeRTOS includes
#  include "libfreertos.h"
#  define SAMGUI_TaskHandle xTaskHandle
#  define SAMGUI_QueueHandle xQueueHandle
#  define SAMGUI_SemaphoreHandle xSemaphoreHandle
#endif // SAM_PORTING == SAM_PORTING_FREERTOS

// Tasks functions
typedef void (*SAMGUI_fnTask)( void* pvParam ) ;

extern int SAMGUI_TaskCreate( SAMGUI_fnTask fnTask, const uint8_t* pucName, uint8_t* pucStack, uint32_t dwStackSize,
                              uint32_t dwPriority, void* pvParameters, SAMGUI_TaskHandle* pHandle ) ;
extern void SAMGUI_TaskDelete( SAMGUI_TaskHandle Handle ) ;
extern void SAMGUI_TaskDelay( uint32_t dwDelayMs ) ;

// Memory functions
extern void* SAMGUI_Malloc( uint32_t dwSize ) ;
extern void SAMGUI_Free( void *pv ) ;


// Message queue functions
extern SAMGUI_QueueHandle SAMGUI_QueueCreate( uint32_t dwMsgNumber, uint32_t dwMsgSize ) ;

extern uint32_t SAMGUI_QueueReceive( SAMGUI_QueueHandle hQueue, void* pMsg, uint32_t dwDelay ) ;
extern uint32_t SAMGUI_QueueSendToFront( SAMGUI_QueueHandle hQueue, void* pMsg, uint32_t dwDelay )  ;
extern uint32_t SAMGUI_QueueSendToBack( SAMGUI_QueueHandle hQueue, void* pMsg, uint32_t dwDelay ) ;
extern uint32_t SAMGUI_QueueSendToFrontFromISR( SAMGUI_QueueHandle hQueue, void* pMsg )  ;
extern uint32_t SAMGUI_QueueSendToBackFromISR( SAMGUI_QueueHandle hQueue, void* pMsg ) ;

// Semaphore functions
extern SAMGUI_SemaphoreHandle SAMGUI_SemaphoreCreate( void ) ;
extern uint32_t SAMGUI_SemaphoreTake( SAMGUI_SemaphoreHandle hSem, uint32_t dwDelay ) ;
extern uint32_t SAMGUI_SemaphoreRelease( SAMGUI_SemaphoreHandle hSem ) ;

#endif // _SAM_GUI_PORTING_
