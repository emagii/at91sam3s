#include "sam_gui_porting.h"

#if SAM_PORTING == SAM_PORTING_FREERTOS

extern void SAMGUI_TaskDelay( uint32_t dwDelayMs )
{
    vTaskDelay( dwDelayMs/portTICK_RATE_MS ) ;
}

extern int SAMGUI_TaskCreate( SAMGUI_fnTask fnTask, const uint8_t* pucName, uint8_t* pucStack, uint32_t dwStackSize,
                              uint32_t dwPriority, void* pvParameters, SAMGUI_TaskHandle* pHandle )
{
    xTaskCreate( fnTask, (signed char const*)pucName, dwStackSize, pvParameters, tskIDLE_PRIORITY+dwPriority, pHandle ) ;

    if ( pHandle != NULL )
    {
        return SAMGUI_E_OK ;
    }

    return SAMGUI_E_OS_TASK_CREATE_FAILED ;
}

extern void SAMGUI_TaskDelete( SAMGUI_TaskHandle Handle )
{
    vTaskDelete( Handle ) ;
}

extern void* SAMGUI_Malloc( uint32_t dwSize )
{
    return pvPortMalloc( dwSize ) ;
}

extern void SAMGUI_Free( void *pv )
{
    vPortFree( pv ) ;
}


extern SAMGUI_QueueHandle SAMGUI_QueueCreate( uint32_t dwMsgNumber, uint32_t dwMsgSize )
{
    return xQueueCreate( dwMsgNumber, dwMsgSize ) ;
}

extern uint32_t SAMGUI_QueueReceive( SAMGUI_QueueHandle hQueue, void* pMsg, uint32_t dwDelay )
{
    uint32_t dw ;

    if ( dwDelay == portMAX_DELAY )
    {
        dw=xQueueReceive( hQueue, pMsg, dwDelay ) ;
    }
    else
    {
        dw=xQueueReceive( hQueue, pMsg, dwDelay/portTICK_RATE_MS ) ;
    }

    if ( dw != pdTRUE )
    {
        return SAMGUI_E_MSQUEUE ;
    }

    return SAMGUI_E_OK ;
}

extern uint32_t SAMGUI_QueueSendToFront( SAMGUI_QueueHandle hQueue, void* pMsg, uint32_t dwDelay )
{
    /* Send message */
    if ( xQueueSendToFront( hQueue, pMsg, (portTickType)dwDelay ) != pdPASS )
    {
        return SAMGUI_E_MSQUEUE ;
    }

    return SAMGUI_E_OK ;
}

extern uint32_t SAMGUI_QueueSendToBack( SAMGUI_QueueHandle hQueue, void* pMsg, uint32_t dwDelay )
{
    /* Send message */
    if ( xQueueSendToBack( hQueue, pMsg, (portTickType)dwDelay ) != pdPASS )
    {
        return SAMGUI_E_MSQUEUE ;
    }

    return SAMGUI_E_OK ;
}

extern uint32_t SAMGUI_QueueSendToFrontFromISR( SAMGUI_QueueHandle hQueue, void* pMsg )
{
    /* Send message */
    if ( xQueueSendToFrontFromISR( hQueue, pMsg, NULL ) != pdPASS )
    {
        return SAMGUI_E_MSQUEUE ;
    }

    return SAMGUI_E_OK ;
}

extern uint32_t SAMGUI_QueueSendToBackFromISR( SAMGUI_QueueHandle hQueue, void* pMsg )
{
    /* Send message */
    if ( xQueueSendToBackFromISR( hQueue, pMsg, (portTickType)0 ) != pdPASS )
    {
        return SAMGUI_E_MSQUEUE ;
    }

    return SAMGUI_E_OK ;
}

extern SAMGUI_SemaphoreHandle SAMGUI_SemaphoreCreate( void )
{
    return xSemaphoreCreateMutex() ;
}

extern uint32_t SAMGUI_SemaphoreTake( SAMGUI_SemaphoreHandle hSem, uint32_t dwDelay )
{
    if ( xSemaphoreTake( hSem, (portTickType)dwDelay ) == pdTRUE )
    {
        return SAMGUI_E_OK ;
    }

    return SAMGUI_E_OS_SEM_TAKE_FAILED ;
}

extern uint32_t SAMGUI_SemaphoreRelease( SAMGUI_SemaphoreHandle hSem )
{
    if ( xSemaphoreGive( hSem ) == pdTRUE )
    {
        return SAMGUI_E_OK ;
    }

    return SAMGUI_E_OS_SEM_RELEASE_FAILED ;
}




#endif // SAM_PORTING == SAM_PORTING_FREERTOS
