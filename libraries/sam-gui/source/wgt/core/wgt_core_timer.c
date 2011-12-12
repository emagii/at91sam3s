#include "libsam_gui.h"

#include <string.h>

/**
 * \addtogroup SAMGUI
 * @{
 *   \addtogroup SAMGUI_WGT
 *   @{
 *     \addtogroup SAMGUI_WGT_CORE
 *     @{
 *       \addtogroup SAMGUI_WGT_CORE_TIMERS WGT Core Timers
 *       @{
 */

static __no_init SWGTTimer* gs_apWGTTimers[WGT_TIMER_MAX] ;

extern uint32_t WGT_Timer_Create( SWGTTimer* pTimer, uint32_t dwID, uint32_t dwDelay )
{
    return SAMGUI_E_OK ;
}

extern uint32_t WGT_Timer_Start( SWGTTimer* pTimer )
{
    return SAMGUI_E_OK ;
}

extern uint32_t WGT_Timer_Stop( SWGTTimer* pTimer )
{
    return SAMGUI_E_OK ;
}

extern uint32_t WGT_Timer_Initialize( void )
{
    memset( gs_apWGTTimers, 0, sizeof( gs_apWGTTimers ) ) ;

    return SAMGUI_E_OK ;
}

extern uint32_t WGT_Timer_Process( void )
{
    return SAMGUI_E_OK ;
}

/** @}
 * @}
 * @}
 * @} */
