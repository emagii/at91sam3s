#ifndef _JCONFIG_SAM_
#define _JCONFIG_SAM_

#define NO_GETENV

#undef USE_MSDOS_MEMMGR
#undef USE_MAC_MEMMGR
#define USE_HEAP_MEMMGR // Atmel SAM specific

/* Select right header file */
#if defined   ( __CC_ARM   )
    #include "jconfig_mdk.h"
#elif defined ( __ICCARM__ )
    #include "jconfig_iar.h"
#elif defined (  __GNUC__  )
    #include "jconfig_gcc.h"
#endif


#endif // _JCONFIG_SAM_
