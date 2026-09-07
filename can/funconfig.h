#ifndef _FUNCONFIG_H
#define _FUNCONFIG_H

#include "appconfig.h"

#ifdef APPCONF_UART
#define FUNCONF_NULL_PRINTF 1
#else
// Though this should be on by default we can extra force it on.
#define FUNCONF_USE_DEBUGPRINTF 1
#define FUNCONF_DEBUGPRINTF_TIMEOUT (1 << 31) // Wait for a very very long time.
#endif

#define FUNCONF_PLL_MULTIPLIER 18
#define FUNCONF_USE_HSI 1
#define FUNCONF_USE_PLL 1

#endif
