/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "sensor/altitude.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_UORB_FORMAT
static const char fusion_altitude_format[] =
    "fusion_altitude - timestamp:%" PRIu64 ",altitude:%hf";
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

ORB_DEFINE(fusion_altitude, struct fusion_altitude, fusion_altitude_format);
