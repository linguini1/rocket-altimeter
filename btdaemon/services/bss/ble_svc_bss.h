#ifndef H_BLE_SVC_BSS_
#define H_BLE_SVC_BSS_

/* This implementation is based on the Bluetooth specification for the Binary
 * Sensor Service:
 * https://www.bluetooth.com/specifications/specs/binary-sensor-service-1-0/
 */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 16 bit Binary Sensor Service UUID */

#define BLE_SVC_BSS_UUID16 (0x183b)

/* 16 Bit Binary Sensor Service Characteristic UUIDs */

#define BLE_SVC_BSS_CHR_UUID16_BSS_CONTROL_POINT (0x2b2b)
#define BLE_SVC_BSS_CHR_UUID16_BSS_RESPONSE (0x2b2c)

/* BSS Control Point command IDs */

#define BLE_SVC_BSS_CMD_GET_SEN_STATUS (0x00)
#define BLE_SVC_BSS_CMD_SET_SEN (0x02)

/* BSS Response response IDs */

#define BLE_SVC_BSS_RESP_GET_SEN_STATUS (0x01)
#define BLE_SVC_BSS_RESP_SEN_SET (0x03)
#define BLE_SVC_BSS_RESP_SEN_STATUS_EVENT (0x04)

/* Error Definitions */

/* TODO correct? */

#define BLE_SVC_BSS_ERR_CMD_NOT_SUPPORTED BLE_ERR_CMD_DISALLOWED

/* Parameter IDs */

#define BLE_SVC_BSS_PRM_RESULT (0x00)
#define BLE_SVC_BSS_PRM_CANCEL (0x01)
#define BLE_SVC_BSS_PRM_SENTYPE (0x02)
#define BLE_SVC_BSS_PRM_REPSTAT (0x03)
#define BLE_SVC_BSS_PRM_SENSTAT (0x0a)
#define BLE_SVC_BSS_PRM_MSENSTAT (0x0b)
#define BLE_SVC_BSS_PRM_NAME (0x0c)

/* Sensor types */

#define BLE_SVC_BSS_SENTYPE_OPCLOSE (0x00)   /* Opening & closing */
#define BLE_SVC_BSS_SENTYPE_HUMAN (0x01)     /* Human detection */
#define BLE_SVC_BSS_SENTYPE_VIBE (0x02)      /* Vibration detection */
#define BLE_SVC_BSS_SENTYPE_OPCLOSE_M (0x80) /* Multi open & close */
#define BLE_SVC_BSS_SENTYPE_HUMAN_M (0x81)   /* Multi human detection */
#define BLE_SVC_BSS_SENTYPE_VIBE_M (0x82)    /* Multi vibration detection */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* TODO: Set values of binary sensor services etc. */

void ble_svc_bss_init(void);

#endif /* H_BLE_SVC_BSS_ */
