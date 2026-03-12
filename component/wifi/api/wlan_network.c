/******************************************************************************
  *
  * This module is a confidential and proprietary property of RealTek and
  * possession or use of this module requires written permission of RealTek.
  *
  * Copyright(c) 2016, Realtek Semiconductor Corporation. All rights reserved.
  *
******************************************************************************/

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "main.h"

#if CONFIG_WLAN
#include "wifi_conf.h"
#include "wlan_intf.h"
#include "wifi_constants.h"
#include "wifi_structures.h"
#endif
#if CONFIG_LWIP_LAYER
#include "lwip_netconf.h"
#endif
#include <platform_stdlib.h>
#include "osdep_service.h"

#ifndef CONFIG_INIT_NET
#define CONFIG_INIT_NET             1
#endif

#define STACKSIZE                   (512 + 768)

_WEAK void wlan_init_start_time(void)
{

}
_WEAK void wlan_init_end_time(void)
{

}

void init_thread(void *param)
{
	wlan_init_start_time();
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
	rtw_create_secure_context(configMINIMAL_SECURE_STACK_SIZE);
#endif
	/* To avoid gcc warnings */
	(void) param;
#if CONFIG_INIT_NET
#if CONFIG_LWIP_LAYER
	/* Initilaize the LwIP stack */
	LwIP_Init();
#endif
#endif

#if CONFIG_WLAN
	if (wifi_on(RTW_MODE_AP) < 0) {
		printf("\n\r[AP MODE] wifi_on failed\n");
		vTaskDelete(NULL);
		return;
	}

	rtw_softap_info_t softAP_config = {0};

	strncpy((char *)softAP_config.ssid.val, AP_MODE_SSID, sizeof(softAP_config.ssid.val) - 1);
	softAP_config.ssid.len = strlen(AP_MODE_SSID);
	softAP_config.password = (unsigned char *)WPA_PASSPHRASE;
	softAP_config.password_len = strlen(WPA_PASSPHRASE);
	softAP_config.channel = AP_DEFAULT_CH;
	softAP_config.security_type = RTW_SECURITY_WPA2_AES_PSK;
	softAP_config.hidden_ssid = 0;

//	if (wifi_start_ap(&softAP_config) < 0) {
//		printf("\n\r[AP MODE] wifi_start_ap failed\n");
//		vTaskDelete(NULL);
//		return;
//	}

//	printf("\n\r[AP MODE] Started SSID: %s\n", AP_MODE_SSID);
	wifi_on(RTW_MODE_STA);
	printf("\n\r%s(%d), Available heap 0x%x\n", __FUNCTION__, __LINE__, xPortGetFreeHeapSize());
#endif
	wlan_init_end_time();
	/* Kill init thread after all init tasks done */
	vTaskDelete(NULL);
}

void wlan_network()
{
	if (xTaskCreate(init_thread, ((const char *)"init"), STACKSIZE, NULL, tskIDLE_PRIORITY + 3 + PRIORITIE_OFFSET, NULL) != pdPASS) {
		printf("\n\r%s xTaskCreate(init_thread) failed\n", __FUNCTION__);
	}
}
