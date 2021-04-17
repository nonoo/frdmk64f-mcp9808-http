/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "lwip/opt.h"

#if LWIP_IPV4 && LWIP_DHCP && LWIP_NETCONN

#include "lwip/dhcp.h"
#include "lwip/ip_addr.h"
#include "lwip/netifapi.h"
#include "lwip/prot/dhcp.h"
#include "lwip/tcpip.h"
#include "lwip/sys.h"
#include "enet_ethernetif.h"

#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_phy.h"

#include "fsl_device_registers.h"
#include "fsl_phyksz8081.h"
#include "fsl_enet_mdio.h"

#include "config.h"
#include "httpcln.h"
#include "float.h"
#include "common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Address of PHY interface. */
#define EXAMPLE_PHY_ADDRESS BOARD_ENET0_PHY_ADDRESS

/* MDIO operations. */
#define EXAMPLE_MDIO_OPS enet_ops

/* PHY operations. */
#define EXAMPLE_PHY_OPS phyksz8081_ops

/* ENET clock frequency. */
#define EXAMPLE_CLOCK_FREQ CLOCK_GetFreq(kCLOCK_CoreSysClk)

/* GPIO pin configuration. */
#define BOARD_LED_GPIO       BOARD_LED_RED_GPIO
#define BOARD_LED_GPIO_PIN   BOARD_LED_RED_GPIO_PIN
#define BOARD_SW_GPIO        BOARD_SW3_GPIO
#define BOARD_SW_GPIO_PIN    BOARD_SW3_GPIO_PIN
#define BOARD_SW_PORT        BOARD_SW3_PORT
#define BOARD_SW_IRQ         BOARD_SW3_IRQ
#define BOARD_SW_IRQ_HANDLER BOARD_SW3_IRQ_HANDLER


#ifndef EXAMPLE_NETIF_INIT_FN
/*! @brief Network interface initialization function. */
#define EXAMPLE_NETIF_INIT_FN ethernetif0_init
#endif /* EXAMPLE_NETIF_INIT_FN */

// These values represent uint16 words.
#define DHCP_THREAD_STACKSIZE 256
#define TEMP_THREAD_STACKSIZE 1024

/*! @brief Priority of the thread which prints DHCP info. */
#define DHCP_THREAD_PRIO DEFAULT_THREAD_PRIO
#define TEMP_THREAD_PRIO DEFAULT_THREAD_PRIO

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

static mdio_handle_t mdioHandle = {.ops = &EXAMPLE_MDIO_OPS};
static phy_handle_t phyHandle   = {.phyAddr = EXAMPLE_PHY_ADDRESS, .mdioHandle = &mdioHandle, .ops = &EXAMPLE_PHY_OPS};

static struct {
	SemaphoreHandle_t mutex;

	uint8_t dhcp_last_state;
} app_state = {
	.dhcp_last_state = DHCP_STATE_OFF,
};

/*******************************************************************************
 * Code
 ******************************************************************************/

static double temp_read(void) {
	// TODO
	return 12.34;
}

static void temp_thread(void *arg) {
	char url[HTTPCLN_MAX_HTTP_URL_PATH_LENGTH];
	httpcln_params_t httpcln_params;
	httpcln_result_t http_res;
	bool is_dhcp_up;
	double temp;
	char tmp[10];
	int f_int;
	char s_frac[5];
	int i;

	LED_RED_INIT(1);
	LED_GREEN_INIT(1);

	while (1) {
        xSemaphoreTake(app_state.mutex, portMAX_DELAY);
        is_dhcp_up = (app_state.dhcp_last_state == DHCP_STATE_BOUND);
        xSemaphoreGive(app_state.mutex);

        if (!is_dhcp_up) {
        	LED_RED_TOGGLE();
        	LED_GREEN_TOGGLE();
        	vTaskDelay(1000 / portTICK_PERIOD_MS);
        	continue;
        }

        LED_RED_ON();
        LED_GREEN_ON();

        temp = temp_read();

        float_explode(temp, &f_int, s_frac, sizeof(s_frac));
        if (temp < 0)
        	i = snprintf(url, sizeof(url), URL_BASE "-%u.%s", abs(f_int), s_frac);
        else
        	i = snprintf(url, sizeof(url), URL_BASE "%u.%s", f_int, s_frac);

        // Trimming zeroes from the end.
        while (url[--i] == '0')
        	url[i] = 0;

        memset(&httpcln_params, 0, sizeof(httpcln_params_t));
        httpcln_params.max_resp_content_length = 128;
        http_res = httpcln_get(url, &httpcln_params, NULL);

        switch (http_res) {
       	case HTTPCLN_RESULT_OK:
            LED_RED_OFF();
       		PRINTF("temp: send ok\n");
       		vTaskDelay(1000 / portTICK_PERIOD_MS);
            LED_GREEN_OFF();
            for (i = TEMP_READ_INTERVAL_MS-1000; i > 0; i -= 10000) {
            	LED_GREEN_ON();
            	vTaskDelay(100 / portTICK_PERIOD_MS);
            	LED_GREEN_OFF();
            	vTaskDelay(min(i, 9900) / portTICK_PERIOD_MS);
            }
       		break;
       	default:
            LED_GREEN_OFF();
       		PRINTF("temp: http err, res %d\n", http_res);
            for (i = TEMP_READ_INTERVAL_MS; i > 0; i -= 1000) {
            	vTaskDelay(1000 / portTICK_PERIOD_MS);
            	LED_RED_TOGGLE();
            }
        	LED_RED_OFF();
       		break;
       	}
	}

    vTaskDelete(NULL);
}

static void dhcp_thread(void *arg) {
    struct netif *netif = (struct netif *)arg;
    struct dhcp *dhcp;
    bool last_eth_link_state;
    bool is_eth_link_up;

    while (1) {
    	// https://community.nxp.com/t5/i-MX-RT/Best-way-to-check-Ethernet-link/m-p/925844#M3714
    	PHY_GetLinkStatus(&phyHandle, &is_eth_link_up);
    	if (last_eth_link_state != is_eth_link_up) {
    		if (is_eth_link_up) {
    			PRINTF("dhcp: eth link up\n");
    		    netifapi_dhcp_start(netif);
    		} else {
    			PRINTF("dhcp: eth link down\n");
    			netifapi_dhcp_stop(netif);
    		}
    		last_eth_link_state = is_eth_link_up;
    	}

        dhcp = netif_dhcp_data(netif);

        xSemaphoreTake(app_state.mutex, portMAX_DELAY);

        if (dhcp == NULL)
            app_state.dhcp_last_state = DHCP_STATE_OFF;
        else if (app_state.dhcp_last_state != dhcp->state) {
            app_state.dhcp_last_state = dhcp->state;

            PRINTF("dhcp: state ");
            switch (app_state.dhcp_last_state) {
                case DHCP_STATE_OFF: PRINTF("off"); break;
                case DHCP_STATE_REQUESTING: PRINTF("requesting"); break;
                case DHCP_STATE_INIT: PRINTF("init"); break;
                case DHCP_STATE_REBOOTING: PRINTF("rebooting"); break;
                case DHCP_STATE_REBINDING: PRINTF("rebinding"); break;
                case DHCP_STATE_RENEWING: PRINTF("renewing"); break;
                case DHCP_STATE_SELECTING: PRINTF("selecting"); break;
                case DHCP_STATE_INFORMING: PRINTF("informing"); break;
                case DHCP_STATE_CHECKING: PRINTF("checking"); break;
                case DHCP_STATE_BOUND: PRINTF("bound"); break;
                case DHCP_STATE_BACKING_OFF: PRINTF("backing off"); break;
                default:
                	PRINTF("%u", app_state.dhcp_last_state);
                	assert(0);
                	break;
            }
            PRINTF("\n");

            if (app_state.dhcp_last_state == DHCP_STATE_BOUND) {
                PRINTF("dhcp: bound to addr %s mask %s gw %s\n", ipaddr_ntoa(&netif->ip_addr),
                		ipaddr_ntoa(&netif->netmask), ipaddr_ntoa(&netif->gw));
            }
        }

        xSemaphoreGive(app_state.mutex);

        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

int main(void) {
    static struct netif netif;
#if defined(FSL_FEATURE_SOC_LPC_ENET_COUNT) && (FSL_FEATURE_SOC_LPC_ENET_COUNT > 0)
    static mem_range_t non_dma_memory[] = NON_DMA_MEMORY_ARRAY;
#endif /* FSL_FEATURE_SOC_LPC_ENET_COUNT */
    ip4_addr_t netif_ipaddr, netif_netmask, netif_gw;
    ethernetif_config_t enet_config = {
        .phyHandle  = &phyHandle,
        .macAddress = MAC_ADDR,
#if defined(FSL_FEATURE_SOC_LPC_ENET_COUNT) && (FSL_FEATURE_SOC_LPC_ENET_COUNT > 0)
        .non_dma_memory = non_dma_memory,
#endif /* FSL_FEATURE_SOC_LPC_ENET_COUNT */
    };

    SYSMPU_Type *base = SYSMPU;
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();
    /* Disable SYSMPU. */
    base->CESR &= ~SYSMPU_CESR_VLD_MASK;

    mdioHandle.resource.csrClock_Hz = EXAMPLE_CLOCK_FREQ;

    IP4_ADDR(&netif_ipaddr, 0U, 0U, 0U, 0U);
    IP4_ADDR(&netif_netmask, 0U, 0U, 0U, 0U);
    IP4_ADDR(&netif_gw, 0U, 0U, 0U, 0U);

    tcpip_init(NULL, NULL);

    netifapi_netif_add(&netif, &netif_ipaddr, &netif_netmask, &netif_gw, &enet_config, EXAMPLE_NETIF_INIT_FN,
                       tcpip_input);
    netifapi_netif_set_default(&netif);
    netifapi_netif_set_up(&netif);

    app_state.mutex = xSemaphoreCreateMutex();

    if (xTaskCreate(temp_thread, "temp", TEMP_THREAD_STACKSIZE, NULL, TEMP_THREAD_PRIO, NULL) != pdPASS)
        LWIP_ASSERT("stack_init(): read_temp task creation failed.", 0);

    if (xTaskCreate(dhcp_thread, "dhcp", DHCP_THREAD_STACKSIZE, &netif, DHCP_THREAD_PRIO, NULL) != pdPASS)
        LWIP_ASSERT("stack_init(): print_dhcp_state task creation failed.", 0);

    vTaskStartScheduler();

    /* Will not get here unless a task calls vTaskEndScheduler ()*/
    return 0;
}
#endif
