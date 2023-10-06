/*******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Empty Application Example
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
********************************************************************************
* Copyright 2021-2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define PIN_ADC (P10_0)
#define ACQUISITION_TIME_NS (1000u)
#define CONVERSION_A_MILI (1000u)
int32_t voltaje;
float32_t tem;

/*******************************************************************************
* Global Variables
*******************************************************************************/
cy_rslt_t result;
cyhal_adc_t adc_obj;
cyhal_adc_channel_t adc_chan_0_obj;

const cyhal_adc_config_t adc_config = {
		.continuous_scanning=false,
		.average_count=1,
		.vref=CYHAL_ADC_REF_VDDA,
		.vneg=CYHAL_ADC_VNEG_VSSA,
		.resolution = 12u,
		.ext_vref = NC,
		.bypass_pin = NC
};

/*******************************************************************************
* Function Prototypes
*******************************************************************************/


/*******************************************************************************
* Function Definitions
*******************************************************************************/

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CPU. It...
*    1.
*    2.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

#if defined (CY_DEVICE_SECURE)
    cyhal_wdt_t wdt_obj;

    /* Clear watchdog timer so that it doesn't trigger a reset */
    result = cyhal_wdt_init(&wdt_obj, cyhal_wdt_get_max_timeout_ms());
    CY_ASSERT(CY_RSLT_SUCCESS == result);
    cyhal_wdt_free(&wdt_obj);
#endif

    /* Initialize the device and board peripherals */

    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /*Inicializacion de UART*/
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
    if (result != CY_RSLT_SUCCESS)
    {
            CY_ASSERT(0);
    }
    /*Inicializacion del ADC*/
    result = cyhal_adc_init(&adc_obj, PIN_ADC, NULL);
    if (result != CY_RSLT_SUCCESS)
     {
          CY_ASSERT(0);
     }
    /* Enable global interrupts */
    __enable_irq();

    /*Configuracion del canal ADC*/
    const cyhal_adc_channel_config_t channel_config = {
    		.enable_averaging = false,
			.min_acquisition_ns = ACQUISITION_TIME_NS,
			.enabled = true
    };
    /*Inicializacion del canal*/
    result = cyhal_adc_channel_init_diff(&adc_chan_0_obj, &adc_obj, PIN_ADC, CYHAL_ADC_VNEG, &channel_config);
    if (result != CY_RSLT_SUCCESS)
    {
                CY_ASSERT(0);
    }
    for (;;)
    {
    	voltaje = cyhal_adc_read_uv(&adc_chan_0_obj)/CONVERSION_A_MILI;
    	tem = (voltaje/10.00);
    	printf("Voltaje: %4ldmV\r\n", (long int)voltaje);
    	printf("Temperatura: %0.1f°C\r\n", (float)tem);
    	cyhal_system_delay_ms(500);
    }
}

/* [] END OF FILE */
