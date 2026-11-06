/* Prueba independiente. Usar como unico main en un proyecto LPC1769
 * enlazado con CMSISv2p00_LPC17xx y los drivers de la catedra.
 * ADC: P0.23 (AD0.0). UART2 TX: P0.10 -> RX del USB-UART de 3.3 V.
 * GND del panel, LPC y adaptador unidos. VCC del adaptador sin conectar.
 * Panel+ -> 10 kohm -> P0.23 -> 4.7 kohm -> GND.
 * Agregar 100 nF de P0.23 a GND. Ver LEEME antes de conectar.
 */
#include "LPC17xx.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_systick.h"
#include <stdio.h>
#include <string.h>

#define ADC_REFERENCE_MV 3300UL
#define R_TOP_OHM 10000UL
#define R_BOTTOM_OHM 4700UL
#define SAMPLE_COUNT 32UL

volatile uint32_t panelMv;
volatile uint32_t adcRaw;

static void wait_ms(uint32_t milliseconds)
{
    SYSTICK_ClearCounterFlag();
    while (milliseconds > 0) {
        if (SYSTICK_HasFired() == SET) {
            milliseconds--;
        }
    }
}

int main(void)
{
    SystemCoreClockUpdate();
    SYSTICK_InternalInit(1);
    SYSTICK_IntCmd(DISABLE);
    SYSTICK_Cmd(ENABLE);

    UART_CFG_T serial = {
        .baudRate = 115200,
        .parity = UART_PARITY_NONE,
        .dataBits = UART_DBITS_8,
        .stopBits = UART_STOPBIT_1
    };
    UART_Init(UART2, &serial);
    UART_PinConfig(UART_TX2_P0_10);
    UART_TxEnable(UART2);

    ADC_Init(10000);
    ADC_PinConfig(ADC_CHANNEL_0);
    ADC_ChannelEnable(ADC_CHANNEL_0);
    ADC_BurstEnable();
    wait_ms(10);

    while (1) {
        uint32_t sum = 0;
        for (uint32_t i = 0; i < SAMPLE_COUNT; i++) {
            while (ADC_ChannelGetStatus(ADC_CHANNEL_0, ADC_DATA_DONE) == RESET) {
            }
            sum += ADC_ChannelGetData(ADC_CHANNEL_0);
        }
        adcRaw = (sum + SAMPLE_COUNT / 2) / SAMPLE_COUNT;
        panelMv = (uint32_t)(((uint64_t)adcRaw * ADC_REFERENCE_MV *
                   (R_TOP_OHM + R_BOTTOM_OHM) +
                   (uint64_t)4095 * R_BOTTOM_OHM / 2) /
                   ((uint64_t)4095 * R_BOTTOM_OHM));

        char message[96];
        snprintf(message, sizeof(message),
                 "{\"panel_mv\":%lu,\"adc\":%lu}\r\n",
                 (unsigned long)panelMv, (unsigned long)adcRaw);
        UART_Send(UART2, (const uint8_t *)message,
                  (uint32_t)strlen(message), BLOCKING);
        wait_ms(250);
    }
}
