/**
 * @file    SolarTracker_TPF_ED3.c
 * @brief   Prueba de LDRs: lee 4 LDR con el ADC en modo burst y enciende el LED
 *          correspondiente a la LDR que recibe mas luz.
 *
 * Conexiones:
 *   LDR0 -> P0.23 (AD0.0)    LED0 -> P0.0
 *   LDR1 -> P0.24 (AD0.1)    LED1 -> P0.1
 *   LDR2 -> P0.25 (AD0.2)    LED2 -> P0.2
 *   LDR3 -> P0.26 (AD0.3)    LED3 -> P0.3
 *
 * Divisor de cada LDR:
 *   3V3 ---[10k]---+--- pin ADC
 *                  |
 *                [LDR]
 *                  |
 *                 GND
 *
 *   Vadc = 3V3 * R_LDR / (R_LDR + 10k)
 *   Mas luz -> menor R_LDR -> menor tension -> MENOR cuenta del ADC.
 *   Por eso la LDR mas iluminada es la que tiene el valor MINIMO.
 */

#include "LPC17xx.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_pinsel.h"

/* ------------------------------ Configuracion ------------------------------ */
#define NUM_LDR       4
#define ADC_RATE_HZ   100000U   /* Frecuencia de conversion del ADC (max 200 kHz) */
#define AVG_SAMPLES   16        /* Muestras promediadas por canal */
#define HYSTERESIS    40        /* Cuentas (~32 mV) para evitar parpadeo entre LEDs */
#define LOOP_DELAY    500000U   /* Retardo aproximado entre lecturas (~20-30 ms) */

#define LED_MASK      ((1U << 0) | (1U << 1) | (1U << 2) | (1U << 3))
#define NO_LDR        0xFF

static const ADC_CHANNEL ldrChannel[NUM_LDR] = {
    ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3
};

static const LPC_PIN ledPin[NUM_LDR] = {
    PIN_0, PIN_1, PIN_2, PIN_3
};

/* Valores promediados; globales para poder verlos en el debugger */
volatile uint16_t ldrValue[NUM_LDR];

/* ------------------------------- Funciones -------------------------------- */
static void delay(volatile uint32_t n) {
    while (n--) {
    }
}

/** Configura P0.0..P0.3 como GPIO de salida, con los LEDs apagados. */
static void LED_Init(void) {
    PINSEL_CFG_T cfg = {
        .port      = PORT_0,
        .pin       = PIN_0,           /* Ignorado por ConfigMultiplePins */
        .func      = PINSEL_FUNC_00,  /* Funcion GPIO */
        .mode      = PINSEL_TRISTATE,
        .openDrain = DISABLE
    };
    PINSEL_ConfigMultiplePins(&cfg, LED_MASK);

    GPIO_SetDir(PORT_0, LED_MASK, GPIO_OUTPUT);
    GPIO_ClearPins(PORT_0, LED_MASK);
}

/** Configura AD0.0..AD0.3 (P0.23..P0.26) y arranca el ADC en modo burst. */
static void ADC_LDR_Init(void) {
    ADC_Init(ADC_RATE_HZ);  /* Deja START = 000, requisito para burst */

    for (uint8_t i = 0; i < NUM_LDR; i++) {
        ADC_PinConfig(ldrChannel[i]);
        ADC_ChannelEnable(ldrChannel[i]);
    }

    ADC_BurstEnable();      /* Convierte los 4 canales en forma continua */
}

/**
 * Lee los 4 canales, promediando AVG_SAMPLES conversiones nuevas por canal.
 * Leer el registro ADDRx limpia su flag DONE, asi que cada espera garantiza
 * una conversion nueva.
 */
static void LDR_Read(volatile uint16_t out[NUM_LDR]) {
    uint32_t acc[NUM_LDR] = {0};

    for (uint8_t s = 0; s < AVG_SAMPLES; s++) {
        for (uint8_t i = 0; i < NUM_LDR; i++) {
            while (ADC_ChannelGetStatus(ldrChannel[i], ADC_DATA_DONE) == RESET) {
            }
            acc[i] += ADC_ChannelGetData(ldrChannel[i]);
        }
    }

    for (uint8_t i = 0; i < NUM_LDR; i++) {
        out[i] = (uint16_t)(acc[i] / AVG_SAMPLES);
    }
}

/**
 * Devuelve el indice de la LDR mas iluminada (valor ADC minimo).
 * Con histeresis: solo cambia si la nueva supera a la actual por HYSTERESIS cuentas.
 */
static uint8_t LDR_GetBrightest(const volatile uint16_t v[NUM_LDR], uint8_t current) {
    uint8_t min = 0;

    for (uint8_t i = 1; i < NUM_LDR; i++) {
        if (v[i] < v[min]) {
            min = i;
        }
    }

    if (current < NUM_LDR && min != current && (v[current] - v[min]) < HYSTERESIS) {
        return current;
    }
    return min;
}

/** Enciende solo el LED indicado. */
static void LED_Show(uint8_t idx) {
    GPIO_ClearPins(PORT_0, LED_MASK);
    GPIO_SetPins(PORT_0, 1U << ledPin[idx]);
}

/* ---------------------------------- Main ---------------------------------- */
int main(void) {
    uint8_t brightest = NO_LDR;

    LED_Init();
    ADC_LDR_Init();

    while (1) {
        LDR_Read(ldrValue);

        uint8_t now = LDR_GetBrightest(ldrValue, brightest);
        if (now != brightest) {
            brightest = now;
            LED_Show(brightest);
        }

        delay(LOOP_DELAY);
    }

    return 0;
}
