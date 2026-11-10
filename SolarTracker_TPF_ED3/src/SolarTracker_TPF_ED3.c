/**
 * @file    SolarTracker_TPF_ED3.c
 * @brief   Seguidor solar: dos servos (azimut e inclinacion), joystick, LDR, OLED,
 *          medicion de tension del panel y comunicacion bidireccional por UART2
 *          con la pagina HTML.
 *
 *          Dos modos, que se alternan con el pulsador del joystick:
 *          - AUTOMATICO: lee las LDR y muestra cual recibe mas luz (servos quietos).
 *          - MANUAL:     el joystick mueve los dos servos a la vez:
 *                          izquierda/derecha -> servo de AZIMUT (rota todo el panel)
 *                          arriba/abajo      -> servo de INCLINACION del panel
 *
 * Conexiones:
 *   LDR0 -> P0.23 (AD0.0)   LDR1 -> P0.24 (AD0.1)
 *   LDR2 -> P0.25 (AD0.2)   LDR3 -> P0.26 (AD0.3)
 *
 *   Panel (divisor 10k / 4k7) -> P0.3 (AD0.6, J2-22)
 *     Panel+ --[10k]--+--[4k7]-- GND      Vpanel max medible = 3,3 V * 14,7 / 4,7 = 10,3 V
 *                     |
 *                   P0.3  (+ 100 nF a GND recomendado)
 *
 *   UART2 con adaptador USB-UART externo (logica 3,3 V), 115200 8N1:
 *     P0.10 (TXD2, J2-40) -> RX del adaptador
 *     P0.11 (RXD2, J2-41) <- TX del adaptador
 *     GND comun. VCC del adaptador sin conectar.
 *
 *   Joystick VRx -> P1.30 (AD0.4)
 *   Joystick VRy -> P1.31 (AD0.5)
 *   Joystick SW  -> P0.21 (GPIO con interrupcion, pull-up interno, activo en bajo)
 *   Joystick +5V -> 3V3 de la placa (NO a 5V: el ADC admite como maximo 3,3 V)
 *
 *   Servo AZIMUT      senal -> P2.4 (PWM1.5, J2-46)
 *   Servo INCLINACION senal -> P2.5 (PWM1.6, J2-47)
 *   Ambos servos: +5V de la fuente externa, GND de la fuente unido al GND de la placa.
 *
 *   OLED I2C: GND -> GND, VDD -> 3V3, SDA -> P0.27 (SDA0, J2-25), SCK -> P0.28 (SCL0, J2-26)
 *
 * Indicador de modo (LED RGB de la placa, activo en bajo):
 *   Verde (P3.25) = automatico, Azul (P3.26) = manual.
 *
 * Comandos por UART (cada uno termina en '\n'; el '\r' se ignora):
 *   SCREEN,0   Estado general
 *   SCREEN,1   Tension del panel
 *   SCREEN,2   Promedios del panel
 *   SCREEN,3   Lecturas de las LDR
 *   SCREEN,4   Servos y joystick
 *   WINDOW,10 | WINDOW,50 | WINDOW,100   Tamano del promedio movil
 *   RESET      Reinicia las estadisticas
 * Respuestas: {"type":"ack",...} o {"type":"error",...}
 * Telemetria cada 500 ms: {"type":"data",...}
 */

#include <string.h>
#include "LPC17xx.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_clkpwr.h"
#include "lpc17xx_uart.h"
#include "oled.h"

/* ------------------------------ Configuracion ------------------------------ */
/* Canal del panel. P0.3 = ADC_CHANNEL_6. Si la lectura queda clavada cerca de
 * 4095 aun con el panel tapado, el debugger de la placa esta forzando P0.3:
 * en ese caso mover el cable a P0.2 y poner ADC_CHANNEL_7. */
#define PANEL_CHANNEL         ADC_CHANNEL_7

/* LDR: 0 = mas luz da MENOR cuenta (LDR a GND, 10k a 3V3, el montaje actual).
 *      1 = mas luz da MAYOR cuenta (LDR a 3V3, 10k a GND). */
#define LDR_MORE_LIGHT_HIGHER 0

#define NUM_LDR               4
#define ADC_RATE_HZ           100000U   /* Frecuencia de conversion del ADC (max 200 kHz) */
#define AVG_SAMPLES           16U       /* Muestras promediadas para LDR y joystick */
#define PANEL_SAMPLES         32U       /* Muestras promediadas para el panel */
#define ADC_TIMEOUT           100000U   /* Iteraciones maximas esperando una conversion */
#define LDR_HYSTERESIS        40        /* Cuentas para evitar saltos entre LDR */
#define NO_LDR                0xFF

/* Periodos de las tareas del lazo principal */
#define CONTROL_PERIOD_MS     20U       /* LDR, joystick y servos */
#define PANEL_PERIOD_MS       100U      /* Medicion del panel y estadisticas */
#define DISPLAY_PERIOD_MS     250U      /* Redibujo de la OLED */
#define TELEMETRY_PERIOD_MS   500U      /* Envio de datos por UART */

/* Medicion del panel */
#define ADC_REFERENCE_MV      3300UL
#define R_SUPERIOR            10000UL
#define R_INFERIOR            4700UL
#define ADC_SATURATED         4090
#define HISTORY_MAX           100U      /* Ventana maxima del promedio movil */

/* UART2 */
#define UART_BAUDRATE         115200
#define RX_SIZE               128U      /* Buffer circular de recepcion */
#define CMD_MAX               48U       /* Largo maximo de un comando */

/* Pantallas */
#define NUM_SCREENS           5

/* Joystick */
#define JOY_INVERT_X          0         /* Poner en 1 si izquierda/derecha quedan al reves */
#define JOY_INVERT_Y          0         /* Poner en 1 si arriba/abajo quedan al reves */

/* Servos (PWM1, periodo comun de 20 ms) */
#define SERVO_PERIOD_US       20000U    /* 50 Hz */
#define SERVO_CENTER_US       1500U
#define SERVO_MAX_STEP        20        /* us por ciclo de 20 ms con el joystick al tope */
#define SERVO_DEADZONE        200       /* Cuentas alrededor del centro sin movimiento */

/* Limites de cada servo: ajustarlos a lo que permita la mecanica */
#define SERVO_AZ_MIN_US       500U
#define SERVO_AZ_MAX_US       2400U
#define SERVO_INC_MIN_US      500U
#define SERVO_INC_MAX_US      2400U

/* Correspondencia pulso -> grados, solo para mostrar el angulo aproximado */
#define SERVO_US_0DEG         500U
#define SERVO_US_180DEG       2400U

#define SERVO_AZ_INVERT       0
#define SERVO_INC_INVERT      0

/* Pulsador */
#define SW_PORT               PORT_0
#define SW_PIN                PIN_21
#define SW_MASK               (1U << 21)
#define DEBOUNCE_MS           80

/* LED RGB de la placa (activo en bajo) */
#define RGB_RED_PORT          PORT_0
#define RGB_RED_MASK          (1U << 22)
#define RGB_GB_PORT           PORT_3
#define RGB_GREEN_MASK        (1U << 25)
#define RGB_BLUE_MASK         (1U << 26)

/* ------------------------------- Tipos ------------------------------------- */
typedef enum {
    SERVO_AZ = 0,   /* Azimut:      P2.4 / PWM1.5 */
    SERVO_INC,      /* Inclinacion: P2.5 / PWM1.6 */
    NUM_SERVOS
} SERVO_ID;

/* ------------------------------- Variables -------------------------------- */
static const ADC_CHANNEL ldrChannel[NUM_LDR] = {
    ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3
};
static const ADC_CHANNEL joyChannel[2]   = { ADC_CHANNEL_4, ADC_CHANNEL_5 };
static const ADC_CHANNEL panelChannel[1] = { PANEL_CHANNEL };

static const uint8_t  servoPwmCh[NUM_SERVOS] = { 5, 6 };
static const uint32_t servoMin[NUM_SERVOS]   = { SERVO_AZ_MIN_US, SERVO_INC_MIN_US };
static const uint32_t servoMax[NUM_SERVOS]   = { SERVO_AZ_MAX_US, SERVO_INC_MAX_US };

/* Globales y volatile para poder verlas en el debugger */
volatile uint16_t ldrValue[NUM_LDR];
volatile uint8_t  brightestLdr = NO_LDR;
volatile uint16_t joyValue[2];       /* [0] = X, [1] = Y */
volatile uint16_t joyCenter[2];
volatile uint8_t  manualMode = 0;    /* 0 = automatico, 1 = manual */
volatile uint32_t servoPulseUs[NUM_SERVOS] = { SERVO_CENTER_US, SERVO_CENTER_US };
volatile uint8_t  oledOk = 0;
volatile uint8_t  screen = 0;

/* Panel */
volatile uint8_t  panelValid = 0;
volatile uint16_t panelAdc   = 0;    /* Promedio de 32 muestras (0..4095) */
volatile uint32_t panelMv    = 0;    /* Tension promediada */
volatile uint32_t rawMv      = 0;    /* Tension de la ultima muestra */
volatile uint32_t averageMv  = 0;    /* Promedio movil */
volatile uint32_t minimumMv  = 0;
volatile uint32_t maximumMv  = 0;

static uint32_t history[HISTORY_MAX];
static uint32_t historyIndex = 0;
static uint32_t historyCount = 0;
static uint32_t historySum   = 0;
static uint32_t windowSize   = 50;
static uint64_t sessionSum   = 0;
static uint32_t sampleCount  = 0;

/* Recepcion UART por interrupcion (buffer circular) */
static volatile uint8_t  rxBuffer[RX_SIZE];
static volatile uint32_t rxHead  = 0;
static volatile uint32_t rxTail  = 0;
static volatile uint8_t  rxError = 0;

static uint8_t displayDirty = 1;     /* Fuerza un redibujo inmediato */

static volatile uint32_t msTicks    = 0;
static volatile uint32_t lastEdgeMs = 0;

/* ------------------------------ Interrupciones ----------------------------- */
void SysTick_Handler(void) {
    msTicks++;
}

/**
 * Pulsador del joystick. Las interrupciones GPIO de los puertos 0 y 2 comparten
 * el vector de EINT3. Solo cambia de modo en un flanco de bajada si no hubo
 * ningun flanco en los ultimos DEBOUNCE_MS (ignora rebotes al apretar y al soltar).
 */
void EINT3_IRQHandler(void) {
    const uint8_t fell = GPIO_GetPinIntStatus(SW_PORT, SW_PIN, GPIO_INT_FALLING) == SET;
    const uint8_t rose = GPIO_GetPinIntStatus(SW_PORT, SW_PIN, GPIO_INT_RISING) == SET;

    if (fell || rose) {
        const uint32_t now = msTicks;

        GPIO_ClearInt(SW_PORT, SW_MASK);

        if (fell && !rose && (now - lastEdgeMs) > DEBOUNCE_MS) {
            manualMode = !manualMode;
        }
        lastEdgeMs = now;
    }
}

/** Recepcion UART2: guarda cada byte en el buffer circular. */
void UART2_IRQHandler(void) {
    (void)UART_GetIntId(UART2);

    uint8_t status = UART_GetLineStatus(UART2);

    while (status & UART_LSR_RDR) {
        const uint8_t value = UART_ReceiveByte(UART2);
        const uint32_t next = (rxHead + 1U) % RX_SIZE;

        if (next != rxTail) {
            rxBuffer[rxHead] = value;
            rxHead = next;
        } else {
            rxError = 1;   /* Buffer lleno: se descarta el comando en curso */
        }

        status = UART_GetLineStatus(UART2);
    }
}

/* -------------------------------- Utilidades ------------------------------- */
static void delay_ms(uint32_t ms) {
    const uint32_t start = msTicks;
    while ((msTicks - start) < ms) {
    }
}

/* Construccion de textos sin printf. Cada funcion escribe en p, deja el '\0'
 * al final y devuelve el puntero a ese '\0' para poder seguir concatenando. */
static char* str_cat(char* p, const char* s) {
    while (*s) {
        *p++ = *s++;
    }
    *p = '\0';
    return p;
}

static char* str_uint(char* p, uint32_t v) {
    char tmp[10];
    uint8_t n = 0;
    do {
        tmp[n++] = (char)('0' + v % 10);
        v /= 10;
    } while (v);
    while (n) {
        *p++ = tmp[--n];
    }
    *p = '\0';
    return p;
}

/** mV -> "X.XX" (voltios con dos decimales, redondeado). */
static char* str_volts(char* p, uint32_t mv) {
    const uint32_t cv = (mv + 5) / 10;  /* Centesimas de voltio */
    p = str_uint(p, cv / 100);
    *p++ = '.';
    *p++ = (char)('0' + (cv % 100) / 10);
    *p++ = (char)('0' + cv % 10);
    *p = '\0';
    return p;
}

/** Agrega ,"nombre":valor a un JSON en construccion. */
static char* json_field(char* p, const char* name, uint32_t value) {
    p = str_cat(p, ",\"");
    p = str_cat(p, name);
    p = str_cat(p, "\":");
    return str_uint(p, value);
}

/* ---------------------------------- GPIO ----------------------------------- */
/** LED RGB de la placa como salida, todo apagado (activo en bajo -> nivel alto). */
static void RGB_Init(void) {
    PINSEL_CFG_T cfg = {
        .port      = RGB_RED_PORT,
        .pin       = PIN_22,
        .func      = PINSEL_FUNC_00,
        .mode      = PINSEL_TRISTATE,
        .openDrain = DISABLE
    };
    PINSEL_ConfigPin(&cfg);

    cfg.port = RGB_GB_PORT;
    PINSEL_ConfigMultiplePins(&cfg, RGB_GREEN_MASK | RGB_BLUE_MASK);

    GPIO_SetDir(RGB_RED_PORT, RGB_RED_MASK, GPIO_OUTPUT);
    GPIO_SetDir(RGB_GB_PORT, RGB_GREEN_MASK | RGB_BLUE_MASK, GPIO_OUTPUT);

    GPIO_SetPins(RGB_RED_PORT, RGB_RED_MASK);
    GPIO_SetPins(RGB_GB_PORT, RGB_GREEN_MASK | RGB_BLUE_MASK);
}

/** Verde = automatico, azul = manual. */
static void RGB_ShowMode(uint8_t manual) {
    GPIO_SetPins(RGB_GB_PORT, RGB_GREEN_MASK | RGB_BLUE_MASK);
    GPIO_ClearPins(RGB_GB_PORT, manual ? RGB_BLUE_MASK : RGB_GREEN_MASK);
}

/** P0.21 como entrada con pull-up e interrupcion por ambos flancos. */
static void SW_Init(void) {
    PINSEL_CFG_T cfg = {
        .port      = SW_PORT,
        .pin       = SW_PIN,
        .func      = PINSEL_FUNC_00,
        .mode      = PINSEL_PULLUP,
        .openDrain = DISABLE
    };
    PINSEL_ConfigPin(&cfg);

    GPIO_SetDir(SW_PORT, SW_MASK, GPIO_INPUT);
    GPIO_IntConfigPin(SW_PORT, SW_PIN, GPIO_INT_FALLING, ENABLE);
    GPIO_IntConfigPin(SW_PORT, SW_PIN, GPIO_INT_RISING, ENABLE);
    GPIO_ClearInt(SW_PORT, SW_MASK);

    NVIC_EnableIRQ(EINT3_IRQn);
}

/* ---------------------------------- UART2 ---------------------------------- */
/** UART2 a 115200 8N1 en P0.10/P0.11, con interrupcion de recepcion. */
static void UART_Setup(void) {
    UART_CFG_T cfg = {
        .baudRate = UART_BAUDRATE,
        .parity   = UART_PARITY_NONE,
        .dataBits = UART_DBITS_8,
        .stopBits = UART_STOPBIT_1
    };

    UART_Init(UART2, &cfg);
    UART_PinConfig(UART_TX2_P0_10);
    UART_PinConfig(UART_RX2_P0_11);

    UART_FIFO_CFG_T fifo = {
        .resetRxBuf = ENABLE,
        .resetTxBuf = ENABLE,
        .dmaMode    = DISABLE,
        .level      = UART_FIFO_TRGLEV0
    };
    UART_FIFOConfig(UART2, &fifo);
    UART_TxEnable(UART2);

    UART_IntConfig(UART2, UART_INT_RBR, ENABLE);
    NVIC_ClearPendingIRQ(UART2_IRQn);
    NVIC_EnableIRQ(UART2_IRQn);
}

/** Envia desde 'start' hasta 'end' (sin incluir). */
static void UART_SendRange(const char* start, const char* end) {
    UART_Send(UART2, (const uint8_t*)start, (uint32_t)(end - start), BLOCKING);
}

/* --------------------------------- Servos ---------------------------------- */
/** Fija el ancho de pulso de un servo; se aplica al comienzo del proximo periodo. */
static void SERVO_SetPulse(SERVO_ID s, uint32_t us) {
    if (us < servoMin[s]) {
        us = servoMin[s];
    } else if (us > servoMax[s]) {
        us = servoMax[s];
    }
    servoPulseUs[s] = us;

    if (s == SERVO_AZ) {
        LPC_PWM1->MR5 = us;
    } else {
        LPC_PWM1->MR6 = us;
    }
    /* |= para no pisar un latch pendiente del otro servo en el mismo periodo */
    LPC_PWM1->LER |= (1U << servoPwmCh[s]);
}

/**
 * PWM1 con el contador a 1 MHz (1 tick = 1 us):
 * MR0 = periodo de 20 ms (comun a los dos servos),
 * MR5 = pulso del azimut (P2.4), MR6 = pulso de la inclinacion (P2.5).
 */
static void SERVO_Init(void) {
    PINSEL_CFG_T cfg = {
        .port      = PORT_2,
        .pin       = PIN_4,
        .func      = PINSEL_FUNC_01,  /* PWM1.5 */
        .mode      = PINSEL_TRISTATE,
        .openDrain = DISABLE
    };
    PINSEL_ConfigPin(&cfg);
    cfg.pin = PIN_5;                  /* PWM1.6 */
    PINSEL_ConfigPin(&cfg);

    CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCPWM1, ENABLE);
    const uint32_t pclk = CLKPWR_GetPCLK(CLKPWR_PCLKSEL_PWM1);

    LPC_PWM1->TCR = (1U << 1);                    /* Reset del contador */
    LPC_PWM1->PR  = pclk / 1000000U - 1U;         /* Tick de 1 us */
    LPC_PWM1->MR0 = SERVO_PERIOD_US;
    LPC_PWM1->MR5 = SERVO_CENTER_US;
    LPC_PWM1->MR6 = SERVO_CENTER_US;
    LPC_PWM1->MCR = (1U << 1);                    /* Reinicia el contador en MR0 */
    LPC_PWM1->PCR = (1U << 13) | (1U << 14);      /* Salidas PWM1.5 y PWM1.6 (simple flanco) */
    LPC_PWM1->LER = (1U << 0) | (1U << 5) | (1U << 6);
    LPC_PWM1->TCR = (1U << 0) | (1U << 3);        /* Contador + modo PWM habilitados */

    servoPulseUs[SERVO_AZ]  = SERVO_CENTER_US;
    servoPulseUs[SERVO_INC] = SERVO_CENTER_US;
}

/**
 * Control por velocidad de un servo: cuanto mayor la desviacion 'd' del
 * joystick, mas rapido se mueve. d > 0 aumenta el pulso, d < 0 lo disminuye.
 * Dentro de la zona muerta no se mueve, asi que al soltar queda en su lugar.
 */
static void SERVO_Move(SERVO_ID s, int32_t d) {
    const int32_t mag = d < 0 ? -d : d;
    if (mag <= SERVO_DEADZONE) {
        return;
    }

    int32_t step = (mag - SERVO_DEADZONE) * SERVO_MAX_STEP / (2048 - SERVO_DEADZONE);
    if (step < 1) {
        step = 1;
    }

    int32_t pulse = (int32_t)servoPulseUs[s] + (d > 0 ? step : -step);
    if (pulse < 0) {
        pulse = 0;
    }
    SERVO_SetPulse(s, (uint32_t)pulse);
}

/** Angulo aproximado (0..180) a partir del ancho de pulso, solo para mostrar. */
static uint32_t SERVO_GetDegrees(SERVO_ID s) {
    const uint32_t us = servoPulseUs[s];
    if (us <= SERVO_US_0DEG) {
        return 0;
    }
    if (us >= SERVO_US_180DEG) {
        return 180;
    }
    const uint32_t span = SERVO_US_180DEG - SERVO_US_0DEG;
    return ((us - SERVO_US_0DEG) * 180U + span / 2) / span;
}

/* ----------------------------------- ADC ----------------------------------- */
/** LDR (AD0.0-3), joystick (AD0.4-5) y panel (PANEL_CHANNEL) en modo burst. */
static void ADC_Setup(void) {
    ADC_Init(ADC_RATE_HZ);  /* Deja START = 000, requisito para burst */

    for (uint8_t i = 0; i < NUM_LDR; i++) {
        ADC_PinConfig(ldrChannel[i]);
        ADC_ChannelEnable(ldrChannel[i]);
    }
    for (uint8_t i = 0; i < 2; i++) {
        ADC_PinConfig(joyChannel[i]);
        ADC_ChannelEnable(joyChannel[i]);
    }
    ADC_PinConfig(panelChannel[0]);
    ADC_ChannelEnable(panelChannel[0]);

    ADC_BurstEnable();
}

/**
 * Promedia 'samples' conversiones nuevas de cada canal de la lista (con redondeo).
 * Leer ADDRx limpia su flag DONE, asi que cada espera garantiza una conversion nueva.
 * Si se pasa 'lastFirst', guarda ahi la ultima muestra del primer canal.
 * Devuelve 0 si alguna conversion no llego a tiempo (y no modifica 'out').
 */
static uint8_t ADC_ReadAverage(const ADC_CHANNEL ch[], uint8_t n, uint32_t samples,
                               volatile uint16_t out[], uint16_t* lastFirst) {
    uint32_t acc[8] = {0};

    for (uint32_t s = 0; s < samples; s++) {
        for (uint8_t i = 0; i < n; i++) {
            uint32_t t = ADC_TIMEOUT;
            while (ADC_ChannelGetStatus(ch[i], ADC_DATA_DONE) == RESET) {
                if (--t == 0) {
                    return 0;
                }
            }
            const uint16_t value = ADC_ChannelGetData(ch[i]);
            acc[i] += value;
            if (i == 0 && lastFirst) {
                *lastFirst = value;
            }
        }
    }

    for (uint8_t i = 0; i < n; i++) {
        out[i] = (uint16_t)((acc[i] + samples / 2) / samples);
    }
    return 1;
}

/* ---------------------------------- Panel ---------------------------------- */
static uint32_t PANEL_AdcToMv(uint32_t adc) {
    const uint64_t num = (uint64_t)adc * ADC_REFERENCE_MV * (R_SUPERIOR + R_INFERIOR);
    const uint64_t den = (uint64_t)4095 * R_INFERIOR;
    return (uint32_t)((num + den / 2) / den);
}

static void PANEL_ResetHistory(void) {
    historyIndex = 0;
    historyCount = 0;
    historySum   = 0;
    averageMv    = 0;
}

static void PANEL_ResetStatistics(void) {
    PANEL_ResetHistory();
    sessionSum  = 0;
    sampleCount = 0;
    minimumMv   = 0;
    maximumMv   = 0;
}

/** Mide el panel y actualiza promedio movil, minimo, maximo y promedio de sesion. */
static void PANEL_Measure(void) {
    volatile uint16_t avg;
    uint16_t last = 0;

    panelValid = ADC_ReadAverage(panelChannel, 1, PANEL_SAMPLES, &avg, &last);
    if (!panelValid) {
        return;
    }

    panelAdc = avg;
    panelMv  = PANEL_AdcToMv(avg);
    rawMv    = PANEL_AdcToMv(last);

    /* Promedio movil sobre las ultimas 'windowSize' mediciones */
    if (historyCount == windowSize) {
        historySum -= history[historyIndex];
    } else {
        historyCount++;
    }
    history[historyIndex] = panelMv;
    historySum += panelMv;
    historyIndex = (historyIndex + 1U) % windowSize;
    averageMv = historySum / historyCount;

    if (sampleCount == 0 || panelMv < minimumMv) {
        minimumMv = panelMv;
    }
    if (sampleCount == 0 || panelMv > maximumMv) {
        maximumMv = panelMv;
    }

    sessionSum += panelMv;
    sampleCount++;
}

static uint32_t PANEL_SessionAverage(void) {
    return sampleCount ? (uint32_t)(sessionSum / sampleCount) : 0;
}

/* ---------------------------------- LDR ------------------------------------ */
/** 1 si la lectura 'a' indica mas luz que 'b' segun el montaje. */
static uint8_t LDR_IsBrighter(uint16_t a, uint16_t b) {
#if LDR_MORE_LIGHT_HIGHER
    return a > b;
#else
    return a < b;
#endif
}

/**
 * Lee las 4 LDR y devuelve la mas iluminada.
 * Con histeresis: solo cambia si la nueva supera a la actual por LDR_HYSTERESIS cuentas.
 */
static uint8_t LDR_Update(uint8_t current) {
    ADC_ReadAverage(ldrChannel, NUM_LDR, AVG_SAMPLES, ldrValue, 0);

    uint8_t best = 0;
    for (uint8_t i = 1; i < NUM_LDR; i++) {
        if (LDR_IsBrighter(ldrValue[i], ldrValue[best])) {
            best = i;
        }
    }

    if (current < NUM_LDR && best != current) {
        const int32_t diff = (int32_t)ldrValue[current] - (int32_t)ldrValue[best];
        if ((diff < 0 ? -diff : diff) < LDR_HYSTERESIS) {
            return current;
        }
    }
    return best;
}

/* -------------------------------- Joystick --------------------------------- */
/** Toma la posicion de reposo del joystick como centro. No tocarlo al arrancar. */
static void JOY_Calibrate(void) {
    ADC_ReadAverage(joyChannel, 2, AVG_SAMPLES, joyCenter, 0);
}

static void JOY_Read(void) {
    ADC_ReadAverage(joyChannel, 2, AVG_SAMPLES, joyValue, 0);
}

/**
 * Mueve los dos servos a la vez segun el joystick:
 *   derecha -> azimut aumenta el pulso, izquierda -> lo disminuye
 *   arriba  -> inclinacion aumenta el pulso, abajo -> lo disminuye
 */
static void SERVOS_UpdateFromJoystick(void) {
    int32_t dx = (int32_t)joyValue[0] - (int32_t)joyCenter[0];  /* > 0: derecha */
    int32_t dy = (int32_t)joyValue[1] - (int32_t)joyCenter[1];  /* > 0: abajo   */
#if JOY_INVERT_X
    dx = -dx;
#endif
#if JOY_INVERT_Y
    dy = -dy;
#endif

    int32_t az  = dx;
    int32_t inc = -dy;
#if SERVO_AZ_INVERT
    az = -az;
#endif
#if SERVO_INC_INVERT
    inc = -inc;
#endif

    SERVO_Move(SERVO_AZ, az);
    SERVO_Move(SERVO_INC, inc);
}

/* ------------------------------ Comunicacion ------------------------------- */
/** Respuesta a un comando: {"type":"ack|error","command":"...","screen":n,"window":n} */
static void COMM_Reply(const char* type, const char* command) {
    char buf[128];
    char* p = buf;

    p = str_cat(p, "{\"type\":\"");
    p = str_cat(p, type);
    p = str_cat(p, "\",\"command\":\"");
    p = str_cat(p, command);
    p = str_cat(p, "\"");
    p = json_field(p, "screen", screen);
    p = json_field(p, "window", windowSize);
    p = str_cat(p, "}\n");

    UART_SendRange(buf, p);
}

static void COMM_Execute(const char* cmd) {
    if (strlen(cmd) == 8 && strncmp(cmd, "SCREEN,", 7) == 0 &&
        cmd[7] >= '0' && cmd[7] < (char)('0' + NUM_SCREENS)) {
        screen       = (uint8_t)(cmd[7] - '0');
        displayDirty = 1;
        COMM_Reply("ack", "SCREEN");

    } else if (strcmp(cmd, "RESET") == 0) {
        PANEL_ResetStatistics();
        displayDirty = 1;
        COMM_Reply("ack", "RESET");

    } else if (strcmp(cmd, "WINDOW,10") == 0) {
        windowSize = 10;
        PANEL_ResetHistory();
        displayDirty = 1;
        COMM_Reply("ack", "WINDOW");

    } else if (strcmp(cmd, "WINDOW,50") == 0) {
        windowSize = 50;
        PANEL_ResetHistory();
        displayDirty = 1;
        COMM_Reply("ack", "WINDOW");

    } else if (strcmp(cmd, "WINDOW,100") == 0) {
        windowSize = 100;
        PANEL_ResetHistory();
        displayDirty = 1;
        COMM_Reply("ack", "WINDOW");

    } else {
        COMM_Reply("error", "INVALID");
    }
}

/**
 * Arma comandos con los bytes recibidos. Un comando termina en '\n'.
 * Si llega algo invalido o demasiado largo, se descarta hasta el proximo '\n'.
 */
static void COMM_Process(void) {
    static char     cmd[CMD_MAX];
    static uint32_t len     = 0;
    static uint8_t  discard = 0;

    NVIC_DisableIRQ(UART2_IRQn);
    if (rxError) {
        rxTail  = rxHead;
        rxError = 0;
        len     = 0;
        discard = 1;
    }
    NVIC_EnableIRQ(UART2_IRQn);

    for (uint32_t count = 0; count < RX_SIZE && rxTail != rxHead; count++) {
        const uint8_t c = rxBuffer[rxTail];
        rxTail = (rxTail + 1U) % RX_SIZE;

        if (c == '\n') {
            if (discard) {
                COMM_Reply("error", "FRAME");
            } else {
                cmd[len] = '\0';
                COMM_Execute(cmd);
            }
            len     = 0;
            discard = 0;
        } else if (c != '\r' && !discard) {
            if (c < 32 || c > 126 || len >= CMD_MAX - 1U) {
                discard = 1;
            } else {
                cmd[len++] = (char)c;
            }
        }
    }
}

/** Telemetria en una linea JSON (mismo formato que usa la pagina). */
static void COMM_SendTelemetry(void) {
    char buf[384];
    char* p = buf;

    p = str_cat(p, "{\"type\":\"data\"");
    p = json_field(p, "valid", panelValid);
    p = json_field(p, "raw_mv", rawMv);
    p = json_field(p, "panel_mv", panelMv);
    p = json_field(p, "avg_mv", averageMv);
    p = json_field(p, "session_mv", PANEL_SessionAverage());
    p = json_field(p, "min_mv", minimumMv);
    p = json_field(p, "max_mv", maximumMv);
    p = json_field(p, "adc", panelAdc);
    p = json_field(p, "samples", sampleCount);
    p = json_field(p, "window", windowSize);
    p = json_field(p, "screen", screen);
    p = json_field(p, "manual", manualMode);
    p = json_field(p, "oled", oledOk);
    p = json_field(p, "tx_dropped", 0);

    p = str_cat(p, ",\"ldr\":[");
    for (uint8_t i = 0; i < NUM_LDR; i++) {
        if (i) {
            p = str_cat(p, ",");
        }
        p = str_uint(p, ldrValue[i]);
    }
    p = str_cat(p, "],\"joy\":[");
    p = str_uint(p, joyValue[0]);
    p = str_cat(p, ",");
    p = str_uint(p, joyValue[1]);
    p = str_cat(p, "],\"servo\":[");
    p = str_uint(p, servoPulseUs[SERVO_AZ]);
    p = str_cat(p, ",");
    p = str_uint(p, servoPulseUs[SERVO_INC]);
    p = str_cat(p, "]}\n");

    UART_SendRange(buf, p);
}

/* --------------------------------- Pantalla -------------------------------- */
static void DrawCentered(int y, const char* str, uint8_t scale) {
    OLED_DrawString((OLED_WIDTH - OLED_StringWidth(str, scale)) / 2, y, str, scale);
}

/** Linea "etiqueta + numero + unidad" alineada a la izquierda. */
static void DrawValue(int y, const char* label, uint32_t value, const char* unit) {
    char buf[32];
    char* p = str_cat(buf, label);
    p = str_uint(p, value);
    str_cat(p, unit);
    OLED_DrawString(0, y, buf, 1);
}

/** Linea "etiqueta + X.XX V". */
static void DrawVolts(int y, const char* label, uint32_t mv) {
    char buf[32];
    char* p = str_cat(buf, label);
    p = str_volts(p, mv);
    str_cat(p, " V");
    OLED_DrawString(0, y, buf, 1);
}

/** Pantalla 0: modo, tension en grande, ADC y servos / LDR. */
static void Screen_Status(void) {
    char buf[24];
    char* p;

    DrawCentered(0, manualMode ? "MODO MANUAL" : "MODO AUTOMATICO", 1);

    if (!panelValid) {
        DrawCentered(20, "ERROR ADC", 2);
    } else {
        p = str_volts(buf, panelMv);
        str_cat(p, " V");
        DrawCentered(12, buf, 3);

        if (panelAdc >= ADC_SATURATED) {
            DrawCentered(40, "ADC SATURADO", 1);
        } else {
            p = str_cat(buf, "ADC ");
            p = str_uint(p, panelAdc);
            str_cat(p, " / 4095");
            DrawCentered(40, buf, 1);
        }
    }

    if (manualMode) {
        p = str_cat(buf, "AZ ");
        p = str_uint(p, SERVO_GetDegrees(SERVO_AZ));
        p = str_cat(p, "   INC ");
        str_uint(p, SERVO_GetDegrees(SERVO_INC));
    } else {
        p = str_cat(buf, "MAS LUZ: LDR ");
        if (brightestLdr < NUM_LDR) {
            str_uint(p, brightestLdr);
        } else {
            str_cat(p, "-");
        }
    }
    DrawCentered(54, buf, 1);
}

static void Screen_Panel(void) {
    OLED_DrawString(0, 0, "TENSION DEL PANEL", 1);
    if (!panelValid) {
        OLED_DrawString(0, 25, "ERROR ADC", 1);
        return;
    }
    DrawVolts(17, "Actual ", rawMv);
    DrawVolts(31, "Prom32 ", panelMv);
    DrawValue(45, "ADC ", panelAdc, " / 4095");
}

static void Screen_Averages(void) {
    OLED_DrawString(0, 0, "PROMEDIOS DEL PANEL", 1);
    DrawVolts(12, "Movil  ", averageMv);
    DrawVolts(23, "Sesion ", PANEL_SessionAverage());
    DrawVolts(34, "Min    ", minimumMv);
    DrawVolts(45, "Max    ", maximumMv);
    DrawValue(56, "Ventana ", windowSize, " muestras");
}

static void Screen_Ldr(void) {
    OLED_DrawString(0, 0, "LECTURAS LDR", 1);
    DrawValue(12, "LDR0 ", ldrValue[0], "");
    DrawValue(22, "LDR1 ", ldrValue[1], "");
    DrawValue(32, "LDR2 ", ldrValue[2], "");
    DrawValue(42, "LDR3 ", ldrValue[3], "");
    if (brightestLdr < NUM_LDR) {
        DrawValue(54, "Mas luz: LDR", brightestLdr, "");
    }
}

static void Screen_Servos(void) {
    OLED_DrawString(0, 0, "SERVOS Y JOYSTICK", 1);
    DrawValue(14, "Azimut  ", SERVO_GetDegrees(SERVO_AZ), " grados");
    DrawValue(26, "Inclina ", SERVO_GetDegrees(SERVO_INC), " grados");
    DrawValue(38, "Joy X ", joyValue[0], "");
    DrawValue(50, "Joy Y ", joyValue[1], "");
}

static void Display_Draw(void) {
    if (!oledOk) {
        return;
    }

    OLED_Clear();
    switch (screen) {
        case 1:  Screen_Panel();    break;
        case 2:  Screen_Averages(); break;
        case 3:  Screen_Ldr();      break;
        case 4:  Screen_Servos();   break;
        default: Screen_Status();   break;
    }
    oledOk = (uint8_t)OLED_Update();
}

/* ---------------------------------- Main ---------------------------------- */
int main(void) {
    uint8_t lastMode = 0xFF;

    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000);  /* Tick de 1 ms */

    RGB_Init();
    ADC_Setup();
    UART_Setup();
    delay_ms(20);
    JOY_Calibrate();
    SERVO_Init();
    SW_Init();

    delay_ms(100);                 /* La OLED necesita un momento tras encenderse */
    oledOk = (uint8_t)OLED_Init();

    /* Cada tarea corre con su propio periodo, sin bloquear el lazo */
    uint32_t lastControl   = msTicks;
    uint32_t lastPanel     = msTicks - PANEL_PERIOD_MS;
    uint32_t lastDisplay   = msTicks - DISPLAY_PERIOD_MS;
    uint32_t lastTelemetry = msTicks - TELEMETRY_PERIOD_MS;

    while (1) {
        COMM_Process();

        const uint32_t now  = msTicks;
        const uint8_t  mode = manualMode;

        if (mode != lastMode) {
            lastMode = mode;
            RGB_ShowMode(mode);
            displayDirty = 1;
        }

        if ((now - lastControl) >= CONTROL_PERIOD_MS) {
            lastControl  = now;
            brightestLdr = LDR_Update(brightestLdr);
            JOY_Read();
            if (mode) {
                SERVOS_UpdateFromJoystick();
            }
        }

        if ((now - lastPanel) >= PANEL_PERIOD_MS) {
            lastPanel = now;
            PANEL_Measure();
        }

        if (displayDirty || (now - lastDisplay) >= DISPLAY_PERIOD_MS) {
            lastDisplay  = now;
            displayDirty = 0;
            Display_Draw();
        }

        if ((now - lastTelemetry) >= TELEMETRY_PERIOD_MS) {
            lastTelemetry = now;
            COMM_SendTelemetry();
        }
    }

    return 0;
}
