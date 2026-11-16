/**
 * SolarTracker - Trabajo Practico Final de Electronica Digital 3
 * Alumnos: Ignacio Giraudo y Enzo Ferrando. Profesor: Marcos Blasco.
 * Plataforma: LPCXpresso LPC1769, MCUXpresso y drivers de la catedra.
 *

/* ================= 1. INCLUDES Y CONFIGURACION ================= */
#include <string.h>
#include "LPC17xx.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_clkpwr.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_gpdma.h"
#include "oled.h"

/* ------------------------------ Configuracion ------------------------------ */
/* Canal del panel: P0.2 = AD0.7 */
#define PANEL_CHANNEL         ADC_CHANNEL_7

/* LDR: 0 = mas luz da MENOR cuenta (LDR a GND, 10k a 3V3).
 *      1 = mas luz da MAYOR cuenta (LDR a 3V3, 10k a GND). */
#define LDR_MORE_LIGHT_HIGHER 1

#define NUM_LDR               3
#define ADC_RATE_HZ           100000U   /* Frecuencia de conversion del ADC (max 200 kHz) */

/* ADC por DMA: un canal por bloque, sin rafagas sobre ADDR0..ADDR7.
 * 65 conversiones = 1 descartada + 64 utiles; se requieren al menos 32 validas. */
#define ADC_DMA_SAMPLES       64U
#define ADC_DMA_BUF_LEN       (ADC_DMA_SAMPLES + 1U)
#define ADC_DMA_MIN_SAMPLES   32U
#define ADC_DMA_CH            GPDMA_CH_0
#define ADC_DMA_REGS          LPC_GPDMACH0
#define ADC_DMA_TIMEOUT_MS    100U
#define ADC_DMA_STALE_MS      750U
#define ADC_DMA_START_MS      1U

/* DAC por DMA */
#define DAC_DMA_CH            GPDMA_CH_1
#define DAC_DMA_REGS          LPC_GPDMACH1
#define DAC_UPDATE_HZ         1000U     /* Frecuencia de las peticiones DMA del DAC */

/* Carga del panel */
#define R_LOAD_OHM            10000UL

/* Los buffers DMA se mantienen en SRAM AHB, como en el codigo recibido.
 * Direcciones fijas del banco 1 de la SRAM AHB (0x20080000, 16 KB), que el programa
 * no usa para nada mas. Asi no depende de como el linker ubique las secciones. */
#define AHB_RAM_BASE          0x20080000UL
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
#define NUM_SCREENS           6

/* Joystick */
#define JOY_INVERT_X          0         /* Poner en 1 si izquierda/derecha quedan al reves */
#define JOY_INVERT_Y          1         /* Invertido segun prueba del joystick */

/* Servos (PWM1, periodo comun de 20 ms) */
#define SERVO_PERIOD_US       20000U    /* 50 Hz */
#define SERVO_CENTER_US       1500U
#define SERVO_MAX_STEP        20        /* us por ciclo de 20 ms con el joystick al tope */
#define SERVO_DEADZONE        200       /* Cuentas alrededor del centro sin movimiento */

/* Seguimiento automatico con las LDR.
 * Errores normalizados en milesimos (0..1000) para que no dependan del nivel de luz. */
/* Zona muerta con histeresis: empieza a corregir si el error supera START y
 * sigue corrigiendo hasta que baja de STOP. Evita el arranca-frena en el borde
 * y deja el panel mejor apuntado que una zona muerta unica. */
#define TRACK_DEADBAND_START  100       /* 10 % -> empieza a moverse */
#define TRACK_DEADBAND_STOP   40        /*  4 % -> se detiene */
#define TRACK_GAIN_DIV        4         /* Paso en us = (error - STOP) / GAIN_DIV */
#define TRACK_MIN_STEP        3         /* us: menos que esto el SG90 no lo mueve (tironea) */
#define TRACK_MAX_STEP        15        /* us por actualizacion de 20 ms (~750 us/s) */
#define TRACK_PERIOD_MS       20U       /* Igual al periodo del PWM: un paso por pulso */
#define TRACK_FILTER_DIV      4         /* Filtro del error: cada lectura aporta 1/4 */
#define TRACK_MIN_LIGHT       300       /* Brillo total minimo para seguir (de noche no se mueve) */
#define TRACK_SEQUENTIAL      0         /* 0 = corrige ambos ejes a la vez (recomendado)
                                           1 = primero inclinacion, despues giro */
#define TRACK_AZ_INVERT       0         /* Giro automatico invertido segun prueba mecanica */
#define TRACK_INC_INVERT      1         /* Poner en 1 si en automatico la inclinacion se aleja */
/* Compensacion en milesimos: con luz uniforme, observar Err H/V en SCREEN,4.
 * Si existe un desbalance fijo, copiar el error observado al offset del eje. */
#define TRACK_H_OFFSET        0
#define TRACK_V_OFFSET        0

/*
 * Limites de cada servo: ajustarlos a lo que permita la mecanica.
 * Como encontrarlos: modo manual + pantalla 4 (SCREEN,4). Llevar el servo
 * despacio hacia un extremo hasta que zumbe, volver hasta que se calle y
 * anotar los us que muestra la pantalla.
 */
#define SERVO_AZ_MIN_US       500U
#define SERVO_AZ_MAX_US       2400U
#define SERVO_INC_MIN_US      606U  /* Servo 10 deg -> panel -30 deg */
#define SERVO_INC_MAX_US      1872U /* Servo 130 deg -> panel +90 deg */

/* Correspondencia pulso -> grados, solo para mostrar el angulo aproximado */
#define SERVO_US_0DEG         500U
#define SERVO_US_180DEG       2400U
#define SERVO_INC_HORIZONTAL_US 922U /* Servo 40 deg -> panel horizontal */
#define PANEL_INC_ZERO_SERVO_DEG 40

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

/* ================= 2. TIPOS Y ESTADO DEL SISTEMA ================= */
typedef enum {
    SERVO_AZ = 0,   /* Azimut:      P2.4 / PWM1.5 */
    SERVO_INC,      /* Inclinacion: P2.5 / PWM1.6 */
    NUM_SERVOS
} SERVO_ID;

typedef enum {
    STAGE_INCLINATION = 0,  /* Corrigiendo inclinacion (y quizas giro) */
    STAGE_ROTATION,         /* Corrigiendo solo el giro */
    STAGE_BALANCED,         /* Errores dentro de la zona muerta */
    STAGE_LOW_LIGHT,        /* Poca luz: no se mueve */
    STAGE_LDR_ERROR,        /* Fallo la lectura de las LDR */
    STAGE_MANUAL,           /* Modo manual */
    STAGE_AT_LIMIT          /* Hay error pero el servo ya esta en su limite */
} TRACK_STAGE;

/* Estado compartido:
 * static limita la visibilidad a este archivo; const indica tablas que no cambian.
 * volatile evita que el compilador omita lecturas/escrituras observables por ISR,
 * perifericos o debugger; NO convierte por si solo una operacion en atomica.
 * Unidades del panel: tension mV, corriente uA, potencia uW, energia interna nJ.
 * servoPulseUs son consignas PWM en microsegundos; manualMode: 0 auto, 1 manual.
 */
static const ADC_CHANNEL ldrChannel[NUM_LDR] = {
    ADC_CHANNEL_0,  /* LDR0 arriba centrada  - P0.23 */
    ADC_CHANNEL_1,  /* LDR1 abajo derecha    - P0.24 */
    ADC_CHANNEL_2   /* LDR2 abajo izquierda  - P0.25 */
};
static const char* const ldrName[NUM_LDR] = { "ARRIBA", "AB DER", "AB IZQ" };
static const ADC_CHANNEL joyChannel[2]   = { ADC_CHANNEL_4, ADC_CHANNEL_5 };
static const ADC_CHANNEL panelChannel[1] = { PANEL_CHANNEL };

static const uint8_t  servoPwmCh[NUM_SERVOS] = { 5, 6 };
static const uint32_t servoMin[NUM_SERVOS]   = { SERVO_AZ_MIN_US, SERVO_INC_MIN_US };
static const uint32_t servoMax[NUM_SERVOS]   = { SERVO_AZ_MAX_US, SERVO_INC_MAX_US };

volatile uint16_t ldrValue[NUM_LDR];
volatile uint8_t  ldrValid = 0;      /* Inhibe seguimiento si falla la adquisicion */
volatile uint8_t  brightestLdr = NO_LDR;
volatile uint16_t joyValue[2];       /* [0] = X, [1] = Y */
volatile uint16_t joyCenter[2];
volatile uint8_t  manualMode = 0;    /* 0 = automatico, 1 = manual */
volatile uint32_t servoPulseUs[NUM_SERVOS] = { SERVO_CENTER_US, SERVO_INC_HORIZONTAL_US };
volatile uint8_t  oledOk = 0;
volatile uint8_t  screen = 0;

volatile int32_t  trackErrH = 0;
volatile int32_t  trackErrV = 0;
volatile uint8_t  trackStage = STAGE_BALANCED;
static uint32_t   trackLastMs = 0;
static uint8_t    trackActive[NUM_SERVOS];   /* Estado de la histeresis de cada eje */

volatile uint8_t  panelValid = 0;
volatile uint16_t panelAdc   = 0;    /* Promedio de 32 muestras (0..4095) */
volatile uint32_t panelMv    = 0;    /* Tension promediada */
volatile uint32_t rawMv      = 0;    /* Tension de la ultima muestra */
volatile uint32_t averageMv  = 0;    /* Promedio movil */
volatile uint32_t minimumMv  = 0;
volatile uint32_t maximumMv  = 0;
volatile uint32_t currentUa  = 0;    /* Corriente en la carga (uA) */
volatile uint32_t powerUw    = 0;    /* Potencia (uW) */
static uint64_t   energyNj   = 0;    /* Energia acumulada (nJ = uW * ms) */

static volatile uint32_t* const adcDmaBuf = (volatile uint32_t*)AHB_RAM_BASE;
static const uint8_t adcScan[] = {0, 1, 2, 4, 5, 7};
static uint8_t adcScanIndex = 0;
static uint8_t adcDmaBusy = 0;
static uint32_t adcDmaStarted = 0;
static uint32_t adcDmaLastStart = 0;
static uint32_t adcLastGood[8];
static uint16_t adcLatest[8];
volatile uint16_t adcAvg[8];
volatile uint16_t adcCount[8];
volatile uint32_t adcDmaBlocks = 0;
volatile uint32_t adcDmaErrors = 0;
volatile uint32_t adcDmaTimeouts = 0;
volatile uint32_t adcDmaBadSamples = 0;
volatile uint8_t adcDmaLastFailure = 0; /* 0 OK, 1 DMA, 2 timeout, 3 muestras */
volatile uint8_t dacDmaOk = 0;

static volatile uint32_t* const dacWord = (volatile uint32_t*)(AHB_RAM_BASE + 0xC00U);
static GPDMA_LLI_T* const       dacLli  = (GPDMA_LLI_T*)(AHB_RAM_BASE + 0xC10U);

/* Promedio movil: RAM, independiente del historico persistente logMv[6]. */
static uint32_t history[HISTORY_MAX];
static uint32_t historyIndex = 0;
static uint32_t historyCount = 0;
static uint32_t historySum   = 0;
static uint32_t windowSize   = 50;
static uint64_t sessionSum   = 0;
static uint32_t sampleCount  = 0;

static volatile uint8_t  rxBuffer[RX_SIZE];
static volatile uint32_t rxHead  = 0;
static volatile uint32_t rxTail  = 0;
static volatile uint8_t  rxError = 0;

static uint8_t displayDirty = 1;     /* Fuerza un redibujo inmediato */

static volatile uint32_t msTicks    = 0;
static volatile uint32_t lastEdgeMs = 0;

/* ================= 3. INTERRUPCIONES: registrar eventos; el trabajo pesado queda en main ================= */

/* Interrupcion de 1 ms: solo actualiza el reloj que usa el lazo principal. */
void SysTick_Handler(void) {
    msTicks++;
}

/* Pulsador: detecta flancos, filtra rebotes y alterna manualMode. No escribe flash aqui. */
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

/* Recepcion UART: copia los bytes al buffer circular. main interpretara los comandos. */
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

/* ================= 4. UTILIDADES DE TIEMPO Y FORMATO (sin printf) ================= */

/* Espera activa usada al encender perifericos; necesita SysTick funcionando. */
static void delay_ms(uint32_t ms) {
    const uint32_t start = msTicks;
    while ((msTicks - start) < ms) {
    }
}

/* Agrega texto y devuelve el final para seguir concatenando. El llamador reserva espacio. */
static char* str_cat(char* p, const char* s) {
    while (*s) {
        *p++ = *s++;
    }
    *p = '\0';
    return p;
}

/* Convierte un entero sin signo a decimal sin printf ni dependencias de Redlib. */
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

/* Como str_uint, pero admite valores negativos (inclinacion y errores de seguimiento). */
static char* str_int(char* p, int32_t v) {
    if (v < 0) {
        *p++ = '-';
        v = -v;
    }
    return str_uint(p, (uint32_t)v);
}

/* Convierte mV a voltios con dos decimales y redondeo, para la OLED. */
static char* str_volts(char* p, uint32_t mv) {
    const uint32_t cv = (mv + 5) / 10;  /* Centesimas de voltio */
    p = str_uint(p, cv / 100);
    *p++ = '.';
    *p++ = (char)('0' + (cv % 100) / 10);
    *p++ = (char)('0' + cv % 10);
    *p = '\0';
    return p;
}

/* Formatea un valor entero con escala decimal; evita calculos con float para mostrar unidades. */
static char* str_dec(char* p, uint32_t value, uint8_t decimals) {
    uint32_t div = 1;
    for (uint8_t i = 0; i < decimals; i++) {
        div *= 10U;
    }
    p = str_uint(p, value / div);
    if (decimals) {
        uint32_t frac = value % div;
        *p++ = '.';
        for (uint32_t d = div / 10U; d > 0; d /= 10U) {
            *p++ = (char)('0' + (frac / d) % 10U);
        }
        *p = '\0';
    }
    return p;
}

/* Agrega un campo numerico al mensaje JSON que se esta construyendo. */
static char* json_field(char* p, const char* name, uint32_t value) {
    p = str_cat(p, ",\"");
    p = str_cat(p, name);
    p = str_cat(p, "\":");
    return str_uint(p, value);
}

/* ================= 5. GPIO, PULSADOR Y UART ================= */

/* Configura las salidas RGB; son activas en bajo (0 enciende, 1 apaga). */
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

/* Verde = automatico; azul = manual. */
static void RGB_ShowMode(uint8_t manual) {
    GPIO_SetPins(RGB_GB_PORT, RGB_GREEN_MASK | RGB_BLUE_MASK);
    GPIO_ClearPins(RGB_GB_PORT, manual ? RGB_BLUE_MASK : RGB_GREEN_MASK);
}

/* Configura el pulsador con pull-up y flancos de subida/bajada para filtrar rebotes. */
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

/* UART2: 115200 baudios, 8N1, TX P0.10 y RX P0.11; recepcion por interrupcion. */
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

/* Transmite los bytes entre start y end. Es bloqueante, como en la version probada. */
static void UART_SendRange(const char* start, const char* end) {
    UART_Send(UART2, (const uint8_t*)start, (uint32_t)(end - start), BLOCKING);
}

/* ================= 6. SERVOS: PWM y conversion mecanica ================= */

/* Limita el pulso a la mecanica y actualiza el registro PWM y su latch para el siguiente periodo. */
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

    LPC_PWM1->LER |= (1U << servoPwmCh[s]);
}

/* Carga primero los pulsos recuperados de flash y luego activa PWM. No pasa por el centro al arrancar. */
static void SERVO_Init(void) {
    PINSEL_CFG_T cfg = {
        .port      = PORT_2,
        .pin       = PIN_4,
        .func      = PINSEL_FUNC_01,  /* PWM1.5 */
        .mode      = PINSEL_TRISTATE,
        .openDrain = DISABLE
    };
    CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCPWM1, ENABLE);
    const uint32_t pclk = CLKPWR_GetPCLK(CLKPWR_PCLKSEL_PWM1);

    LPC_PWM1->TCR = (1U << 1);                    /* Reset del contador */
    LPC_PWM1->PR  = pclk / 1000000U - 1U;         /* Tick de 1 us */
    LPC_PWM1->MR0 = SERVO_PERIOD_US;
    LPC_PWM1->MR5 = servoPulseUs[SERVO_AZ];
    LPC_PWM1->MR6 = servoPulseUs[SERVO_INC];
    LPC_PWM1->MCR = (1U << 1);                    /* Reinicia el contador en MR0 */
    LPC_PWM1->PCR = (1U << 13) | (1U << 14);      /* Salidas PWM1.5 y PWM1.6 (simple flanco) */
    LPC_PWM1->LER = (1U << 0) | (1U << 5) | (1U << 6);

    PINSEL_ConfigPin(&cfg);
    cfg.pin = PIN_5;
    PINSEL_ConfigPin(&cfg);
    LPC_PWM1->TCR = (1U << 0) | (1U << 3);        /* Contador + modo PWM habilitados */

}

/* Control manual por velocidad: mas desviacion del joystick produce un paso mayor; al soltar se detiene. */
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

/* Indica si un movimiento en ese sentido chocaria con el limite de pulso configurado. */
static uint8_t SERVO_AtLimit(SERVO_ID s, int32_t dir) {
    return (dir > 0 && servoPulseUs[s] >= servoMax[s]) ||
           (dir < 0 && servoPulseUs[s] <= servoMin[s]);
}

/* Convierte el pulso solicitado a grados. En inclinacion resta 40: horizontal=0, vertical=90. */
static int32_t SERVO_GetDegrees(SERVO_ID s) {
    const uint32_t us = servoPulseUs[s];
    const uint32_t span = SERVO_US_180DEG - SERVO_US_0DEG;
    int32_t deg;
    if (us <= SERVO_US_0DEG) deg = 0;
    else if (us >= SERVO_US_180DEG) deg = 180;
    else deg = (int32_t)(((us - SERVO_US_0DEG) * 180U + span / 2U) / span);
    return s == SERVO_INC ? deg - PANEL_INC_ZERO_SERVO_DEG : deg;
}

/* ================= 7. ADC + DMA: un canal por bloque, sin interrupcion DMA ================= */

/* Prepara reloj y pines analogicos. La adquisicion se inicia despues, desde ADCDMA_Start. */
static void ADC_Setup(void) {
    ADC_Init(ADC_RATE_HZ);
    LPC_ADC->ADCR &= ~((7UL << 24) | (1UL << 16));
    LPC_ADC->ADINTEN = 0;
    NVIC_DisableIRQ(ADC_IRQn);
    for (uint8_t i = 0; i < NUM_LDR; i++) {
        ADC_PinConfig(ldrChannel[i]);
    }
    for (uint8_t i = 0; i < 2; i++) {
        ADC_PinConfig(joyChannel[i]);
    }
    ADC_PinConfig(panelChannel[0]);
}

/* Detiene conversiones y solicitudes DMA del ADC; marca el canal como disponible. */
static void ADCDMA_Stop(void) {
    ADC_BurstDisable();
    LPC_ADC->ADINTEN = 0;
    ADC_DMA_REGS->DMACCConfig = 0;
    __DSB();
    adcDmaBusy = 0;
}

/* Avanza circularmente por los seis canales: LDR 0,1,2; joystick 4,5; panel 7. */
static void ADCDMA_Advance(void) {
    adcScanIndex = (uint8_t)((adcScanIndex + 1U) % (sizeof(adcScan) / sizeof(adcScan[0])));
}

/* Adquiere 65 palabras de UN canal: origen ADDRx fijo, destino incremental; DMA canal 0. */
static void ADCDMA_Start(void) {
    const uint32_t ch = adcScan[adcScanIndex];
    volatile const uint32_t* const data = &LPC_ADC->ADDR0 + ch;

    ADCDMA_Stop();
    LPC_GPDMA->DMACIntTCClear = 1U;
    LPC_GPDMA->DMACIntErrClr = 1U;
    LPC_ADC->ADCR = (LPC_ADC->ADCR & ~((7UL << 24) | (1UL << 16) | 0xFFUL)) | (1UL << ch);
    (void)*data; /* Limpiar DONE anterior del canal que se va a adquirir. */
    LPC_ADC->ADINTEN = 1UL << ch; /* ADGINTEN=0 en burst. */

    ADC_DMA_REGS->DMACCSrcAddr = (uint32_t)data;
    ADC_DMA_REGS->DMACCDestAddr = (uint32_t)adcDmaBuf;
    ADC_DMA_REGS->DMACCLLI = 0;
    ADC_DMA_REGS->DMACCControl =
        GPDMA_DMACCxControl_TransferSize(ADC_DMA_BUF_LEN) |
        GPDMA_DMACCxControl_SBSize(GPDMA_BSIZE_1) |
        GPDMA_DMACCxControl_DBSize(GPDMA_BSIZE_1) |
        GPDMA_DMACCxControl_SWidth(GPDMA_WORD) |
        GPDMA_DMACCxControl_DWidth(GPDMA_WORD) |
        GPDMA_DMACCxControl_DI |
        GPDMA_DMACCxControl_I;

    adcDmaStarted = msTicks;
    adcDmaLastStart = msTicks;
    adcDmaBusy = 1;
    __DMB();
    ADC_DMA_REGS->DMACCConfig =
        GPDMA_DMACCxConfig_E |
        GPDMA_DMACCxConfig_SrcPeripheral(4U) | /* ADC: pedido 4 */
        GPDMA_DMACCxConfig_TransferType(GPDMA_P2M) |
        GPDMA_DMACCxConfig_IE | GPDMA_DMACCxConfig_ITC;
    ADC_BurstEnable();
}

/* Limpia el estado de adquisicion e inicia el primer bloque. ADC_Setup debe haberse ejecutado. */
static void ADCDMA_Init(void) {
    NVIC_DisableIRQ(DMA_IRQn);
    NVIC_ClearPendingIRQ(DMA_IRQn);
    for (uint32_t i = 0; i < ADC_DMA_BUF_LEN; i++) {
        adcDmaBuf[i] = 0;
    }
    for (uint8_t ch = 0; ch < 8; ch++) {
        adcAvg[ch] = 0;
        adcCount[ch] = 0;
        adcLatest[ch] = 0;
        adcLastGood[ch] = 0;
    }
    adcScanIndex = 0;
    adcDmaBlocks = adcDmaErrors = adcDmaTimeouts = adcDmaBadSamples = 0;
    adcDmaLastFailure = 0;
    ADCDMA_Start();
}

/* Descarta la primera muestra y las invalidas; promedia las restantes si hay al menos 32 utiles. */
static void ADCDMA_Consume(void) {
    const uint8_t ch = adcScan[adcScanIndex];
    uint32_t sum = 0;
    uint32_t count = 0;
    uint16_t latest = 0;

    for (uint32_t i = 1; i < ADC_DMA_BUF_LEN; i++) {
        const uint32_t word = adcDmaBuf[i];
        if (!(word & ADC_DR_DONE_FLAG) || (word & ADC_DR_OVERRUN_FLAG)) {
            adcDmaBadSamples++;
            continue;
        }
        latest = (uint16_t)ADC_DR_RESULT(word);
        sum += latest;
        count++;
    }
    adcDmaBlocks++;
    if (count < ADC_DMA_MIN_SAMPLES) {
        adcCount[ch] = 0;
        adcDmaErrors++;
        adcDmaLastFailure = 3;
        return;
    }
    adcAvg[ch] = (uint16_t)((sum + count / 2U) / count);
    adcLatest[ch] = latest;
    adcLastGood[ch] = adcDmaStarted;
    adcCount[ch] = (uint16_t)count;
    adcDmaLastFailure = 0;
}

/* Atiende fin de bloque, errores y timeout por consulta de flags. Invalida datos viejos y lanza el siguiente canal. */
static void ADCDMA_Update(void) {
    const uint32_t now = msTicks;

    if (LPC_GPDMA->DMACRawIntErrStat & 2U) {
        LPC_GPDMACH1->DMACCConfig = 0;
        LPC_DAC->DACCTRL = 0;
        LPC_DAC->DACR = 0;
        LPC_GPDMA->DMACIntErrClr = 2U;
        dacDmaOk = 0;
    }

    if (adcDmaBusy) {
        if (LPC_GPDMA->DMACRawIntErrStat & 1U) {
            ADCDMA_Stop();
            LPC_GPDMA->DMACIntErrClr = 1U;
            LPC_GPDMA->DMACIntTCClear = 1U;
            adcCount[adcScan[adcScanIndex]] = 0;
            adcDmaErrors++;
            adcDmaLastFailure = 1;
            ADCDMA_Advance();
        } else if (LPC_GPDMA->DMACRawIntTCStat & 1U) {
            ADCDMA_Stop();
            LPC_GPDMA->DMACIntTCClear = 1U;
            __DMB();
            ADCDMA_Consume();
            ADCDMA_Advance();
        } else if ((now - adcDmaStarted) > ADC_DMA_TIMEOUT_MS) {
            ADCDMA_Stop();
            LPC_GPDMA->DMACIntTCClear = 1U;
            LPC_GPDMA->DMACIntErrClr = 1U;
            adcCount[adcScan[adcScanIndex]] = 0;
            adcDmaTimeouts++;
            adcDmaLastFailure = 2;
            ADCDMA_Advance();
        }
    }
    for (uint8_t ch = 0; ch < 8; ch++) {
        if (adcCount[ch] && (now - adcLastGood[ch]) > ADC_DMA_STALE_MS) {
            adcCount[ch] = 0;
        }
    }
    if (!adcDmaBusy && (now - adcDmaLastStart) >= ADC_DMA_START_MS) {
        ADCDMA_Start();
    }
}

/* Copia promedios de canales solo si todos tienen muestras recientes; devuelve 1 si pudo hacerlo. */
static uint8_t ADCDMA_Get(const ADC_CHANNEL ch[], uint8_t n, volatile uint16_t out[]) {
    for (uint8_t i = 0; i < n; i++) {
        if (!adcCount[ch[i]] || (msTicks - adcLastGood[ch[i]]) > ADC_DMA_STALE_MS) {
            return 0;
        }
    }
    for (uint8_t i = 0; i < n; i++) {
        out[i] = adcAvg[ch[i]];
    }
    return 1;
}

/* Devuelve la ultima conversion individual del canal, sin el promedio del bloque. */
static uint16_t ADCDMA_Latest(ADC_CHANNEL ch) {
    return adcLatest[ch];
}

/* ================= 8. DAC + DMA: salida analogica proporcional ================= */

/* Configura P0.26 y DMA canal 1: un descriptor circular envia dacWord al DAC a 1 kHz. */
static void DAC_Setup(void) {
    LPC_PINCON->PINSEL1 = (LPC_PINCON->PINSEL1 & ~(3U << 20)) | (2U << 20);
    LPC_PINCON->PINMODE1 = (LPC_PINCON->PINMODE1 & ~(3U << 20)) | (2U << 20);
    *dacWord = 0;
    LPC_DAC->DACR = 0;
    LPC_DAC->DACCTRL = 0;
    uint32_t period = CLKPWR_GetPCLK(CLKPWR_PCLKSEL_DAC) / DAC_UPDATE_HZ;
    if (!period) period = 1;
    if (period > 65536U) period = 65536U;
    LPC_DAC->DACCNTVAL = period - 1U;

    dacLli->srcAddr = (uint32_t)dacWord;
    dacLli->dstAddr = (uint32_t)&LPC_DAC->DACR;
    dacLli->nextLLI = (uint32_t)dacLli;
    dacLli->control =
        GPDMA_DMACCxControl_TransferSize(1U) |
        GPDMA_DMACCxControl_SBSize(GPDMA_BSIZE_1) |
        GPDMA_DMACCxControl_DBSize(GPDMA_BSIZE_1) |
        GPDMA_DMACCxControl_SWidth(GPDMA_WORD) |
        GPDMA_DMACCxControl_DWidth(GPDMA_WORD);
    DAC_DMA_REGS->DMACCConfig = 0;
    LPC_GPDMA->DMACIntTCClear = 2U;
    LPC_GPDMA->DMACIntErrClr = 2U;
    DAC_DMA_REGS->DMACCSrcAddr = dacLli->srcAddr;
    DAC_DMA_REGS->DMACCDestAddr = dacLli->dstAddr;
    DAC_DMA_REGS->DMACCLLI = (uint32_t)dacLli;
    DAC_DMA_REGS->DMACCControl = dacLli->control;
    __DMB();
    DAC_DMA_REGS->DMACCConfig =
        GPDMA_DMACCxConfig_E |
        GPDMA_DMACCxConfig_DestPeripheral(7U) |
        GPDMA_DMACCxConfig_TransferType(GPDMA_M2P) |
        GPDMA_DMACCxConfig_IE;
    LPC_DAC->DACCTRL = (1U << 1) | (1U << 2) | (1U << 3);
    dacDmaOk = 1;
}

/* Escala el promedio ADC de 12 bits al DAC de 10 bits. El DMA transmite el valor actualizado. */
static void DAC_SetFromAdc(uint16_t adc12) {
    *dacWord = ((uint32_t)(adc12 >> 2) & 0x3FFU) << 6;
}

/* ================= 9. PANEL: tension, estadisticas, corriente calculada y energia ================= */

/* Deshace el divisor 10k/4k7: Vpanel_mV = ADC*3300*(10000+4700)/(4095*4700). Usa 64 bits. */
static uint32_t PANEL_AdcToMv(uint32_t adc) {
    const uint64_t num = (uint64_t)adc * ADC_REFERENCE_MV * (R_SUPERIOR + R_INFERIOR);
    const uint64_t den = (uint64_t)4095 * R_INFERIOR;
    return (uint32_t)((num + den / 2) / den);
}

/* Reinicia SOLO la ventana del promedio movil. No borra las seis lecturas persistentes. */
static void PANEL_ResetHistory(void) {
    historyIndex = 0;
    historyCount = 0;
    historySum   = 0;
    averageMv    = 0;
}

/* Reinicia energia y estadisticas en RAM. El historico de seis tensiones se conserva. */
static void PANEL_ResetStatistics(void) {
    PANEL_ResetHistory();
    energyNj    = 0;
    sessionSum  = 0;
    sampleCount = 0;
    minimumMv   = 0;
    maximumMv   = 0;
}

/* Convierte energia acumulada en nJ a uWh: dividir por 3.600.000. */
static uint32_t PANEL_EnergyUwh(void) {
    return (uint32_t)(energyNj / 3600000ULL);
}

/* Usa el ADC promediado: I_uA=V_mV*1000/R; P_uW=V_mV^2/R; E_nJ+=P_uW*dt_ms. Actualiza DAC y estadisticas. */
static void PANEL_Measure(void) {
    static uint32_t lastMs  = 0;
    static uint8_t  started = 0;
    const uint32_t  now     = msTicks;

    panelValid = adcCount[PANEL_CHANNEL] > 0;
    if (!panelValid) {
        started = 0;
        currentUa = 0;
        powerUw = 0;
        DAC_SetFromAdc(0);
        return;
    }

    panelAdc = adcAvg[PANEL_CHANNEL];
    panelMv  = PANEL_AdcToMv(panelAdc);
    rawMv    = PANEL_AdcToMv(ADCDMA_Latest(PANEL_CHANNEL));

    currentUa = (uint32_t)((uint64_t)panelMv * 1000ULL / R_LOAD_OHM);
    powerUw   = (uint32_t)((uint64_t)panelMv * panelMv / R_LOAD_OHM);

    if (started) {
        energyNj += (uint64_t)powerUw * (now - lastMs);
    }
    lastMs  = now;
    started = 1;

    DAC_SetFromAdc(panelAdc);

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

/* Promedio de todas las mediciones validas de tension desde el inicio o RESET. */
static uint32_t PANEL_SessionAverage(void) {
    return sampleCount ? (uint32_t)(sessionSum / sampleCount) : 0;
}

/* ================= 10. LDR Y JOYSTICK: lecturas para los modos de control ================= */

/* Compara cuentas ADC segun la polaridad del divisor LDR configurada al principio. */
static uint8_t LDR_IsBrighter(uint16_t a, uint16_t b) {
#if LDR_MORE_LIGHT_HIGHER
    return a > b;
#else
    return a < b;
#endif
}

/* Actualiza las tres LDR y el sensor mas iluminado; la histeresis evita alternar ante diferencias pequenas. */
static uint8_t LDR_Update(uint8_t current) {
    ldrValid = ADCDMA_Get(ldrChannel, NUM_LDR, ldrValue);
    if (!ldrValid) {
        return NO_LDR;
    }

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

/* Mide el centro del joystick al encender. No tocarlo durante el arranque; fallback=2048 si no hay datos. */
static void JOY_Calibrate(void) {
    const uint32_t started = msTicks;
    while ((!adcCount[joyChannel[0]] || !adcCount[joyChannel[1]]) &&
           (msTicks - started) < 250U) {
        ADCDMA_Update();
    }
    if (!ADCDMA_Get(joyChannel, 2, joyCenter)) {
        joyCenter[0] = 2048;                      /* Sin muestras: centro teorico */
        joyCenter[1] = 2048;
    }
}

/* Copia los promedios X/Y disponibles; no realiza conversiones nuevas por su cuenta. */
static void JOY_Read(void) {
    ADCDMA_Get(joyChannel, 2, joyValue);
}

/* Resta el centro, aplica las inversiones de sentido y mueve cada eje con zona muerta. */
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

/* Expresa siempre mayor iluminacion como mayor numero, segun LDR_MORE_LIGHT_HIGHER. */
static int32_t LDR_Brightness(uint16_t adc) {
#if LDR_MORE_LIGHT_HIGHER
    return adc;
#else
    return 4095 - (int32_t)adc;
#endif
}

/* ================= 11. SEGUIMIENTO AUTOMATICO ================= */

/* Histeresis: empieza a corregir sobre START y deja de hacerlo debajo de STOP. */
static uint8_t TRACK_AxisActive(SERVO_ID s, int32_t mag) {
    if (trackActive[s]) {
        if (mag < TRACK_DEADBAND_STOP) {
            trackActive[s] = 0;
        }
    } else if (mag > TRACK_DEADBAND_START) {
        trackActive[s] = 1;
    }
    return trackActive[s];
}

/* Convierte el error de luz en un paso PWM limitado por TRACK_MIN_STEP/MAX_STEP. */
static void TRACK_MoveAxis(SERVO_ID s, int32_t err) {
    const int32_t mag = err < 0 ? -err : err;

    int32_t step = (mag - TRACK_DEADBAND_STOP) / TRACK_GAIN_DIV;
    if (step < TRACK_MIN_STEP) {
        step = TRACK_MIN_STEP;
    } else if (step > TRACK_MAX_STEP) {
        step = TRACK_MAX_STEP;
    }

    int32_t pulse = (int32_t)servoPulseUs[s] + (err > 0 ? step : -step);
    if (pulse < 0) {
        pulse = 0;
    }
    SERVO_SetPulse(s, (uint32_t)pulse);
}

/* Automatico probado: compara derecha/izquierda y arriba/promedio inferior; filtra errores, aplica histeresis y limites. */
static void TRACK_Update(void) {
    if (!ldrValid) {
        trackErrH = 0;
        trackErrV = 0;
        trackActive[SERVO_AZ]  = 0;
        trackActive[SERVO_INC] = 0;
        trackStage = STAGE_LDR_ERROR;
        return;
    }

    const int32_t top   = LDR_Brightness(ldrValue[0]);
    const int32_t right = LDR_Brightness(ldrValue[1]);
    const int32_t left  = LDR_Brightness(ldrValue[2]);
    const int32_t total = top + right + left;

    if (total < TRACK_MIN_LIGHT) {
        trackErrH = 0;
        trackErrV = 0;
        trackActive[SERVO_AZ]  = 0;
        trackActive[SERVO_INC] = 0;
        trackStage = STAGE_LOW_LIGHT;
        return;
    }

    const int32_t bottomSum = right + left;
    const int32_t bottomAvg = bottomSum / 2;
    const int32_t vertSum   = top + bottomAvg;

    const int32_t rawH = bottomSum > 0 ? (right - left) * 1000 / bottomSum - TRACK_H_OFFSET : 0;
    const int32_t rawV = vertSum   > 0 ? (top - bottomAvg) * 1000 / vertSum - TRACK_V_OFFSET : 0;

    trackErrH += (rawH - trackErrH) / TRACK_FILTER_DIV;
    trackErrV += (rawV - trackErrV) / TRACK_FILTER_DIV;

    const uint32_t now = msTicks;
    if ((now - trackLastMs) < TRACK_PERIOD_MS) {
        return;
    }
    trackLastMs = now;

    int32_t az  = trackErrH;
    int32_t inc = trackErrV;
#if SERVO_AZ_INVERT
    az = -az;
#endif
#if SERVO_INC_INVERT
    inc = -inc;
#endif
#if TRACK_AZ_INVERT
    az = -az;
#endif
#if TRACK_INC_INVERT
    inc = -inc;
#endif

    const int32_t magV = trackErrV < 0 ? -trackErrV : trackErrV;
    const int32_t magH = trackErrH < 0 ? -trackErrH : trackErrH;
    const uint8_t errInc = TRACK_AxisActive(SERVO_INC, magV);
    const uint8_t errAz  = TRACK_AxisActive(SERVO_AZ, magH);
    const uint8_t needInc = errInc && !SERVO_AtLimit(SERVO_INC, inc);
    const uint8_t needAz  = errAz  && !SERVO_AtLimit(SERVO_AZ, az);

#if TRACK_SEQUENTIAL
    if (needInc) {
        TRACK_MoveAxis(SERVO_INC, inc);   /* Giro quieto hasta terminar la inclinacion */
    } else if (needAz) {
        TRACK_MoveAxis(SERVO_AZ, az);
    }
    const uint8_t movedAz = !needInc && needAz;
#else
    if (needInc) {
        TRACK_MoveAxis(SERVO_INC, inc);
    }
    if (needAz) {
        TRACK_MoveAxis(SERVO_AZ, az);
    }
    const uint8_t movedAz = needAz;
#endif

    if (needInc) {
        trackStage = STAGE_INCLINATION;
    } else if (movedAz) {
        trackStage = STAGE_ROTATION;
    } else if (errInc || errAz) {
        trackStage = STAGE_AT_LIMIT;      /* Hay error pero los servos no pueden seguir */
    } else {
        trackStage = STAGE_BALANCED;
    }
}

/* ================= 12. RESPUESTAS UART ================= */

/* Envia confirmacion o rechazo JSON. El HTML espera type=ack/error y command. */
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

#define STORE_BASE_A 0x00070000UL
#define STORE_BASE_B 0x00078000UL
#define STORE_MAGIC  0x534F4C33UL
#define LOG_COUNT 6U
#define LOG_PERIOD_MS 10000U
static uint32_t logMv[LOG_COUNT];
static uint32_t logCount = 0, logNext = 0;
static uint32_t logRevision = 0, storeLogRevision = 0;
static uint8_t storeScreen = 0;
static uint32_t storeAddress = 0, storeSequence = 0;
static uint8_t storeMode = 0, storeRestored = 0;
static uint32_t storeStatus = 0; /* 0 OK; 100 linker; 101 verificacion; otros: ROM IAP */
/* Registro flash de 256 bytes (64 palabras de 32 bits):
 * w[0..2]: firma, version, secuencia; w[3]: modo; w[4..5]: pulsos PWM;
 * w[6..7]: energia de 64 bits; w[8..9]: pantalla y ventana;
 * w[10..11]: cantidad/indice del historico; w[12..17]: seis tensiones mV;
 * w[18..62]: espacio reservado; w[63]: CRC32.
 */
typedef struct { uint32_t w[64]; } StoreRecord;
static StoreRecord storeBuffer __attribute__((aligned(256)));
extern uint32_t __data_section_table[] __attribute__((weak));
extern uint32_t __data_section_table_end[] __attribute__((weak));
extern uint32_t _etext __attribute__((weak));
/* ================= 13. FLASH: diario de capturas con CRC ================= */

/* Comprueba que la imagen de codigo/datos del linker MCUXpresso no invade los sectores de guardado. */
static int STORE_AreaSafe(void) {
    uint32_t start=(uint32_t)__data_section_table;
    uint32_t end=(uint32_t)__data_section_table_end;
    if (!start || end<=start || end-start>256U || (end-start)%12U || end>STORE_BASE_A) return 0;
    uint32_t imageEnd=(uint32_t)&_etext;
    for (const uint32_t *p=__data_section_table; p<__data_section_table_end; p+=3) {
        uint32_t load=p[0], size=p[2];
        if (load>=0x80000U || size>0x80000U-load) return 0;
        if (load+size>imageEnd) imageEnd=load+size;
    }
    return imageEnd>0 && imageEnd<=STORE_BASE_A;
}

/* CRC32 sobre los primeros 252 bytes del registro; permite detectar datos incompletos o corruptos. */
static uint32_t STORE_Crc(const StoreRecord *r) {
    uint32_t c = 0xFFFFFFFFUL;
    const uint8_t *p = (const uint8_t*)r;
    for (uint32_t i = 0; i < 252; i++) {
        c ^= p[i];
        for (uint32_t k = 0; k < 8; k++)
            c = (c >> 1) ^ ((c & 1U) ? 0xEDB88320UL : 0);
    }
    return ~c;
}
/* Valida identificador, version, rangos y CRC antes de aceptar un registro de flash. */
static uint8_t STORE_Valid(const StoreRecord *r) {
    return r->w[0] == STORE_MAGIC && (r->w[1] == 1U || r->w[1] == 2U) && r->w[3] <= 1U &&
        r->w[4] >= SERVO_AZ_MIN_US && r->w[4] <= SERVO_AZ_MAX_US &&
        r->w[5] >= SERVO_US_0DEG && r->w[5] <= SERVO_US_180DEG &&
        r->w[8] < NUM_SCREENS &&
        (r->w[9] == 10 || r->w[9] == 50 || r->w[9] == 100) &&
        (r->w[1] == 1U || (r->w[10] <= LOG_COUNT && r->w[11] < LOG_COUNT)) &&
        r->w[63] == STORE_Crc(r);
}

/* Busca la secuencia valida mas reciente y restaura RAM. Se ejecuta ANTES de habilitar los servos. */
static void STORE_Load(void) {
    storeMode = manualMode; storeScreen = screen;
    if (!STORE_AreaSafe()) { storeStatus = 100; return; }
    for (uint32_t a = STORE_BASE_A; a < 0x80000UL; a += 256U) {
        const StoreRecord *r = (const StoreRecord*)a;
        if (STORE_Valid(r) && (!storeAddress || (int32_t)(r->w[2]-storeSequence) > 0)) {
            storeAddress = a; storeSequence = r->w[2];
        }
    }
    if (storeAddress) {
        const StoreRecord *r = (const StoreRecord*)storeAddress;
        manualMode = (uint8_t)r->w[3]; storeMode = manualMode;
        for (uint32_t i = 0; i < NUM_SERVOS; i++) {
            uint32_t pulse = r->w[4U + i];
            if (pulse < servoMin[i]) pulse = servoMin[i];
            if (pulse > servoMax[i]) pulse = servoMax[i];
            servoPulseUs[i] = pulse;
        }
        energyNj = (uint64_t)r->w[6] | ((uint64_t)r->w[7] << 32);
        screen = (uint8_t)r->w[8]; windowSize = r->w[9];
        storeScreen = screen;
        if (r->w[1] == 2U) {
            logCount = r->w[10]; logNext = r->w[11];
            for (uint32_t i = 0; i < LOG_COUNT; i++) logMv[i] = r->w[12U+i];
        }
        storeRestored = 1;
    }
}
/* Invoca la rutina de programacion de flash en ROM. Devuelve 0 si la operacion fue correcta. */
static uint32_t STORE_Iap(uint32_t cmd, uint32_t a, uint32_t b, uint32_t c, uint32_t d) {
    uint32_t in[5] = {cmd,a,b,c,d}, out[5] = {0};
    ((void (*)(uint32_t*,uint32_t*))0x1FFF1FF1UL)(in,out);
    return out[0];
}
/* Agrega una captura de 256 bytes. Al llenarse el sector cambia al otro y lo borra; preserva el ultimo registro. */
static uint8_t STORE_Save(void) {
    if (!STORE_AreaSafe()) { storeStatus = 100; return 0; }
    uint32_t base = storeAddress >= STORE_BASE_B ? STORE_BASE_B : STORE_BASE_A;
    uint32_t target = storeAddress ? storeAddress + 256U : base;
    uint8_t erase = 0;

    for (; target < base + 0x8000U; target += 256U) {
        const uint32_t *p = (const uint32_t*)target;
        uint32_t i;
        for (i = 0; i < 64 && p[i] == 0xFFFFFFFFUL; i++) {}
        if (i == 64) break;
    }
    if (target >= base + 0x8000U) {
        base = base == STORE_BASE_A ? STORE_BASE_B : STORE_BASE_A;
        target = base; erase = 1;
    }
    memset(&storeBuffer, 0xFF, sizeof storeBuffer);
    uint32_t mask = __get_PRIMASK(); __disable_irq();
    storeBuffer.w[0] = STORE_MAGIC; storeBuffer.w[1] = 2;
    storeBuffer.w[2] = storeSequence + 1U; storeBuffer.w[3] = manualMode;
    storeBuffer.w[4] = servoPulseUs[0]; storeBuffer.w[5] = servoPulseUs[1];
    storeBuffer.w[6] = (uint32_t)energyNj; storeBuffer.w[7] = (uint32_t)(energyNj >> 32);
    storeBuffer.w[8] = screen; storeBuffer.w[9] = windowSize;
    storeBuffer.w[10] = logCount; storeBuffer.w[11] = logNext;
    for (uint32_t i = 0; i < LOG_COUNT; i++) storeBuffer.w[12U+i] = logMv[i];
    storeBuffer.w[63] = STORE_Crc(&storeBuffer);
    ADCDMA_Stop();
    uint32_t dacCtrl = LPC_DAC->DACCTRL;
    uint32_t dacConfig = LPC_GPDMACH1->DMACCConfig;
    LPC_DAC->DACCTRL = 0; LPC_GPDMACH1->DMACCConfig = 0;

    while ((ADC_DMA_REGS->DMACCConfig | LPC_GPDMACH1->DMACCConfig) & (1UL << 17)) {}
    __DSB();
    uint32_t sector = base == STORE_BASE_A ? 28U : 29U;
    uint32_t khz = SystemCoreClock / 1000U;
    uint32_t result = STORE_Iap(50,sector,sector,0,0);
    if (!result && erase) result = STORE_Iap(52,sector,sector,khz,0);
    if (!result) result = STORE_Iap(50,sector,sector,0,0);
    if (!result) result = STORE_Iap(51,target,(uint32_t)&storeBuffer,256U,khz);
    LPC_GPDMA->DMACIntTCClear = 1U;
    LPC_GPDMA->DMACIntErrClr = 1U;
    LPC_GPDMACH1->DMACCConfig = dacConfig;
    LPC_DAC->DACCTRL = dacCtrl;
    __set_PRIMASK(mask);
    storeStatus = result;
    if (!result && STORE_Valid((const StoreRecord*)target)) {
        storeAddress = target; storeSequence = storeBuffer.w[2];
        storeMode = (uint8_t)storeBuffer.w[3];
        storeScreen = (uint8_t)storeBuffer.w[8]; storeLogRevision = logRevision;
        return 1;
    }
    if (!result) storeStatus = 101;
    return 0;
}

/* Guarda si cambio modo, pantalla o historico. Ante un error espera 1 segundo antes de reintentar. */
static void STORE_Process(void) {
    static uint32_t lastAttempt = 0;
    static uint8_t failed = 0;
    if (manualMode == storeMode && screen == storeScreen && logRevision == storeLogRevision) return;
    if (failed && (msTicks - lastAttempt) < 1000U) return;
    lastAttempt = msTicks;
    failed = !STORE_Save();
}

static uint32_t logLastMs = 0;
/* ================= 14. HISTORICO: seis tensiones persistentes ================= */

/* Cada 10 s agrega la tension valida al anillo de seis posiciones y solicita persistir la captura completa. */
static void LOG_Process(void) {
    const uint32_t now = msTicks;
    if ((now - logLastMs) < LOG_PERIOD_MS) return;
    logLastMs = now;
    if (!panelValid || !adcCount[PANEL_CHANNEL] ||
        (now - adcLastGood[PANEL_CHANNEL]) > ADC_DMA_STALE_MS) return;
    logMv[logNext] = panelMv;
    logNext = (logNext + 1U) % LOG_COUNT;
    if (logCount < LOG_COUNT) logCount++;
    logRevision++;
    displayDirty = 1;
    STORE_Process();
}

/* Consulta las seis lecturas en orden cronologico sin mover el arreglo: primero la mas antigua. */
static uint32_t LOG_Value(uint32_t row) {
    uint32_t first = logCount == LOG_COUNT ? logNext : 0U;
    return logMv[(first + row) % LOG_COUNT];
}

/* ================= 15. COMANDOS Y TELEMETRIA JSON ================= */

/* Valida y ejecuta comandos completos; un comando desconocido recibe error sin ejecutar acciones. */
static void COMM_Execute(const char* cmd) {
    if (strcmp(cmd, "MODO,AUTO") == 0 || strcmp(cmd, "MODO,MANUAL") == 0) {
        manualMode = (uint8_t)(strcmp(cmd, "MODO,MANUAL") == 0);
        displayDirty = 1;
        COMM_Reply(STORE_Save() ? "ack" : "error", "MODO");
    } else if (strcmp(cmd, "GUARDAR") == 0) {
        COMM_Reply(STORE_Save() ? "ack" : "error", "GUARDAR");
    } else if (strlen(cmd) == 8 && strncmp(cmd, "SCREEN,", 7) == 0 &&
        cmd[7] >= '0' && cmd[7] < (char)('0' + NUM_SCREENS)) {
        screen       = (uint8_t)(cmd[7] - '0');
        displayDirty = 1;
        COMM_Reply(STORE_Save() ? "ack" : "error", "SCREEN");

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

/* Extrae bytes del buffer RX, arma lineas LF/CRLF y descarta tramas largas o invalidas. */
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

/* Construye y envia una linea JSON con mediciones, modo, posiciones e historico cada 500 ms. */
static void COMM_SendTelemetry(void) {
    char buf[1024];
    char* p = buf;

    p = str_cat(p, "{\"type\":\"data\"");
    p = json_field(p, "valid", panelValid);
    p = json_field(p, "raw_mv", rawMv);
    p = json_field(p, "panel_mv", panelMv);
    p = json_field(p, "avg_mv", averageMv);
    p = json_field(p, "session_mv", PANEL_SessionAverage());
    p = json_field(p, "min_mv", minimumMv);
    p = json_field(p, "max_mv", maximumMv);
    p = json_field(p, "current_ua", currentUa);
    p = json_field(p, "power_uw", powerUw);
    p = json_field(p, "energy_uwh", PANEL_EnergyUwh());
    p = json_field(p, "adc", panelAdc);
    p = json_field(p, "adc_n", adcCount[PANEL_CHANNEL]);
    p = json_field(p, "adc_dma_blocks", adcDmaBlocks);
    p = json_field(p, "adc_dma_errors", adcDmaErrors);
    p = json_field(p, "adc_dma_timeouts", adcDmaTimeouts);
    p = json_field(p, "adc_dma_bad", adcDmaBadSamples);
    p = json_field(p, "adc_dma_failure", adcDmaLastFailure);
    p = json_field(p, "adc_dma_channel", adcScan[adcScanIndex]);
    p = json_field(p, "dac_dma_ok", dacDmaOk);
    p = json_field(p, "samples", sampleCount);
    p = json_field(p, "window", windowSize);
    p = json_field(p, "screen", screen);
    p = json_field(p, "manual", manualMode);
    p = json_field(p, "flash_status", storeStatus);
    p = json_field(p, "flash_sequence", storeSequence);
    p = json_field(p, "flash_restored", storeRestored);
    p = json_field(p, "saved_manual", storeMode);
    p = json_field(p, "history_count", logCount);
    p = json_field(p, "history_period_s", LOG_PERIOD_MS / 1000U);
    p = str_cat(p, ",\"history_mv\":[");
    for (uint32_t i = 0; i < logCount; i++) {
        if (i) p = str_cat(p, ",");
        p = str_uint(p, LOG_Value(i));
    }
    p = str_cat(p, "]");
    p = json_field(p, "track_stage", trackStage);
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
    p = str_cat(p, "],\"servo_deg\":[");
    p = str_int(p, SERVO_GetDegrees(SERVO_AZ));
    p = str_cat(p, ",");
    p = str_int(p, SERVO_GetDegrees(SERVO_INC));
    p = str_cat(p, "],\"err\":[");
    p = str_int(p, trackErrH);
    p = str_cat(p, ",");
    p = str_int(p, trackErrV);
    p = str_cat(p, "]}\n");

    UART_SendRange(buf, p);
}

/* ================= 16. OLED: funciones de dibujo y seis pantallas ================= */

/* Centra un texto usando su ancho en pixeles. */
static void DrawCentered(int y, const char* str, uint8_t scale) {
    OLED_DrawString((OLED_WIDTH - OLED_StringWidth(str, scale)) / 2, y, str, scale);
}

/* Dibuja etiqueta, entero sin signo y unidad en la OLED. */
static void DrawValue(int y, const char* label, uint32_t value, const char* unit) {
    char buf[32];
    char* p = str_cat(buf, label);
    p = str_uint(p, value);
    str_cat(p, unit);
    OLED_DrawString(0, y, buf, 1);
}

/* Dibuja una tension recibida en mV con formato de voltios. */
static void DrawVolts(int y, const char* label, uint32_t mv) {
    char buf[32];
    char* p = str_cat(buf, label);
    p = str_volts(p, mv);
    str_cat(p, " V");
    OLED_DrawString(0, y, buf, 1);
}

/* Pantalla 0: estado general, tension y modo. */
static void Screen_Status(void) {
    char buf[24];
    char* p;

    DrawCentered(0, manualMode ? "MODO MANUAL" : "MODO AUTOMATICO", 1);

    if (!panelValid) {
        DrawCentered(20, "ERROR ADC", 2);
        DrawValue(41, "DMA bloques ", adcDmaBlocks, "");
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
        p = str_int(p, SERVO_GetDegrees(SERVO_AZ));
        p = str_cat(p, "   INC ");
        str_int(p, SERVO_GetDegrees(SERVO_INC));
    } else {
        p = str_cat(buf, "MAS LUZ: ");
        if (brightestLdr < NUM_LDR) {
            str_cat(p, ldrName[brightestLdr]);
        } else {
            str_cat(p, "-");
        }
    }
    DrawCentered(54, buf, 1);
}

/* Dibuja un entero con decimales y unidad, por ejemplo corriente en mA o energia en mWh. */
static void DrawFixed(int y, const char* label, uint32_t value, uint8_t decimals,
                      const char* unit) {
    char buf[32];
    char* p = str_cat(buf, label);
    p = str_dec(p, value, decimals);
    str_cat(p, unit);
    OLED_DrawString(0, y, buf, 1);
}

/* Pantalla 1: tension, corriente, potencia y energia; muestra error si no hay ADC valido. */
static void Screen_Panel(void) {
    OLED_DrawString(0, 0, "MEDICIONES PANEL", 1);
    if (!panelValid) {
        OLED_DrawString(0, 12, "ERROR ADC", 1);
        DrawValue(23, "Bloques ", adcDmaBlocks, "");
        DrawValue(34, "Errores ", adcDmaErrors, "");
        DrawValue(45, "Timeouts ", adcDmaTimeouts, "");
        DrawValue(55, "Fallo ", adcDmaLastFailure, "");
        return;
    }
    DrawVolts(11, "V ", panelMv);
    DrawFixed(22, "I ", currentUa / 100U, 1, " mA");
    DrawFixed(33, "P ", powerUw / 100U, 1, " mW");
    DrawFixed(44, "E ", PANEL_EnergyUwh(), 3, " mWh");
    DrawValue(55, "ADC ", panelAdc, " / 4095");
}

/* Pantalla 2: promedio movil, de sesion, minimo y maximo. */
static void Screen_Averages(void) {
    OLED_DrawString(0, 0, "PROMEDIOS DEL PANEL", 1);
    DrawVolts(12, "Movil  ", averageMv);
    DrawVolts(23, "Sesion ", PANEL_SessionAverage());
    DrawVolts(34, "Min    ", minimumMv);
    DrawVolts(45, "Max    ", maximumMv);
    DrawValue(56, "Ventana ", windowSize, " muestras");
}

/* Pantalla 3: las tres cuentas ADC y la LDR mas iluminada. */
static void Screen_Ldr(void) {
    char label[16];

    OLED_DrawString(0, 0, "LECTURAS LDR", 1);
    for (uint8_t i = 0; i < NUM_LDR; i++) {

        char line[32];
        char* p = str_uint(line, i);
        p = str_cat(p, " ");
        p = str_cat(p, ldrName[i]);
        p = str_cat(p, " ");
        p = str_uint(p, ldrValue[i]);
        p = str_cat(p, " ");
        p = str_uint(p, (uint32_t)(LDR_Brightness(ldrValue[i]) * 100 / 4095));
        str_cat(p, "%");
        OLED_DrawString(0, 12 + 10 * i, line, 1);
    }
    if (brightestLdr < NUM_LDR) {
        char* p = str_cat(label, "Mas luz: ");
        str_cat(p, ldrName[brightestLdr]);
        OLED_DrawString(0, 54, label, 1);
    }
}

/* Dibuja el pulso y los grados solicitados de un servo; no es una medida de posicion real. */
static void DrawServoLine(int y, const char* label, SERVO_ID s) {
    char buf[32];
    char* p = str_cat(buf, label);
    p = str_uint(p, servoPulseUs[s]);
    p = str_cat(p, "us  ");
    p = str_int(p, SERVO_GetDegrees(s));
    str_cat(p, "g");
    OLED_DrawString(0, y, buf, 1);
}

/* Dibuja limites PWM de un eje para consultar la calibracion. */
static void DrawLimitLine(int y, const char* label, SERVO_ID s) {
    char buf[32];
    char* p = str_cat(buf, label);
    p = str_uint(p, servoMin[s]);
    p = str_cat(p, "-");
    str_uint(p, servoMax[s]);
    OLED_DrawString(0, y, buf, 1);
}

/* Pantalla 4: posiciones y estado de los ejes. */
static void Screen_Servos(void) {
    static const char* const stageTitle[] = {
        "AUTO: INCLINACION", "AUTO: ROTACION", "AUTO: EQUILIBRADO",
        "AUTO: POCA LUZ", "AUTO: ERROR LDR", "SERVOS: MANUAL", "AUTO: EN LIMITE"
    };
    char buf[32];
    char* p;

    if (manualMode) {
        OLED_DrawString(0, 0, "SERVOS: MANUAL", 1);
    } else {
        OLED_DrawString(0, 0, stageTitle[trackStage <= STAGE_AT_LIMIT ? trackStage : 0], 1);
    }
    DrawServoLine(11, "AZ  ", SERVO_AZ);
    DrawServoLine(21, "INC ", SERVO_INC);
    DrawLimitLine(33, "Lim AZ  ", SERVO_AZ);
    DrawLimitLine(43, "Lim INC ", SERVO_INC);

    if (manualMode) {
        p = str_cat(buf, "Joy X");
        p = str_uint(p, joyValue[0]);
        p = str_cat(p, " Y");
        str_uint(p, joyValue[1]);
    } else {
        p = str_cat(buf, "Err H");
        p = str_int(p, trackErrH);
        p = str_cat(p, " V");
        str_int(p, trackErrV);
    }
    OLED_DrawString(0, 55, buf, 1);
}

/* Pantalla 5: seis filas de tension, separadas por intervalos de adquisicion de 10 s. */
static void Screen_History(void) {
    OLED_DrawString(0, 0, "HISTORICO CADA 10s", 1);
    OLED_DrawString(0, 8, "N   PANEL (V)", 1);
    for (uint32_t row = 0; row < LOG_COUNT; row++) {
        char buf[24];
        char *p = str_uint(buf, row + 1U);
        p = str_cat(p, "   ");
        if (row < logCount) {
            p = str_volts(p, LOG_Value(row));
            str_cat(p, " V");
        } else str_cat(p, "--");
        OLED_DrawString(0, 16 + (int)row * 8, buf, 1);
    }
}

/* Limpia el framebuffer, dibuja la pantalla elegida y la envia por I2C a la OLED. */
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
        case 5:  Screen_History();  break;
        default: Screen_Status();   break;
    }
    oledOk = (uint8_t)OLED_Update();
}

/* ================= 17. ARRANQUE Y LAZO PRINCIPAL ================= */

/* Inicializa el hardware y ejecuta las tareas periodicas. Es el punto de entrada funcional del programa. */
__attribute__((used,noinline,noreturn)) void Tracker_Run(void) {
    uint8_t lastMode = 0xFF;

    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000);  /* Tick de 1 ms */

    RGB_Init();
    GPDMA_Init();
    ADC_Setup();                   /* Habilitar reloj y configurar pines primero. */
    ADCDMA_Init();                 /* Iniciar el primer bloque de un solo canal. */
    DAC_Setup();
    UART_Setup();
    delay_ms(20);
    JOY_Calibrate();
    STORE_Load(); /* Recuperar pulsos en RAM antes de activar PWM. */
    SERVO_Init();
    SW_Init();

    delay_ms(100);                 /* La OLED necesita un momento tras encenderse */
    oledOk = (uint8_t)OLED_Init();

    logLastMs = msTicks;
    uint32_t lastControl   = msTicks;
    uint32_t lastPanel     = msTicks - PANEL_PERIOD_MS;
    uint32_t lastDisplay   = msTicks - DISPLAY_PERIOD_MS;
    uint32_t lastTelemetry = msTicks - TELEMETRY_PERIOD_MS;

    /* Cada resta de tiempos es sin signo: tolera el desborde de msTicks.
     * Las tareas rapidas se atienden en cada vuelta; el resto usa su periodo. */
    while (1) {
        COMM_Process();
        STORE_Process(); /* Persistir cambios del pulsador fuera de la ISR. */
        ADCDMA_Update(); /* Avanzar DMA en cada vuelta, no solo cada 20 ms. */

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
                trackStage  = STAGE_MANUAL;
                trackLastMs = now;
                SERVOS_UpdateFromJoystick();
            } else {
                TRACK_Update();
            }
        }

        if ((now - lastPanel) >= PANEL_PERIOD_MS) {
            lastPanel = now;
            PANEL_Measure();
        }

        LOG_Process();

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

}

/* Entrada del startup: reserva 32 bytes superiores de SRAM para IAP y salta al programa. No simplificar este ensamblador. */
__attribute__((naked)) int main(void) {
    __asm volatile ("sub sp, sp, #32\n" "b Tracker_Run\n");
}
