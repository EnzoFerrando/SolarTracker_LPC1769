/**
 * @file    SolarTracker_TPF_ED3.c
 * @brief   Prueba LDR + joystick con dos modos, que se alternan con el pulsador del joystick:
 *          - AUTOMATICO: enciende el LED de la LDR que recibe mas luz.
 *          - MANUAL:     enciende el LED del cuadrante hacia donde apunta el joystick.
 *
 * Conexiones:
 *   LDR0 -> P0.23 (AD0.0)    LED0 -> P0.0   (arriba-izquierda)
 *   LDR1 -> P0.24 (AD0.1)    LED1 -> P0.1   (arriba-derecha)
 *   LDR2 -> P0.25 (AD0.2)    LED2 -> P0.2   (abajo-izquierda)
 *   LDR3 -> P0.26 (AD0.3)    LED3 -> P0.3   (abajo-derecha)
 *
 *   Joystick VRx -> P1.30 (AD0.4)
 *   Joystick VRy -> P1.31 (AD0.5)
 *   Joystick SW  -> P0.21 (GPIO con interrupcion, pull-up interno, activo en bajo)
 *   Joystick +5V -> 3V3 de la placa (NO a 5V: el ADC admite como maximo 3,3 V)
 *
 *   Servo SG90 senal (naranja) -> P2.4 (PWM1.5)
 *   Servo SG90 +5V (rojo)      -> fuente externa de 5 V
 *   Servo SG90 GND (marron)    -> GND de la fuente, unido al GND de la placa
 *   En modo MANUAL, joystick arriba/abajo mueve el servo; en AUTOMATICO queda quieto.
 *
 * Indicador de modo (LED RGB de la placa, activo en bajo):
 *   Verde (P3.25) = automatico, Azul (P3.26) = manual.
 *
 * Divisor de cada LDR: 3V3 --[10k]--+-- ADC --[LDR]-- GND
 *   Mas luz -> menor R_LDR -> menor tension -> MENOR cuenta del ADC.
 */

#include "LPC17xx.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_clkpwr.h"

/* ------------------------------ Configuracion ------------------------------ */
#define NUM_LDR         4
#define ADC_RATE_HZ     100000U   /* Frecuencia de conversion del ADC (max 200 kHz) */
#define AVG_SAMPLES     16        /* Muestras promediadas por canal */
#define LDR_HYSTERESIS  40        /* Cuentas (~32 mV) para evitar parpadeo entre LEDs */
#define LOOP_PERIOD_MS  20        /* Periodo del lazo principal */

/* LEDs de las LDR */
#define LED_PORT        PORT_0
#define LED_MASK        ((1U << 0) | (1U << 1) | (1U << 2) | (1U << 3))
#define NO_LED          0xFF

/* Joystick */
#define JOY_THRESHOLD   600       /* Cuentas desde el centro para cambiar de lado */
#define JOY_INVERT_X    0         /* Poner en 1 si izquierda/derecha quedan al reves */
#define JOY_INVERT_Y    0         /* Poner en 1 si arriba/abajo quedan al reves */

/* Servo (PWM1.5 en P2.4) */
#define SERVO_PERIOD_US 20000U    /* 50 Hz */
#define SERVO_MIN_US    1000U     /* Extremo 1 (ampliar a ~500 si el servo lo permite) */
#define SERVO_MAX_US    2000U     /* Extremo 2 (ampliar a ~2400 si el servo lo permite) */
#define SERVO_CENTER_US 1500U
#define SERVO_MAX_STEP  20        /* us por ciclo de 20 ms con el joystick al tope */
#define SERVO_DEADZONE  200       /* Cuentas alrededor del centro sin movimiento */
#define SERVO_INVERT    0         /* Poner en 1 para invertir el sentido */

/* Pulsador */
#define SW_PORT         PORT_0
#define SW_PIN          PIN_21
#define SW_MASK         (1U << 21)
#define DEBOUNCE_MS     80

/* LED RGB de la placa (activo en bajo) */
#define RGB_RED_PORT    PORT_0
#define RGB_RED_MASK    (1U << 22)
#define RGB_GB_PORT     PORT_3
#define RGB_GREEN_MASK  (1U << 25)
#define RGB_BLUE_MASK   (1U << 26)

/* ------------------------------- Variables -------------------------------- */
static const ADC_CHANNEL ldrChannel[NUM_LDR] = {
    ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3
};
static const ADC_CHANNEL joyChannel[2] = { ADC_CHANNEL_4, ADC_CHANNEL_5 };
static const LPC_PIN ledPin[NUM_LDR]   = { PIN_0, PIN_1, PIN_2, PIN_3 };

/* Globales y volatile para poder verlas en el debugger */
volatile uint16_t ldrValue[NUM_LDR];
volatile uint16_t joyValue[2];       /* [0] = X, [1] = Y */
volatile uint16_t joyCenter[2];
volatile uint8_t  manualMode = 0;    /* 0 = automatico, 1 = manual */
volatile uint32_t servoPulseUs = SERVO_CENTER_US;

static volatile uint32_t msTicks    = 0;
static volatile uint32_t lastEdgeMs = 0;

/* ------------------------------ Interrupciones ----------------------------- */
void SysTick_Handler(void) {
    msTicks++;
}

/**
 * Las interrupciones GPIO de los puertos 0 y 2 comparten el vector de EINT3.
 * Antirrebote: se registra cada flanco (subida o bajada) y solo se cambia de
 * modo en un flanco de bajada si no hubo ningun flanco en los ultimos
 * DEBOUNCE_MS. Asi se ignoran los rebotes al presionar y tambien al soltar.
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

/* -------------------------------- Utilidades ------------------------------- */
static void delay_ms(uint32_t ms) {
    const uint32_t start = msTicks;
    while ((msTicks - start) < ms) {
    }
}

/* ---------------------------------- GPIO ----------------------------------- */
/** P0.0..P0.3 como salidas, con los LEDs apagados. */
static void LED_Init(void) {
    PINSEL_CFG_T cfg = {
        .port      = LED_PORT,
        .pin       = PIN_0,           /* Ignorado por ConfigMultiplePins */
        .func      = PINSEL_FUNC_00,  /* GPIO */
        .mode      = PINSEL_TRISTATE,
        .openDrain = DISABLE
    };
    PINSEL_ConfigMultiplePins(&cfg, LED_MASK);

    GPIO_SetDir(LED_PORT, LED_MASK, GPIO_OUTPUT);
    GPIO_ClearPins(LED_PORT, LED_MASK);
}

/** Enciende solo el LED indicado. */
static void LED_Show(uint8_t idx) {
    GPIO_ClearPins(LED_PORT, LED_MASK);
    GPIO_SetPins(LED_PORT, 1U << ledPin[idx]);
}

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

/* ---------------------------------- Servo ---------------------------------- */
/** Fija el ancho de pulso; se aplica al comienzo del proximo periodo. */
static void SERVO_SetPulse(uint32_t us) {
    if (us < SERVO_MIN_US) {
        us = SERVO_MIN_US;
    } else if (us > SERVO_MAX_US) {
        us = SERVO_MAX_US;
    }
    servoPulseUs  = us;
    LPC_PWM1->MR5 = us;
    LPC_PWM1->LER = (1U << 5);  /* Latch de MR5 */
}

/**
 * PWM1 con el contador a 1 MHz (1 tick = 1 us):
 * MR0 = periodo de 20 ms, MR5 = ancho de pulso. Salida simple flanco en PWM1.5.
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

    CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCPWM1, ENABLE);
    const uint32_t pclk = CLKPWR_GetPCLK(CLKPWR_PCLKSEL_PWM1);

    LPC_PWM1->TCR = (1U << 1);              /* Reset del contador */
    LPC_PWM1->PR  = pclk / 1000000U - 1U;   /* Tick de 1 us */
    LPC_PWM1->MR0 = SERVO_PERIOD_US;
    LPC_PWM1->MR5 = SERVO_CENTER_US;
    LPC_PWM1->MCR = (1U << 1);              /* Reinicia el contador en MR0 */
    LPC_PWM1->PCR = (1U << 13);             /* Habilita salida PWM1.5 (simple flanco) */
    LPC_PWM1->LER = (1U << 0) | (1U << 5);
    LPC_PWM1->TCR = (1U << 0) | (1U << 3);  /* Contador + modo PWM habilitados */

    servoPulseUs = SERVO_CENTER_US;
}

/**
 * Control por velocidad: cuanto mas se inclina el joystick en Y, mas rapido se
 * mueve el servo. Al soltarlo, el servo se queda en la ultima posicion.
 */
static void SERVO_UpdateFromJoystick(void) {
    int32_t dy = (int32_t)joyValue[1] - (int32_t)joyCenter[1];
#if JOY_INVERT_Y
    dy = -dy;
#endif
#if SERVO_INVERT
    dy = -dy;
#endif

    int32_t mag = dy < 0 ? -dy : dy;
    if (mag <= SERVO_DEADZONE) {
        return;
    }

    int32_t step = (mag - SERVO_DEADZONE) * SERVO_MAX_STEP / (2048 - SERVO_DEADZONE);
    if (step < 1) {
        step = 1;
    }

    /* Joystick arriba (dy < 0) aumenta el pulso; abajo lo disminuye */
    int32_t pulse = (int32_t)servoPulseUs + (dy < 0 ? step : -step);
    if (pulse < 0) {
        pulse = 0;
    }
    SERVO_SetPulse((uint32_t)pulse);
}

/* ----------------------------------- ADC ----------------------------------- */
/** AD0.0..AD0.5 configurados y ADC en modo burst. */
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

    ADC_BurstEnable();
}

/**
 * Promedia AVG_SAMPLES conversiones nuevas de cada canal de la lista.
 * Leer ADDRx limpia su flag DONE, asi que cada espera garantiza una conversion nueva.
 */
static void ADC_ReadAverage(const ADC_CHANNEL ch[], uint8_t n, volatile uint16_t out[]) {
    uint32_t acc[6] = {0};

    for (uint8_t s = 0; s < AVG_SAMPLES; s++) {
        for (uint8_t i = 0; i < n; i++) {
            while (ADC_ChannelGetStatus(ch[i], ADC_DATA_DONE) == RESET) {
            }
            acc[i] += ADC_ChannelGetData(ch[i]);
        }
    }

    for (uint8_t i = 0; i < n; i++) {
        out[i] = (uint16_t)(acc[i] / AVG_SAMPLES);
    }
}

/* ------------------------------- Modo automatico --------------------------- */
/**
 * Indice de la LDR mas iluminada (valor ADC minimo).
 * Con histeresis: solo cambia si la nueva supera a la actual por LDR_HYSTERESIS cuentas.
 */
static uint8_t LDR_GetBrightest(uint8_t current) {
    ADC_ReadAverage(ldrChannel, NUM_LDR, ldrValue);

    uint8_t min = 0;
    for (uint8_t i = 1; i < NUM_LDR; i++) {
        if (ldrValue[i] < ldrValue[min]) {
            min = i;
        }
    }

    if (current < NUM_LDR && min != current &&
        (ldrValue[current] - ldrValue[min]) < LDR_HYSTERESIS) {
        return current;
    }
    return min;
}

/* -------------------------------- Modo manual ------------------------------ */
/** Toma la posicion de reposo del joystick como centro. No tocarlo al arrancar. */
static void JOY_Calibrate(void) {
    ADC_ReadAverage(joyChannel, 2, joyCenter);
}

/**
 * LED segun el cuadrante del joystick: LED = 2*filaY + columnaX
 *   0 = arriba-izq, 1 = arriba-der, 2 = abajo-izq, 3 = abajo-der.
 * Cada eje solo cambia de lado si se aleja mas de JOY_THRESHOLD del centro.
 * Si no, conserva el lado actual. Asi, al empujar solo hacia arriba se
 * mantiene la columna, y en reposo queda encendido el ultimo LED elegido.
 */
static void JOY_Read(void) {
    ADC_ReadAverage(joyChannel, 2, joyValue);
}

/** Usa la ultima lectura de JOY_Read(). */
static uint8_t JOY_GetLed(uint8_t current) {
    int32_t dx = (int32_t)joyValue[0] - (int32_t)joyCenter[0];
    int32_t dy = (int32_t)joyValue[1] - (int32_t)joyCenter[1];
#if JOY_INVERT_X
    dx = -dx;
#endif
#if JOY_INVERT_Y
    dy = -dy;
#endif

    if (current >= NUM_LDR) {
        current = 0;
    }
    uint8_t col = current % 2;  /* 0 = izquierda, 1 = derecha */
    uint8_t row = current / 2;  /* 0 = arriba,    1 = abajo   */

    if (dx < -JOY_THRESHOLD) {
        col = 0;
    } else if (dx > JOY_THRESHOLD) {
        col = 1;
    }

    if (dy < -JOY_THRESHOLD) {
        row = 0;
    } else if (dy > JOY_THRESHOLD) {
        row = 1;
    }

    return (uint8_t)(row * 2 + col);
}

/* ---------------------------------- Main ---------------------------------- */
int main(void) {
    uint8_t current  = NO_LED;
    uint8_t lastMode = 0xFF;

    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000);  /* Tick de 1 ms */

    LED_Init();
    RGB_Init();
    ADC_Setup();
    delay_ms(10);
    JOY_Calibrate();
    SERVO_Init();
    SW_Init();

    while (1) {
        const uint8_t mode = manualMode;

        if (mode != lastMode) {
            lastMode = mode;
            RGB_ShowMode(mode);
        }

        uint8_t next;
        if (mode) {
            JOY_Read();
            next = JOY_GetLed(current);
            SERVO_UpdateFromJoystick();
        } else {
            next = LDR_GetBrightest(current);
        }

        if (next != current) {
            current = next;
            LED_Show(current);
        }

        delay_ms(LOOP_PERIOD_MS);
    }

    return 0;
}
