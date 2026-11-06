/**
 * @file    SolarTracker_TPF_ED3.c
 * @brief   Seguidor solar: dos servos (azimut e inclinacion), joystick, LDR y OLED.
 *          Dos modos, que se alternan con el pulsador del joystick:
 *          - AUTOMATICO: lee las LDR y muestra cual recibe mas luz (los servos quedan quietos).
 *          - MANUAL:     el joystick mueve los dos servos a la vez:
 *                          izquierda/derecha -> servo de AZIMUT (rota todo el panel)
 *                          arriba/abajo      -> servo de INCLINACION del panel
 *
 * Conexiones:
 *   LDR0 -> P0.23 (AD0.0)   LDR1 -> P0.24 (AD0.1)
 *   LDR2 -> P0.25 (AD0.2)   LDR3 -> P0.26 (AD0.3)
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
 * Divisor de cada LDR: 3V3 --[10k]--+-- ADC --[LDR]-- GND
 *   Mas luz -> menor R_LDR -> menor tension -> MENOR cuenta del ADC.
 */

#include "LPC17xx.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_clkpwr.h"
#include "oled.h"

/* ------------------------------ Configuracion ------------------------------ */
#define NUM_LDR         4
#define ADC_RATE_HZ     100000U   /* Frecuencia de conversion del ADC (max 200 kHz) */
#define AVG_SAMPLES     16        /* Muestras promediadas por canal */
#define LDR_HYSTERESIS  40        /* Cuentas (~32 mV) para evitar saltos entre LDR */
#define LOOP_PERIOD_MS  20        /* Periodo del lazo principal */
#define NO_LDR          0xFF

/* Joystick */
#define JOY_THRESHOLD   600       /* Cuentas desde el centro para mostrar una direccion */
#define JOY_INVERT_X    0         /* Poner en 1 si izquierda/derecha quedan al reves */
#define JOY_INVERT_Y    0         /* Poner en 1 si arriba/abajo quedan al reves */

/* Servos (PWM1, periodo comun de 20 ms) */
#define SERVO_PERIOD_US 20000U    /* 50 Hz */
#define SERVO_CENTER_US 1500U
#define SERVO_MAX_STEP  20        /* us por ciclo de 20 ms con el joystick al tope */
#define SERVO_DEADZONE  200       /* Cuentas alrededor del centro sin movimiento */

/* Limites de cada servo: ajustarlos a lo que permita la mecanica */
#define SERVO_AZ_MIN_US   500U    /* ~0 grados   */
#define SERVO_AZ_MAX_US   2400U   /* ~180 grados */
#define SERVO_INC_MIN_US  500U
#define SERVO_INC_MAX_US  2400U

#define SERVO_AZ_INVERT   0       /* Poner en 1 para invertir el sentido del azimut */
#define SERVO_INC_INVERT  0       /* Poner en 1 para invertir el sentido de la inclinacion */

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

/* ------------------------------- Tipos ------------------------------------- */
typedef enum {
    SERVO_AZ = 0,   /* Azimut:      P2.4 / PWM1.5 */
    SERVO_INC,      /* Inclinacion: P2.5 / PWM1.6 */
    NUM_SERVOS
} SERVO_ID;

typedef enum {
    DIR_CENTER = 0,
    DIR_UP,
    DIR_DOWN,
    DIR_LEFT,
    DIR_RIGHT
} JOY_DIR;

/* ------------------------------- Variables -------------------------------- */
static const ADC_CHANNEL ldrChannel[NUM_LDR] = {
    ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3
};
static const ADC_CHANNEL joyChannel[2] = { ADC_CHANNEL_4, ADC_CHANNEL_5 };

static const uint8_t  servoPwmCh[NUM_SERVOS] = { 5, 6 };
static const uint32_t servoMin[NUM_SERVOS]   = { SERVO_AZ_MIN_US, SERVO_INC_MIN_US };
static const uint32_t servoMax[NUM_SERVOS]   = { SERVO_AZ_MAX_US, SERVO_INC_MAX_US };

/* Globales y volatile para poder verlas en el debugger */
volatile uint16_t ldrValue[NUM_LDR];
volatile uint16_t joyValue[2];       /* [0] = X, [1] = Y */
volatile uint16_t joyCenter[2];
volatile uint8_t  manualMode = 0;    /* 0 = automatico, 1 = manual */
volatile uint32_t servoPulseUs[NUM_SERVOS] = { SERVO_CENTER_US, SERVO_CENTER_US };
volatile uint8_t  oledOk = 0;        /* 1 si la pantalla respondio por I2C */

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

static void JOY_Read(void) {
    ADC_ReadAverage(joyChannel, 2, joyValue);
}

/** Desviacion de cada eje respecto del centro: dx > 0 = derecha, dy > 0 = abajo. */
static void JOY_GetDeltas(int32_t* dx, int32_t* dy) {
    *dx = (int32_t)joyValue[0] - (int32_t)joyCenter[0];
    *dy = (int32_t)joyValue[1] - (int32_t)joyCenter[1];
#if JOY_INVERT_X
    *dx = -*dx;
#endif
#if JOY_INVERT_Y
    *dy = -*dy;
#endif
}

/**
 * Mueve los dos servos a la vez segun el joystick:
 *   derecha -> azimut aumenta el pulso, izquierda -> lo disminuye
 *   arriba  -> inclinacion aumenta el pulso, abajo -> lo disminuye
 * En diagonal se mueven los dos al mismo tiempo.
 */
static void SERVOS_UpdateFromJoystick(void) {
    int32_t dx, dy;
    JOY_GetDeltas(&dx, &dy);

    int32_t az  = dx;
    int32_t inc = -dy;  /* Arriba es dy < 0 */
#if SERVO_AZ_INVERT
    az = -az;
#endif
#if SERVO_INC_INVERT
    inc = -inc;
#endif

    SERVO_Move(SERVO_AZ, az);
    SERVO_Move(SERVO_INC, inc);
}

/** Direccion dominante del joystick, para mostrar en la pantalla. */
static JOY_DIR JOY_GetDirection(void) {
    int32_t dx, dy;
    JOY_GetDeltas(&dx, &dy);

    const int32_t ax = dx < 0 ? -dx : dx;
    const int32_t ay = dy < 0 ? -dy : dy;

    if (ax < JOY_THRESHOLD && ay < JOY_THRESHOLD) {
        return DIR_CENTER;
    }
    if (ay >= ax) {
        return dy < 0 ? DIR_UP : DIR_DOWN;
    }
    return dx < 0 ? DIR_LEFT : DIR_RIGHT;
}

/* --------------------------------- Pantalla -------------------------------- */
static void DrawCentered(int y, const char* str, uint8_t scale) {
    OLED_DrawString((OLED_WIDTH - OLED_StringWidth(str, scale)) / 2, y, str, scale);
}

/** Flecha en la zona central (filas 12..46). */
static void DrawArrow(JOY_DIR dir) {
    switch (dir) {
        case DIR_UP:
            OLED_FillTriangle(64, 12, 48, 28, 80, 28);
            OLED_FillRect(59, 28, 69, 45);
            break;
        case DIR_DOWN:
            OLED_FillRect(59, 13, 69, 30);
            OLED_FillTriangle(48, 30, 80, 30, 64, 46);
            break;
        case DIR_LEFT:
            OLED_FillTriangle(42, 29, 60, 13, 60, 45);
            OLED_FillRect(60, 24, 86, 34);
            break;
        case DIR_RIGHT:
            OLED_FillRect(42, 24, 68, 34);
            OLED_FillTriangle(68, 13, 68, 45, 86, 29);
            break;
        default:
            OLED_FillCircle(64, 29, 7);
            break;
    }
}

static void Display_Draw(uint8_t manual, JOY_DIR dir, uint8_t ldr) {
    static const char* dirName[] = { "CENTRO", "ARRIBA", "ABAJO", "IZQUIERDA", "DERECHA" };

    if (!oledOk) {
        return;
    }

    OLED_Clear();
    DrawCentered(0, manual ? "MODO MANUAL" : "MODO AUTOMATICO", 1);

    if (manual) {
        DrawArrow(dir);
        DrawCentered(48, dirName[dir], 2);
    } else {
        char txt[] = "LDR ?";
        if (ldr < NUM_LDR) {
            txt[4] = (char)('0' + ldr);
        }
        DrawCentered(24, txt, 3);
    }

    oledOk = (uint8_t)OLED_Update();
}

/* ---------------------------------- Main ---------------------------------- */
int main(void) {
    uint8_t brightest = NO_LDR;
    uint8_t lastMode  = 0xFF;

    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000);  /* Tick de 1 ms */

    RGB_Init();
    ADC_Setup();
    delay_ms(10);
    JOY_Calibrate();
    SERVO_Init();
    SW_Init();

    delay_ms(100);                 /* La OLED necesita un momento tras encenderse */
    oledOk = (uint8_t)OLED_Init();

    JOY_DIR dir     = DIR_CENTER;
    JOY_DIR lastDir = DIR_CENTER;
    uint8_t lastShownMode = 0xFF;
    uint8_t lastShownLdr  = NO_LDR;

    while (1) {
        const uint8_t mode = manualMode;

        if (mode != lastMode) {
            lastMode = mode;
            RGB_ShowMode(mode);
        }

        if (mode) {
            JOY_Read();
            dir = JOY_GetDirection();
            SERVOS_UpdateFromJoystick();
        } else {
            brightest = LDR_GetBrightest(brightest);
        }

        /* Redibujar solo si cambio algo: enviar la pantalla tarda ~25 ms */
        if (mode != lastShownMode || (mode && dir != lastDir) ||
            (!mode && brightest != lastShownLdr)) {
            lastShownMode = mode;
            lastDir       = dir;
            lastShownLdr  = brightest;
            Display_Draw(mode, dir, brightest);
        }

        delay_ms(LOOP_PERIOD_MS);
    }

    return 0;
}
