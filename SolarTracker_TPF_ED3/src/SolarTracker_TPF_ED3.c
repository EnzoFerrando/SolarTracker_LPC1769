#include "LPC17xx.h"
#include "lpc17xx_pwm.h"
#include "lpc17xx_systick.h"

/* Cable de senal del servo en P2.0 (PWM1.1).
 * Rojo a fuente de 5 V; marron/negro a GND comun con la LPC.
 * Probar inicialmente sin carga mecanica.
 */

/* Agregar a Expressions para comprobar que el programa avanza. */
volatile uint32_t servoPulseUs = 1500;
volatile uint32_t servoCycles = 0;

static void delay_ms(uint32_t milliseconds)
{
    SYSTICK_ClearCounterFlag();
    while (milliseconds > 0) {
        if (SYSTICK_HasFired() == SET) {
            milliseconds--;
        }
    }
}

static void servo_set_pulse(uint32_t pulseUs)
{
    PWM_MatchUpdateSingle(PWM_MATCH_1, pulseUs);
    servoPulseUs = pulseUs;
}

static void servo_init(void)
{
    PWM_TIMERCFG_T timerCfg = {
        .prescaleOpt = PWM_US,
        .prescaleValue = 1
    };
    PWM_InitTimer(&timerCfg);
    PWM_PinConfig(PWM1_P2_0);
    PWM_ChannelConfig(PWM_CHANNEL_1, PWM_SINGLE_EDGE);

    PWM_MATCHCFG_T periodCfg = {
        .channel = PWM_MATCH_0,
        .intEn = DISABLE,
        .stopEn = DISABLE,
        .resetEn = ENABLE,
        .matchValue = 20000
    };
    PWM_ConfigMatch(&periodCfg);

    PWM_MATCHCFG_T pulseCfg = {
        .channel = PWM_MATCH_1,
        .intEn = DISABLE,
        .stopEn = DISABLE,
        .resetEn = DISABLE,
        .matchValue = 1500
    };
    PWM_ConfigMatch(&pulseCfg);

    PWM_ResetCounter();
    PWM_ChannelEnable(PWM_CHANNEL_1);
    PWM_Enable();
    PWM_CounterEnable();
}

int main(void)
{
    SystemCoreClockUpdate();

    SYSTICK_InternalInit(1);
    SYSTICK_IntCmd(DISABLE);
    SYSTICK_Cmd(ENABLE);

    servo_init();
    delay_ms(1000);

    while (1) {
        servo_set_pulse(1200);
        delay_ms(1000);
        servo_set_pulse(1500);
        delay_ms(1000);
        servo_set_pulse(1800);
        delay_ms(1000);
        servo_set_pulse(1500);
        delay_ms(1000);
        servoCycles++;
    }
}
