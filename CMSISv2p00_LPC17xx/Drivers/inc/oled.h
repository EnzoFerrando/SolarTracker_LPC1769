/**
 * @file    oled.h
 * @brief   Driver minimo para pantalla OLED I2C SSD1306 / SH1106 (128x64) sobre I2C0
 *          del LPC1769 (P0.27 = SDA0, P0.28 = SCL0; pull-ups de 2k2 ya en la placa).
 *
 * Funciona con un framebuffer de 1 KB en RAM: se dibuja con las funciones OLED_*
 * y luego se envia todo a la pantalla con OLED_Update().
 */

#ifndef OLED_H_
#define OLED_H_

#include <stdint.h>

/* ------------------------------ Configuracion ------------------------------ */
#define OLED_ADDR        0x3C     /* Direccion I2C de 7 bits (algunos modulos usan 0x3D) */
#define OLED_WIDTH       128
#define OLED_HEIGHT      64       /* 64 o 32 segun el modulo */
#define OLED_COL_OFFSET  0        /* 0 para SSD1306 (0,96"), 2 para SH1106 (1,3") */
#define OLED_I2C_HZ      400000U  /* Bajar a 100000 si hay errores de comunicacion */

/* --------------------------------- Funciones ------------------------------- */
/** Inicializa I2C1 y la pantalla. Devuelve 1 si la pantalla respondio, 0 si no. */
int OLED_Init(void);

/** Envia el framebuffer a la pantalla. Devuelve 1 si salio bien. */
int OLED_Update(void);

void OLED_Clear(void);
void OLED_SetPixel(int x, int y, uint8_t on);
void OLED_FillRect(int x0, int y0, int x1, int y1);
void OLED_FillTriangle(int x0, int y0, int x1, int y1, int x2, int y2);
void OLED_FillCircle(int cx, int cy, int r);

/** Dibuja texto ASCII con fuente 5x7. scale = 1, 2, 3... (cada caracter ocupa 6*scale px). */
void OLED_DrawString(int x, int y, const char* str, uint8_t scale);

/** Ancho en pixeles que ocupa un texto con la escala dada. */
int OLED_StringWidth(const char* str, uint8_t scale);

#endif /* OLED_H_ */
