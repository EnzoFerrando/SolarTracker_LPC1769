/**
 * @file    oled.c
 * @brief   Driver minimo para OLED I2C SSD1306 / SH1106 sobre I2C0 (P0.27 SDA0, P0.28 SCL0).
 *
 * I2C en modo maestro por polling, escrito sobre los registros de LPC_I2C0.
 * Todas las esperas tienen timeout: si la pantalla no esta conectada, el
 * programa no se cuelga, solo OLED_Init()/OLED_Update() devuelven 0.
 */

#include "oled.h"
#include "LPC17xx.h"
#include "lpc17xx_clkpwr.h"

/* ---------------------------- Registros de I2C ---------------------------- */
#define I2C_AA      0x04
#define I2C_SI      0x08
#define I2C_STO     0x10
#define I2C_STA     0x20
#define I2C_EN      0x40

#define I2C_STAT_START      0x08
#define I2C_STAT_REPSTART   0x10
#define I2C_STAT_SLAW_ACK   0x18
#define I2C_STAT_DATA_ACK   0x28

#define I2C_TIMEOUT 100000U

/* Bytes de control del SSD1306 */
#define OLED_CTRL_CMD   0x00
#define OLED_CTRL_DATA  0x40

#define OLED_PAGES (OLED_HEIGHT / 8)

static uint8_t fb[OLED_WIDTH * OLED_PAGES];

/* ------------------------- Fuente 5x7 (ASCII 32-126) ----------------------- */
/* Cada caracter son 5 columnas; bit 0 = fila superior. */
static const uint8_t font5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, /*   */ {0x00,0x00,0x5F,0x00,0x00}, /* ! */
    {0x00,0x07,0x00,0x07,0x00}, /* " */ {0x14,0x7F,0x14,0x7F,0x14}, /* # */
    {0x24,0x2A,0x7F,0x2A,0x12}, /* $ */ {0x23,0x13,0x08,0x64,0x62}, /* % */
    {0x36,0x49,0x55,0x22,0x50}, /* & */ {0x00,0x05,0x03,0x00,0x00}, /* ' */
    {0x00,0x1C,0x22,0x41,0x00}, /* ( */ {0x00,0x41,0x22,0x1C,0x00}, /* ) */
    {0x08,0x2A,0x1C,0x2A,0x08}, /* * */ {0x08,0x08,0x3E,0x08,0x08}, /* + */
    {0x00,0x50,0x30,0x00,0x00}, /* , */ {0x08,0x08,0x08,0x08,0x08}, /* - */
    {0x00,0x60,0x60,0x00,0x00}, /* . */ {0x20,0x10,0x08,0x04,0x02}, /* / */
    {0x3E,0x51,0x49,0x45,0x3E}, /* 0 */ {0x00,0x42,0x7F,0x40,0x00}, /* 1 */
    {0x42,0x61,0x51,0x49,0x46}, /* 2 */ {0x21,0x41,0x45,0x4B,0x31}, /* 3 */
    {0x18,0x14,0x12,0x7F,0x10}, /* 4 */ {0x27,0x45,0x45,0x45,0x39}, /* 5 */
    {0x3C,0x4A,0x49,0x49,0x30}, /* 6 */ {0x01,0x71,0x09,0x05,0x03}, /* 7 */
    {0x36,0x49,0x49,0x49,0x36}, /* 8 */ {0x06,0x49,0x49,0x29,0x1E}, /* 9 */
    {0x00,0x36,0x36,0x00,0x00}, /* : */ {0x00,0x56,0x36,0x00,0x00}, /* ; */
    {0x08,0x14,0x22,0x41,0x00}, /* < */ {0x14,0x14,0x14,0x14,0x14}, /* = */
    {0x00,0x41,0x22,0x14,0x08}, /* > */ {0x02,0x01,0x51,0x09,0x06}, /* ? */
    {0x32,0x49,0x79,0x41,0x3E}, /* @ */ {0x7E,0x11,0x11,0x11,0x7E}, /* A */
    {0x7F,0x49,0x49,0x49,0x36}, /* B */ {0x3E,0x41,0x41,0x41,0x22}, /* C */
    {0x7F,0x41,0x41,0x22,0x1C}, /* D */ {0x7F,0x49,0x49,0x49,0x41}, /* E */
    {0x7F,0x09,0x09,0x01,0x01}, /* F */ {0x3E,0x41,0x41,0x51,0x32}, /* G */
    {0x7F,0x08,0x08,0x08,0x7F}, /* H */ {0x00,0x41,0x7F,0x41,0x00}, /* I */
    {0x20,0x40,0x41,0x3F,0x01}, /* J */ {0x7F,0x08,0x14,0x22,0x41}, /* K */
    {0x7F,0x40,0x40,0x40,0x40}, /* L */ {0x7F,0x02,0x04,0x02,0x7F}, /* M */
    {0x7F,0x04,0x08,0x10,0x7F}, /* N */ {0x3E,0x41,0x41,0x41,0x3E}, /* O */
    {0x7F,0x09,0x09,0x09,0x06}, /* P */ {0x3E,0x41,0x51,0x21,0x5E}, /* Q */
    {0x7F,0x09,0x19,0x29,0x46}, /* R */ {0x46,0x49,0x49,0x49,0x31}, /* S */
    {0x01,0x01,0x7F,0x01,0x01}, /* T */ {0x3F,0x40,0x40,0x40,0x3F}, /* U */
    {0x1F,0x20,0x40,0x20,0x1F}, /* V */ {0x7F,0x20,0x18,0x20,0x7F}, /* W */
    {0x63,0x14,0x08,0x14,0x63}, /* X */ {0x03,0x04,0x78,0x04,0x03}, /* Y */
    {0x61,0x51,0x49,0x45,0x43}, /* Z */ {0x00,0x7F,0x41,0x41,0x00}, /* [ */
    {0x02,0x04,0x08,0x10,0x20}, /* \ */ {0x00,0x41,0x41,0x7F,0x00}, /* ] */
    {0x04,0x02,0x01,0x02,0x04}, /* ^ */ {0x40,0x40,0x40,0x40,0x40}, /* _ */
    {0x00,0x01,0x02,0x04,0x00}, /* ` */ {0x20,0x54,0x54,0x54,0x78}, /* a */
    {0x7F,0x48,0x44,0x44,0x38}, /* b */ {0x38,0x44,0x44,0x44,0x20}, /* c */
    {0x38,0x44,0x44,0x48,0x7F}, /* d */ {0x38,0x54,0x54,0x54,0x18}, /* e */
    {0x08,0x7E,0x09,0x01,0x02}, /* f */ {0x08,0x14,0x54,0x54,0x3C}, /* g */
    {0x7F,0x08,0x04,0x04,0x78}, /* h */ {0x00,0x44,0x7D,0x40,0x00}, /* i */
    {0x20,0x40,0x44,0x3D,0x00}, /* j */ {0x00,0x7F,0x10,0x28,0x44}, /* k */
    {0x00,0x41,0x7F,0x40,0x00}, /* l */ {0x7C,0x04,0x18,0x04,0x78}, /* m */
    {0x7C,0x08,0x04,0x04,0x78}, /* n */ {0x38,0x44,0x44,0x44,0x38}, /* o */
    {0x7C,0x14,0x14,0x14,0x08}, /* p */ {0x08,0x14,0x14,0x18,0x7C}, /* q */
    {0x7C,0x08,0x04,0x04,0x08}, /* r */ {0x48,0x54,0x54,0x54,0x20}, /* s */
    {0x04,0x3F,0x44,0x40,0x20}, /* t */ {0x3C,0x40,0x40,0x20,0x7C}, /* u */
    {0x1C,0x20,0x40,0x20,0x1C}, /* v */ {0x3C,0x40,0x30,0x40,0x3C}, /* w */
    {0x44,0x28,0x10,0x28,0x44}, /* x */ {0x0C,0x50,0x50,0x50,0x3C}, /* y */
    {0x44,0x64,0x54,0x4C,0x44}, /* z */ {0x00,0x08,0x36,0x41,0x00}, /* { */
    {0x00,0x00,0x7F,0x00,0x00}, /* | */ {0x00,0x41,0x36,0x08,0x00}, /* } */
    {0x08,0x04,0x08,0x10,0x08}, /* ~ */
};

/* ---------------------------------- I2C1 ----------------------------------- */
static int i2c_waitSI(void) {
    uint32_t t = I2C_TIMEOUT;
    while (!(LPC_I2C0->I2CONSET & I2C_SI)) {
        if (--t == 0) {
            return 0;
        }
    }
    return 1;
}

static void i2c_stop(void) {
    LPC_I2C0->I2CONSET = I2C_STO;
    LPC_I2C0->I2CONCLR = I2C_SI;

    uint32_t t = I2C_TIMEOUT;
    while ((LPC_I2C0->I2CONSET & I2C_STO) && --t) {
    }
}

static int i2c_start(uint8_t addr7) {
    LPC_I2C0->I2CONSET = I2C_STA;
    if (!i2c_waitSI()) {
        return 0;
    }

    const uint32_t st = LPC_I2C0->I2STAT;
    if (st != I2C_STAT_START && st != I2C_STAT_REPSTART) {
        return 0;
    }

    LPC_I2C0->I2DAT    = (uint32_t)(addr7 << 1);  /* Bit 0 = 0 -> escritura */
    LPC_I2C0->I2CONCLR = I2C_STA | I2C_SI;
    if (!i2c_waitSI()) {
        return 0;
    }
    return LPC_I2C0->I2STAT == I2C_STAT_SLAW_ACK;
}

static int i2c_writeByte(uint8_t b) {
    LPC_I2C0->I2DAT    = b;
    LPC_I2C0->I2CONCLR = I2C_SI;
    if (!i2c_waitSI()) {
        return 0;
    }
    return LPC_I2C0->I2STAT == I2C_STAT_DATA_ACK;
}

/** Una transaccion: START, direccion, byte de control, datos, STOP. */
static int i2c_write(uint8_t addr7, uint8_t ctrl, const uint8_t* data, uint16_t len) {
    int ok = i2c_start(addr7) && i2c_writeByte(ctrl);

    for (uint16_t i = 0; ok && i < len; i++) {
        ok = i2c_writeByte(data[i]);
    }

    i2c_stop();
    return ok;
}

static void i2c_init(void) {
    /*
     * P0.27 = SDA0, P0.28 = SCL0 (funcion 01). Son pines I2C dedicados, ya open-drain,
     * y no tienen PINMODE ni PINMODE_OD (esos bits son reservados). Por eso solo se
     * escribe PINSEL1 directamente y no se usa PINSEL_ConfigPin(), que tocaria esos bits.
     * I2CPADCFG queda en su valor de reset (modo estandar/fast con filtro).
     */
    LPC_PINCON->PINSEL1 = (LPC_PINCON->PINSEL1 & ~(0xFU << 22)) | (0x5U << 22);

    CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCI2C0, ENABLE);
    const uint32_t pclk = CLKPWR_GetPCLK(CLKPWR_PCLKSEL_I2C0);
    const uint32_t half = pclk / (2U * OLED_I2C_HZ);

    LPC_I2C0->I2SCLH   = half;
    LPC_I2C0->I2SCLL   = half;
    LPC_I2C0->I2CONCLR = I2C_AA | I2C_SI | I2C_STA | I2C_EN;
    LPC_I2C0->I2CONSET = I2C_EN;
}

/* --------------------------------- Pantalla -------------------------------- */
static int oled_cmds(const uint8_t* cmds, uint16_t len) {
    return i2c_write(OLED_ADDR, OLED_CTRL_CMD, cmds, len);
}

int OLED_Init(void) {
    static const uint8_t initSeq[] = {
        0xAE,                         /* Display OFF */
        0xD5, 0x80,                   /* Clock */
        0xA8, OLED_HEIGHT - 1,        /* Multiplex */
        0xD3, 0x00,                   /* Offset vertical */
        0x40,                         /* Linea de inicio 0 */
        0x8D, 0x14,                   /* Charge pump ON */
        0xA1,                         /* Espejado de columnas */
        0xC8,                         /* Barrido COM invertido */
        0xDA, (OLED_HEIGHT == 64) ? 0x12 : 0x02, /* Configuracion de pines COM */
        0x81, 0xCF,                   /* Contraste */
        0xD9, 0xF1,                   /* Precarga */
        0xDB, 0x40,                   /* VCOMH */
        0xA4,                         /* Mostrar contenido de la RAM */
        0xA6,                         /* Normal (no invertido) */
        0xAF                          /* Display ON */
    };

    i2c_init();
    if (!oled_cmds(initSeq, sizeof(initSeq))) {
        return 0;
    }

    OLED_Clear();
    return OLED_Update();
}

/** Modo de direccionamiento por paginas: compatible con SSD1306 y SH1106. */
int OLED_Update(void) {
    for (uint8_t page = 0; page < OLED_PAGES; page++) {
        const uint8_t col    = OLED_COL_OFFSET;
        const uint8_t cmds[] = { (uint8_t)(0xB0 | page),
                                 (uint8_t)(0x00 | (col & 0x0F)),
                                 (uint8_t)(0x10 | (col >> 4)) };

        if (!oled_cmds(cmds, sizeof(cmds))) {
            return 0;
        }
        if (!i2c_write(OLED_ADDR, OLED_CTRL_DATA, &fb[page * OLED_WIDTH], OLED_WIDTH)) {
            return 0;
        }
    }
    return 1;
}

/* --------------------------------- Dibujo ---------------------------------- */
void OLED_Clear(void) {
    for (uint16_t i = 0; i < sizeof(fb); i++) {
        fb[i] = 0;
    }
}

void OLED_SetPixel(int x, int y, uint8_t on) {
    if (x < 0 || x >= OLED_WIDTH || y < 0 || y >= OLED_HEIGHT) {
        return;
    }
    uint8_t* b = &fb[(y / 8) * OLED_WIDTH + x];
    if (on) {
        *b |= (uint8_t)(1U << (y % 8));
    } else {
        *b &= (uint8_t)~(1U << (y % 8));
    }
}

void OLED_FillRect(int x0, int y0, int x1, int y1) {
    for (int y = y0; y <= y1; y++) {
        for (int x = x0; x <= x1; x++) {
            OLED_SetPixel(x, y, 1);
        }
    }
}

static int edge(int ax, int ay, int bx, int by, int px, int py) {
    return (bx - ax) * (py - ay) - (by - ay) * (px - ax);
}

void OLED_FillTriangle(int x0, int y0, int x1, int y1, int x2, int y2) {
    int minX = x0, maxX = x0, minY = y0, maxY = y0;
    if (x1 < minX) minX = x1;
    if (x2 < minX) minX = x2;
    if (x1 > maxX) maxX = x1;
    if (x2 > maxX) maxX = x2;
    if (y1 < minY) minY = y1;
    if (y2 < minY) minY = y2;
    if (y1 > maxY) maxY = y1;
    if (y2 > maxY) maxY = y2;

    for (int y = minY; y <= maxY; y++) {
        for (int x = minX; x <= maxX; x++) {
            const int w0 = edge(x1, y1, x2, y2, x, y);
            const int w1 = edge(x2, y2, x0, y0, x, y);
            const int w2 = edge(x0, y0, x1, y1, x, y);
            if ((w0 >= 0 && w1 >= 0 && w2 >= 0) || (w0 <= 0 && w1 <= 0 && w2 <= 0)) {
                OLED_SetPixel(x, y, 1);
            }
        }
    }
}

void OLED_FillCircle(int cx, int cy, int r) {
    for (int y = -r; y <= r; y++) {
        for (int x = -r; x <= r; x++) {
            if (x * x + y * y <= r * r) {
                OLED_SetPixel(cx + x, cy + y, 1);
            }
        }
    }
}

static void drawChar(int x, int y, char c, uint8_t scale) {
    if (c < 32 || c > 126) {
        c = '?';
    }
    const uint8_t* glyph = font5x7[c - 32];

    for (int col = 0; col < 5; col++) {
        for (int row = 0; row < 8; row++) {
            if (glyph[col] & (1U << row)) {
                OLED_FillRect(x + col * scale, y + row * scale,
                              x + col * scale + scale - 1, y + row * scale + scale - 1);
            }
        }
    }
}

void OLED_DrawString(int x, int y, const char* str, uint8_t scale) {
    while (*str) {
        drawChar(x, y, *str++, scale);
        x += 6 * scale;
    }
}

int OLED_StringWidth(const char* str, uint8_t scale) {
    int n = 0;
    while (str[n]) {
        n++;
    }
    return n > 0 ? n * 6 * scale - scale : 0;  /* Sin el espacio final */
}
