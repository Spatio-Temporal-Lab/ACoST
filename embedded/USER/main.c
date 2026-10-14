/**
 * main.c — STM32F103C8T6 Trajectory Compression Benchmark
 *
 * Two modes (compile-time switch):
 *
 *   MODE_DATASET (default):
 *     Mac sends dataset points via USART1 (PA9/PA10, 115200 baud, USB-TTL).
 *     No GPS/OLED needed. Best for reproducible experiments.
 *
 *   MODE_GPS:
 *     Real-time GPS via USART2 (PA2/PA3, 9600 baud, NEO-M8N/NEO-6M).
 *     Results displayed on OLED + dumped via USART1.
 *
 * To select mode, define one of these before including this file,
 * or in Makefile: -DMODE_GPS   (default is MODE_DATASET)
 *
 * Hardware:
 *   - STM32F103C8T6 minimum system board
 *   - ST-Link V2 (SWD: SWDIO=PA13, SWCLK=PA14)
 *   - USB-TTL → PA9(TX) / PA10(RX)  [both modes]
 *   - NEO-M8N → PA2(TX) / PA3(RX)   [MODE_GPS only]
 *   - SSD1306 OLED → PB10/PB11 IIC  [MODE_GPS only]
 *
 * Serial protocol (MODE_DATASET, Mac → STM32):
 *   GPS point: 24 bytes binary LE [lon f64][lat f64][ts_ms u64]
 *   Commands:  0xFF = dump report, 0xFE = reset
 *
 *   STM32 → Mac: ASCII text (ACK + CSV report)
 */

/* ── Mode selection ─────────────────────────────────────────────────── */
#if !defined(MODE_GPS) && !defined(MODE_DATASET)
#define MODE_DATASET   /* default: Mac feeds dataset via serial */
#endif

#include "delay.h"
#include "sys.h"
#include "acost_benchmark.h"

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"

#ifdef MODE_GPS
#include "oled.h"
#include "gps.h"
extern void GpsDataRead(void);
#endif

/* ══════════════════════════════════════════════════════════════════════
 *  USART1 helpers (both modes use USART1 for benchmark output)
 * ══════════════════════════════════════════════════════════════════════ */
static void uart_putc(uint8_t c) {
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
    USART_SendData(USART1, (uint16_t)c);
}
static void uart_puts(const char *s) {
    while (*s) uart_putc((uint8_t)*s++);
}

#ifdef MODE_DATASET
/* ── UART1 RX Ring Buffer (interrupt-driven) ──────────────────────────── */
#define RX_BUF_SIZE 512  /* must be power of 2 */
static volatile uint8_t rx_ring[RX_BUF_SIZE];
static volatile uint16_t rx_head = 0;  /* ISR writes here */
static volatile uint16_t rx_tail = 0;  /* main reads here */

void USART1_IRQHandler(void) {
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        uint8_t ch = (uint8_t)USART_ReceiveData(USART1);
        uint16_t next = (rx_head + 1) & (RX_BUF_SIZE - 1);
        if (next != rx_tail) {  /* not full */
            rx_ring[rx_head] = ch;
            rx_head = next;
        }
        /* if full, byte is dropped (overflow) */
    }
    if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET) {
        USART_ReceiveData(USART1);  /* clear ORE */
    }
}

static inline uint16_t rx_available(void) {
    return (rx_head - rx_tail) & (RX_BUF_SIZE - 1);
}

static uint8_t uart_getc(void) {
    while (rx_head == rx_tail) {}  /* wait for data */
    uint8_t c = rx_ring[rx_tail];
    rx_tail = (rx_tail + 1) & (RX_BUF_SIZE - 1);
    return c;
}
static void uart_read(uint8_t *buf, int n) {
    for (int i = 0; i < n; ++i) buf[i] = uart_getc();
}

static void init_usart1_irq(void) {
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}
#endif

/* ══════════════════════════════════════════════════════════════════════
 *  Hardware init
 * ══════════════════════════════════════════════════════════════════════ */

static void init_usart1(void) {
    RCC_APB2PeriphClockCmd(
        RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO,
        ENABLE);

    GPIO_InitTypeDef gpio;
    /* PA9 = TX */
    gpio.GPIO_Pin   = GPIO_Pin_9;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);
    /* PA10 = RX */
    gpio.GPIO_Pin  = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    USART_InitTypeDef usart;
    USART_StructInit(&usart);
    usart.USART_BaudRate = 115200;
    usart.USART_Mode     = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &usart);
    USART_Cmd(USART1, ENABLE);

    if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET)
        USART_ReceiveData(USART1);
}

#ifdef MODE_GPS
static void init_usart2(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    GPIO_InitTypeDef gpio;
    /* PA2 = TX */
    gpio.GPIO_Pin   = GPIO_Pin_2;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);
    /* PA3 = RX */
    gpio.GPIO_Pin  = GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    USART_InitTypeDef usart;
    USART_StructInit(&usart);
    usart.USART_BaudRate = 9600;    /* NEO-M8N default */
    usart.USART_Mode     = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART2, &usart);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
    USART_Cmd(USART2, ENABLE);
}

static void init_nvic(void) {
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    NVIC_InitTypeDef nvic;
    /* USART2 (GPS): highest priority */
    nvic.NVIC_IRQChannel = USART2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    /* USART1 (benchmark/dump): lower priority */
    nvic.NVIC_IRQChannel = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&nvic);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}
#endif /* MODE_GPS */

/* ══════════════════════════════════════════════════════════════════════
 *  MODE_DATASET: binary packet from Mac
 * ══════════════════════════════════════════════════════════════════════ */
#ifdef MODE_DATASET

#define PKT_SIZE 24

typedef struct {
    double   lon;
    double   lat;
    uint64_t ts_ms;
} __attribute__((packed)) GpsPacket;

int main(void)
{
    delay_init();
    init_usart1();
    init_usart1_irq();  /* enable RX interrupt + ring buffer */
    AcostBench_Init(1e-5);

    uart_puts("\r\n=== STM32 Benchmark [DATASET MODE] ===\r\n");
    uart_puts("Protocol: [0xAA][0x55][type][payload]\r\n");
    uart_puts("  type=0x01: GPS point (24B payload)\r\n");
    uart_puts("  type=0x02: dump report\r\n");
    uart_puts("  type=0x03: reset\r\n\r\n");

    uint32_t count = 0;
    uint8_t buf[PKT_SIZE];

    while (1) {
        /* Wait for sync header: 0xAA 0x55 */
        uint8_t b = uart_getc();
        if (b != 0xAA) continue;
        b = uart_getc();
        if (b != 0x55) continue;

        /* Read command type */
        uint8_t type = uart_getc();

        if (type == 0x02) {
            /* Dump report */
            AcostBench_Report();
            uart_putc('!');  /* end-of-report marker */
            continue;
        }
        if (type == 0x03) {
            /* Reset */
            AcostBench_Init(1e-5);
            count = 0;
            uart_puts("RESET OK\r\n");
            continue;
        }
        if (type == 0x01) {
            /* GPS data point: read 24 bytes */
            uart_read(buf, PKT_SIZE);
            GpsPacket *pkt = (GpsPacket *)buf;
            AcostBench_Feed(pkt->lon, pkt->lat, pkt->ts_ms);
            ++count;
            /* Per-point ACK for flow control */
            uart_putc('.');
            continue;
        }
        /* Unknown type: ignore */
    }
}

#endif /* MODE_DATASET */

/* ══════════════════════════════════════════════════════════════════════
 *  MODE_GPS: real-time NEO-M8N GPS + OLED display
 * ══════════════════════════════════════════════════════════════════════ */
#ifdef MODE_GPS

static void OLED_DrawStaticTitle(void) {
    OLED_ShowCHinese(18,0,0);
    OLED_ShowCHinese(36,0,1);
    OLED_ShowCHinese(54,0,2);
    OLED_ShowCHinese(72,0,3);
    OLED_ShowCHinese(90,0,4);
}

int main(void)
{
    uint8_t key = 0xFF;

    delay_init();
    init_usart1();
    init_usart2();
    init_nvic();

    OLED_Init();
    OLED_Clear();
    OLED_DrawStaticTitle();

    AcostBench_Init(1e-5);

    delay_ms(100);

    /* Configure NEO-M8N (UBX protocol, backward compatible with NEO-6M) */
    Ublox_Cfg_Msg(1, 0);   /* disable GLL */
    delay_ms(200);
    Ublox_Cfg_Msg(2, 0);   /* disable GSA */
    delay_ms(200);
    Ublox_Cfg_Msg(3, 0);   /* disable GSV */
    delay_ms(200);

    if (Ublox_Cfg_Rate(1000, 1) != 0) {
        OLED_ShowCHinese(18,3,10);
        OLED_ShowCHinese(36,3,11);
        OLED_ShowCHinese(54,3,12);
        OLED_ShowCHinese(72,3,13);
        OLED_ShowCHinese(90,3,14);
        while ((Ublox_Cfg_Rate(1000, 1) != 0) && key) {
            key = Ublox_Cfg_Cfg_Save();
            delay_ms(100);
        }
        delay_ms(500);
    }

    uart_puts("\r\n=== STM32 Benchmark [GPS MODE: NEO-M8N] ===\r\n");
    uart_puts("Send any byte on USART1 to dump report.\r\n\r\n");

    while (1) {
        GpsDataRead();
        if (AcostBench_IsDumpRequested()) {
            AcostBench_Report();
        }
        delay_ms(800);
    }
}

/* ── IRQ handlers (GPS mode) ───────────────────────────────────────── */
void USART1_IRQHandler(void) {
    AcostBench_USART1_IRQ();
}

#endif /* MODE_GPS */

/* ── Fallback USART2 IRQ (not used in DATASET mode) ───────────────── */
#ifdef MODE_DATASET
void USART2_IRQHandler(void) {
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
        USART_ReceiveData(USART2);
}
#endif
