/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (Node B, DS3231 @ I2C2, Rain LED @ PB9)
  ******************************************************************************
  * H/W:
  *  - Soil AO  : PA0 (ADC1_IN0)
  *  - Rain AO  : PA1 (ADC1_IN1)
  *  - DHT11    : PB8 (DATA)
  *  - Relay    : PB5 (active-HIGH)
  *  - LED_W    : PB4 (active-HIGH)
  *  - LED_R    : PB9 (active-HIGH)
  *  - TX LED   : PC13 (active-LOW)
  *  - LoRa SX1278: SPI1 (PA5/6/7, PA4 CS), PB1 RESET, PB0 DIO0
  *  - LCD I2C1 : PB6 SCL, PB7 SDA
  *  - DS3231   : I2C2 PB10 SCL2, PB11 SDA2
  *  - USB CDC  : PA11/PA12
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"
extern USBD_HandleTypeDef hUsbDeviceFS;

#include "DHT.h"
#include "i2c-lcd.h"
#include "LoRa.h"

#include "W25Qxx.h"
#include "flashlog.h"
#include "uplink_patch.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum { ST_IDLE=0, ST_WATERING, ST_COOLDOWN, ST_RAIN_LOCK } state_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ================== NODE IDENT ================== */
#define NODE_ID_STR   "B"   /* Node B */

/*** ACK & RTC options ***/
#ifndef ACK_ENABLE
#define ACK_ENABLE         1
#endif
#ifndef ACK_WINDOW_MS
#define ACK_WINDOW_MS      1500
#endif

/* vao RX som hon sau TX (giam miss ACK) */
#ifndef ACK_GUARD_MS
#define ACK_GUARD_MS       10
#endif

/* gap sau TX (giam chiem kenh qua lau) */
#ifndef TX_POST_GAP_MS
#define TX_POST_GAP_MS     600
#endif

/* Timestamp source */
#define USE_DS3231_TS      1
#define DS3231_I2C_ADDR7   0x68
#define DS3231_HOLDS_LOCAL 1
#ifndef USE_RTC_TS
#define USE_RTC_TS         1
#endif

/* DS3231 giu gio VN local -> epoch_utc tru 7h */
#ifndef TZ_OFFSET_SECS
#define TZ_OFFSET_SECS     (7*3600)
#endif

#ifndef INIT_RTC_ONCE
#define INIT_RTC_ONCE      0
#endif

/* ---- Set DS3231 once (safe) ---- */
#define SET_DS3231_ONCE      1
#define DS3231_INIT_Y        2025
#define DS3231_INIT_M        11
#define DS3231_INIT_D        10
#define DS3231_INIT_H        00
#define DS3231_INIT_MIN      59
#define DS3231_INIT_S        00
#define DS3231_BKP_MAGIC     0xD321

/*** Thresholds & timings ***/
#define SOIL_ON_TH            30
#define SOIL_OFF_TH           40
#define RH_BLOCK_TH           95

#define RAIN_LOCK_MIN         1
#define SOAK_ON_MIN           1
#define SOAK_OFF_MIN          1
#define COOLDOWN_MIN          1
#define MAX_ON_TIME_MIN       2

/*** ADC channels ***/
#define SOIL_ADC_CH    ADC_CHANNEL_0   // PA0
#define RAIN_ADC_CH    ADC_CHANNEL_1   // PA1

/*** ADC mapping ***/
#define SOIL_ADC_DRY   0x0F44
#define SOIL_ADC_WET   0x07F7
#define SOIL_SMOOTH_N  8

#define RAIN_ADC_DRY   0x0F90
#define RAIN_ADC_WET   0x0679
#define RAIN_SMOOTH_N  4
#define RAIN_SAMPLE_MS 100
#define RAIN_WET_THR_PCT 55
#define RAIN_DEB_TICKS   3

/*** GPIO helpers (REL/LED) ***/
#define RELAY_GPIO_Port GPIOB
#define RELAY_Pin       GPIO_PIN_5
#define RELAY_ON()      HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_SET)
#define RELAY_OFF()     HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_RESET)

#define LEDW_Port GPIOB
#define LEDW_Pin  GPIO_PIN_4
#define LEDR_Port GPIOB
#define LEDR_Pin  GPIO_PIN_9
#define LEDW_ON()  HAL_GPIO_WritePin(LEDW_Port, LEDW_Pin, GPIO_PIN_SET)
#define LEDW_OFF() HAL_GPIO_WritePin(LEDW_Port, LEDW_Pin, GPIO_PIN_RESET)
#define LEDR_ON()  HAL_GPIO_WritePin(LEDR_Port, LEDR_Pin, GPIO_PIN_SET)
#define LEDR_OFF() HAL_GPIO_WritePin(LEDR_Port, LEDR_Pin, GPIO_PIN_RESET)

/*** TX LED (PC13 active-LOW) ***/
#define TXLED_Port GPIOC
#define TXLED_Pin  GPIO_PIN_13
#define TXLED_ON()  HAL_GPIO_WritePin(TXLED_Port, TXLED_Pin, GPIO_PIN_RESET)
#define TXLED_OFF() HAL_GPIO_WritePin(TXLED_Port, TXLED_Pin, GPIO_PIN_SET)

/*** LoRa pins ***/
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;

#define TRANSMIT_TIMEOUT 2000

/*** LCD EPS ***/
#define EPS_SOIL  1.0f
#define EPS_RAIN  5.0f
#define EPS_TEMP  0.5f
#define EPS_RH    2.0f

/*** Heartbeat ***/
#define HEARTBEAT_MS 4000u

/* jitter tranh trung nhip */
#define HEARTBEAT_JITTER_MS  400u

/* Policy */
#define ONLY_TX_ON_CHANGE   0

/* Debug periods */
#define DEBUG_RAW_EVERY_MS  0
#define FLASHDBG_HEAD_SUMMARY_EVERY_MS 0

/* -------- Buttons (active-LOW) -------- */
#define BTN_MODE_Port  GPIOA
#define BTN_MODE_Pin   GPIO_PIN_8
#define BTN_PUMP_Port  GPIOB
#define BTN_PUMP_Pin   GPIO_PIN_3
static inline uint8_t BTN_MODE_PRESSED(void){ return HAL_GPIO_ReadPin(BTN_MODE_Port, BTN_MODE_Pin) == GPIO_PIN_RESET; }
static inline uint8_t BTN_PUMP_PRESSED(void){ return HAL_GPIO_ReadPin(BTN_PUMP_Port, BTN_PUMP_Pin) == GPIO_PIN_RESET; }

/* -------- Status LEDs (active-LOW) -------- */
#define LED_MODE_Port  GPIOA
#define LED_MODE_Pin   GPIO_PIN_9
#define LED_PUMP_Port  GPIOA
#define LED_PUMP_Pin   GPIO_PIN_15
#define LED_MODE_ON()   HAL_GPIO_WritePin(LED_MODE_Port, LED_MODE_Pin, GPIO_PIN_RESET)
#define LED_MODE_OFF()  HAL_GPIO_WritePin(LED_MODE_Port, LED_MODE_Pin, GPIO_PIN_SET)
#define LED_PUMP_ON()   HAL_GPIO_WritePin(LED_PUMP_Port, LED_PUMP_Pin, GPIO_PIN_RESET)
#define LED_PUMP_OFF()  HAL_GPIO_WritePin(LED_PUMP_Port, LED_PUMP_Pin, GPIO_PIN_SET)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MIN_TO_MS(x) ((x) * 60UL * 1000UL)
static inline uint32_t now_ms(void){ return HAL_GetTick(); }

/* ============ LECH PHA NODE B (KHAC NODE A) ============ */
/* offset 900..2900ms theo UID => Node B khong phat trung nhip voi Node A */
static inline uint32_t hb_phase_offset_ms(void){
  uint32_t uid = (uint32_t)HAL_GetUIDw0() ^ (uint32_t)HAL_GetUIDw1() ^ (uint32_t)HAL_GetUIDw2();
  return 900u + (uid % 2000u);
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static volatile state_t state = ST_IDLE;
static state_t state_prev = ST_IDLE;

static volatile uint32_t t_onStart=0, t_coolStart=0, t_rainStart=0;
static const uint32_t RAIN_LOCK_MS   = MIN_TO_MS(RAIN_LOCK_MIN);
static const uint32_t SOAK_ON_MS     = MIN_TO_MS(SOAK_ON_MIN);
static const uint32_t SOAK_OFF_MS    = MIN_TO_MS(SOAK_OFF_MIN);
static const uint32_t COOLDOWN_MS    = MIN_TO_MS(COOLDOWN_MIN);
static const uint32_t MAX_ON_TIME_MS = MIN_TO_MS(MAX_ON_TIME_MIN);

static volatile float   g_soil_percent=NAN, g_rain_percent=NAN, g_temp_c=NAN, g_rh=NAN;
static volatile uint8_t g_rain=0, dht_fault=0;
static volatile int8_t  rain_integrator=0;

static char  lcd_cache[4][21];
static float prev_soil=NAN, prev_rainpct=NAN, prev_t=NAN, prev_rh=NAN;
static int   prev_pump=-1;

typedef struct { uint8_t left; uint16_t on_ms, off_ms; uint8_t phase_on; uint32_t t; } blink_t;
static blink_t txblink = {0};

static LoRa lora;
static volatile uint32_t seq = 0, tx_count = 0;
static volatile uint32_t next_hb_due = 0;

static float   snap_soil=NAN, snap_rainpct=NAN, snap_t=NAN, snap_rh=NAN;
static int     snap_rain=-1, snap_pump=-1, snap_dht=-1;

static uint32_t rng_state = 0x12345678;

static char json_no_crc[208];

static volatile uint32_t t_rain_last = 0;
static volatile uint32_t t_meas_last = 0;

#if DEBUG_RAW_EVERY_MS > 0
static uint32_t t_dbg_last  = 0;
#endif
#if FLASHDBG_HEAD_SUMMARY_EVERY_MS > 0
static uint32_t t_flash_head_dbg_last = 0;
#endif

static volatile uint16_t dbg_soil_adc = 0;
static volatile uint16_t dbg_rain_adc = 0;

static uint32_t g_w25_id = 0;
static uint8_t g_sync_word = 0x34;

extern I2C_HandleTypeDef hi2c2;

/* ===================== TX MANAGER (NON-BLOCKING) ===================== */
#define TX_ITEM_MAX   240
#define TXQ_SIZE      6

typedef struct {
  uint8_t  used;
  uint8_t  from_flash;      // 1: replay tu flash
  uint8_t  log_type;        // LOG_TYPE_DATA / LOG_TYPE_EVENT
  uint32_t ts_meta;
  uint32_t seq_expected;
  uint16_t len;
  char     buf[TX_ITEM_MAX];
} tx_item_t;

static tx_item_t txq[TXQ_SIZE];
static uint8_t txq_head=0, txq_tail=0, txq_cnt=0;

typedef enum { TXS_IDLE=0, TXS_WAIT_GUARD, TXS_WAIT_ACK, TXS_COOLDOWN } tx_state_t;
static tx_state_t txs = TXS_IDLE;

static tx_item_t cur_tx;
static uint32_t  tx_guard_until = 0;
static uint32_t  tx_ack_deadline = 0;
static uint32_t  tx_cool_until = 0;

/* Replay backoff neu fail */
static uint32_t  replay_backoff_until = 0;

/* Control mode */
typedef enum { MODE_AUTO = 0, MODE_MANUAL = 1 } ctrl_mode_t;
static volatile ctrl_mode_t g_mode = MODE_AUTO;
static volatile uint32_t    t_last_cmd_ms = 0;
#define MANUAL_TIMEOUT_MS   (60u * 1000u)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
static uint32_t rtc_epoch_utc(void);
static int      json_extract_int(const char* json, const char* key, int defval);
static void     RTC_SetOnce_IfNeeded(void);

static inline uint8_t bcd2bin(uint8_t x){ return (uint8_t)(x - 6*(x>>4)); }
static inline uint8_t bin2bcd(uint8_t x){ return (uint8_t)(x + 6*(x/10)); }
static HAL_StatusTypeDef ds3231_read_regs(uint8_t reg, uint8_t *dst, uint8_t len);
static HAL_StatusTypeDef ds3231_write_regs(uint8_t reg, const uint8_t *src, uint8_t len);
static int ds3231_get_tm(struct tm *out);
static int ds3231_set_tm(const struct tm *t);
static int ds3231_read_status(uint8_t *st);
static void ds3231_clear_osf(void);
static void DS3231_SetOnce_Safe(void);

static uint32_t days_from_civil(int y, unsigned m, unsigned d);
static uint32_t tm_to_epoch_utc(const struct tm* t);

static void civil_from_days(int z, int *y, unsigned *m, unsigned *d);
static void iso_from_tm_local(const struct tm* t, char *out, size_t n);

static void cdc_send_line(const char* s);
static uint16_t crc16_ccitt(const uint8_t *data, size_t len);

static uint16_t adc_read_single(uint32_t ch);
static uint16_t adc_read_avg(uint32_t ch, int n);
static float soil_adc_to_percent(uint16_t adc);
static float rain_adc_to_wetpct(uint16_t adc);
static void  update_rain_adc_debounce(void);
static void  update_soil_and_dht(void);

static inline void pad20(char* dst, const char* src);
static inline void center20(char* dst, const char* src);
static inline void lcd_put_line_if_changed(uint8_t row, const char* s20);
static int  changed_enough(void);
static inline void snapshot_prev(void);
static void lcd_boot(void);
static void lcd_show(void);
static void dot_to_comma(char* s);

static void txled_start(uint8_t times, uint16_t on_ms, uint16_t off_ms);
static void txled_task(void);

static void lora_init_params(void);
static void send_meas_request(bool forced);
static void send_event_request(const char* ev);
static inline void snapshot_meas(void);
static inline int  changed_for_meas(void);
static void heartbeat_kick_task(void);

static uint8_t rh_allows_watering(void);
static void fsm_step(void);

static uint32_t rng_next(void);

static void diag_dump_raw(void);

static const char* log_type_str(uint8_t type);
static void flash_debug_head_once(void);

static const char* sf_to_str(uint8_t sf);
static const char* bw_to_str(uint8_t bw);
static const char* cr_to_str(uint8_t cr);
static void        log_rf_config(void);

static void apply_cmd_from_ack(const char* json);

static void buttons_task(void);
static void leds_update(void);

static uint8_t txq_push(const char* payload, uint16_t len, uint32_t seq_expected,
                        uint32_t ts_meta, uint8_t log_type, uint8_t from_flash);
static uint8_t txq_pop(tx_item_t* out);
static uint8_t txq_is_full(void);
static uint8_t txq_is_empty(void);

static void tx_start_current(const tx_item_t* it);
static void tx_finish_success(void);
static void tx_finish_fail(void);
static void tx_task(void);
static uint8_t tx_handle_rx_and_check_ack(uint32_t expected_seq);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void cdc_send_line(const char* s){
  if(!s) return;
  static char buf[256];
  int n = snprintf(buf, sizeof(buf), "%s\r\n", s);
  if(n <= 0) return;

  uint32_t t0 = HAL_GetTick();
  while (HAL_GetTick() - t0 < 50) {
    if (CDC_Transmit_FS((uint8_t*)buf, (uint16_t)n) == USBD_OK) {
      USBD_CDC_HandleTypeDef *hcdc =
        (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;

      if (hcdc) {
        uint32_t t1 = HAL_GetTick();
        while (hcdc->TxState && (HAL_GetTick() - t1) < 20) { }
      }
      break;
    }
    HAL_Delay(2);
  }
}

static uint16_t crc16_ccitt(const uint8_t *data, size_t len){
  uint16_t crc = 0xFFFF;
  for(size_t i=0;i<len;i++){
    crc ^= (uint16_t)data[i] << 8;
    for(int b=0;b<8;b++) crc = (crc & 0x8000) ? (uint16_t)((crc<<1) ^ 0x1021) : (uint16_t)(crc<<1);
  }
  return crc;
}

/*** DS3231 over I2C2 ***/
static HAL_StatusTypeDef ds3231_read_regs(uint8_t reg, uint8_t *dst, uint8_t len){
  uint16_t a = (DS3231_I2C_ADDR7 << 1);
  if (HAL_I2C_Master_Transmit(&hi2c2, a, &reg, 1, 50) != HAL_OK) return HAL_ERROR;
  if (HAL_I2C_Master_Receive(&hi2c2, a, dst, len, 50) != HAL_OK)  return HAL_ERROR;
  return HAL_OK;
}
static HAL_StatusTypeDef ds3231_write_regs(uint8_t reg, const uint8_t *src, uint8_t len){
  uint16_t a = (DS3231_I2C_ADDR7 << 1);
  uint8_t buf[16];
  if (len + 1 > sizeof(buf)) return HAL_ERROR;
  buf[0] = reg;
  memcpy(&buf[1], src, len);
  return HAL_I2C_Master_Transmit(&hi2c2, a, buf, len+1, 50);
}
static int ds3231_get_tm(struct tm *out){
  uint8_t b[7];
  if (ds3231_read_regs(0x00, b, 7) != HAL_OK) return 0;
  memset(out, 0, sizeof(*out));
  out->tm_sec  = bcd2bin(b[0] & 0x7F);
  out->tm_min  = bcd2bin(b[1] & 0x7F);
  out->tm_hour = bcd2bin(b[2] & 0x3F);
  out->tm_mday = bcd2bin(b[4] & 0x3F);
  out->tm_mon  = bcd2bin(b[5] & 0x1F) - 1;
  out->tm_year = bcd2bin(b[6]) + 100;
  return 1;
}
static int ds3231_set_tm(const struct tm *t){
  uint8_t b[7];
  b[0] = bin2bcd((uint8_t)t->tm_sec);
  b[1] = bin2bcd((uint8_t)t->tm_min);
  b[2] = bin2bcd((uint8_t)t->tm_hour) & 0x3F;
  b[3] = 1;
  b[4] = bin2bcd((uint8_t)t->tm_mday);
  b[5] = bin2bcd((uint8_t)(t->tm_mon + 1));
  b[6] = bin2bcd((uint8_t)((t->tm_year + 1900) - 2000));
  return (ds3231_write_regs(0x00, b, 7) == HAL_OK) ? 1 : 0;
}

static int ds3231_read_status(uint8_t *st){
  return (ds3231_read_regs(0x0F, st, 1) == HAL_OK) ? 1 : 0;
}
static void ds3231_clear_osf(void){
  uint8_t st;
  if (!ds3231_read_status(&st)) return;
  st &= ~(1u<<7);
  (void)ds3231_write_regs(0x0F, &st, 1);
}

static void DS3231_SetOnce_Safe(void){
#if SET_DS3231_ONCE
  uint16_t bkp = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2);

  struct tm now; int have = ds3231_get_tm(&now);
  int year = have ? (1900 + now.tm_year) : 0;

  uint8_t st = 0; int have_st = ds3231_read_status(&st);
  int osf = (have_st && (st & 0x80)) ? 1 : 0;

  int need_set = 0;
  if (!have || year < 2024) need_set = 1;
  if (osf) need_set = 1;

  if (need_set){
    struct tm t = {0};
    t.tm_year = DS3231_INIT_Y - 1900;
    t.tm_mon  = DS3231_INIT_M - 1;
    t.tm_mday = DS3231_INIT_D;
    t.tm_hour = DS3231_INIT_H;
    t.tm_min  = DS3231_INIT_MIN;
    t.tm_sec  = DS3231_INIT_S;

    if (ds3231_set_tm(&t)) {
      ds3231_clear_osf();
      HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, DS3231_BKP_MAGIC);
      cdc_send_line("DS3231 set OK (safe).");
    } else {
      cdc_send_line("DS3231 set FAIL!");
    }
  } else {
    if (bkp != DS3231_BKP_MAGIC)
      HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, DS3231_BKP_MAGIC);
    cdc_send_line("DS3231 keep existing.");
  }
#else
  (void)ds3231_clear_osf;
#endif
}

/*** ADC utils & mapping ***/
static uint16_t adc_read_single(uint32_t ch){
  ADC_ChannelConfTypeDef s={0};
  s.Channel=ch; s.Rank=ADC_REGULAR_RANK_1; s.SamplingTime=ADC_SAMPLETIME_71CYCLES_5;
  HAL_ADC_ConfigChannel(&hadc1,&s);
  HAL_ADC_Start(&hadc1); HAL_ADC_PollForConversion(&hadc1,10);
  uint16_t v=HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1); return v;
}
static uint16_t adc_read_avg(uint32_t ch, int n){
  uint32_t acc=0; for(int i=0;i<n;i++) acc+=adc_read_single(ch);
  return (uint16_t)(acc/n);
}
static float soil_adc_to_percent(uint16_t adc){
  float den = (float)SOIL_ADC_DRY - (float)SOIL_ADC_WET;
  if (den <= 0.0f) return NAN;
  float pct = ((float)SOIL_ADC_DRY - (float)adc) * 100.0f / den;
  if (pct < 0.0f)   pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  return pct;
}
static float rain_adc_to_wetpct(uint16_t adc){
  float num=(float)adc-(float)RAIN_ADC_WET, den=(float)RAIN_ADC_DRY-(float)RAIN_ADC_WET;
  if(den==0) return NAN;
  float pct_dry=100.0f*num/den; float pct_wet=100.0f-pct_dry;
  if(pct_wet<0)pct_wet=0; if(pct_wet>100)pct_wet=100; return pct_wet;
}
static void update_rain_adc_debounce(void){
  uint16_t adc = adc_read_avg(RAIN_ADC_CH, RAIN_SMOOTH_N);
  dbg_rain_adc = adc;
  g_rain_percent = rain_adc_to_wetpct(adc);
  uint8_t wet_now = (!isnan(g_rain_percent) && g_rain_percent >= RAIN_WET_THR_PCT) ? 1 : 0;
  rain_integrator += wet_now ? 1 : -1;
  if(rain_integrator<0) rain_integrator=0;
  if(rain_integrator>RAIN_DEB_TICKS) rain_integrator=RAIN_DEB_TICKS;
  g_rain = (rain_integrator >= RAIN_DEB_TICKS) ? 1 : 0;
}
static void update_soil_and_dht(void){
  uint16_t rs = adc_read_avg(SOIL_ADC_CH, SOIL_SMOOTH_N);
  dbg_soil_adc = rs;
  g_soil_percent = soil_adc_to_percent(rs);

  DHT_DataTypedef d = (DHT_DataTypedef){0};
  DHT_GetData(&d);
  if (d.Temperature==0 && d.Humidity==0) dht_fault = 1;
  else { dht_fault = 0; g_temp_c = d.Temperature; g_rh = d.Humidity; }
}

/*** LCD helpers ***/
static inline void pad20(char* dst, const char* src){
  size_t n = strlen(src); if(n>20) n=20; memcpy(dst, src, n);
  for(size_t i=n;i<20;i++) dst[i]=' '; dst[20]='\0';
}
static inline void center20(char* dst, const char* src){
  size_t n = strlen(src); if(n>20) n=20;
  size_t left = (20 - n)/2;
  for(size_t i=0;i<left;i++) dst[i]=' ';
  memcpy(dst+left, src, n);
  for(size_t j=left+n;j<20;j++) dst[j]=' ';
  dst[20]='\0';
}
static inline void lcd_put_line_if_changed(uint8_t row, const char* s20){
  if (strncmp(lcd_cache[row], s20, 20) != 0){
    lcd_put_cur(row,0); lcd_send_string(s20);
    strncpy(lcd_cache[row], s20, 20); lcd_cache[row][20]='\0';
  }
}
static int changed_enough(void){
  int ch=0;
  if(isnan(prev_soil) || fabsf(g_soil_percent - prev_soil) > EPS_SOIL) ch=1;
  if(isnan(prev_rainpct) || fabsf(g_rain_percent - prev_rainpct) > EPS_RAIN) ch=1;
  if(!dht_fault){
    if(isnan(prev_t)  || fabsf(g_temp_c - prev_t) > EPS_TEMP) ch=1;
    if(isnan(prev_rh) || fabsf(g_rh - prev_rh)     > EPS_RH)  ch=1;
  }
  if(state_prev != state) ch=1;
  int pump = (state==ST_WATERING);
  if(prev_pump != pump) ch=1;
  return ch;
}
static inline void snapshot_prev(void){
  prev_soil = g_soil_percent; prev_rainpct = g_rain_percent;
  prev_t = g_temp_c; prev_rh = g_rh; prev_pump = (state==ST_WATERING);
  state_prev = state;
}
static void dot_to_comma(char* s){ for(char* p=s; *p; ++p) if(*p=='.') *p=','; }
static void lcd_boot(void){
  lcd_init();
  HAL_Delay(30);
  lcd_clear();
  lcd_put_cur(0,0); lcd_send_string("NODE B - IRRIGATION");
  lcd_put_cur(1,0); lcd_send_string("Booting...");
}
static void lcd_show(void){
  char tmp[48], ln20[21];

  center20(ln20, "NODE B"); lcd_put_line_if_changed(0, ln20);

  snprintf(tmp,sizeof(tmp),"S:%5.2f%%  R:%5.2f%%",
           isnan(g_soil_percent)?0.0f:g_soil_percent,
           isnan(g_rain_percent)?0.0f:g_rain_percent);
  dot_to_comma(tmp); pad20(ln20,tmp); lcd_put_line_if_changed(1, ln20);

  if(!dht_fault) {
    snprintf(tmp,sizeof(tmp),"T:%5.2fC RH:%5.2f%%",
             isnan(g_temp_c)?0.0f:g_temp_c,
             isnan(g_rh)?0.0f:g_rh);
    dot_to_comma(tmp);
  } else {
    strcpy(tmp,"T/RH: -- (DHT ERR)  ");
  }
  pad20(ln20,tmp); lcd_put_line_if_changed(2, ln20);

  const char* st = (state==ST_IDLE)?"IDLE":(state==ST_WATERING)?"WATERING":
                 (state==ST_COOLDOWN)?"COOLDOWN":"RAINLOCK";
  const char* p  = (state==ST_WATERING) ? "ON" : "OFF";
  const char* md = (g_mode==MODE_MANUAL) ? "M" : "A";
  snprintf(tmp,sizeof(tmp),"C:%-8s P:%-3s M:%-1s", st, p, md);
  pad20(ln20,tmp); lcd_put_line_if_changed(3, ln20);
}

/*** LED TX non-blocking ***/
static void txled_start(uint8_t times, uint16_t on_ms, uint16_t off_ms){
  txblink.left = times; txblink.on_ms = on_ms; txblink.off_ms = off_ms;
  txblink.phase_on = 1; txblink.t = now_ms(); TXLED_ON();
}
static void txled_task(void){
  if (txblink.left == 0) return;
  uint32_t dt = now_ms() - txblink.t;
  if (txblink.phase_on && dt >= txblink.on_ms) {
    TXLED_OFF(); txblink.phase_on = 0; txblink.t = now_ms();
  } else if (!txblink.phase_on && dt >= txblink.off_ms) {
    txblink.left--; if (txblink.left) { TXLED_ON(); txblink.phase_on = 1; txblink.t = now_ms(); }
  }
}

/*** RNG ***/
static uint32_t rng_next(void){ rng_state = 1664525u*rng_state + 1013904223u; return rng_state; }

/*** LoRa init ***/
static void lora_init_params(void){
  lora = newLoRa();
  lora.CS_port   = GPIOA; lora.CS_pin   = GPIO_PIN_4;
  lora.reset_port= GPIOB; lora.reset_pin= GPIO_PIN_1;
  lora.DIO0_port = GPIOB; lora.DIO0_pin = GPIO_PIN_0;
  lora.hSPIx     = &hspi1;

  lora.frequency      = 433;
  lora.spredingFactor = SF_9;
  lora.bandWidth      = BW_125KHz;
  lora.crcRate        = CR_4_5;
  lora.power          = POWER_17db;
  lora.overCurrentProtection = 100;
  lora.preamble       = 14;

  uint16_t st = LoRa_init(&lora);
  if (st != LORA_OK) cdc_send_line("ERR: LoRa init");
  else {
    cdc_send_line("LoRa OK");
    int8_t desired_dbm = 17;
    LoRa_setPower_dBm(&lora, desired_dbm);
    char s[32]; snprintf(s, sizeof(s), "TX power = %d dBm", desired_dbm);
    cdc_send_line(s);
  }

  LoRa_setTOMsb_setCRCon(&lora);
  g_sync_word = 0x34;
  LoRa_setSyncWord(&lora, g_sync_word);

  LoRa_startReceiving(&lora);
}

/*** RF config log ***/
static const char* sf_to_str(uint8_t sf){
  switch(sf){
    case SF_7: return "7"; case SF_8: return "8"; case SF_9: return "9";
    case SF_10:return "10";case SF_11:return "11";case SF_12:return "12";
    default:  return "?";
  }
}
static const char* bw_to_str(uint8_t bw){
  switch(bw){
    case BW_7_8KHz:   return "7.8";
    case BW_10_4KHz:  return "10.4";
    case BW_15_6KHz:  return "15.6";
    case BW_20_8KHz:  return "20.8";
    case BW_31_25KHz: return "31.25";
    case BW_41_7KHz:  return "41.7";
    case BW_62_5KHz:  return "62.5";
    case BW_125KHz:   return "125";
    case BW_250KHz:   return "250";
    case BW_500KHz:   return "500";
    default:          return "?";
  }
}
static const char* cr_to_str(uint8_t cr){
  switch(cr){
    case CR_4_5: return "4/5";
    case CR_4_6: return "4/6";
    case CR_4_7: return "4/7";
    case CR_4_8: return "4/8";
    default:     return "?";
  }
}
static void log_rf_config(void){
  char line[128];
  snprintf(line, sizeof(line),
    "RF: freq=%lu MHz SF=%s BW=%s kHz CR=%s SW=0x%02X preamble=%u",
    (unsigned long)lora.frequency,
    sf_to_str(lora.spredingFactor),
    bw_to_str(lora.bandWidth),
    cr_to_str(lora.crcRate),
    (unsigned)g_sync_word,
    (unsigned)lora.preamble);
  cdc_send_line(line);
}

/*** Flash debug ***/
static const char* log_type_str(uint8_t type) {
  switch (type) { case LOG_TYPE_DATA:  return "DATA";
                  case LOG_TYPE_EVENT: return "LOG";
                  default:             return "UNK"; }
}
static void flash_debug_head_once(void) {
  if (Log_Empty()) { cdc_send_line("[FLASH] queue empty"); return; }
  uint8_t  buf[256]; uint16_t len; uint8_t type; uint32_t ts;
  if (!Log_Peek(buf, sizeof(buf)-1, &len, &type, &ts)) {
    cdc_send_line("[FLASH] peek failed"); return;
  }
  buf[len] = '\0';
  char meta[96];
  snprintf(meta, sizeof(meta),"[FLASH HEAD] len=%u type=%s ts=%lu",
           (unsigned)len, log_type_str(type), (unsigned long)ts);
  cdc_send_line(meta);
  cdc_send_line((char*)buf);
}

/*** ISO-8601 helpers ***/
static void civil_from_days(int z, int *y, unsigned *m, unsigned *d){
  z += 719468;
  const int era = (z >= 0 ? z : z - 146096) / 146097;
  const unsigned doe = (unsigned)(z - era * 146097);
  const unsigned yoe = (doe - doe/1460 + doe/36524 - doe/146096) / 365;
  const int y2 = (int)yoe + era * 400;
  const unsigned doy = doe - (365 * yoe + yoe/4 - yoe/100);
  const unsigned mp  = (5 * doy + 2) / 153;
  *d  = doy - (153 * mp + 2) / 5 + 1;
  *m  = mp + (mp < 10 ? 3 : -9);
  *y  = y2 + (*m <= 2);
}
static void iso_from_tm_local(const struct tm* t, char *out, size_t n){
  snprintf(out, n, "%04d-%02d-%02d %02d:%02d:%02d",
           1900 + t->tm_year, t->tm_mon + 1, t->tm_mday,
           t->tm_hour, t->tm_min, t->tm_sec);
}

/* === Time helpers === */
static uint32_t days_from_civil(int y, unsigned m, unsigned d){
  y -= (m <= 2);
  const int era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = (unsigned)(y - era * 400);
  const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return (uint32_t)(era * 146097 + (int)doe - 719468);
}
static uint32_t tm_to_epoch_utc(const struct tm* t){
  uint32_t days = days_from_civil(1900 + t->tm_year, (unsigned)(t->tm_mon+1), (unsigned)t->tm_mday);
  return days*86400u + (uint32_t)t->tm_hour*3600u + (uint32_t)t->tm_min*60u + (uint32_t)t->tm_sec;
}
static uint32_t rtc_epoch_utc(void){
#if USE_DS3231_TS
  struct tm t;
  if (ds3231_get_tm(&t)){
    uint32_t secs = tm_to_epoch_utc(&t);
#if DS3231_HOLDS_LOCAL && (TZ_OFFSET_SECS != 0)
    if (secs >= (uint32_t)TZ_OFFSET_SECS) secs -= (uint32_t)TZ_OFFSET_SECS;
#endif
    return secs;
  }
#endif
#if USE_RTC_TS
  RTC_TimeTypeDef rt; RTC_DateTypeDef rd;
  HAL_RTC_GetTime(&hrtc, &rt, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &rd, RTC_FORMAT_BIN);
  int year = 2000 + rd.Year;
  unsigned mon = rd.Month;
  unsigned day = rd.Date;
  uint32_t days = days_from_civil(year, mon, day);
  uint32_t secs = days*86400u + (uint32_t)rt.Hours*3600u + (uint32_t)rt.Minutes*60u + (uint32_t)rt.Seconds;
#if (TZ_OFFSET_SECS != 0)
  if (secs >= (uint32_t)TZ_OFFSET_SECS) secs -= (uint32_t)TZ_OFFSET_SECS;
#endif
  return secs;
#else
  return (now_ms()/1000u);
#endif
}
static int json_extract_int(const char* json, const char* key, int defval){
  if(!json || !key) return defval;
  const char* p = strstr(json, key);
  if(!p) return defval;
  p = strchr(p, ':'); if(!p) return defval; ++p;
  while (*p==' ' || *p=='\t' || *p=='\r' || *p=='\n') ++p;
  int sign = 1; if(*p=='-'){ sign=-1; ++p; }
  int val = 0; int any=0;
  while(*p>='0' && *p<='9'){ val = val*10 + (*p - '0'); ++p; any=1; }
  if(!any) return defval;
  return sign*val;
}

/* ==== Apply CMD from ACK JSON ==== */
static void apply_cmd_from_ack(const char* json)
{
    if (!json) return;

    int any_cmd_applied = 0;

    int pump     = json_extract_int(json, "\"pump\"", -2);
    int mode_num = json_extract_int(json, "\"mode\"", -1);
    int override = json_extract_int(json, "\"override\"", 0);
    (void)override;

    if (pump != -2) {
        g_mode = MODE_MANUAL;

        if (pump > 0) {
            RELAY_ON();
            state      = ST_WATERING;
            t_onStart  = now_ms();
            cdc_send_line("[CMD] pump=1 -> RELAY_ON");
        } else {
            RELAY_OFF();
            if (state == ST_WATERING || state == ST_IDLE) {
                state       = ST_COOLDOWN;
                t_coolStart = now_ms();
            }
            cdc_send_line("[CMD] pump=0 -> RELAY_OFF");
        }
        any_cmd_applied = 1;
    }

    if (mode_num == 0 || strstr(json, "\"mode\":\"AUTO\"")) {
        g_mode = MODE_AUTO;
        cdc_send_line("[CMD] mode=AUTO");
        any_cmd_applied = 1;
    } else if (mode_num == 1 || strstr(json, "\"mode\":\"MANUAL\"")) {
        g_mode = MODE_MANUAL;
        cdc_send_line("[CMD] mode=MANUAL");
        any_cmd_applied = 1;
    }

    if (any_cmd_applied) {
        t_last_cmd_ms = now_ms();
        if (!(mode_num == 0 || strstr(json, "\"mode\":\"AUTO\""))) {
            g_mode = MODE_MANUAL;
        }
        snapshot_prev();
        lcd_show();
        leds_update();
    }
}

/* ===================== TX QUEUE ===================== */
static uint8_t txq_is_full(void){ return (txq_cnt >= TXQ_SIZE); }
static uint8_t txq_is_empty(void){ return (txq_cnt == 0); }

static uint8_t txq_push(const char* payload, uint16_t len, uint32_t seq_expected,
                        uint32_t ts_meta, uint8_t log_type, uint8_t from_flash)
{
  if (!payload || len == 0) return 0;
  if (len >= TX_ITEM_MAX) return 0;
  if (txq_is_full()) return 0;

  tx_item_t* it = &txq[txq_tail];
  memset(it, 0, sizeof(*it));
  it->used = 1;
  it->from_flash = from_flash;
  it->log_type = log_type;
  it->ts_meta = ts_meta;
  it->seq_expected = seq_expected;
  it->len = len;
  memcpy(it->buf, payload, len);
  it->buf[len] = '\0';

  txq_tail = (uint8_t)((txq_tail + 1) % TXQ_SIZE);
  txq_cnt++;
  return 1;
}

static uint8_t txq_pop(tx_item_t* out)
{
  if (!out) return 0;
  if (txq_is_empty()) return 0;

  tx_item_t* it = &txq[txq_head];
  *out = *it;

  memset(it, 0, sizeof(*it));
  txq_head = (uint8_t)((txq_head + 1) % TXQ_SIZE);
  txq_cnt--;
  return 1;
}

/* ===================== TX MANAGER (NON-BLOCKING) ===================== */
static void tx_start_current(const tx_item_t* it)
{
  if (!it || it->len == 0) return;

  txled_start(1,60,60);

  uint8_t ok = LoRa_transmit(&lora, (uint8_t*)it->buf, (uint8_t)it->len, TRANSMIT_TIMEOUT);

  cdc_send_line(it->buf);

  if (!ok) {
    tx_finish_fail();
    return;
  }

#if ACK_ENABLE
  LoRa_startReceiving(&lora);
  tx_guard_until = now_ms() + ACK_GUARD_MS;
  tx_ack_deadline = now_ms() + ACK_WINDOW_MS;
  txs = TXS_WAIT_GUARD;
#else
  tx_finish_success();
#endif
}

static void tx_finish_success(void)
{
  if (cur_tx.from_flash) {
    Log_Drop();
  } else {
    seq++;
  }
  tx_count++;
  tx_cool_until = now_ms() + TX_POST_GAP_MS;
  txs = TXS_COOLDOWN;
}

static void tx_finish_fail(void)
{
  if (!cur_tx.from_flash) {
    Log_Push((const uint8_t*)cur_tx.buf, cur_tx.len, cur_tx.log_type, cur_tx.ts_meta);
  } else {
    replay_backoff_until = now_ms() + 2000u + (rng_next() % 3000u);
  }
  tx_cool_until = now_ms() + TX_POST_GAP_MS;
  txs = TXS_COOLDOWN;
}

static uint8_t tx_handle_rx_and_check_ack(uint32_t expected_seq)
{
  uint8_t rx[128];
  uint8_t n = LoRa_receive(&lora, rx, sizeof(rx) - 1);
  if (n == 0) return 0;

  rx[n] = '\0';
  char line[160];
  int cut = (n > 100) ? 100 : n;
  snprintf(line, sizeof(line), "[ACK RX] len=%u payload=\"%.*s\"", (unsigned)n, cut, (char*)rx);
  cdc_send_line(line);

  int a = json_extract_int((const char*)rx, "\"ack\"", -1);
  int s = json_extract_int((const char*)rx, "\"seq\"", -1);

  if ((a >= 0 && ((uint32_t)a == expected_seq || (uint32_t)a == (expected_seq + 1u))) ||
      (s >= 0 && ((uint32_t)s == expected_seq || (uint32_t)s == (expected_seq + 1u)))) {
    cdc_send_line("[ACK] received");
    if (strstr((const char*)rx, "\"cmd\"")) {
      apply_cmd_from_ack((const char*)rx);
    }
    return 1;
  }
  return 0;
}

static void tx_task(void)
{
  uint32_t now = now_ms();

  if (txs == TXS_COOLDOWN) {
    if ((int32_t)(now - tx_cool_until) >= 0) txs = TXS_IDLE;
    else return;
  }

  if (txs == TXS_WAIT_GUARD) {
    if ((int32_t)(now - tx_guard_until) >= 0) txs = TXS_WAIT_ACK;
    else return;
  }

  if (txs == TXS_WAIT_ACK) {
    if (tx_handle_rx_and_check_ack(cur_tx.seq_expected)) { tx_finish_success(); return; }
    if ((int32_t)(now - tx_ack_deadline) >= 0) { cdc_send_line("[ACK] timeout"); tx_finish_fail(); return; }
    return;
  }

  if (txs != TXS_IDLE) return;
  if ((int32_t)(now - tx_cool_until) < 0) return;

  if (!txq_is_empty()) {
    if (txq_pop(&cur_tx)) tx_start_current(&cur_tx);
    return;
  }

  if (!Log_Empty() && (int32_t)(now - replay_backoff_until) >= 0) {
    uint8_t  buf[TX_ITEM_MAX];
    uint16_t len; uint8_t type; uint32_t ts;

    if (Log_Peek(buf, sizeof(buf)-1, &len, &type, &ts)) {
      buf[len] = '\0';
      (void)uplink_set_hist_and_fix_crc((char*)buf, sizeof(buf), 1);

      uint32_t seq_sent = (uint32_t)json_extract_int((char*)buf, "\"seq\"", -1);

      if (txq_push((const char*)buf, (uint16_t)strlen((char*)buf), seq_sent, ts, type, 1)) {
        if (txq_pop(&cur_tx)) tx_start_current(&cur_tx);
      }
    }
  }
}

/* ===================== DATA/LOG REQUEST (enqueue) ===================== */
static void send_meas_request(bool forced)
{
  (void)forced;

  const char* st=(state==ST_IDLE)?"IDLE":(state==ST_WATERING)?"WATERING":
                 (state==ST_COOLDOWN)?"COOLDOWN":"RAIN_LOCK";
  int pump=(state==ST_WATERING)?1:0;

  char iso[32] = "1970-01-01 00:00:00";
  uint32_t ts_meta = 0;

  struct tm t;
  if (ds3231_get_tm(&t)) {
    iso_from_tm_local(&t, iso, sizeof(iso));
    ts_meta = tm_to_epoch_utc(&t);
#if DS3231_HOLDS_LOCAL && (TZ_OFFSET_SECS != 0)
    if (ts_meta >= (uint32_t)TZ_OFFSET_SECS) ts_meta -= (uint32_t)TZ_OFFSET_SECS;
#endif
  } else {
    ts_meta = rtc_epoch_utc();
    uint32_t day = ts_meta / 86400u, sec=ts_meta%86400u;
    unsigned hh = sec/3600u, mm=(sec%3600u)/60u, ss=sec%60u;
    int y; unsigned mo, da; civil_from_days((int)day,&y,&mo,&da);
    snprintf(iso, sizeof(iso), "%04d-%02u-%02u %02u:%02u:%02u", y,mo,da,hh,mm,ss);
  }

  uint32_t seq_sent = seq;
  const char* mode_str = (g_mode==MODE_MANUAL) ? "MANUAL" : "AUTO";

  int n = snprintf(json_no_crc, sizeof(json_no_crc),
    "{\"ver\":1,\"node\":\"%s\",\"type\":\"DATA\",\"seq\":%lu,"
    "\"time\":\"%s\",\"soil\":%.2f,\"rainpct\":%.2f,\"rain\":%d,"
    "\"rh\":%.2f,\"t\":%.2f,\"st\":\"%s\",\"pump\":%d,"
    "\"dht_fault\":%d,\"hist\":0,\"mode\":\"%s\"}",
    NODE_ID_STR, (unsigned long)seq_sent, iso,
    g_soil_percent, isnan(g_rain_percent)?-1.0f:g_rain_percent, g_rain,
    isnan(g_rh)?-1.0f:g_rh, isnan(g_temp_c)?-100.0f:g_temp_c,
    st, pump, dht_fault, mode_str);
  if(n<=0 || n >= (int)sizeof(json_no_crc)) return;

  uint16_t crc = crc16_ccitt((const uint8_t*)json_no_crc, (size_t)n);

  char json_final[TX_ITEM_MAX];
  int m = snprintf(json_final, sizeof(json_final),
                   "%.*s,\"crc\":\"%04X\"}", n-1, json_no_crc, crc);
  if (m <= 0 || m >= (int)sizeof(json_final)) return;

  if (!txq_push(json_final, (uint16_t)strlen(json_final), seq_sent, ts_meta, LOG_TYPE_DATA, 0)) {
    cdc_send_line("[TXQ] full, skip meas");
  }
}

static void send_event_request(const char* ev)
{
  if (!ev) return;

  char iso[32] = "1970-01-01 00:00:00";
  uint32_t ts_meta = 0;

  struct tm t;
  if (ds3231_get_tm(&t)) {
    iso_from_tm_local(&t, iso, sizeof(iso));
    ts_meta = tm_to_epoch_utc(&t);
#if DS3231_HOLDS_LOCAL && (TZ_OFFSET_SECS != 0)
    if (ts_meta >= (uint32_t)TZ_OFFSET_SECS) ts_meta -= (uint32_t)TZ_OFFSET_SECS;
#endif
  } else {
    ts_meta = rtc_epoch_utc();
    uint32_t day = ts_meta / 86400u, sec=ts_meta%86400u;
    unsigned hh = sec/3600u, mm=(sec%3600u)/60u, ss=sec%60u;
    int y; unsigned mo, da; civil_from_days((int)day,&y,&mo,&da);
    snprintf(iso, sizeof(iso), "%04d-%02u-%02u %02u:%02u:%02u", y,mo,da,hh,mm,ss);
  }

  uint32_t seq_sent = seq;
  const char* mode_str = (g_mode==MODE_MANUAL) ? "MANUAL" : "AUTO";

  int n = snprintf(json_no_crc, sizeof(json_no_crc),
    "{\"ver\":1,\"node\":\"%s\",\"type\":\"LOG\",\"seq\":%lu,"
    "\"time\":\"%s\",\"st\":\"%s\",\"hist\":0,\"mode\":\"%s\"}",
    NODE_ID_STR, (unsigned long)seq_sent, iso, ev, mode_str);
  if(n<=0 || n >= (int)sizeof(json_no_crc)) return;

  uint16_t crc = crc16_ccitt((const uint8_t*)json_no_crc, (size_t)n);

  char json_final[TX_ITEM_MAX];
  int m = snprintf(json_final, sizeof(json_final),
                   "%.*s,\"crc\":\"%04X\"}", n-1, json_no_crc, crc);
  if (m <= 0 || m >= (int)sizeof(json_final)) return;

  if (!txq_push(json_final, (uint16_t)strlen(json_final), seq_sent, ts_meta, LOG_TYPE_EVENT, 0)) {
    Log_Push((const uint8_t*)json_final, (uint16_t)strlen(json_final), LOG_TYPE_EVENT, ts_meta);
    cdc_send_line("[TXQ] full -> event pushed to FLASH");
  }
}

/*** Change detection policy ***/
static inline int changed_for_meas(void){
  if(isnan(snap_soil)   || fabsf(snap_soil - g_soil_percent) > 1.5f) return 1;
  if(isnan(snap_rainpct)|| fabsf(snap_rainpct - g_rain_percent)>10.0f) return 1;
  if(!dht_fault){
    if(isnan(snap_t)    || fabsf(snap_t - g_temp_c) > 0.8f) return 1;
    if(isnan(snap_rh)   || fabsf(snap_rh - g_rh)    > 3.0f) return 1;
  }
  if(snap_rain != (int)g_rain) return 1;
  if(snap_pump != (state==ST_WATERING)) return 1;
  if(snap_dht  != dht_fault) return 1;
  return 0;
}
static inline void snapshot_meas(void){
  snap_soil=g_soil_percent; snap_rainpct=g_rain_percent; snap_t=g_temp_c; snap_rh=g_rh;
  snap_rain=g_rain; snap_pump=(state==ST_WATERING); snap_dht=dht_fault;
}

static void heartbeat_kick_task(void){
  uint32_t now = now_ms();
  if (now >= next_hb_due){
#if ONLY_TX_ON_CHANGE
    if (changed_for_meas()) {
      send_meas_request(false);
      snapshot_meas();
    }
#else
    send_meas_request(false);
    snapshot_meas();
#endif
    next_hb_due = now + HEARTBEAT_MS + (rng_next() % HEARTBEAT_JITTER_MS);
  }
}

/*** FSM ***/
static uint8_t rh_allows_watering(void){
  if(dht_fault) return 1;
  if(isnan(g_rh)) return 1;
  return (g_rh < RH_BLOCK_TH) ? 1 : 0;
}
static void fsm_step(void){
  if (g_mode == MODE_MANUAL) {
    if (state == ST_WATERING && (now_ms() - t_onStart) > MAX_ON_TIME_MS) {
      RELAY_OFF(); t_coolStart = now_ms(); state = ST_COOLDOWN;
      send_event_request("SAFETY_MAX_ON");
    }
    return;
  }
  switch(state){
  case ST_IDLE:
    if(g_rain==1){ state=ST_RAIN_LOCK; t_rainStart=now_ms(); RELAY_OFF(); send_event_request("RAIN_LOCK"); break; }
    if(!isnan(g_soil_percent) && g_soil_percent<SOIL_ON_TH && rh_allows_watering()){
      RELAY_ON(); t_onStart=now_ms(); state=ST_WATERING; send_event_request("WATER_ON");
    }
    break;
  case ST_WATERING:
    if(g_rain==1 ||
       (!isnan(g_soil_percent) && g_soil_percent>SOIL_OFF_TH) ||
       (now_ms()-t_onStart)>MAX_ON_TIME_MS ||
       (now_ms()-t_onStart)>SOAK_ON_MS)
    { RELAY_OFF(); t_coolStart=now_ms(); state=ST_COOLDOWN; send_event_request("WATER_OFF"); }
    break;
  case ST_COOLDOWN:
    if(g_rain==1){ state=ST_RAIN_LOCK; t_rainStart=now_ms(); send_event_request("RAIN_LOCK"); break; }
    if((now_ms()-t_coolStart)>COOLDOWN_MS || (now_ms()-t_coolStart)>SOAK_OFF_MS)
      state=ST_IDLE;
    break;
  case ST_RAIN_LOCK:
    RELAY_OFF();
    if(g_rain==0 && (now_ms()-t_rainStart)>RAIN_LOCK_MS) { state=ST_IDLE; send_event_request("RAIN_CLEAR"); }
    break;
  }
}

/*** LEDs ***/
static void leds_update(void){
  if(state==ST_WATERING) LEDW_ON(); else LEDW_OFF();
  if(g_rain) LEDR_ON();
  else if(state==ST_RAIN_LOCK){ if(((now_ms()/500)%2)==0) LEDR_ON(); else LEDR_OFF(); }
  else LEDR_OFF();

  if (state == ST_WATERING) LED_PUMP_ON(); else LED_PUMP_OFF();
  if (g_mode == MODE_MANUAL) LED_MODE_ON(); else LED_MODE_OFF();
}

static void diag_dump_raw(void){
  uint16_t a0 = adc_read_avg(SOIL_ADC_CH, 32);
  uint16_t a1 = adc_read_avg(RAIN_ADC_CH, 32);
  char line[64];
  snprintf(line, sizeof(line), "RAW ADC: A0(Soil)=%u  A1(Rain)=%u", a0, a1);
  cdc_send_line(line);
}

static void RTC_SetOnce_IfNeeded(void){
  uint32_t magic = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1);
  if (magic == 0xBEEF) return;

  RTC_TimeTypeDef t = {0};
  RTC_DateTypeDef d = {0};
  t.Hours = 0; t.Minutes = 0; t.Seconds = 5;
  d.WeekDay = RTC_WEEKDAY_WEDNESDAY;
  d.Date = 1; d.Month = 1; d.Year = 25;
  HAL_RTC_SetTime(&hrtc, &t, RTC_FORMAT_BIN);
  HAL_RTC_SetDate(&hrtc, &d, RTC_FORMAT_BIN);
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0xBEEF);
}

/* Buttons debounce */
typedef struct {
  uint8_t stable;
  uint8_t last_raw;
  uint32_t t_edge;
} btn_db_t;

static btn_db_t b_mode = {0,0,0}, b_pump = {0,0,0};
#define DEBOUNCE_MS 30u

static uint8_t debounce_step(btn_db_t *b, uint8_t raw_now){
  if (raw_now != b->last_raw){ b->last_raw = raw_now; b->t_edge = now_ms(); }
  if ((uint32_t)(now_ms() - b->t_edge) >= DEBOUNCE_MS){
    if (b->stable != raw_now){
      uint8_t prev = b->stable;
      b->stable = raw_now;
      return (prev == 0 && b->stable == 1);
    }
  }
  return 0;
}

static void buttons_task(void){
  if (debounce_step(&b_mode, BTN_MODE_PRESSED())){
    if (g_mode == MODE_AUTO){
      g_mode = MODE_MANUAL; t_last_cmd_ms = now_ms(); send_event_request("MODE_MANUAL_BTN");
    } else {
      g_mode = MODE_AUTO;   send_event_request("MODE_AUTO_BTN");
    }
  }

  if (g_mode == MODE_MANUAL && debounce_step(&b_pump, BTN_PUMP_PRESSED())){
    if (state == ST_WATERING){
      RELAY_OFF(); state = ST_COOLDOWN; t_coolStart = now_ms();
      send_event_request("PUMP_OFF_BTN");
    } else {
      RELAY_ON();  state = ST_WATERING; t_onStart = now_ms();
      send_event_request("PUMP_ON_BTN");
    }
    t_last_cmd_ms = now_ms();
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_RTC_Init();
#if INIT_RTC_ONCE
  RTC_SetOnce_IfNeeded();
#endif

  DS3231_SetOnce_Safe();

  /* USER CODE BEGIN 2 */
  RELAY_OFF(); LEDW_OFF(); LEDR_OFF(); TXLED_OFF();

  lcd_boot();
  lora_init_params();
  log_rf_config();

  {
    struct tm t;
    if (ds3231_get_tm(&t)){
      char s[64];
      snprintf(s,sizeof(s),"DS3231: %04d-%02d-%02d %02d:%02d:%02d",
               1900+t.tm_year, t.tm_mon+1, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec);
      cdc_send_line(s);
    } else {
      cdc_send_line("DS3231: read fail");
    }
  }

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
  W25Q_ForceSPI2_Mode0();
  W25Q_ReleasePowerDown();
  W25Q_Reset();
  g_w25_id = W25Q_ReadID();
  Log_Init();
  { char info[48]; snprintf(info,sizeof(info),"FLASH: JEDEC=0x%06lX", (unsigned long)g_w25_id); cdc_send_line(info); }

  flash_debug_head_once();

  update_rain_adc_debounce();
  update_soil_and_dht();
  lcd_show();
  snapshot_prev();

  txled_start(3,80,80);
  cdc_send_line("BOOT: Node B ready");

  /* seed RNG: uid + tick */
  rng_state ^= (uint32_t)HAL_GetUIDw0() ^ (uint32_t)HAL_GetUIDw1() ^ (uint32_t)HAL_GetUIDw2() ^ (now_ms()<<16);

  /* ========== LECH PHA NGAY TU DAU (KHAC NODE A) ========== */
  next_hb_due = now_ms()
              + hb_phase_offset_ms()
              + (rng_next() % HEARTBEAT_JITTER_MS);

  snapshot_meas();

  t_rain_last = now_ms();
  t_meas_last = now_ms();

#if DEBUG_RAW_EVERY_MS > 0
  t_dbg_last = now_ms();
#endif
#if FLASHDBG_HEAD_SUMMARY_EVERY_MS > 0
  t_flash_head_dbg_last = now_ms();
#endif
  /* USER CODE END 2 */

  while (1)
  {
    uint32_t now = now_ms();

    if ((uint32_t)(now - t_rain_last) >= (uint32_t)RAIN_SAMPLE_MS) {
      t_rain_last = now;
      update_rain_adc_debounce();
    }

    if ((uint32_t)(now - t_meas_last) >= 3000u) {
      t_meas_last = now;
      update_soil_and_dht();
      if (changed_enough()) { lcd_show(); snapshot_prev(); }
    }

#if DEBUG_RAW_EVERY_MS > 0
    if ((uint32_t)(now - t_dbg_last) >= (uint32_t)DEBUG_RAW_EVERY_MS) {
      t_dbg_last = now;
      diag_dump_raw();
    }
#endif

#if (FLASHDBG_HEAD_SUMMARY_EVERY_MS > 0)
    if ((uint32_t)(now - t_flash_head_dbg_last) >= FLASHDBG_HEAD_SUMMARY_EVERY_MS) {
      t_flash_head_dbg_last = now;
      if (!Log_Empty()) flash_debug_head_once();
      else cdc_send_line("[FLASH] queue empty");
    }
#endif

    heartbeat_kick_task();
    tx_task();

    if (g_mode == MODE_MANUAL && (now_ms() - t_last_cmd_ms) > MANUAL_TIMEOUT_MS) {
      g_mode = MODE_AUTO;
      send_event_request("MODE_AUTO_RESUME");
    }

    fsm_step();
    buttons_task();
    leds_update();
    txled_task();
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_RCC_BKP_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();

  RCC_OscInitStruct.OscillatorType   = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSEState         = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue   = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState         = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState         = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState     = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource    = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL       = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                   |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection    = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection    = RCC_USBCLKSOURCE_PLL_DIV1_5;
  PeriphClkInit.RTCClockSelection    = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) { Error_Handler(); }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file; (void)line;
}
#endif /* USE_FULL_ASSERT */
