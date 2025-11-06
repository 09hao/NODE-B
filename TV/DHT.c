/************** DHT for STM32F1 – robust, with timeouts ********************/
#include "stm32f1xx_hal.h"
#include "DHT.h"

/* ---------- Select sensor & pin ---------- */
#define TYPE_DHT11        1
/* #define TYPE_DHT22     1 */

#define DHT_PORT GPIOB
#define DHT_PIN  GPIO_PIN_8

/* ---------- Microsecond delay via DWT ---------- */
static inline void DWT_Delay_Init(void) {
  /* enable trace */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  /* enable cycle counter */
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline void delay_us(uint32_t us) {
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = (HAL_RCC_GetHCLKFreq()/1000000U) * us;
  while ((DWT->CYCCNT - start) < ticks) { /* spin */ }
}

/* ---------- GPIO helpers ---------- */
static inline void set_output_od(GPIO_TypeDef *GPIOx, uint16_t pin) {
  GPIO_InitTypeDef i = {0};
  i.Pin = pin;
  i.Mode = GPIO_MODE_OUTPUT_OD;     // open-drain -> an toàn cho bus 1 dây
  i.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOx, &i);
}
static inline void set_input_pu(GPIO_TypeDef *GPIOx, uint16_t pin) {
  GPIO_InitTypeDef i = {0};
  i.Pin = pin;
  i.Mode = GPIO_MODE_INPUT;
  i.Pull = GPIO_PULLUP;             // tránh float n?u thi?u R pull-up ngoài
  HAL_GPIO_Init(GPIOx, &i);
}

/* Wait until pin becomes state, with timeout in us. Return 1=ok, 0=timeout */
static inline uint8_t wait_level(GPIO_PinState state, uint32_t timeout_us) {
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = (HAL_RCC_GetHCLKFreq()/1000000U) * timeout_us;
  while (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN) != state) {
    if ((DWT->CYCCNT - start) > ticks) return 0;
  }
  return 1;
}

/* ---------- Low level bus primitives ---------- */
static void DHT_Start(void) {
  DWT_Delay_Init();

  set_output_od(DHT_PORT, DHT_PIN);
  HAL_GPIO_WritePin(DHT_PORT, DHT_PIN, GPIO_PIN_RESET);  // MCU kéo xu?ng

#if defined(TYPE_DHT11)
  delay_us(18000);                 // =18ms cho DHT11
#else
  delay_us(1200);                  // >1ms cho DHT22
#endif

  HAL_GPIO_WritePin(DHT_PORT, DHT_PIN, GPIO_PIN_SET);    // th? lên
  delay_us(30);                                          // ~20–40us
  set_input_pu(DHT_PORT, DHT_PIN);                       // nh? bus cho sensor
}

/* Tr? v? 1 n?u sensor có ph?n h?i start (80us low + 80us high) */
static uint8_t DHT_Check_Response(void) {
  /* 1) Sensor kéo LOW ~80us */
  if (!wait_level(GPIO_PIN_RESET, 120)) return 0;
  /* 2) Sau dó kéo HIGH ~80us */
  if (!wait_level(GPIO_PIN_SET,   120)) return 0;
  /* 3) K?t thúc phase high -> v? LOW d? b?t d?u bit0 */
  if (!wait_level(GPIO_PIN_RESET, 120)) return 0;
  return 1;
}

/* Ð?c 1 byte, tr? 1=ok, 0=timeout */
static uint8_t DHT_ReadByte(uint8_t *val) {
  uint8_t i = 0;
  uint8_t data = 0;

  for (i = 0; i < 8; i++) {
    /* M?i bit b?t d?u b?ng LOW ~50us */
    if (!wait_level(GPIO_PIN_SET, 100)) return 0;  // ch? lên HIGH (b?t d?u c?a s? do)
    /* Sau khi lên HIGH:
       ~26-28us -> bit 0
       ~70us    -> bit 1
       M?o: l?y m?u ? ~40-50us t? lúc HIGH b?t d?u */
    delay_us(45);
    data <<= 1;
    if (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN) == GPIO_PIN_SET) {
      data |= 0x01;  // bit = 1
    }
    /* ch? sensor kéo xu?ng k?t thúc bit */
    if (!wait_level(GPIO_PIN_RESET, 100)) return 0;
  }

  *val = data;
  return 1;
}

/* ---------- Public API ---------- */
void DHT_GetData(DHT_DataTypedef *out)
{
  /* m?c d?nh tr? l?i = 0,0 (d? main phát hi?n dht_fault) */
  out->Temperature = 0;
  out->Humidity    = 0;

  DHT_Start();
  if (!DHT_Check_Response()) return;

  uint8_t rh1=0, rh2=0, t1=0, t2=0, sum=0;

  if (!DHT_ReadByte(&rh1)) return;
  if (!DHT_ReadByte(&rh2)) return;
  if (!DHT_ReadByte(&t1 )) return;
  if (!DHT_ReadByte(&t2 )) return;
  if (!DHT_ReadByte(&sum)) return;

  if (((uint8_t)(rh1 + rh2 + t1 + t2)) != sum) {
    /* checksum sai -> gi? 0,0 */
    return;
  }

#if defined(TYPE_DHT11)
  out->Humidity    = rh1;  /* DHT11: byte nguyên */
  out->Temperature = t1;
#else /* DHT22 */
  out->Humidity    = ((rh1 << 8) | rh2) / 10.0f;
  int16_t rawT     = ((t1  << 8) | t2);
  if (rawT & 0x8000) { rawT &= 0x7FFF; out->Temperature = -(rawT/10.0f); }
  else                out->Temperature =  (rawT/10.0f);
#endif
}
