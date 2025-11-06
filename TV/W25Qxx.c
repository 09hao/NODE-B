#include "main.h"
#include "W25Qxx.h"

extern SPI_HandleTypeDef hspi2;
#define W25Q_SPI   hspi2
#define CS_PORT    GPIOB
#define CS_PIN     GPIO_PIN_12
#define csLOW()    HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET)
#define csHIGH()   HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET)

enum {
  CMD_WRITE_ENABLE   = 0x06,
  CMD_READ_STATUS1   = 0x05,
  CMD_SECTOR_ERASE   = 0x20,   /* 4KB */
  CMD_PAGE_PROGRAM   = 0x02,
  CMD_READ_DATA      = 0x03,
  CMD_JEDEC_ID       = 0x9F,
  CMD_RESET_ENABLE   = 0x66,
  CMD_RESET_MEMORY   = 0x99,
};

void W25Q_Reset(void) {
  uint8_t cmd[2] = {CMD_RESET_ENABLE, CMD_RESET_MEMORY};
  csLOW();
  HAL_SPI_Transmit(&W25Q_SPI, &cmd[0], 1, 1000);
  HAL_SPI_Transmit(&W25Q_SPI, &cmd[1], 1, 1000);
  csHIGH();
  HAL_Delay(1);
}

uint32_t W25Q_ReadID(void) {
  uint8_t tx[4] = {CMD_JEDEC_ID, 0,0,0};
  uint8_t rx[4] = {0};
  csLOW();
  HAL_SPI_TransmitReceive(&W25Q_SPI, tx, rx, 4, 1000);
  csHIGH();
  return ((uint32_t)rx[1] << 16) | ((uint32_t)rx[2] << 8) | rx[3];
}

uint8_t W25Q_ReadStatus1(void) {
  uint8_t tx[2] = {CMD_READ_STATUS1, 0};
  uint8_t rx[2] = {0};
  csLOW();
  HAL_SPI_TransmitReceive(&W25Q_SPI, tx, rx, 2, 1000);
  csHIGH();
  return rx[1];
}

void W25Q_WriteEnable(void) {
  uint8_t cmd = CMD_WRITE_ENABLE;
  csLOW();
  HAL_SPI_Transmit(&W25Q_SPI, &cmd, 1, 1000);
  csHIGH();
}

void W25Q_WaitWhileBusy(void) {
  while (W25Q_ReadStatus1() & 0x01) {
    HAL_Delay(1);
  }
}

void W25Q_EraseSector(uint32_t addr) {
  uint8_t cmd[4] = {CMD_SECTOR_ERASE, (uint8_t)(addr>>16), (uint8_t)(addr>>8), (uint8_t)addr};
  W25Q_WriteEnable();
  csLOW();
  HAL_SPI_Transmit(&W25Q_SPI, cmd, 4, 1000);
  csHIGH();
  W25Q_WaitWhileBusy();
}

void W25Q_Read(uint32_t addr, uint8_t *dst, uint32_t len) {
  uint8_t hdr[4] = {CMD_READ_DATA, (uint8_t)(addr>>16), (uint8_t)(addr>>8), (uint8_t)addr};
  csLOW();
  HAL_SPI_Transmit(&W25Q_SPI, hdr, 4, 1000);
  HAL_SPI_Receive(&W25Q_SPI, dst, len, 10000);
  csHIGH();
}

static void page_program(uint32_t addr, const uint8_t *src, uint32_t len) {
  uint8_t hdr[4] = {CMD_PAGE_PROGRAM, (uint8_t)(addr>>16), (uint8_t)(addr>>8), (uint8_t)addr};
  W25Q_WriteEnable();
  csLOW();
  HAL_SPI_Transmit(&W25Q_SPI, hdr, 4, 1000);
  HAL_SPI_Transmit(&W25Q_SPI, (uint8_t*)src, len, 10000);
  csHIGH();
  W25Q_WaitWhileBusy();
}

void W25Q_Write(uint32_t addr, const uint8_t *src, uint32_t len) {
  while (len) {
    uint32_t page_off = addr % W25Q_PAGE_SIZE;
    uint32_t chunk = W25Q_PAGE_SIZE - page_off;
    if (chunk > len) chunk = len;
    page_program(addr, src, chunk);
    addr += chunk;
    src  += chunk;
    len  -= chunk;
  }
}

void W25Q_ForceSPI2_Mode0(void){
  if (hspi2.Init.CLKPolarity != SPI_POLARITY_LOW ||
      hspi2.Init.CLKPhase    != SPI_PHASE_1EDGE   ||
      hspi2.Init.DataSize    != SPI_DATASIZE_8BIT ||
      hspi2.Init.FirstBit    != SPI_FIRSTBIT_MSB  ||
      hspi2.Init.NSS         != SPI_NSS_SOFT) {
    HAL_SPI_DeInit(&hspi2);
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;   // MODE 0
    hspi2.Init.CLKPhase    = SPI_PHASE_1EDGE;
    hspi2.Init.DataSize    = SPI_DATASIZE_8BIT;
    hspi2.Init.FirstBit    = SPI_FIRSTBIT_MSB;
    hspi2.Init.NSS         = SPI_NSS_SOFT;       // b?t bu?c Soft NSS
    // Prescaler tu? ý (8–64). 32 ~ 1.125 Mbps là ?n:
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    HAL_SPI_Init(&hspi2);
  }
}

void W25Q_ReleasePowerDown(void){ // 0xAB
  uint8_t cmd=0xAB, dummy=0;
  csLOW();  HAL_SPI_Transmit(&hspi2,&cmd,1,1000);
  HAL_SPI_Receive(&hspi2,&dummy,1,1000);
  csHIGH(); HAL_Delay(1);
}