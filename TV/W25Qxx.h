#ifndef INC_W25QXX_H_
#define INC_W25QXX_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Basic commands */
void     W25Q_Reset(void);
uint32_t W25Q_ReadID(void);
uint8_t  W25Q_ReadStatus1(void);
void     W25Q_WriteEnable(void);
void     W25Q_WaitWhileBusy(void);
void     W25Q_EraseSector(uint32_t addr);
void     W25Q_Read(uint32_t addr, uint8_t *dst, uint32_t len);
void     W25Q_Write(uint32_t addr, const uint8_t *src, uint32_t len);
void W25Q_ForceSPI2_Mode0(void);
void W25Q_ReleasePowerDown(void);

/* Geometry */
#define W25Q_PAGE_SIZE     256u
#define W25Q_SECTOR_SIZE   4096u

#ifdef __cplusplus
}
#endif

#endif /* INC_W25QXX_H_ */
