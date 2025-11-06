/* flashlog.h — Ring buffer logger on W25Q32 (uses W25Qxx low-level API)
 * Layout:
 *   [META sector @ 0x000000]
 *     magic "LOGM", version, head, tail
 *   [LOG  area @ 0x001000 .. 0x001000 + LOG_AREA_SIZE)
 *     variable-length records:
 *       struct rec_hdr { 0xA55A, len, ts, type, flags, crc16 } + payload(len)
 *
 * API:
 *   void  Log_Init(void);
 *   bool  Log_Push(const uint8_t* buf, uint16_t len, uint8_t type, uint32_t ts);
 *   bool  Log_Peek(uint8_t* out, uint16_t max, uint16_t* len, uint8_t* type, uint32_t* ts);
 *   void  Log_Drop(void);
 *   bool  Log_Empty(void);
 */
#ifndef FLASHLOG_H
#define FLASHLOG_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --- Config (can be tuned) --- */
#define LOG_META_ADDR     0x000000u             /* sector 0 */
#define LOG_AREA_BASE     0x001000u             /* sector 1 */
#define LOG_AREA_SECTORS  32u                   /* 32 * 4KB = 128 KB for logs */
#define LOG_AREA_SIZE     (LOG_AREA_SECTORS * 4096u)
#define LOG_AREA_END      (LOG_AREA_BASE + LOG_AREA_SIZE)

/* Types for "type" field */
#define LOG_TYPE_DATA     1u
#define LOG_TYPE_EVENT    2u

/* Flags bits */
#define LOG_FLAG_NONE     0u

/* API */
void  Log_Init(void);
bool  Log_Push(const uint8_t* buf, uint16_t len, uint8_t type, uint32_t ts);
bool  Log_Peek(uint8_t* out, uint16_t max, uint16_t* len, uint8_t* type, uint32_t* ts);
void  Log_Drop(void);
bool  Log_Empty(void);

/* Helpers */
uint16_t flashlog_crc16_ccitt(const uint8_t* data, uint32_t len);

#ifdef __cplusplus
}
#endif
#endif
