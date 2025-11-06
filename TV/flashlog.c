#include "flashlog.h"
#include "W25Qxx.h"
#include <string.h>

#ifndef MIN
#define MIN(a,b) ((a)<(b)?(a):(b))
#endif

#define REC_MAGIC   0xA55Au
#define META_MAGIC  0x4C4F474Du  /* "LOGM" */
#define VERSION     0x0001u
#define SECTOR_SZ   4096u
#define PAGE_SZ     256u

typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint16_t version;
  uint16_t rsv;
  uint32_t head;  /* absolute address in [LOG_AREA_BASE, LOG_AREA_END) */
  uint32_t tail;  /* absolute address in [LOG_AREA_BASE, LOG_AREA_END) */
} meta_t;

typedef struct __attribute__((packed)) {
  uint16_t magic;   /* 0xA55A */
  uint16_t len;     /* payload length */
  uint32_t ts;      /* epoch seconds */
  uint8_t  type;    /* LOG_TYPE_* */
  uint8_t  flags;   /* reserved */
  uint16_t crc16;   /* CRC of payload */
} rec_hdr_t;

static meta_t g_meta;

static uint32_t area_wrap(uint32_t addr){
  if (addr >= LOG_AREA_END) addr = LOG_AREA_BASE + (addr - LOG_AREA_BASE) % LOG_AREA_SIZE;
  return addr;
}

static uint32_t next_rec_addr(uint32_t addr, uint16_t len){
  uint32_t need = sizeof(rec_hdr_t) + len;
  /* align to 2 bytes */
  if (need & 1u) need++;
  addr += need;
  if (addr >= LOG_AREA_END) addr = LOG_AREA_BASE + (addr - LOG_AREA_BASE) % LOG_AREA_SIZE;
  return addr;
}

static void read_meta(meta_t* m){
  uint8_t buf[sizeof(meta_t)];
  W25Q_Read(LOG_META_ADDR, buf, sizeof(buf));
  memcpy(m, buf, sizeof(meta_t));
}

static void write_meta(const meta_t* m){
  /* meta sits at beginning of sector -> erase then write */
  W25Q_EraseSector(LOG_META_ADDR);
  W25Q_Write(LOG_META_ADDR, (const uint8_t*)m, sizeof(meta_t));
}

static void format_area(void){
  /* Erase log sectors */
  for (uint32_t i=0;i<LOG_AREA_SECTORS;i++){
    W25Q_EraseSector(LOG_AREA_BASE + i*SECTOR_SZ);
  }
  g_meta.magic   = META_MAGIC;
  g_meta.version = VERSION;
  g_meta.rsv     = 0;
  g_meta.head    = LOG_AREA_BASE;
  g_meta.tail    = LOG_AREA_BASE;
  write_meta(&g_meta);
}

void Log_Init(void){
  read_meta(&g_meta);
  if (g_meta.magic != META_MAGIC || g_meta.version != VERSION ||
      g_meta.head < LOG_AREA_BASE || g_meta.head >= LOG_AREA_END ||
      g_meta.tail < LOG_AREA_BASE || g_meta.tail >= LOG_AREA_END) {
    format_area();
  }
}

bool Log_Empty(void){
  return g_meta.head == g_meta.tail;
}

/* compute used and free (circular) */
static uint32_t used_bytes(void){
  if (g_meta.head >= g_meta.tail) return g_meta.head - g_meta.tail;
  return LOG_AREA_SIZE - (g_meta.tail - g_meta.head);
}
static uint32_t free_bytes(void){
  /* leave one header margin to distinguish full/empty */
  uint32_t used = used_bytes();
  if (used >= LOG_AREA_SIZE) return 0;
  return LOG_AREA_SIZE - used - sizeof(rec_hdr_t);
}

/* read helper with wrap */
static void area_read(uint32_t addr, uint8_t* out, uint32_t len){
  if (addr + len <= LOG_AREA_END){
    W25Q_Read(addr, out, len);
    return;
  }
  uint32_t first = LOG_AREA_END - addr;
  W25Q_Read(addr, out, first);
  W25Q_Read(LOG_AREA_BASE, out+first, len-first);
}

/* write helper with wrap (assumes sectors already erased) */
static void area_write(uint32_t addr, const uint8_t* in, uint32_t len){
  if (addr + len <= LOG_AREA_END){
    W25Q_Write(addr, in, len);
    return;
  }
  uint32_t first = LOG_AREA_END - addr;
  W25Q_Write(addr, in, first);
  W25Q_Write(LOG_AREA_BASE, in+first, len-first);
}

static void ensure_erased_for_range(uint32_t addr, uint32_t len){
  /* erase any sector overlapped by [addr, addr+len) BEFORE writing */
  uint32_t end = addr + len;
  for (uint32_t a = addr; a < end; ){
    uint32_t sec = (a / SECTOR_SZ) * SECTOR_SZ;
    uint8_t probe = 0;
    W25Q_Read(sec, &probe, 1);
    if (probe != 0xFF){
      W25Q_EraseSector(sec);
    }
    a = sec + SECTOR_SZ;
  }
}

bool Log_Push(const uint8_t* buf, uint16_t len, uint8_t type, uint32_t ts){
  if (len == 0 || len > 1500) return false; /* guard */
  uint32_t need = sizeof(rec_hdr_t) + len;
  if (need & 1u) need++;

  /* ensure capacity: drop oldest until enough free */
  while (free_bytes() < need){
    /* drop one record */
    if (Log_Empty()) break; /* should not happen */
    Log_Drop();
  }

  rec_hdr_t h;
  h.magic = REC_MAGIC;
  h.len   = len;
  h.ts    = ts;
  h.type  = type;
  h.flags = 0;
  h.crc16 = flashlog_crc16_ccitt(buf, len);

  /* erase sectors as needed before writing */
  ensure_erased_for_range(g_meta.head, need);

  /* write header then payload with wrap handling */
  area_write(g_meta.head, (const uint8_t*)&h, sizeof(h));
  uint32_t payload_addr = area_wrap(g_meta.head + sizeof(h));
  area_write(payload_addr, buf, len);

  /* advance head */
  g_meta.head = next_rec_addr(g_meta.head, len);
  write_meta(&g_meta);
  return true;
}

bool Log_Peek(uint8_t* out, uint16_t max, uint16_t* len, uint8_t* type, uint32_t* ts){
  if (Log_Empty()) return false;

  rec_hdr_t h;
  area_read(g_meta.tail, (uint8_t*)&h, sizeof(h));
  if (h.magic != REC_MAGIC || h.len == 0 || h.len > max) {
    return false;
  }
  if (type) *type = h.type;
  if (ts)   *ts   = h.ts;
  area_read(area_wrap(g_meta.tail + sizeof(h)), out, h.len);
  if (len) *len = h.len;

  /* optional: verify CRC here if you want */
  return true;
}

void Log_Drop(void){
  if (Log_Empty()) return;
  rec_hdr_t h;
  area_read(g_meta.tail, (uint8_t*)&h, sizeof(h));
  uint32_t next = next_rec_addr(g_meta.tail, h.len);
  g_meta.tail = next;
  write_meta(&g_meta);
}

/* CRC16-CCITT (0xFFFF, poly 0x1021) */
uint16_t flashlog_crc16_ccitt(const uint8_t* data, uint32_t len){
  uint16_t crc = 0xFFFFu;
  for (uint32_t i=0;i<len;i++){
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b=0;b<8;b++){
      crc = (crc & 0x8000) ? (uint16_t)((crc<<1) ^ 0x1021u) : (uint16_t)(crc<<1);
    }
  }
  return crc;
}
