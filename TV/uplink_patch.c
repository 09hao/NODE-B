#include "uplink_patch.h"
#include <string.h>
#include <stdio.h>

/* Keil/ARMCC5 doesn't have strnlen in the default C library.
   Provide a tiny safe replacement. */
static size_t my_strnlen(const char* s, size_t maxn){
  size_t n = 0;
  if (!s) return 0;
  while (n < maxn && s[n] != '\0') n++;
  return n;
}

static uint16_t crc16_acc_byte(uint16_t crc, uint8_t byte){
  crc ^= (uint16_t)byte << 8;
  for(int b=0;b<8;b++){
    crc = (crc & 0x8000) ? (uint16_t)((crc<<1) ^ 0x1021) : (uint16_t)(crc<<1);
  }
  return crc;
}

uint16_t uplink_crc16_ccitt(const uint8_t* data, size_t len){
  uint16_t crc = 0xFFFF;
  for(size_t i=0;i<len;i++){
    crc = crc16_acc_byte(crc, data[i]);
  }
  return crc;
}

/* json shape expected (order arbitrary but has these tail tokens):
   ... ,"hist":0,"crc":"ABCD"}
   We will: 1) set hist to 0/1; 2) recompute CRC over base (without ,"crc":...).
*/
int uplink_set_hist_and_fix_crc(char* json, size_t buflen, int hist){
  if (!json) return 0;
  size_t L = my_strnlen(json, buflen);
  if (L==buflen) return 0; /* not terminated within buffer */

  char* crcpos = strstr(json, ",\"crc\":\"");
  if (!crcpos) return 0;

  char* histpos = strstr(json, "\"hist\":");
  if (!histpos) return 0;
  char* val = histpos + 7; /* points to number */
  if (*val=='0' || *val=='1'){
    *val = hist ? '1' : '0';
  } else {
    return 0;
  }

  /* Recompute CRC over base part ending right before ,\"crc\":\" */
  size_t baselen = (size_t)(crcpos - json);
  if (baselen + 2 > buflen) return 0;

  /* Build a tiny temp: base + '}' to match original crc scope */
  char tmp[384];
  if (baselen + 2 > sizeof(tmp)) return 0;
  memcpy(tmp, json, baselen);
  tmp[baselen] = '}';
  tmp[baselen+1] = '\0';

  uint16_t crc = uplink_crc16_ccitt((const uint8_t*)tmp, baselen+1);

  /* Overwrite the CRC part */
  snprintf(crcpos, buflen - (size_t)(crcpos - json), ",\"crc\":\"%04X\"}", crc);
  return 1;
}
