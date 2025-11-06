/* uplink_patch.h — helpers to patch JSON "hist" and CRC in-place */
#ifndef UPLINK_PATCH_H
#define UPLINK_PATCH_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Returns 1 on success, 0 on failure */
int uplink_set_hist_and_fix_crc(char* json, size_t buflen, int hist);

/* Compute CRC16-CCITT same as in main */
uint16_t uplink_crc16_ccitt(const uint8_t* data, size_t len);

#ifdef __cplusplus
}
#endif
#endif
