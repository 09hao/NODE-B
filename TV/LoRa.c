/*
 * LoRa_4s.c — driver kept in sync with main_4s.c (Node B heartbeat every 4s)
 * No functional changes vs. your current LoRa.c; this file is provided to pair with main_4s.c.
 */
#include "LoRa.h"

/* ----------------------------------------------------------------------------- */
LoRa newLoRa(){
    LoRa new_LoRa;

    new_LoRa.frequency             = 433;
    new_LoRa.spredingFactor        = SF_9;
    new_LoRa.bandWidth             = BW_125KHz;
    new_LoRa.crcRate               = CR_4_5;
    new_LoRa.power                 = POWER_17db;
    new_LoRa.overCurrentProtection = 100;
    new_LoRa.preamble              = 14;

    return new_LoRa;
}
/* ----------------------------------------------------------------------------- */
void LoRa_reset(LoRa* _LoRa){
    HAL_GPIO_WritePin(_LoRa->reset_port, _LoRa->reset_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(_LoRa->reset_port, _LoRa->reset_pin, GPIO_PIN_SET);
    HAL_Delay(100);
}
/* ----------------------------------------------------------------------------- */
void LoRa_gotoMode(LoRa* _LoRa, int mode){
    uint8_t read = LoRa_read(_LoRa, RegOpMode);
    uint8_t data = read;

    if (mode == SLEEP_MODE)        { data = (read & 0xF8) | 0x00; _LoRa->current_mode = SLEEP_MODE; }
    else if (mode == STNBY_MODE)   { data = (read & 0xF8) | 0x01; _LoRa->current_mode = STNBY_MODE; }
    else if (mode == TRANSMIT_MODE){ data = (read & 0xF8) | 0x03; _LoRa->current_mode = TRANSMIT_MODE; }
    else if (mode == RXCONTIN_MODE){ data = (read & 0xF8) | 0x05; _LoRa->current_mode = RXCONTIN_MODE; }
    else if (mode == RXSINGLE_MODE){ data = (read & 0xF8) | 0x06; _LoRa->current_mode = RXSINGLE_MODE; }

    LoRa_write(_LoRa, RegOpMode, data);
}
/* ----------------------------------------------------------------------------- */
void LoRa_readReg(LoRa* _LoRa, uint8_t* address, uint16_t r_length, uint8_t* output, uint16_t w_length){
    HAL_GPIO_WritePin(_LoRa->CS_port, _LoRa->CS_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(_LoRa->hSPIx, address, r_length, TRANSMIT_TIMEOUT);
    while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY) {;}
    HAL_SPI_Receive(_LoRa->hSPIx, output, w_length, RECEIVE_TIMEOUT);
    while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY) {;}
    HAL_GPIO_WritePin(_LoRa->CS_port, _LoRa->CS_pin, GPIO_PIN_SET);
}
/* ----------------------------------------------------------------------------- */
void LoRa_writeReg(LoRa* _LoRa, uint8_t* address, uint16_t r_length, uint8_t* values, uint16_t w_length){
    HAL_GPIO_WritePin(_LoRa->CS_port, _LoRa->CS_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(_LoRa->hSPIx, address, r_length, TRANSMIT_TIMEOUT);
    while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY) {;}
    HAL_SPI_Transmit(_LoRa->hSPIx, values, w_length, TRANSMIT_TIMEOUT);
    while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY) {;}
    HAL_GPIO_WritePin(_LoRa->CS_port, _LoRa->CS_pin, GPIO_PIN_SET);
}
/* ----------------------------------------------------------------------------- */
void LoRa_setLowDaraRateOptimization(LoRa* _LoRa, uint8_t value){
    uint8_t read = LoRa_read(_LoRa, RegModemConfig3);
    uint8_t data = value ? (uint8_t)(read | 0x08) : (uint8_t)(read & 0xF7);
    LoRa_write(_LoRa, RegModemConfig3, data);
    HAL_Delay(10);
}
/* ----------------------------------------------------------------------------- */
void LoRa_setAutoLDO(LoRa* _LoRa){
    double BW[] = {7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125.0, 250.0, 500.0};
    LoRa_setLowDaraRateOptimization(_LoRa,
        (long)((1 << _LoRa->spredingFactor) / ((double)BW[_LoRa->bandWidth])) > 16.0);
}
/* ----------------------------------------------------------------------------- */
void LoRa_setFrequency(LoRa* _LoRa, int freq){
    // freq in MHz; FRF = freq * (2^19 / 32) = freq * 16384
    uint32_t F = (uint32_t)( (freq * 524288u) >> 5 ); // = freq * 16384

    LoRa_write(_LoRa, RegFrMsb, (uint8_t)(F >> 16)); HAL_Delay(5);
    LoRa_write(_LoRa, RegFrMid, (uint8_t)(F >>  8)); HAL_Delay(5);
    LoRa_write(_LoRa, RegFrLsb, (uint8_t)(F >>  0)); HAL_Delay(5);
}
/* ----------------------------------------------------------------------------- */
void LoRa_setSpreadingFactor(LoRa* _LoRa, int SF){
    if (SF > 12) SF = 12;
    if (SF < 7 ) SF = 7;

    uint8_t read = LoRa_read(_LoRa, RegModemConfig2);
    uint8_t data = (uint8_t)((SF << 4) | (read & 0x0F));
    LoRa_write(_LoRa, RegModemConfig2, data);
    HAL_Delay(10);

    LoRa_setAutoLDO(_LoRa);
}
/* ----------------------------------------------------------------------------- */
void LoRa_setPower(LoRa* _LoRa, uint8_t power){
    LoRa_write(_LoRa, RegPaConfig, power);
    HAL_Delay(10);
}

// Ch?n công su?t theo dBm (2..17 dBm, và 20 dBm d?c bi?t cho PA_BOOST)
void LoRa_setPower_dBm(LoRa* _LoRa, int8_t dBm){
    // d?m b?o dang ? Standby d? ghi thanh ghi an toàn
    if (_LoRa->current_mode != STNBY_MODE) LoRa_gotoMode(_LoRa, STNBY_MODE);

    if (dBm >= 20){
        // 20 dBm: c?n PaDac high-power + OCP cao, OutputPower=15
        LoRa_write(_LoRa, RegPaDac, 0x87);        // High power PaDac
        LoRa_setOCP(_LoRa, 140);                  // kho?ng 140 mA
        LoRa_write(_LoRa, RegPaConfig, 0xFF);     // PA_BOOST + MaxPower=111 + Out=15
    } else {
        // Gi?i h?n v? 2..17 dBm theo PA_BOOST
        if (dBm < 2)  dBm = 2;
        if (dBm > 17) dBm = 17;
        uint8_t out = (uint8_t)(dBm - 2);         // 0..15  -> 2..17 dBm
        uint8_t pac = (uint8_t)(0x80 | (0x7 << 4) | out); // PA_BOOST + MaxPower=111 + Out
        LoRa_write(_LoRa, RegPaConfig, pac);
        LoRa_write(_LoRa, RegPaDac, 0x84);        // PaDac normal
        LoRa_setOCP(_LoRa, 100);                  // OCP m?c d?nh c?a b?n
    }
    HAL_Delay(10);
}

/* ----------------------------------------------------------------------------- */
void LoRa_setOCP(LoRa* _LoRa, uint8_t current){
    if (current < 45)  current = 45;
    if (current > 240) current = 240;

    uint8_t OcpTrim;
    if (current <= 120) OcpTrim = (current - 45)/5;
    else                OcpTrim = (current + 30)/10;

    OcpTrim = (uint8_t)(OcpTrim + (1 << 5));   // enable OCP + trim
    LoRa_write(_LoRa, RegOcp, OcpTrim);
    HAL_Delay(10);
}
/* ----------------------------------------------------------------------------- */
void LoRa_setTOMsb_setCRCon(LoRa* _LoRa){
    uint8_t read = LoRa_read(_LoRa, RegModemConfig2);
    uint8_t data = (uint8_t)(read | 0x07); // RxPayloadCrcOn[2]=1, SymbTimeoutMsb[1:0]=11
    LoRa_write(_LoRa, RegModemConfig2, data);
    HAL_Delay(10);
}
/* ----------------------------------------------------------------------------- */
void LoRa_setSyncWord(LoRa* _LoRa, uint8_t syncword){
    LoRa_write(_LoRa, RegSyncWord, syncword);
    HAL_Delay(10);
}
/* ----------------------------------------------------------------------------- */
uint8_t LoRa_read(LoRa* _LoRa, uint8_t address){
    uint8_t read_data;
    uint8_t data_addr = address & 0x7F;
    LoRa_readReg(_LoRa, &data_addr, 1, &read_data, 1);
    return read_data;
}
/* ----------------------------------------------------------------------------- */
void LoRa_write(LoRa* _LoRa, uint8_t address, uint8_t value){
    uint8_t addr = address | 0x80;
    LoRa_writeReg(_LoRa, &addr, 1, &value, 1);
}
/* ----------------------------------------------------------------------------- */
void LoRa_BurstWrite(LoRa* _LoRa, uint8_t address, uint8_t *value, uint8_t length){
    uint8_t addr = address | 0x80;

    HAL_GPIO_WritePin(_LoRa->CS_port, _LoRa->CS_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(_LoRa->hSPIx, &addr, 1, TRANSMIT_TIMEOUT);
    while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY) {;}
    HAL_SPI_Transmit(_LoRa->hSPIx, value, length, TRANSMIT_TIMEOUT);
    while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY) {;}
    HAL_GPIO_WritePin(_LoRa->CS_port, _LoRa->CS_pin, GPIO_PIN_SET);
}
/* ----------------------------------------------------------------------------- */
uint8_t LoRa_isvalid(LoRa* _LoRa){
    (void)_LoRa;
    return 1;
}
/* ----------------------------------------------------------------------------- */
uint8_t LoRa_transmit(LoRa* _LoRa, uint8_t* data, uint8_t length, uint16_t timeout){
    uint8_t read;
    int prev_mode = _LoRa->current_mode;

    read = LoRa_read(_LoRa, RegFiFoTxBaseAddr);
    LoRa_write(_LoRa, RegFiFoAddPtr, read);
    LoRa_write(_LoRa, RegPayloadLength, length);
    LoRa_BurstWrite(_LoRa, RegFiFo, data, length);

    LoRa_gotoMode(_LoRa, TRANSMIT_MODE);

    while (1){
        read = LoRa_read(_LoRa, RegIrqFlags);
        if ((read & 0x08) != 0){                 // TxDone
            LoRa_write(_LoRa, RegIrqFlags, 0xFF);
            LoRa_gotoMode(_LoRa, prev_mode);     // kh i ph?c mode tru?c d 
            return 1;
        } else {
            if (--timeout == 0){
                LoRa_gotoMode(_LoRa, prev_mode);
                return 0;
            }
        }
        HAL_Delay(1);
    }
}
/* ----------------------------------------------------------------------------- */
void LoRa_startReceiving(LoRa* _LoRa){
    // Reset FIFO RX & map DIO0 = RxDone
    LoRa_write(_LoRa, RegFiFoRxBaseAddr, 0x00);
    LoRa_write(_LoRa, RegFiFoAddPtr,     0x00);

    uint8_t map1 = LoRa_read(_LoRa, RegDioMapping1);
    map1 &= 0x3F;                      // DIO0[7:6] = 00 -> RxDone
    LoRa_write(_LoRa, RegDioMapping1, map1);

    LoRa_write(_LoRa, RegIrqFlags, 0xFF);   // clear all IRQs
    LoRa_gotoMode(_LoRa, RXCONTIN_MODE);    // RX li n t?c
}
/* ----------------------------------------------------------------------------- */
uint8_t LoRa_receive(LoRa* _LoRa, uint8_t* data, uint8_t length){
    uint8_t flags = LoRa_read(_LoRa, RegIrqFlags);
    if ((flags & 0x40) == 0) return 0;            // chua c  g i

    if (flags & 0x20){                             // CRC PHY error
        LoRa_write(_LoRa, RegIrqFlags, 0xFF);
        uint8_t base = LoRa_read(_LoRa, RegFiFoRxBaseAddr);
        LoRa_write(_LoRa, RegFiFoAddPtr, base);
        if (_LoRa->current_mode != RXCONTIN_MODE) LoRa_gotoMode(_LoRa, RXCONTIN_MODE);
        return 0;
    }

    uint8_t n   = LoRa_read(_LoRa, RegRxNbBytes);
    uint8_t cur = LoRa_read(_LoRa, RegFiFoRxCurrentAddr);
    LoRa_write(_LoRa, RegFiFoAddPtr, cur);

    uint8_t cnt = (n <= length) ? n : length;
    for (uint8_t i = 0; i < cnt; i++) data[i] = LoRa_read(_LoRa, RegFiFo);
    for (uint8_t i = cnt; i < n; i++) (void)LoRa_read(_LoRa, RegFiFo); // x? ph?n du

    LoRa_write(_LoRa, RegIrqFlags, 0xFF);
    if (_LoRa->current_mode != RXCONTIN_MODE) LoRa_gotoMode(_LoRa, RXCONTIN_MODE);
    return cnt;
}
/* ----------------------------------------------------------------------------- */
uint16_t LoRa_init(LoRa* _LoRa){
    if (!LoRa_isvalid(_LoRa)) return LORA_UNAVAILABLE;

    // Sleep
    HAL_Delay(10);

    // Turn on LoRa mode
    uint8_t read = LoRa_read(_LoRa, RegOpMode);
    LoRa_write(_LoRa, RegOpMode, (uint8_t)(read | 0x80));
    HAL_Delay(100);

    // Frequency / Power / OCP / LNA
    LoRa_setFrequency(_LoRa, _LoRa->frequency);
    LoRa_setPower(_LoRa, _LoRa->power);
    LoRa_setOCP(_LoRa, _LoRa->overCurrentProtection);
    LoRa_write(_LoRa, RegLna, 0x23);

    // CRC on + TimeoutMSB, SF, TimeoutLSB
    LoRa_setTOMsb_setCRCon(_LoRa);
    LoRa_setSpreadingFactor(_LoRa, _LoRa->spredingFactor);
    LoRa_write(_LoRa, RegSymbTimeoutL, 0xFF);

    // BW / CR / explicit header
    uint8_t mc1 = (uint8_t)((_LoRa->bandWidth << 4) | (_LoRa->crcRate << 1)); // Implicit=0
    LoRa_write(_LoRa, RegModemConfig1, mc1);
    LoRa_setAutoLDO(_LoRa);

    // Preamble
    LoRa_write(_LoRa, RegPreambleMsb, (uint8_t)(_LoRa->preamble >> 8));
    LoRa_write(_LoRa, RegPreambleLsb, (uint8_t)(_LoRa->preamble >> 0));

    // DIO0 = RxDone
    uint8_t map1 = LoRa_read(_LoRa, RegDioMapping1);
    map1 &= 0x3F;                                   // DIO0[7:6] = 00
    LoRa_write(_LoRa, RegDioMapping1, map1);

    // Standby & check chip
    _LoRa->current_mode = STNBY_MODE;
    HAL_Delay(10);

    read = LoRa_read(_LoRa, RegVersion);
    if (read == 0x12) return LORA_OK;
    return LORA_NOT_FOUND;
}
