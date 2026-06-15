/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 * SPDX-FileCopyrightText: 2026 Ignacy Oziero
 * SPDX-License-Identifier: Unlicense
 */
#include "ir_nec_parser.h"

/**
 * @brief Check whether a duration is within expected range
 */
static inline bool nec_check_in_range(uint32_t signal_duration, uint32_t spec_duration)
{
    return (signal_duration < (spec_duration + EXAMPLE_IR_NEC_DECODE_MARGIN)) &&
           (signal_duration > (spec_duration - EXAMPLE_IR_NEC_DECODE_MARGIN));
}

bool nec_parse_logic0(rmt_symbol_word_t *rmt_nec_symbols)
{
    return nec_check_in_range(rmt_nec_symbols->duration0, NEC_PAYLOAD_ZERO_DURATION_0) &&
           nec_check_in_range(rmt_nec_symbols->duration1, NEC_PAYLOAD_ZERO_DURATION_1);
}

bool nec_parse_logic1(rmt_symbol_word_t *rmt_nec_symbols)
{
    return nec_check_in_range(rmt_nec_symbols->duration0, NEC_PAYLOAD_ONE_DURATION_0) &&
           nec_check_in_range(rmt_nec_symbols->duration1, NEC_PAYLOAD_ONE_DURATION_1);
}

/*static inline void nec_set_bit(uint8_t *byte, int bit_index, bool lsb)
{
    if (lsb) {
		*byte |= (1 << bit_index);
    } else {
        *byte |= (1 << (7 - bit_index));
    }
}*/

static uint8_t reverse_byte(uint8_t x)
{
    x = (x & 0xF0) >> 4 | (x & 0x0F) << 4;
    x = (x & 0xCC) >> 2 | (x & 0x33) << 2;
    x = (x & 0xAA) >> 1 | (x & 0x55) << 1;
    return x;
}

bool nec_parse_frame(rmt_symbol_word_t *rmt_nec_symbols, bool err_cor, bool lsb_format, nec_scan_code_t *ret)
{
    if (!rmt_nec_symbols || !ret) return false;

    rmt_symbol_word_t *cur = rmt_nec_symbols;

    // Check leader
    bool leader_ok =
    (nec_check_in_range(cur->duration0, NEC_LEADING_CODE_DURATION_0) &&
     nec_check_in_range(cur->duration1, NEC_LEADING_CODE_DURATION_1)) ||

    (nec_check_in_range(cur->duration0, NECX_LEADING_CODE_DURATION_0) &&
     nec_check_in_range(cur->duration1, NECX_LEADING_CODE_DURATION_1));

	if (!leader_ok) {
	    return false;
	}

    cur++;

    uint8_t addr_l = 0;
    uint8_t addr_h = 0;
    uint8_t cmd_l  = 0;
    uint8_t cmd_h  = 0;

    // Decode 16-bit address (LSB-first per byte)
    for (int bit = 0; bit < 8; bit++) {
        if (nec_parse_logic1(cur)) {
            addr_l |= (1 << bit);
        } else if (!nec_parse_logic0(cur)) {
            return false;
        }
        cur++;
    }

    for (int bit = 0; bit < 8; bit++) {
        if (nec_parse_logic1(cur)) {
            addr_h |= (1 << bit);
        } else if (!nec_parse_logic0(cur)) {
            return false;
        }
        cur++;
    }

    //Decode 16-bit command
    for (int bit = 0; bit < 8; bit++) {
        if (nec_parse_logic1(cur)) {
            cmd_l |= (1 << bit);
        } else if (!nec_parse_logic0(cur)) {
            return false;
        }
        cur++;
    }

    for (int bit = 0; bit < 8; bit++) {
        if (nec_parse_logic1(cur)) {
            cmd_h |= (1 << bit);
        } else if (!nec_parse_logic0(cur)) {
            return false;
        }
        cur++;
    }

    // Standard NEC: command is inverted in extended frame
    // or address/command checksum depending on variant

    
    if (err_cor) {
		// Validate command
		if ((uint8_t)(cmd_l ^ cmd_h) != 0xFF) {
		    return false;
		}
		
		// Validate address (classic NEC, not extended)
		if ((uint8_t)(addr_l ^ addr_h) != 0xFF) {
		    return false;
		}
	}
	
	
    
	if (!lsb_format) {
	    addr_l = reverse_byte(addr_l);
	    addr_h = reverse_byte(addr_h);
	    cmd_l  = reverse_byte(cmd_l);
	    cmd_h  = reverse_byte(cmd_h);
	}


    // Save result
    ret->address = ((uint16_t)addr_h << 8) | addr_l;
	ret->command = ((uint16_t)cmd_h << 8) | cmd_l;

    return true;
}

bool nec_parse_frame_repeat(rmt_symbol_word_t *rmt_nec_symbols)
{
    return nec_check_in_range(rmt_nec_symbols->duration0, NEC_REPEAT_CODE_DURATION_0) &&
           nec_check_in_range(rmt_nec_symbols->duration1, NEC_REPEAT_CODE_DURATION_1);
}

bool parse_received_symbols_to_nec(rmt_symbol_word_t *rmt_nec_symbols, size_t num_symbols, bool err_cor, nec_scan_code_t *ret) {
	 printf("NEC frame start---\r\n");
    for (size_t i = 0; i < num_symbols; i++) {
        printf("{%d:%d},{%d:%d}\r\n", rmt_nec_symbols[i].level0, rmt_nec_symbols[i].duration0,
               rmt_nec_symbols[i].level1, rmt_nec_symbols[i].duration1);
    }
    printf("---NEC frame end: ");
    // decode RMT symbols
    switch (num_symbols) {
    case 34: // NEC normal frame
        if (nec_parse_frame(rmt_nec_symbols, err_cor, 1, ret)) {
			return true;
        }
        break;
    case 2: // NEC repeat frame
        if (nec_parse_frame_repeat(rmt_nec_symbols)) {
            printf("Repeat\n");
        }
        break;
    default:
        printf("Unknown NEC frame\r\n\r\n");
        break;
    }
    return false;
}