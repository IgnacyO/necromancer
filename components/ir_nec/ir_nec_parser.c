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

/*bool nec_parse_frame(rmt_symbol_word_t *rmt_nec_symbols, ir_nec_scan_code_t *nec_frame)
{
    rmt_symbol_word_t *cur = rmt_nec_symbols;
    uint16_t address = 0;
    uint16_t command = 0;
    bool valid_leading_code = nec_check_in_range(cur->duration0, NEC_LEADING_CODE_DURATION_0) &&
                              nec_check_in_range(cur->duration1, NEC_LEADING_CODE_DURATION_1);
    if (!valid_leading_code) {
        return false;
    }
    cur++;
    for (int i = 0; i < 16; i++) {
        if (nec_parse_logic1(cur)) {
            address |= 1 << i;
        } else if (nec_parse_logic0(cur)) {
            address &= ~(1 << i);
        } else {
            return false;
        }
        cur++;
    }
    for (int i = 0; i < 16; i++) {
        if (nec_parse_logic1(cur)) {
            command |= 1 << i;
        } else if (nec_parse_logic0(cur)) {
            command &= ~(1 << i);
        } else {
            return false;
        }
        cur++;
    }
    // save address and command
    nec_frame->address = address;
    nec_frame->command = command;
    return true;
}*/

bool nec_parse_frame(rmt_symbol_word_t *s, ir_nec_scan_code_t *out)
{
    if (!s || !out) return false;

    rmt_symbol_word_t *cur = s;

    // Check leader
    if (!nec_check_in_range(cur->duration0, NEC_LEADING_CODE_DURATION_0) ||
        !nec_check_in_range(cur->duration1, NEC_LEADING_CODE_DURATION_1)) {
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

    // Combine bytes (NEC standard)
    uint16_t address = ((uint16_t)addr_h << 8) | addr_l;
    uint16_t command = ((uint16_t)cmd_h << 8) | cmd_l;

    // Standard NEC: command is inverted in extended frame
    // or address/command checksum depending on variant

    uint8_t cmd_low = cmd_l;
    uint8_t cmd_high = cmd_h;

    if ((uint8_t)(cmd_low ^ cmd_high) != 0xFF) {
        // not strict NEC extended format, but many remotes ignore this
    }

    // Save result
    out->address = address;
    out->command = (cmd_h << 8) | cmd_l;

    return true;
}

bool nec_parse_frame_repeat(rmt_symbol_word_t *rmt_nec_symbols)
{
    return nec_check_in_range(rmt_nec_symbols->duration0, NEC_REPEAT_CODE_DURATION_0) &&
           nec_check_in_range(rmt_nec_symbols->duration1, NEC_REPEAT_CODE_DURATION_1);
}

bool parse_received_symbols_to_nec(rmt_symbol_word_t *rmt_symbols, size_t num_symbols, ir_nec_scan_code_t *res) {
	 printf("NEC frame start---\r\n");
    for (size_t i = 0; i < num_symbols; i++) {
        printf("{%d:%d},{%d:%d}\r\n", rmt_symbols[i].level0, rmt_symbols[i].duration0,
               rmt_symbols[i].level1, rmt_symbols[i].duration1);
    }
    printf("---NEC frame end: ");
    // decode RMT symbols
    switch (num_symbols) {
    case 34: // NEC normal frame
        if (nec_parse_frame(rmt_symbols, res)) {
			return true;
        }
        break;
    case 2: // NEC repeat frame
        if (nec_parse_frame_repeat(rmt_symbols)) {
            printf("Repeat\n");
        }
        break;
    default:
        printf("Unknown NEC frame\r\n\r\n");
        break;
    }
    return false;
}