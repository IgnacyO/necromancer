/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 * SPDX-FileCopyrightText: 2026 Ignacy Oziero
 * SPDX-License-Identifier: Unlicense
 */
#ifndef COMPONENTS_IR_NEC_INCLUDE_IR_NEC_PARSER_H_
#define COMPONENTS_IR_NEC_INCLUDE_IR_NEC_PARSER_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include <stdint.h>

#define EXAMPLE_IR_NEC_DECODE_MARGIN 400     // Tolerance for parsing RMT symbols into bit stream

/**
 * @brief NEC timing spec
 */
#define NEC_LEADING_CODE_DURATION_0  9000
#define NEC_LEADING_CODE_DURATION_1  4500
#define NEC_PAYLOAD_ZERO_DURATION_0  560
#define NEC_PAYLOAD_ZERO_DURATION_1  560
#define NEC_PAYLOAD_ONE_DURATION_0   560
#define NEC_PAYLOAD_ONE_DURATION_1   1690
#define NEC_REPEAT_CODE_DURATION_0   9000
#define NEC_REPEAT_CODE_DURATION_1   2250

/**
 * @brief Check whether a RMT symbol represents NEC logic zero
 */
bool nec_parse_logic0(rmt_symbol_word_t *rmt_nec_symbols);
/**
 * @brief Check whether a RMT symbol represents NEC logic one
 */
bool nec_parse_logic1(rmt_symbol_word_t *rmt_nec_symbols);
/**S
 * @brief Decode RMT symbols into NEC address and command
 */
bool nec_parse_frame(rmt_symbol_word_t *rmt_nec_symbols, uint16_t *out_address, uint16_t *out_command);
/**
 * @brief Check whether the RMT symbols represent NEC repeat code
 */
bool nec_parse_frame_repeat(rmt_symbol_word_t *rmt_nec_symbols);

#endif /* COMPONENTS_IR_NEC_INCLUDE_IR_NEC_PARSER_H_ */
