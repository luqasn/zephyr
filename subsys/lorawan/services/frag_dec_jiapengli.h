/*
 * Copyright (c) 2022 Jiapeng Li
 *
 * Original source: https://github.com/JiapengLi/LoRaWANFragmentedDataBlockTransportAlgorithm
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef FRAG_DEC_H_
#define FRAG_DEC_H_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "bitmap.h"

/*
 * https://github.com/brocaar/lorawan/blob/master/applayer/fragmentation/encode.go
 * https://github.com/brocaar/lorawan/blob/master/applayer/fragmentation/encode_test.go
 */

#define FRAG_DEC_ONGOING                 (-1)
#define FRAG_DEC_ERR_INVALID_FRAME       (-2)
#define FRAG_DEC_ERR_TOO_MANY_FRAME_LOST (-3)
#define FRAG_DEC_ERR_1                   (-4)
#define FRAG_DEC_ERR_2                   (-5)

typedef int (*flash_rd_t)(uint32_t addr, uint8_t *buf, uint32_t len);
typedef int (*flash_wr_t)(uint32_t addr, const uint8_t *buf, uint32_t len);

typedef struct {
	uint8_t *dt;
	uint32_t maxlen;
	/** number of fragments */
	uint16_t nb;
	uint8_t size;
	uint16_t tolerence;
	flash_rd_t frd_func;
	flash_wr_t fwr_func;
} frag_dec_cfg_t;

typedef enum {
	/* wait uncoded fragmentations */
	FRAG_DEC_STA_UNCODED,
	/* wait coded fragmentations, uncoded frags are processed as coded ones */
	FRAG_DEC_STA_CODED,
	FRAG_DEC_STA_DONE,
} frag_dec_sta_t;

typedef struct {
	frag_dec_cfg_t cfg;
	frag_dec_sta_t sta;

	uint16_t lost_frm_count;
	uint16_t filled_lost_frm_count;

	uint8_t *row_data_buf;
	uint8_t *xor_row_data_buf;
} frag_dec_t;

int frag_dec_init(frag_dec_t *obj);
int frag_dec(frag_dec_t *obj, uint16_t frameCounter, const uint8_t *buf, int len);

int m2t_map(int x, int y, int m);

#endif /* FRAG_DEC_H_ */
