/*
 * Copyright (c) 2022 Jiapeng Li
 *
 * Original source: https://github.com/JiapengLi/LoRaWANFragmentedDataBlockTransportAlgorithm
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/arch/common/ffs.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include "bitmap.h"

bool bit_get(bm_t *bitmap, int index)
{
	return ((bitmap[index >> BM_OFST] & (1 << (index % BM_UNIT))) != 0);
}

void bit_set(bm_t *bitmap, int index)
{
	bitmap[index >> BM_OFST] |= ((bm_t)1 << (index % BM_UNIT));
}

void bit_clr(bm_t *bitmap, int index)
{
	bitmap[index >> BM_OFST] &= ~(1 << (index % BM_UNIT));
}

int bit_count_ones(bm_t *bitmap, int index)
{
	int i, cnt;
	bm_t val;

	cnt = 0;
	for (i = 0; i < (index >> BM_OFST); i++) {
		cnt += POPCOUNT(bitmap[i]);
	}
	val = bitmap[index >> BM_OFST] & ((bm_t)(-1) >> (BM_UNIT - 1 - index % BM_UNIT));
	cnt += POPCOUNT(val);
	return cnt;
}

int bit_count_ones_new(struct sys_bitarray *bitmap, int index)
{
	size_t count;
	int ret = sys_bitarray_popcount_region(bitmap, index+1, 0, &count);
	__ASSERT_NO_MSG(ret == 0);
	return count;
}

int bit_ffs(bm_t *bitmap, int size)
{
#if 0
	int i, j;
	for (i = 0; i < ((size + BM_UNIT - 1) >> BM_OFST); i++) {
		for (j = 0; j < BM_UNIT; j++) {
			if (bitmap[i] & (1 << j)) {
				return ((i << BM_OFST) + j);
			}
		}
	}
	return -1;
#else
	int i;
	int len;

	len = ((size + BM_UNIT - 1) >> BM_OFST);
	for (i = 0; i < len; i++) {
		if (bitmap[i] != 0) {
			return (i << BM_OFST) + find_lsb_set(bitmap[i]) - 1;
		}
	}
	return -1;
#endif
}

/* find the nth set */
int bit_fns(bm_t *bitmap, int size, int n)
{
	/*
	int i, j, cnt;
	cnt = 0;
	for (i = 0; i < ((size + BM_UNIT - 1) >> BM_OFST); i++) {
		for (j = 0; j < BM_UNIT; j++) {
			if (bitmap[i] & (1 << j)) {
				cnt++;
				if (cnt == n) {
					return ((i << BM_OFST) + j);
				}
			}
		}
	}
	return -1;
	*/
	int i, j, cnt;
	cnt = 0;
	for (i = 0; i < ((size + BM_UNIT - 1) >> BM_OFST); i++) {
		cnt += POPCOUNT(bitmap[i]);
		if (cnt >= n) {
			for (j = BM_UNIT - 1; j >= 0; j--) {
				if (bitmap[i] & (1 << j)) {
					if (cnt == n) {
						return ((i << BM_OFST) + j);
					}
					cnt--;
				}
			}
		}
	}
	return -1;
}

void bit_xor(bm_t *des, bm_t *src, int size)
{
	int i;
	int len;

	len = ((size + BM_UNIT - 1) >> BM_OFST);
	for (i = 0; i < len; i++) {
		des[i] ^= src[i];
	}
}

bool bit_is_all_clear(bm_t *bitmap, int size)
{
	int i;
	int len;

	len = ((size + BM_UNIT - 1) >> BM_OFST);
	for (i = 0; i < len; i++) {
		if (bitmap[i]) {
			return false;
		}
	}
	return true;
}

void bit_clear_all(bm_t *bitmap, int size)
{
	int i;
	int len;

	len = ((size + BM_UNIT - 1) >> BM_OFST);
	for (i = 0; i < len; i++) {
		bitmap[i] = 0;
	}
}

int m2t_map(int x, int y, int m)
{
	if (x < y) {
		return -1;
	}
	/* doesn't check to speed up the process */
	/*
	if ((x >= m) || (y >= m)) {
		return -1;
	}
	*/
	return (y + 1) * (m + m - y) / 2 - (m - x);
}

bool m2t_get(bm_t *m2tbm, int x, int y, int m)
{
	if (x < y) {
		return false;
	}
	return bit_get(m2tbm, (y + 1) * (m + m - y) / 2 - (m - x));
}

void m2t_set(bm_t *m2tbm, int x, int y, int m)
{
	if (x < y) {
		return;
	}
	bit_set(m2tbm, (y + 1) * (m + m - y) / 2 - (m - x));
}

void m2t_clr(bm_t *m2tbm, int x, int y, int m)
{
	if (x < y) {
		return;
	}
	bit_clr(m2tbm, (y + 1) * (m + m - y) / 2 - (m - x));
}

bool m2t_get_new(struct sys_bitarray *m2tbm, int x, int y, int m)
{
	int bit;

	if (x < y) {
		return false;
	}

	sys_bitarray_test_bit(m2tbm, m2t_map(x,y,m), &bit);

	return bit;
}

void m2t_set_new(struct sys_bitarray *m2tbm, int x, int y, int m)
{
	if (x < y) {
		return;
	}
	sys_bitarray_set_bit(m2tbm, m2t_map(x,y,m));
}

void m2t_clr_new(struct sys_bitarray *m2tbm, int x, int y, int m)
{
	if (x < y) {
		return;
	}
	sys_bitarray_clear_bit(m2tbm, m2t_map(x,y,m));
}

bool bit_get_new(struct sys_bitarray *bitmap, int index)
{
	int lost_frm_bit;

	sys_bitarray_test_bit(bitmap, index, &lost_frm_bit);
	return lost_frm_bit;
}

void bit_set_new(struct sys_bitarray *bitmap, int index)
{
    sys_bitarray_set_bit(bitmap, index);
}

void bit_clr_new(struct sys_bitarray *bitmap, int index)
{
    sys_bitarray_clear_bit(bitmap, index);
}

int bit_fns_new(struct sys_bitarray *bitmap, int size, int n)
{
    size_t frame_index;
    int ret = sys_bitarray_find_nth_set(bitmap, n, size, 0, &frame_index);
    if (ret == 1) {
        return -1;
    }
    __ASSERT_NO_MSG(ret == 0);
    return frame_index;
}
