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

void __cmp(bm_t *bm, struct sys_bitarray *bm_new, int count)
{
	bool a,b;
	for (int i = 0; i < count; i++) {
		a = bit_get(bm, i);
		b = bit_get_new(bm_new, i);
		__ASSERT(a == b, "Mismatch at %d", i);
	}
}

void __cpy(struct sys_bitarray *from, bm_t *to, int count)
{
	for (int i = 0; i < count; i++) {
		bool a = bit_get_new(from, i);
		if (a) {
			bit_set(to, i);
		} else {
			bit_clr(to, i);
		}
	}
}

void __cpy_all(struct sys_bitarray *from, bm_t *to)
{
	__cpy(from, to, from->num_bits);
}

void __cmp_all(bm_t *bm, struct sys_bitarray *bm_new)
{
	__cmp(bm, bm_new, bm_new->num_bits);
}

bm_t* __clone(struct sys_bitarray *bitmap) {
	int len = ((bitmap->num_bits + BM_UNIT - 1) >> BM_OFST);
	bm_t* clone = (bm_t*)malloc(len * sizeof(bm_t));
	__cpy(bitmap, clone, bitmap->num_bits);
	return clone;
}

void __frag_dec_log_bits_new(struct sys_bitarray *bitmap, int len)
{
	for (int i = 0; i < len; i++) {
		if (i % (sizeof(bitmap->bundles[0]) * 8) == 0) {
			printf("\n");
		}
		if (bit_get_new(bitmap, i)) {
			printf("1");
		} else {
			printf("0");
		}
	}
	printf("\n");
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

int bit_count_ones_new(struct sys_bitarray *bitmap, int index)
{
	bm_t *clone = __clone(bitmap);
	int a = bit_count_ones(clone, index);
	size_t count;
	int ret = sys_bitarray_popcount_region(bitmap, index+1, 0, &count);
	__ASSERT_NO_MSG(ret == 0);
	int b = count;
	__ASSERT(a == b, "popcount wrong");
	return b;
}

void bit_xor_new(struct sys_bitarray *des, struct sys_bitarray *src, int size)
{
	bm_t *clone_des = __clone(des);
	bm_t *clone_src = __clone(src);
	bit_xor(clone_des, clone_src, size);
    sys_bitarray_xor(des, src, size, 0);
    __cmp_all(clone_des, des);
}

bool bit_is_all_clear_new(struct sys_bitarray *bitmap, int size)
{
	bm_t *clone = __clone(bitmap);
	bool a = bit_is_all_clear(clone, size);
	bool b = sys_bitarray_is_region_cleared(bitmap, size, 0);
	__ASSERT(a == b, "all_clear wrong");
	return b;
}

int bit_ffs_new(struct sys_bitarray *bitmap, int size)
{
	bm_t *clone = __clone(bitmap);
	int a = bit_ffs(clone, size);
	int b = bit_fns_new(bitmap, size, 1);
	__frag_dec_log_bits_new(bitmap, size);
	__ASSERT(a == b, "bit count wrong: old = %d, new = %d, size = %d", a, b, size);

    return b;
}

int bit_fns_new(struct sys_bitarray *bitmap, int size, int n)
{
    bm_t *clone = __clone(bitmap);
    int a = bit_fns(clone, size, n);
    int b;
    size_t found_at;
    int ret = sys_bitarray_find_nth_set(bitmap, n, size, 0, &found_at);
    __ASSERT_NO_MSG(ret >= 0);
    if (ret == 1) {
    	b = -1;
    } else {
    	b = found_at;
    }
    __frag_dec_log_bits_new(bitmap, size);
    __ASSERT(a == b, "bit count wrong: old = %d, new = %d, size = %d, n = %d", a, b, size, n);
    return b;
}


void bit_clear_all_new(struct sys_bitarray *bitmap, int size) {
    bm_t *clone = __clone(bitmap);
	bit_clear_all(clone, size);
	sys_bitarray_clear_region(bitmap, size, 0);
	__cmp(clone, bitmap, size);
}