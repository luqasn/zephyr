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


int m2t_map(int x, int y, int m)
{
	if (x < y) {
		return -1;
	}
	return y * m + x;
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

	int ret = sys_bitarray_test_bit(bitmap, index, &lost_frm_bit);
	__ASSERT_NO_MSG(ret == 0);
	return lost_frm_bit != 0;
}

void bit_set_new(struct sys_bitarray *bitmap, int index)
{
    int ret = sys_bitarray_set_bit(bitmap, index);
    __ASSERT_NO_MSG(ret == 0);
}

void bit_clr_new(struct sys_bitarray *bitmap, int index)
{
    int ret = sys_bitarray_clear_bit(bitmap, index);
    __ASSERT_NO_MSG(ret == 0);
}

int bit_count_ones_new(struct sys_bitarray *bitmap, int index)
{
	size_t count;
	int ret = sys_bitarray_popcount_region(bitmap, index + 1, 0, &count);
	__ASSERT_NO_MSG(ret == 0);
	int b = count;
	return b;
}

void bit_xor_new(struct sys_bitarray *des, struct sys_bitarray *src, int size)
{
    int ret = sys_bitarray_xor(des, src, size, 0);
    __ASSERT_NO_MSG(ret == 0);
}

bool bit_is_all_clear_new(struct sys_bitarray *bitmap, int size)
{
	return sys_bitarray_is_region_cleared(bitmap, size, 0);
}

int bit_ffs_new(struct sys_bitarray *bitmap, int size)
{
	return bit_fns_new(bitmap, size, 1);
}

int bit_fns_new(struct sys_bitarray *bitmap, int size, int n)
{
    size_t found_at;
    int ret = sys_bitarray_find_nth_set(bitmap, n, size, 0, &found_at);
    __ASSERT_NO_MSG(ret >= 0);
    if (ret == 1) {
    	return -1;
    } else {
    	return found_at;
    }
}


void bit_clear_all_new(struct sys_bitarray *bitmap, int size) {
	int ret = sys_bitarray_clear_region(bitmap, size, 0);
	__ASSERT_NO_MSG(ret == 0);
}