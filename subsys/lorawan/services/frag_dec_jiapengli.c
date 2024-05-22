/*
 * Copyright (c) 2022 Jiapeng Li
 *
 * Original source: https://github.com/JiapengLi/LoRaWANFragmentedDataBlockTransportAlgorithm
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "frag_dec_jiapengli.h"
#include "frag_flash.h"

#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/bitarray.h>

LOG_MODULE_REGISTER(lorawan_frag_dec, CONFIG_LORAWAN_SERVICES_LOG_LEVEL);

#define FRAG_MAX_NB (CONFIG_LORAWAN_FRAG_TRANSPORT_IMAGE_SIZE / CONFIG_LORAWAN_FRAG_TRANSPORT_MIN_FRAG_SIZE + 1U)
#define FRAG_MAX_SIZE  (CONFIG_LORAWAN_FRAG_TRANSPORT_MAX_FRAG_SIZE)
#define FRAG_TOLERANCE (FRAG_MAX_NB * CONFIG_LORAWAN_FRAG_TRANSPORT_MAX_REDUNDANCY / 100U)

SYS_BITARRAY_DEFINE_STATIC(lost_frm_bm, FRAG_MAX_NB);
SYS_BITARRAY_DEFINE_STATIC(lost_frm_matrix_bm, (FRAG_TOLERANCE * (FRAG_TOLERANCE + 1) / 2));
SYS_BITARRAY_DEFINE_STATIC(matched_lost_frm_bm0, FRAG_TOLERANCE);
SYS_BITARRAY_DEFINE_STATIC(matched_lost_frm_bm1, FRAG_TOLERANCE);
SYS_BITARRAY_DEFINE_STATIC(matrix_line_bm, FRAG_MAX_NB);

static int m2t_map(int x, int y, int m)
{
	return y * m + x;
}

static bool m2t_get_new(struct sys_bitarray *m2tbm, int x, int y, int m)
{
	/* we are dealing with triangular matrices, so we don't expect actions in the lower half */
	__ASSERT(x >= y, "x: %d, y: %d, m: %d", x, y, m);
	int bit;

	int ret = sys_bitarray_test_bit(m2tbm, m2t_map(x,y,m), &bit);
	__ASSERT_NO_MSG(ret == 0);

	return bit != 0;
}

static void m2t_set_new(struct sys_bitarray *m2tbm, int x, int y, int m)
{
	/* we are dealing with triangular matrices, so we don't expect actions in the lower half */
	__ASSERT(x >= y, "x: %d, y: %d, m: %d", x, y, m);
	sys_bitarray_set_bit(m2tbm, m2t_map(x,y,m));
}

static void m2t_clr_new(struct sys_bitarray *m2tbm, int x, int y, int m)
{
	/* we are dealing with triangular matrices, so we don't expect actions in the lower half */
	__ASSERT(x >= y, "x: %d, y: %d, m: %d", x, y, m);
	sys_bitarray_clear_bit(m2tbm, m2t_map(x,y,m));
}

static bool bit_get_new(struct sys_bitarray *bitmap, int index)
{
	int lost_frm_bit;

	int ret = sys_bitarray_test_bit(bitmap, index, &lost_frm_bit);
	__ASSERT_NO_MSG(ret == 0);
	return lost_frm_bit != 0;
}

static void bit_set_new(struct sys_bitarray *bitmap, int index)
{
	int ret = sys_bitarray_set_bit(bitmap, index);
	__ASSERT_NO_MSG(ret == 0);
}

static void bit_clr_new(struct sys_bitarray *bitmap, int index)
{
	int ret = sys_bitarray_clear_bit(bitmap, index);
	__ASSERT_NO_MSG(ret == 0);
}

static int bit_count_ones_new(struct sys_bitarray *bitmap, int index)
{
	size_t count;
	int ret = sys_bitarray_popcount_region(bitmap, index + 1, 0, &count);
	__ASSERT_NO_MSG(ret == 0);
	int b = count;
	return b;
}

static void bit_xor_new(struct sys_bitarray *des, struct sys_bitarray *src, int size)
{
	int ret = sys_bitarray_xor(des, src, size, 0);
	__ASSERT_NO_MSG(ret == 0);
}

static bool bit_is_all_clear_new(struct sys_bitarray *bitmap, int size)
{
	return sys_bitarray_is_region_cleared(bitmap, size, 0);
}

static int find_nth_set_bit(struct sys_bitarray *bitmap, int size, int n)
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

static int find_first_set_bit(struct sys_bitarray *bitmap, int size)
{
	return find_nth_set_bit(bitmap, size, 1);
}

static void bit_clear_all_new(struct sys_bitarray *bitmap, int size) {
	int ret = sys_bitarray_clear_region(bitmap, size, 0);
	__ASSERT_NO_MSG(ret == 0);
}

/**
 * Generate a 23bit Pseudorandom Binary Sequence (PRBS)
 *
 * @param seed Seed input value
 *
 * @returns Pseudorandom output value
 */
static int32_t prbs23(int32_t seed)
{
	int32_t b0 = seed & 1;
	int32_t b1 = (seed & 32) / 32;

	return (seed / 2) + ((b0 ^ b1) << 22);
}

/**
 * Generate vector for coded fragment n of the MxN parity matrix
 *
 * @param m Total number of uncoded fragments (M)
 * @param n Coded fragment number (starting at 1 and not 0)
 * @param vec Output vector (buffer size must be greater than m)
 */
static void lorawan_fec_parity_matrix_vector(int m, int n, struct sys_bitarray *vec)
{
	int mm, x, r;

	int ret = sys_bitarray_clear_region(vec, m, 0);
	__ASSERT_NO_MSG(ret == 0);

	/*
	 * Powers of 2 must be treated differently to make sure matrix content is close
	 * to random. Powers of 2 tend to generate patterns.
	 */
	if (is_power_of_two(m)) {
		mm = m + 1;
	} else {
		mm = m;
	}

	x = 1 + (1001 * n);

	for (int nb_coeff = 0; nb_coeff < (m / 2); nb_coeff++) {
		r = (1 << 16);
		while (r >= m) {
			x = prbs23(x);
			r = x % mm;
		}
		int ret = sys_bitarray_set_bit(vec, r);
		__ASSERT_NO_MSG(ret == 0);
	}
}

void frag_dec_init(frag_dec_t *obj)
{
	int j;

	/* set all frame lost, from 0 to nb_frag-1 */
	obj->lost_frame_count = obj->cfg.nb_frag;
	for (j = 0; j < obj->cfg.nb_frag; j++) {
		bit_set_new(&lost_frm_bm, j);
	}

	obj->filled_lost_frm_count = 0;
	obj->status = FRAG_DEC_STA_UNCODED;
}

void frag_dec_frame_received(frag_dec_t *obj, uint16_t index)
{
	if (bit_get_new(&lost_frm_bm, index)) {
		bit_clr_new(&lost_frm_bm, index);
		obj->lost_frame_count--;
		/* TODO: check and update other maps */
	}
}

static void frag_dec_write_line(struct sys_bitarray *matrix, uint16_t line_index, struct sys_bitarray *vector, int len)
{
	for (int i = line_index; i < len; i++) {
		if (bit_get_new(vector, i)) {
			m2t_set_new(matrix, i, line_index, len);
		} else {
			m2t_clr_new(matrix, i, line_index, len);
		}
	}
}

static void frag_dec_read_line(struct sys_bitarray *matrix, uint16_t line_index, struct sys_bitarray *vector, int len)
{
	for (int i = 0; i < len; i++) {
		if (i >= line_index && m2t_get_new(matrix, i, line_index, len)) {
			bit_set_new(vector, i);
		} else {
			bit_clr_new(vector, i);
		}
	}
}

/* frameCounter from 1 to nb_frag */
int frag_dec(frag_dec_t *obj, uint16_t frameCounter, const uint8_t *buf, int len)
{
	int i, j;
	int index, unmatched_frame_cnt;
	int lost_frame_index, frame_index;
	static uint8_t row_data_buf[FRAG_MAX_SIZE];
	static uint8_t xor_row_data_buf[FRAG_MAX_SIZE];

	if (obj->status == FRAG_DEC_STA_DONE) {
		return obj->lost_frame_count;
	}

	if (len != obj->cfg.frag_size) {
		return FRAG_DEC_ERR_INVALID_FRAME;
	}

	index = frameCounter - 1;
	if ((index < obj->cfg.nb_frag) && (obj->status == FRAG_DEC_STA_UNCODED)) {
		/* uncoded frames under uncoded process */
		/* mark new received frame */
		frag_dec_frame_received(obj, index);
		/* save data to flash */
		frag_flash_write(index * obj->cfg.frag_size, (uint8_t*)buf, obj->cfg.frag_size);
		/* if no frame lost finish decode process */
		if (obj->lost_frame_count == 0) {
			obj->status = FRAG_DEC_STA_DONE;
			return obj->lost_frame_count;
		}
		return FRAG_DEC_ONGOING;
	}

	/* clear all temporary bm and buf */
	bit_clear_all_new(&matched_lost_frm_bm0, obj->lost_frame_count);
	bit_clear_all_new(&matched_lost_frm_bm1, obj->lost_frame_count);

	/* back up input data so that not to mess input data */
	memcpy(xor_row_data_buf, buf, obj->cfg.frag_size);

	obj->status = FRAG_DEC_STA_CODED;
	/* coded frames start processing, lost_frame_count is now frozen and should be not
	 * changed (!!!)
	 * too many packets are lost, it is not possible to reconstruct data block
	 */
	if (obj->lost_frame_count > FRAG_TOLERANCE) {
		/* too many frames are lost, memory is not enough to reconstruct the
		 * packets
		 */
		return FRAG_DEC_ERR_TOO_MANY_FRAME_LOST;
	}
	unmatched_frame_cnt = 0;
	/* build parity matrix vector for current line */
	lorawan_fec_parity_matrix_vector(obj->cfg.nb_frag, frameCounter - obj->cfg.nb_frag, &matrix_line_bm);
	for (i = 0; i < obj->cfg.nb_frag; i++) {
		if (!bit_get_new(&matrix_line_bm, i)) {
			continue;
		}
		if (bit_get_new(&lost_frm_bm, i)) {
			/* coded frame is not matched one received uncoded frame */
			/* matched_lost_frm_bm0 index is the nth lost frame */
			bit_set_new(&matched_lost_frm_bm0, bit_count_ones_new(&lost_frm_bm, i) - 1);
			unmatched_frame_cnt++;
		} else {
			/* restore frame by xoring with already received frame */
			/* load frame with index i into row_data_buf */
			frag_flash_read(i * obj->cfg.frag_size, row_data_buf, obj->cfg.frag_size);
			/* xor previously received frame with data for current frame */
			mem_xor_n(xor_row_data_buf, xor_row_data_buf, row_data_buf,
				  obj->cfg.frag_size);
		}
	}
	if (unmatched_frame_cnt <= 0) {
		return FRAG_DEC_ONGOING;
	}

	/* &matched_lost_frm_bm0 now saves new coded frame which excludes all received
	 * frames content start to diagonal &matched_lost_frm_bm0
	 */
	do {
		lost_frame_index = find_first_set_bit(&matched_lost_frm_bm0, obj->lost_frame_count);
		/** we know which one is the next lost frame, try to find it in the lost frame bm */
		frame_index = find_nth_set_bit(&lost_frm_bm, obj->cfg.nb_frag, lost_frame_index + 1);
		if (frame_index == -1) {
			//LOG_INF("matched_lost_frm_bm0: ");
			//frag_dec_log_bits(&matched_lost_frm_bm0, obj->lost_frame_count);
			//LOG_INF("lost_frm_bm: ");
			//frag_dec_log_bits(&lost_frm_bm, obj->cfg.nb_frag);
			LOG_INF("frame_index: %d, lost_frame_index: %d\n", frame_index,
				lost_frame_index);
		}
		/* if current frame contains new information, save it */
		if (!m2t_get_new(&lost_frm_matrix_bm, lost_frame_index, lost_frame_index, obj->lost_frame_count)) {
			frag_dec_write_line(&lost_frm_matrix_bm, lost_frame_index,
						      &matched_lost_frm_bm0,
						      obj->lost_frame_count);
			frag_flash_write(frame_index * obj->cfg.frag_size, xor_row_data_buf, obj->cfg.frag_size);
			obj->filled_lost_frm_count++;
			break;
		}

		frag_dec_read_line(&lost_frm_matrix_bm, lost_frame_index,
					      &matched_lost_frm_bm1,
					      obj->lost_frame_count);
		bit_xor_new(&matched_lost_frm_bm0, &matched_lost_frm_bm1, obj->lost_frame_count);
		frag_flash_read(frame_index * obj->cfg.frag_size, row_data_buf, obj->cfg.frag_size);
		mem_xor_n(xor_row_data_buf, xor_row_data_buf, row_data_buf, obj->cfg.frag_size);
	} while (!bit_is_all_clear_new(&matched_lost_frm_bm0, obj->lost_frame_count));

	if (obj->filled_lost_frm_count != obj->lost_frame_count) {
		return FRAG_DEC_ONGOING;
	}

	/* all frame content is received, now to reconstruct the whole frame */
	for (i = (obj->lost_frame_count - 2); i >= 0; i--) {
		frame_index = find_nth_set_bit(&lost_frm_bm, obj->cfg.nb_frag, i + 1);
		frag_flash_read(frame_index * obj->cfg.frag_size, xor_row_data_buf, obj->cfg.frag_size);
		for (j = (obj->lost_frame_count - 1); j > i; j--) {
			frag_dec_read_line(&lost_frm_matrix_bm, i,
					   &matched_lost_frm_bm1,
					   obj->lost_frame_count);
			frag_dec_read_line(&lost_frm_matrix_bm, j,
					   &matched_lost_frm_bm0,
					   obj->lost_frame_count);
			if (!bit_get_new(&matched_lost_frm_bm1, j)) {
				continue;
			}
			lost_frame_index = find_nth_set_bit(&lost_frm_bm,
					       obj->cfg.nb_frag, j + 1);
			bit_xor_new(&matched_lost_frm_bm1,
				&matched_lost_frm_bm0,
				obj->lost_frame_count);
			frag_flash_read(lost_frame_index * obj->cfg.frag_size, row_data_buf, obj->cfg.frag_size);
			mem_xor_n(xor_row_data_buf, xor_row_data_buf,
				row_data_buf, obj->cfg.frag_size);
			frag_dec_write_line(&lost_frm_matrix_bm, i, &matched_lost_frm_bm1,
						      obj->lost_frame_count);
		}
		frag_flash_write(frame_index * obj->cfg.frag_size, xor_row_data_buf, obj->cfg.frag_size);
	}
	obj->status = FRAG_DEC_STA_DONE;
	return obj->lost_frame_count;
}
