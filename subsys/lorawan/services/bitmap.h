/*
 * Copyright (c) 2022 Jiapeng Li
 *
 * Original source: https://github.com/JiapengLi/LoRaWANFragmentedDataBlockTransportAlgorithm
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __BITMAP_H
#define __BITMAP_H

#define BM_4
#define BUILTIN_FUNC

/*
count set bits:
https://stackoverflow.com/questions/109023/how-to-count-the-number-of-set-bits-in-a-32-bit-integer
https://www.geeksforgeeks.org/count-set-bits-in-an-integer/

If you want the fastest way, you will need to use non-portable methods.

Windows/MSVC:

_BitScanForward()
_BitScanReverse()
__popcnt()
GCC:

__builtin_ffs()
__builtin_ctz()
__builtin_clz()
__builtin_popcount()
These typically map directly to native hardware instructions. So it doesn't get much faster than
these.

But since there's no C/C++ functionality for them, they're only accessible via compiler intrinsics.

*/

#include <zephyr/sys/bitarray.h>

#if defined BM_4
typedef uint32_t bm_t;
#define BM_UNIT (sizeof(bm_t) * 8)
#define BM_OFST (5) /* 8: 3, 16: 4, 32: 5 */
#elif defined BM_2
typedef uint16_t bm_t;
#define BM_UNIT (sizeof(bm_t) * 8)
#define BM_OFST (4) /* 8: 3, 16: 4, 32: 5 */
#elif defined BM_1
typedef uint8_t bm_t;
#define BM_UNIT (sizeof(bm_t) * 8)
#define BM_OFST (3) /* 8: 3, 16: 4, 32: 5 */
#endif


int m2t_map(int x, int y, int m);

bool bit_get_new(struct sys_bitarray *bitmap, int index);

void bit_set_new(struct sys_bitarray *bitmap, int index);

void bit_clr_new(struct sys_bitarray *bitmap, int index);

int bit_count_ones_new(struct sys_bitarray *bitmap, int index);

int bit_ffs_new(struct sys_bitarray *bitmap, int size);

/* find the nth set */
int bit_fns_new(struct sys_bitarray *bitmap, int size, int n);

void bit_xor_new(struct sys_bitarray *des, struct sys_bitarray *src, int size);

bool bit_is_all_clear_new(struct sys_bitarray *bitmap, int size);

void bit_clear_all_new(struct sys_bitarray *bitmap, int size);

bool m2t_get_new(struct sys_bitarray *m2tbm, int x, int y, int m);
void m2t_set_new(struct sys_bitarray *m2tbm, int x, int y, int m);
void m2t_clr_new(struct sys_bitarray *m2tbm, int x, int y, int m);



#endif /* __BITMAP_H */
