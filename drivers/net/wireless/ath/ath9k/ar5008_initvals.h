/*
 * Copyright (c) 2010-2011 Atheros Communications Inc.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

static const u32 ar5416Modes[][6] = {
	{0x00001030, 0x00000230, 0x00000460, 0x000002c0, 0x00000160, 0x000001e0},
	{0x00001070, 0x00000168, 0x000002d0, 0x00000318, 0x0000018c, 0x000001e0},
	{0x000010b0, 0x00000e60, 0x00001cc0, 0x00007c70, 0x00003e38, 0x00001180},
	{0x000010f0, 0x0000a000, 0x00014000, 0x00016000, 0x0000b000, 0x00014008},
	{0x00008014, 0x03e803e8, 0x07d007d0, 0x10801600, 0x08400b00, 0x06e006e0},
	{0x0000801c, 0x128d93a7, 0x128d93cf, 0x12e013d7, 0x12e013ab, 0x098813cf},
	{0x00008120, 0x08f04800, 0x08f04800, 0x08f04810, 0x08f04810, 0x08f04810},
	{0x000081d0, 0x00003210, 0x00003210, 0x0000320a, 0x0000320a, 0x0000320a},
	{0x00009804, 0x00000300, 0x000003c4, 0x000003c4, 0x00000300, 0x00000303},
	{0x00009820, 0x02020200, 0x02020200, 0x02020200, 0x02020200, 0x02020200},
	{0x00009824, 0x00000e0e, 0x00000e0e, 0x00000e0e, 0x00000e0e, 0x00000e0e},
	{0x00009828, 0x0a020001, 0x0a020001, 0x0a020001, 0x0a020001, 0x0a020001},
	{0x00009834, 0x00000e0e, 0x00000e0e, 0x00000e0e, 0x00000e0e, 0x00000e0e},
	{0x00009838, 0x00000007, 0x00000007, 0x00000007, 0x00000007, 0x00000007},
	{0x00009844, 0x1372161e, 0x1372161e, 0x137216a0, 0x137216a0, 0x137216a0},
	{0x00009848, 0x001a6a65, 0x001a6a65, 0x00197a68, 0x00197a68, 0x00197a68},
	{0x0000a848, 0x001a6a65, 0x001a6a65, 0x00197a68, 0x00197a68, 0x00197a68},
	{0x0000b848, 0x001a6a65, 0x001a6a65, 0x00197a68, 0x00197a68, 0x00197a68},
	{0x00009850, 0x6c48b4e0, 0x6d48b4e0, 0x6d48b0de, 0x6c48b0de, 0x6c48b0de},
	{0x00009858, 0x7ec82d2e, 0x7ec82d2e, 0x7ec82d2e, 0x7ec82d2e, 0x7ec82d2e},
	{0x0000985c, 0x31395d5e, 0x3139605e, 0x3139605e, 0x31395d5e, 0x31395d5e},
	{0x00009860, 0x00049d18, 0x00049d18, 0x00049d18, 0x00049d18, 0x00049d18},
	{0x00009864, 0x0001ce00, 0x0001ce00, 0x0001ce00, 0x0001ce00, 0x0001ce00},
	{0x00009868, 0x409a4190, 0x409a4190, 0x409a4190, 0x409a4190, 0x409a4190},
	{0x0000986c, 0x050cb081, 0x050cb081, 0x050cb081, 0x050cb081, 0x050cb081},
	{0x00009914, 0x000007d0, 0x00000fa0, 0x00001130, 0x00000898, 0x000007d0},
	{0x00009918, 0x000001b8, 0x00000370, 0x00000268, 0x00000134, 0x00000134},
	{0x00009924, 0xd0058a0b, 0xd0058a0b, 0xd0058a0b, 0xd0058a0b, 0xd0058a0b},
	{0x00009944, 0xffb81020, 0xffb81020, 0xffb81020, 0xffb81020, 0xffb81020},
	{0x00009960, 0x00000900, 0x00000900, 0x00012d80, 0x00012d80, 0x00012d80},
	{0x0000a960, 0x00000900, 0x00000900, 0x00012d80, 0x00012d80, 0x00012d80},
	{0x0000b960, 0x00000900, 0x00000900, 0x00012d80, 0x00012d80, 0x00012d80},
	{0x00009964, 0x00000000, 0x00000000, 0x00001120, 0x00001120, 0x00001120},
	{0x000099bc, 0x001a0a00, 0x001a0a00, 0x001a0a00, 0x001a0a00, 0x001a0a00},
	{0x000099c0, 0x038919be, 0x038919be, 0x038919be, 0x038919be, 0x038919be},
	{0x000099c4, 0x06336f77, 0x06336f77, 0x06336f77, 0x06336f77, 0x06336f77},
	{0x000099c8, 0x6af6532c, 0x6af6532c, 0x6af6532c, 0x6af6532c, 0x6af6532c},
	{0x000099cc, 0x08f186c8, 0x08f186c8, 0x08f186c8, 0x08f186c8, 0x08f186c8},
	{0x000099d0, 0x00046384, 0x00046384, 0x00046384, 0x00046384, 0x00046384},
	{0x000099d4, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000},
	{0x000099d8, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000},
	{0x0000a204, 0x00000880, 0x00000880, 0x00000880, 0x00000880, 0x00000880},
	{0x0000a208, 0xd6be4788, 0xd6be4788, 0xd03e4788, 0xd03e4788, 0xd03e4788},
	{0x0000a20c, 0x002ec1e0, 0x002ec1e0, 0x002ac120, 0x002ac120, 0x002ac120},
	{0x0000b20c, 0x002ec1e0, 0x002ec1e0, 0x002ac120, 0x002ac120, 0x002ac120},
	{0x0000c20c, 0x002ec1e0, 0x002ec1e0, 0x002ac120, 0x002ac120, 0x002ac120},
	{0x0000a21c, 0x1883800a, 0x1883800a, 0x1883800a, 0x1883800a, 0x1883800a},
	{0x0000a230, 0x00000000, 0x00000000, 0x00000210, 0x00000108, 0x00000000},
	{0x0000a274, 0x0a1a9caa, 0x0a1a9caa, 0x0a1a7caa, 0x0a1a7caa, 0x0a1a7caa},
	{0x0000a300, 0x18010000, 0x18010000, 0x18010000, 0x18010000, 0x18010000},
	{0x0000a304, 0x30032602, 0x30032602, 0x2e032402, 0x2e032402, 0x2e032402},
	{0x0000a308, 0x48073e06, 0x48073e06, 0x4a0a3c06, 0x4a0a3c06, 0x4a0a3c06},
	{0x0000a30c, 0x560b4c0a, 0x560b4c0a, 0x621a540b, 0x621a540b, 0x621a540b},
	{0x0000a310, 0x641a600f, 0x641a600f, 0x764f6c1b, 0x764f6c1b, 0x764f6c1b},
	{0x0000a314, 0x7a4f6e1b, 0x7a4f6e1b, 0x845b7a5a, 0x845b7a5a, 0x845b7a5a},
	{0x0000a318, 0x8c5b7e5a, 0x8c5b7e5a, 0x950f8ccf, 0x950f8ccf, 0x950f8ccf},
	{0x0000a31c, 0x9d0f96cf, 0x9d0f96cf, 0xa5cf9b4f, 0xa5cf9b4f, 0xa5cf9b4f},
	{0x0000a320, 0xb51fa69f, 0xb51fa69f, 0xbddfaf1f, 0xbddfaf1f, 0xbddfaf1f},
	{0x0000a324, 0xcb3fbd07, 0xcb3fbcbf, 0xd1ffc93f, 0xd1ffc93f, 0xd1ffc93f},
	{0x0000a328, 0x0000d7bf, 0x0000d7bf, 0x00000000, 0x00000000, 0x00000000},
	{0x0000a32c, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000},
	{0x0000a330, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000},
	{0x0000a334, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000},
};

static const u32 ar5416Common[][2] = {
	/* Addr      allmodes  */
	{0x0000000c, 0x00000000},
	{0x00000030, 0x00020015},
	{0x00000034, 0x00000005},
	{0x00000040, 0x00000000},
	{0x00000044, 0x00000008},
	{0x00000048, 0x00000008},
	{0x0000004c, 0x00000010},
	{0x00000050, 0x00000000},
	{0x00000054, 0x0000001f},
	{0x00000800, 0x00000000},
	{0x00000804, 0x00000000},
	{0x00000808, 0x00000000},
	{0x0000080c, 0x00000000},
	{0x00000810, 0x00000000},
	{0x00000814, 0x00000000},
	{0x00000818, 0x00000000},
	{0x0000081c, 0x00000000},
	{0x00000820, 0x00000000},
	{0x00000824, 0x00000000},
	{0x00001040, 0x002ffc0f},
	{0x00001044, 0x002ffc0f},
	{0x00001048, 0x002ffc0f},
	{0x0000104c, 0x002ffc0f},
	{0x00001050, 0x002ffc0f},
	{0x00001054, 0x002ffc0f},
	{0x00001058, 0x002ffc0f},
	{0x0000105c, 0x002ffc0f},
	{0x00001060, 0x002ffc0f},
	{0x00001064, 0x002ffc0f},
	{0x00001230, 0x00000000},
	{0x00001270, 0x00000000},
	{0x00001038, 0x00000000},
	{0x00001078, 0x00000000},
	{0x000010b8, 0x00000000},
	{0x000010f8, 0x00000000},
	{0x00001138, 0x00000000},
	{0x00001178, 0x00000000},
	{0x000011b8, 0x00000000},
	{0x000011f8, 0x00000000},
	{0x00001238, 0x00000000},
	{0x00001278, 0x00000000},
	{0x000012b8, 0x00000000},
	{0x000012f8, 0x00000000},
	{0x00001338, 0x00000000},
	{0x00001378, 0x00000000},
	{0x000013b8, 0x00000000},
	{0x000013f8, 0x00000000},
	{0x00001438, 0x00000000},
	{0x00001478, 0x00000000},
	{0x000014b8, 0x00000000},
	{0x000014f8, 0x00000000},
	{0x00001538, 0x00000000},
	{0x00001578, 0x00000000},
	{0x000015b8, 0x00000000},
	{0x000015f8, 0x00000000},
	{0x00001638, 0x00000000},
	{0x00001678, 0x00000000},
	{0x000016b8, 0x00000000},
	{0x000016f8, 0x00000000},
	{0x00001738, 0x00000000},
	{0x00001778, 0x00000000},
	{0x000017b8, 0x00000000},
	{0x000017f8, 0x00000000},
	{0x0000103c, 0x00000000},
	{0x0000107c, 0x00000000},
	{0x000010bc, 0x00000000},
	{0x000010fc, 0x00000000},
	{0x0000113c, 0x00000000},
	{0x0000117c, 0x00000000},
	{0x000011bc, 0x00000000},
	{0x000011fc, 0x00000000},
	{0x0000123c, 0x00000000},
	{0x0000127c, 0x00000000},
	{0x000012bc, 0x00000000},
	{0x000012fc, 0x00000000},
	{0x0000133c, 0x00000000},
	{0x0000137c, 0x00000000},
	{0x000013bc, 0x00000000},
	{0x000013fc, 0x00000000},
	{0x0000143c, 0x00000000},
	{0x0000147c, 0x00000000},
	{0x00004030, 0x00000002},
	{0x0000403c, 0x00000002},
	{0x00007010, 0x00000000},
	{0x00007038, 0x000004c2},
	{0x00008004, 0x00000000},
	{0x00008008, 0x00000000},
	{0x0000800c, 0x00000000},
	{0x00008018, 0x00000700},
	{0x00008020, 0x00000000},
	{0x00008038, 0x00000000},
	{0x0000803c, 0x00000000},
	{0x00008048, 0x40000000},
	{0x00008054, 0x00000000},
	{0x00008058, 0x00000000},
	{0x0000805c, 0x000fc78f},
	{0x00008060, 0x0000000f},
	{0x00008064, 0x00000000},
	{0x000080c0, 0x2a82301a},
	{0x000080c4, 0x05dc01e0},
	{0x000080c8, 0x1f402710},
	{0x000080cc, 0x01f40000},
	{0x000080d0, 0x00001e00},
	{0x000080d4, 0x00000000},
	{0x000080d8, 0x00400000},
	{0x000080e0, 0xffffffff},
	{0x000080e4, 0x0000ffff},
	{0x000080e8, 0x003f3f3f},
	{0x000080ec, 0x00000000},
	{0x000080f0, 0x00000000},
	{0x000080f4, 0x00000000},
	{0x000080f8, 0x00000000},
	{0x000080fc, 0x00020000},
	{0x00008100, 0x00020000},
	{0x00008104, 0x00000001},
	{0x00008108, 0x00000052},
	{0x0000810c, 0x00000000},
	{0x00008110, 0x00000168},
	{0x00008118, 0x000100aa},
	{0x0000811c, 0x00003210},
	{0x00008124, 0x00000000},
	{0x00008128, 0x00000000},
	{0x0000812c, 0x00000000},
	{0x00008130, 0x00000000},
	{0x00008134, 0x00000000},
	{0x00008138, 0x00000000},
	{0x0000813c, 0x00000000},
	{0x00008144, 0xffffffff},
	{0x00008168, 0x00000000},
	{0x0000816c, 0x00000000},
	{0x00008170, 0x32143320},
	{0x00008174, 0xfaa4fa50},
	{0x00008178, 0x00000100},
	{0x0000817c, 0x00000000},
	{0x000081c4, 0x00000000},
	{0x000081ec, 0x00000000},
	{0x000081f0, 0x00000000},
	{0x000081f4, 0x00000000},
	{0x000081f8, 0x00000000},
	{0x000081fc, 0x00000000},
	{0x00008200, 0x00000000},
	{0x00008204, 0x00000000},
	{0x00008208, 0x00000000},
	{0x0000820c, 0x00000000},
	{0x00008210, 0x00000000},
	{0x00008214, 0x00000000},
	{0x00008218, 0x00000000},
	{0x0000821c, 0x00000000},
	{0x00008220, 0x00000000},
	{0x00008224, 0x00000000},
	{0x00008228, 0x00000000},
	{0x0000822c, 0x00000000},
	{0x00008230, 0x00000000},
	{0x00008234, 0x00000000},
	{0x00008238, 0x00000000},
	{0x0000823c, 0x00000000},
	{0x00008240, 0x00100000},
	{0x00008244, 0x0010f400},
	{0x00008248, 0x00000100},
	{0x0000824c, 0x0001e800},
	{0x00008250, 0x00000000},
	{0x00008254, 0x00000000},
	{0x00008258, 0x00000000},
	{0x0000825c, 0x400000ff},
	{0x00008260, 0x00080922},
	{0x00008264, 0x88000010},
	{0x00008270, 0x00000000},
	{0x00008274, 0x40000000},
	{0x00008278, 0x003e4180},
	{0x0000827c, 0x00000000},
	{0x00008284, 0x0000002c},
	{0x00008288, 0x0000002c},
	{0x0000828c, 0x00000000},
	{0x00008294, 0x00000000},
	{0x00008298, 0x00000000},
	{0x00008300, 0x00000000},
	{0x00008304, 0x00000000},
	{0x00008308, 0x00000000},
	{0x0000830c, 0x00000000},
	{0x00008310, 0x00000000},
	{0x00008314, 0x00000000},
	{0x00008318, 0x00000000},
	{0x00008328, 0x00000000},
	{0x0000832c, 0x00000007},
	{0x00008330, 0x00000302},
	{0x00008334, 0x00000e00},
	{0x00008338, 0x00070000},
	{0x0000833c, 0x00000000},
	{0x00008340, 0x000107ff},
	{0x00009808, 0x00000000},
	{0x0000980c, 0xad848e19},
	{0x00009810, 0x7d14e000},
	{0x00009814, 0x9c0a9f6b},
	{0x0000981c, 0x00000000},
	{0x0000982c, 0x0000a000},
	{0x00009830, 0x00000000},
	{0x0000983c, 0x00200400},
	{0x00009840, 0x206a002e},
	{0x0000984c, 0x1284233c},
	{0x00009854, 0x00000859},
	{0x00009900, 0x00000000},
	{0x00009904, 0x00000000},
	{0x00009908, 0x00000000},
	{0x0000990c, 0x00000000},
	{0x0000991c, 0x10000fff},
	{0x00009920, 0x05100000},
	{0x0000a920, 0x05100000},
	{0x0000b920, 0x05100000},
	{0x00009928, 0x00000001},
	{0x0000992c, 0x00000004},
	{0x00009934, 0x1e1f2022},
	{0x00009938, 0x0a0b0c0d},
	{0x0000993c, 0x00000000},
	{0x00009948, 0x9280b212},
	{0x0000994c, 0x00020028},
	{0x00009954, 0x5d50e188},
	{0x00009958, 0x00081fff},
	{0x0000c95c, 0x004b6a8e},
	{0x0000c968, 0x000003ce},
	{0x00009970, 0x190fb515},
	{0x00009974, 0x00000000},
	{0x00009978, 0x00000001},
	{0x0000997c, 0x00000000},
	{0x00009980, 0x00000000},
	{0x00009984, 0x00000000},
	{0x00009988, 0x00000000},
	{0x0000998c, 0x00000000},
	{0x00009990, 0x00000000},
	{0x00009994, 0x00000000},
	{0x00009998, 0x00000000},
	{0x0000999c, 0x00000000},
	{0x000099a0, 0x00000000},
	{0x000099a4, 0x00000001},
	{0x000099a8, 0x001fff00},
	{0x000099ac, 0x00000000},
	{0x000099b0, 0x03051000},
	{0x000099dc, 0x00000000},
	{0x000099e0, 0x00000200},
	{0x000099e4, 0xaaaaaaaa},
	{0x000099e8, 0x3c466478},
	{0x000099ec, 0x000000aa},
	{0x000099fc, 0x00001042},
	{0x00009b00, 0x00000000},
	{0x00009b04, 0x00000001},
	{0x00009b08, 0x00000002},
	{0x00009b0c, 0x00000003},
	{0x00009b10, 0x00000004},
	{0x00009b14, 0x00000005},
	{0x00009b18, 0x00000008},
	{0x00009b1c, 0x00000009},
	{0x00009b20, 0x0000000a},
	{0x00009b24, 0x0000000b},
	{0x00009b28, 0x0000000c},
	{0x00009b2c, 0x0000000d},
	{0x00009b30, 0x00000010},
	{0x00009b34, 0x00000011},
	{0x00009b38, 0x00000012},
	{0x00009b3c, 0x00000013},
	{0x00009b40, 0x00000014},
	{0x00009b44, 0x00000015},
	{0x00009b48, 0x00000018},
	{0x00009b4c, 0x00000019},
	{0x00009b50, 0x0000001a},
	{0x00009b54, 0x0000001b},
	{0x00009b58, 0x0000001c},
	{0x00009b5c, 0x0000001d},
	{0x00009b60, 0x00000020},
	{0x00009b64, 0x00000021},
	{0x00009b68, 0x00000022},
	{0x00009b6c, 0x00000023},
	{0x00009b70, 0x00000024},
	{0x00009b74, 0x00000025},
	{0x00009b78, 0x00000028},
	{0x00009b7c, 0x00000029},
	{0x00009b80, 0x0000002a},
	{0x00009b84, 0x0000002b},
	{0x00009b88, 0x0000002c},
	{0x00009b8c, 0x0000002d},
	{0x00009b90, 0x00000030},
	{0x00009b94, 0x00000031},
	{0x00009b98, 0x00000032},
	{0x00009b9c, 0x00000033},
	{0x00009ba0, 0x00000034},
	{0x00009ba4, 0x00000035},
	{0x00009ba8, 0x00000035},
	{0x00009bac, 0x00000035},
	{0x00009bb0, 0x00000035},
	{0x00009bb4, 0x00000035},
	{0x00009bb8, 0x00000035},
	{0x00009bbc, 0x00000035},
	{0x00009bc0, 0x00000035},
	{0x00009bc4, 0x00000035},
	{0x00009bc8, 0x00000035},
	{0x00009bcc, 0x00000035},
	{0x00009bd0, 0x00000035},
	{0x00009bd4, 0x00000035},
	{0x00009bd8, 0x00000035},
	{0x00009bdc, 0x00000035},
	{0x00009be0, 0x00000035},
	{0x00009be4, 0x00000035},
	{0x00009be8, 0x00000035},
	{0x00009bec, 0x00000035},
	{0x00009bf0, 0x00000035},
	{0x00009bf4, 0x00000035},
	{0x00009bf8, 0x00000010},
	{0x00009bfc, 0x0000001a},
	{0x0000a210, 0x40806333},
	{0x0000a214, 0x00106c10},
	{0x0000a218, 0x009c4060},
	{0x0000a220, 0x018830c6},
	{0x0000a224, 0x00000400},
	{0x0000a228, 0x00000bb5},
	{0x0000a22c, 0x00000011},
	{0x0000a234, 0x20202020},
	{0x0000a238, 0x20202020},
	{0x0000a23c, 0x13c889af},
	{0x0000a240, 0x38490a20},
	{0x0000a244, 0x00007bb6},
	{0x0000a248, 0x0fff3ffc},
	{0x0000a24c, 0x00000001},
	{0x0000a250, 0x0000a000},
	{0x0000a254, 0x00000000},
	{0x0000a258, 0x0cc75380},
	{0x0000a25c, 0x0f0f0f01},
	{0x0000a260, 0xdfa91f01},
	{0x0000a268, 0x00000000},
	{0x0000a26c, 0x0e79e5c6},
	{0x0000b26c, 0x0e79e5c6},
	{0x0000c26c, 0x0e79e5c6},
	{0x0000d270, 0x00820820},
	{0x0000a278, 0x1ce739ce},
	{0x0000a27c, 0x051701ce},
	{0x0000a338, 0x00000000},
	{0x0000a33c, 0x00000000},
	{0x0000a340, 0x00000000},
	{0x0000a344, 0x00000000},
	{0x0000a348, 0x3fffffff},
	{0x0000a34c, 0x3fffffff},
	{0x0000a350, 0x3fffffff},
	{0x0000a354, 0x0003ffff},
	{0x0000a358, 0x79a8aa1f},
	{0x0000d35c, 0x07ffffef},
	{0x0000d360, 0x0fffffe7},
	{0x0000d364, 0x17ffffe5},
	{0x0000d368, 0x1fffffe4},
	{0x0000d36c, 0x37ffffe3},
	{0x0000d370, 0x3fffffe3},
	{0x0000d374, 0x57ffffe3},
	{0x0000d378, 0x5fffffe2},
	{0x0000d37c, 0x7fffffe2},
	{0x0000d380, 0x7f3c7bba},
	{0x0000d384, 0xf3307ff0},
	{0x0000a388, 0x08000000},
	{0x0000a38c, 0x20202020},
	{0x0000a390, 0x20202020},
	{0x0000a394, 0x1ce739ce},
	{0x0000a398, 0x000001ce},
	{0x0000a39c, 0x00000001},
	{0x0000a3a0, 0x00000000},
	{0x0000a3a4, 0x00000000},
	{0x0000a3a8, 0x00000000},
	{0x0000a3ac, 0x00000000},
	{0x0000a3b0, 0x00000000},
	{0x0000a3b4, 0x00000000},
	{0x0000a3b8, 0x00000000},
	{0x0000a3bc, 0x00000000},
	{0x0000a3c0, 0x00000000},
	{0x0000a3c4, 0x00000000},
	{0x0000a3c8, 0x00000246},
	{0x0000a3cc, 0x20202020},
	{0x0000a3d0, 0x20202020},
	{0x0000a3d4, 0x20202020},
	{0x0000a3dc, 0x1ce739ce},
	{0x0000a3e0, 0x000001ce},
};

static const u32 ar5416Bank0[][2] = {
	/* Addr      allmodes  */
	{0x000098b0, 0x1e5795e5},
	{0x000098e0, 0x02008020},
};

static const u32 ar5416BB_RfGain[][3] = {
	/* Addr      5G_HT20     5G_HT40   */
	{0x00009a00, 0x00000000, 0x00000000},
	{0x00009a04, 0x00000040, 0x00000040},
	{0x00009a08, 0x00000080, 0x00000080},
	{0x00009a0c, 0x000001a1, 0x00000141},
	{0x00009a10, 0x000001e1, 0x00000181},
	{0x00009a14, 0x00000021, 0x000001c1},
	{0x00009a18, 0x00000061, 0x00000001},
	{0x00009a1c, 0x00000168, 0x00000041},
	{0x00009a20, 0x000001a8, 0x000001a8},
	{0x00009a24, 0x000001e8, 0x000001e8},
	{0x00009a28, 0x00000028, 0x00000028},
	{0x00009a2c, 0x00000068, 0x00000068},
	{0x00009a30, 0x00000189, 0x000000a8},
	{0x00009a34, 0x000001c9, 0x00000169},
	{0x00009a38, 0x00000009, 0x000001a9},
	{0x00009a3c, 0x00000049, 0x000001e9},
	{0x00009a40, 0x00000089, 0x00000029},
	{0x00009a44, 0x00000170, 0x00000069},
	{0x00009a48, 0x000001b0, 0x00000190},
	{0x00009a4c, 0x000001f0, 0x000001d0},
	{0x00009a50, 0x00000030, 0x00000010},
	{0x00009a54, 0x00000070, 0x00000050},
	{0x00009a58, 0x00000191, 0x00000090},
	{0x00009a5c, 0x000001d1, 0x00000151},
	{0x00009a60, 0x00000011, 0x00000191},
	{0x00009a64, 0x00000051, 0x000001d1},
	{0x00009a68, 0x00000091, 0x00000011},
	{0x00009a6c, 0x000001b8, 0x00000051},
	{0x00009a70, 0x000001f8, 0x00000198},
	{0x00009a74, 0x00000038, 0x000001d8},
	{0x00009a78, 0x00000078, 0x00000018},
	{0x00009a7c, 0x00000199, 0x00000058},
	{0x00009a80, 0x000001d9, 0x00000098},
	{0x00009a84, 0x00000019, 0x00000159},
	{0x00009a88, 0x00000059, 0x00000199},
	{0x00009a8c, 0x00000099, 0x000001d9},
	{0x00009a90, 0x000000d9, 0x00000019},
	{0x00009a94, 0x000000f9, 0x00000059},
	{0x00009a98, 0x000000f9, 0x00000099},
	{0x00009a9c, 0x000000f9, 0x000000d9},
	{0x00009aa0, 0x000000f9, 0x000000f9},
	{0x00009aa4, 0x000000f9, 0x000000f9},
	{0x00009aa8, 0x000000f9, 0x000000f9},
	{0x00009aac, 0x000000f9, 0x000000f9},
	{0x00009ab0, 0x000000f9, 0x000000f9},
	{0x00009ab4, 0x000000f9, 0x000000f9},
	{0x00009ab8, 0x000000f9, 0x000000f9},
	{0x00009abc, 0x000000f9, 0x000000f9},
	{0x00009ac0, 0x000000f9, 0x000000f9},
	{0x00009ac4, 0x000000f9, 0x000000f9},
	{0x00009ac8, 0x000000f9, 0x000000f9},
	{0x00009acc, 0x000000f9, 0x000000f9},
	{0x00009ad0, 0x000000f9, 0x000000f9},
	{0x00009ad4, 0x000000f9, 0x000000f9},
	{0x00009ad8, 0x000000f9, 0x000000f9},
	{0x00009adc, 0x000000f9, 0x000000f9},
	{0x00009ae0, 0x000000f9, 0x000000f9},
	{0x00009ae4, 0x000000f9, 0x000000f9},
	{0x00009ae8, 0x000000f9, 0x000000f9},
	{0x00009aec, 0x000000f9, 0x000000f9},
	{0x00009af0, 0x000000f9, 0x000000f9},
	{0x00009af4, 0x000000f9, 0x000000f9},
	{0x00009af8, 0x000000f9, 0x000000f9},
	{0x00009afc, 0x000000f9, 0x000000f9},
};

static const u32 ar5416Bank1[][2] = {
	/* Addr      allmodes  */
	{0x000098b0, 0x02108421},
	{0x000098ec, 0x00000008},
};

static const u32 ar5416Bank2[][2] = {
	/* Addr      allmodes  */
	{0x000098b0, 0x0e73ff17},
	{0x000098e0, 0x00000420},
};

static const u32 ar5416Bank3[][3] = {
	/* Addr      5G_HT20     5G_HT40   */
	{0x000098f0, 0x01400018, 0x01c00018},
};

static const u32 ar5416Bank6[][3] = {
	/* Addr      5G_HT20     5G_HT40   */
	{0x0000989c, 0x00000000, 0x00000000},
	{0x0000989c, 0x00000000, 0x00000000},
	{0x0000989c, 0x00000000, 0x00000000},
	{0x0000989c, 0x00e00000, 0x00e00000},
	{0x0000989c, 0x005e0000, 0x005e0000},
	{0x0000989c, 0x00120000, 0x00120000},
	{0x0000989c, 0x00620000, 0x00620000},
	{0x0000989c, 0x00020000, 0x00020000},
	{0x0000989c, 0x00ff0000, 0x00ff0000},
	{0x0000989c, 0x00ff0000, 0x00ff0000},
	{0x0000989c, 0x00ff0000, 0x00ff0000},
	{0x0000989c, 0x40ff0000, 0x40ff0000},
	{0x0000989c, 0x005f0000, 0x005f0000},
	{0x0000989c, 0x00870000, 0x00870000},
	{0x0000989c, 0x00f90000, 0x00f90000},
	{0x0000989c, 0x007b0000, 0x007b0000},
	{0x0000989c, 0x00ff0000, 0x00ff0000},
	{0x0000989c, 0x00f50000, 0x00f50000},
	{0x0000989c, 0x00dc0000, 0x00dc0000},
	{0x0000989c, 0x00110000, 0x00110000},
	{0x0000989c, 0x006100a8, 0x006100a8},
	{0x0000989c, 0x004210a2, 0x004210a2},
	{0x0000989c, 0x0014008f, 0x0014008f},
	{0x0000989c, 0x00c40003, 0x00c40003},
	{0x0000989c, 0x003000f2, 0x003000f2},
	{0x0000989c, 0x00440016, 0x00440016},
	{0x0000989c, 0x00410040, 0x00410040},
	{0x0000989c, 0x0001805e, 0x0001805e},
	{0x0000989c, 0x0000c0ab, 0x0000c0ab},
	{0x0000989c, 0x000000f1, 0x000000f1},
	{0x0000989c, 0x00002081, 0x00002081},
	{0x0000989c, 0x000000d4, 0x000000d4},
	{0x000098d0, 0x0000000f, 0x0010000f},
};

static const u32 ar5416Bank6TPC[][3] = {
	/* Addr      5G_HT20     5G_HT40   */
	{0x0000989c, 0x00000000, 0x00000000},
	{0x0000989c, 0x00000000, 0x00000000},
	{0x0000989c, 0x00000000, 0x00000000},
	{0x0000989c, 0x00e00000, 0x00e00000},
	{0x0000989c, 0x005e0000, 0x005e0000},
	{0x0000989c, 0x00120000, 0x00120000},
	{0x0000989c, 0x00620000, 0x00620000},
	{0x0000989c, 0x00020000, 0x00020000},
	{0x0000989c, 0x00ff0000, 0x00ff0000},
	{0x0000989c, 0x00ff0000, 0x00ff0000},
	{0x0000989c, 0x00ff0000, 0x00ff0000},
	{0x0000989c, 0x40ff0000, 0x40ff0000},
	{0x0000989c, 0x005f0000, 0x005f0000},
	{0x0000989c, 0x00870000, 0x00870000},
	{0x0000989c, 0x00f90000, 0x00f90000},
	{0x0000989c, 0x007b0000, 0x007b0000},
	{0x0000989c, 0x00ff0000, 0x00ff0000},
	{0x0000989c, 0x00f50000, 0x00f50000},
	{0x0000989c, 0x00dc0000, 0x00dc0000},
	{0x0000989c, 0x00110000, 0x00110000},
	{0x0000989c, 0x006100a8, 0x006100a8},
	{0x0000989c, 0x00423022, 0x00423022},
	{0x0000989c, 0x201400df, 0x201400df},
	{0x0000989c, 0x00c40002, 0x00c40002},
	{0x0000989c, 0x003000f2, 0x003000f2},
	{0x0000989c, 0x00440016, 0x00440016},
	{0x0000989c, 0x00410040, 0x00410040},
	{0x0000989c, 0x0001805e, 0x0001805e},
	{0x0000989c, 0x0000c0ab, 0x0000c0ab},
	{0x0000989c, 0x000000e1, 0x000000e1},
	{0x0000989c, 0x00007081, 0x00007081},
	{0x0000989c, 0x000000d4, 0x000000d4},
	{0x000098d0, 0x0000000f, 0x0010000f},
};

static const u32 ar5416Bank7[][2] = {
	/* Addr      allmodes  */
	{0x0000989c, 0x00000500},
	{0x0000989c, 0x00000800},
	{0x000098cc, 0x0000000e},
};

static const u32 ar5416Addac[][2] = {
	/* Addr      allmodes  */
	{0x0000989c, 0x00000000},
	{0x0000989c, 0x00000003},
	{0x0000989c, 0x00000000},
	{0x0000989c, 0x0000000c},
	{0x0000989c, 0x00000000},
	{0x0000989c, 0x00000030},
	{0x0000989c, 0x00000000},
	{0x0000989c, 0x00000000},
	{0x0000989c, 0x00000000},
	{0x0000989c, 0x00000000},
	{0x0000989c, 0x00000000},
	{0x0000989c, 0x00000000},
	{0x0000989c, 0x00000000},
	{0x0000989c, 0x00000000},
	{0x0000989c, 0x00000000},
	{0x0000989c, 0x00000000},
	{0x0000989c, 0x00000000},
	{0x0000989c, 0x00000000},
	{0x0000989c, 0x00000060},
	{0x0000989c, 0x00000000},
	{0x0000989c, 0x00000000},
	{0x0000989c, 0x00000000},
	{0x0000989c, 0x00000000},
	{0x0000989c, 0x00000000},
	{0x0000989c, 0x00000000},
	{0x0000989c, 0x00000000},
	{0x0000989c, 0x00000000},
	{0x0000989c, 0x00000000},
	{0x0000989c, 0x00000000},
	{0x0000989c, 0x00000000},
	{0x0000989c, 0x00000000},
	{0x0000989c, 0x00000058},
	{0x0000989c, 0x00000000},
	{0x0000989c, 0x00000000},
	{0x0000989c, 0x00000000},
	{0x0000989c, 0x00000000},
	{0x000098cc, 0x00000000},
};

