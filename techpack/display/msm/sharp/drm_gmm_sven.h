/*
 * Copyright (C) 2021 SHARP CORPORATION
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef DRM_GMM_SVEN_H
#define DRM_GMM_SVEN_H
enum {
	DRM_GMM_SVEN_RED = 0,
	DRM_GMM_SVEN_GREEN,
	DRM_GMM_SVEN_BLUE,
};

#define DRM_SVEN_GMM_COLOR_SIZE 17

#define DRM_SVEN_GMM_COLOR_OFFSET 3
#define DRM_SVEN_GAM_MAX_SIZE (DRM_SVEN_GMM_COLOR_OFFSET + (DRM_SVEN_GMM_COLOR_SIZE*2*3))

static const int revise_gamma_3x17[3][DRM_SVEN_GMM_COLOR_SIZE] = {
		{//red
			0, 3, -59, -115, -195, -167, -132, -115, -118, -114, -39, 0, 0, 0, 0, 0, 0,
		},
		{//green
			0, 2, -41,  -82, -166, -142, -110,  -96, -101, -100, -36, 0, 0, 0, 0, 0, 0,
		},
		{//blue
			0, 0, -10,  -21, -283, -243, -204, -162, -144, -124, -42, 0, 0, 0, 0, 0, 0,
		},
};

/* +16 is extra buffer for dsi read */
static unsigned char otp_gamma_addr_value[][2+16] = {
	{0x00, 0x00},	//   0 : Bit[3][2]=VGMP[9:8], Bit[1][0]= VGSP[9:8]
	{0x01, 0x00},	//   1 : VGMP[7:0]
	{0x02, 0x00},	//   2 : VGSP[7:0]
	{0x14, 0x00},	//   3 : R00[11:8]
	{0x15, 0x00},	//   4 : R00[7:0]
	{0x1A, 0x00},	//   5 : R01[11:8]
	{0x1B, 0x00},	//   6 : R01[7:0]
	{0x20, 0x00},	//   7 : R02[11:8]
	{0x21, 0x00},	//   8 : R02[7:0]
	{0x26, 0x00},	//   9 : R03[11:8]
	{0x27, 0x00},	//  10 : R03[7:0]
	{0x2D, 0x00},	//  11 : R04[11:8]
	{0x2F, 0x00},	//  12 : R04[7:0]
	{0x34, 0x00},	//  13 : R05[11:8]
	{0x35, 0x00},	//  14 : R05[7:0]
	{0x3A, 0x00},	//  15 : R06[11:8]
	{0x3B, 0x00},	//  16 : R06[7:0]
	{0x42, 0x00},	//  17 : R07[11:8]
	{0x43, 0x00},	//  18 : R07[7:0]
	{0x48, 0x00},	//  19 : R08[11:8]
	{0x49, 0x00},	//  20 : R08[7:0]
	{0x4E, 0x00},	//  21 : R09[11:8]
	{0x4F, 0x00},	//  22 : R09[7:0]
	{0x54, 0x00},	//  23 : R10[11:8]
	{0x55, 0x00},	//  24 : R10[7:0]
	{0x5B, 0x00},	//  25 : R11[11:8]
	{0x5C, 0x00},	//  26 : R11[7:0]
	{0x61, 0x00},	//  27 : R12[11:8]
	{0x62, 0x00},	//  28 : R12[7:0]
	{0x67, 0x00},	//  29 : R13[11:8]
	{0x68, 0x00},	//  30 : R13[7:0]
	{0x6D, 0x00},	//  31 : R14[11:8]
	{0x6E, 0x00},	//  32 : R14[7:0]
	{0x73, 0x00},	//  33 : R15[11:8]
	{0x74, 0x00},	//  34 : R15[7:0]
	{0x79, 0x00},	//  35 : R16[11:8]
	{0x7A, 0x00},	//  36 : R16[7:0]
	{0x16, 0x00},	//  37 : G00[11:8]
	{0x17, 0x00},	//  38 : G00[7:0]
	{0x1C, 0x00},	//  39 : G01[11:8]
	{0x1D, 0x00},	//  40 : G01[7:0]
	{0x22, 0x00},	//  41 : G02[11:8]
	{0x23, 0x00},	//  42 : G02[7:0]
	{0x28, 0x00},	//  43 : G03[11:8]
	{0x29, 0x00},	//  44 : G03[7:0]
	{0x30, 0x00},	//  45 : G04[11:8]
	{0x31, 0x00},	//  46 : G04[7:0]
	{0x36, 0x00},	//  47 : G05[11:8]
	{0x37, 0x00},	//  48 : G05[7:0]
	{0x3D, 0x00},	//  49 : G06[11:8]
	{0x3F, 0x00},	//  50 : G06[7:0]
	{0x44, 0x00},	//  51 : G07[11:8]
	{0x45, 0x00},	//  52 : G07[7:0]
	{0x4A, 0x00},	//  53 : G08[11:8]
	{0x4B, 0x00},	//  54 : G08[7:0]
	{0x50, 0x00},	//  55 : G09[11:8]
	{0x51, 0x00},	//  56 : G09[7:0]
	{0x56, 0x00},	//  57 : G10[11:8]
	{0x58, 0x00},	//  58 : G10[7:0]
	{0x5D, 0x00},	//  59 : G11[11:8]
	{0x5E, 0x00},	//  60 : G11[7:0]
	{0x63, 0x00},	//  61 : G12[11:8]
	{0x64, 0x00},	//  62 : G12[7:0]
	{0x69, 0x00},	//  63 : G13[11:8]
	{0x6A, 0x00},	//  64 : G13[7:0]
	{0x6F, 0x00},	//  65 : G14[11:8]
	{0x70, 0x00},	//  66 : G14[7:0]
	{0x75, 0x00},	//  67 : G15[11:8]
	{0x76, 0x00},	//  68 : G15[7:0]
	{0x7B, 0x00},	//  69 : G16[11:8]
	{0x7C, 0x00},	//  70 : G16[7:0]
	{0x18, 0x00},	//  71 : B00[11:8]
	{0x19, 0x00},	//  72 : B00[7:0]
	{0x1E, 0x00},	//  73 : B01[11:8]
	{0x1F, 0x00},	//  74 : B01[7:0]
	{0x24, 0x00},	//  75 : B02[11:8]
	{0x25, 0x00},	//  76 : B02[7:0]
	{0x2A, 0x00},	//  77 : B03[11:8]
	{0x2B, 0x00},	//  78 : B03[7:0]
	{0x32, 0x00},	//  79 : B04[11:8]
	{0x33, 0x00},	//  80 : B04[7:0]
	{0x38, 0x00},	//  81 : B05[11:8]
	{0x39, 0x00},	//  82 : B05[7:0]
	{0x40, 0x00},	//  83 : B06[11:8]
	{0x41, 0x00},	//  84 : B06[7:0]
	{0x46, 0x00},	//  85 : B07[11:8]
	{0x47, 0x00},	//  86 : B07[7:0]
	{0x4C, 0x00},	//  87 : B08[11:8]
	{0x4D, 0x00},	//  88 : B08[7:0]
	{0x52, 0x00},	//  89 : B09[11:8]
	{0x53, 0x00},	//  90 : B09[7:0]
	{0x59, 0x00},	//  91 : B10[11:8]
	{0x5A, 0x00},	//  92 : B10[7:0]
	{0x5F, 0x00},	//  93 : B11[11:8]
	{0x60, 0x00},	//  94 : B11[7:0]
	{0x65, 0x00},	//  95 : B12[11:8]
	{0x66, 0x00},	//  96 : B12[7:0]
	{0x6B, 0x00},	//  97 : B13[11:8]
	{0x6C, 0x00},	//  98 : B13[7:0]
	{0x71, 0x00},	//  99 : B14[11:8]
	{0x72, 0x00},	// 100 : B14[7:0]
	{0x77, 0x00},	// 101 : B15[11:8]
	{0x78, 0x00},	// 102 : B15[7:0]
	{0x7D, 0x00},	// 103 : B16[11:8]
	{0x7E, 0x00},	// 104 : B16[7:0]
};

static const unsigned char page_addr_value[][2] = {
	{0xFE, 0x00},

	{0xFE, 0x50},
};

static const unsigned char gmm_band_addr_value[][3] = {
	{0x51, 0x0F, 0xFF},
	{0x51, 0x0F, 0xFF},
	{0x51, 0x00, 0x2C},
};


static const unsigned char write_ram_addr_value[][2] = {
	{0xFE, 0x40},
	{0xE3, 0x48},
};

static const unsigned char write_prev_addr_value[][2] = {
	{0xFE, 0x84},
	{0xDF, 0x93},
	{0xFE, 0x70},
	{0x71, 0x00},
	{0x72, 0x31},
};

static const unsigned char write_post_addr_value[][2] = {
	{0xFE, 0x70},
	{0x71, 0x01},
	{0xFE, 0x70},
	{0x72, 0x30},
	{0xFE, 0x84},
	{0xDF, 0x13},
};

static struct dsi_cmd_desc read_cmd[] = {
	{{0, MIPI_DSI_GENERIC_LONG_WRITE,           MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &page_addr_value[0][0]       , 0, NULL                         }, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE,           MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 3, &gmm_band_addr_value[0][0]   , 0, NULL                         }, 1, 50},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE,           MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &page_addr_value[1][0]       , 0, NULL                         }, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[  0][0], 1, &otp_gamma_addr_value[  0][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[  1][0], 1, &otp_gamma_addr_value[  1][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[  2][0], 1, &otp_gamma_addr_value[  2][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[  3][0], 1, &otp_gamma_addr_value[  3][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[  4][0], 1, &otp_gamma_addr_value[  4][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[  5][0], 1, &otp_gamma_addr_value[  5][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[  6][0], 1, &otp_gamma_addr_value[  6][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[  7][0], 1, &otp_gamma_addr_value[  7][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[  8][0], 1, &otp_gamma_addr_value[  8][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[  9][0], 1, &otp_gamma_addr_value[  9][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 10][0], 1, &otp_gamma_addr_value[ 10][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 11][0], 1, &otp_gamma_addr_value[ 11][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 12][0], 1, &otp_gamma_addr_value[ 12][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 13][0], 1, &otp_gamma_addr_value[ 13][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 14][0], 1, &otp_gamma_addr_value[ 14][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 15][0], 1, &otp_gamma_addr_value[ 15][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 16][0], 1, &otp_gamma_addr_value[ 16][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 17][0], 1, &otp_gamma_addr_value[ 17][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 18][0], 1, &otp_gamma_addr_value[ 18][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 19][0], 1, &otp_gamma_addr_value[ 19][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 20][0], 1, &otp_gamma_addr_value[ 20][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 21][0], 1, &otp_gamma_addr_value[ 21][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 22][0], 1, &otp_gamma_addr_value[ 22][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 23][0], 1, &otp_gamma_addr_value[ 23][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 24][0], 1, &otp_gamma_addr_value[ 24][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 25][0], 1, &otp_gamma_addr_value[ 25][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 26][0], 1, &otp_gamma_addr_value[ 26][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 27][0], 1, &otp_gamma_addr_value[ 27][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 28][0], 1, &otp_gamma_addr_value[ 28][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 29][0], 1, &otp_gamma_addr_value[ 29][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 30][0], 1, &otp_gamma_addr_value[ 30][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 31][0], 1, &otp_gamma_addr_value[ 31][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 32][0], 1, &otp_gamma_addr_value[ 32][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 33][0], 1, &otp_gamma_addr_value[ 33][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 34][0], 1, &otp_gamma_addr_value[ 34][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 35][0], 1, &otp_gamma_addr_value[ 35][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 36][0], 1, &otp_gamma_addr_value[ 36][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 37][0], 1, &otp_gamma_addr_value[ 37][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 38][0], 1, &otp_gamma_addr_value[ 38][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 39][0], 1, &otp_gamma_addr_value[ 39][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 40][0], 1, &otp_gamma_addr_value[ 40][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 41][0], 1, &otp_gamma_addr_value[ 41][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 42][0], 1, &otp_gamma_addr_value[ 42][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 43][0], 1, &otp_gamma_addr_value[ 43][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 44][0], 1, &otp_gamma_addr_value[ 44][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 45][0], 1, &otp_gamma_addr_value[ 45][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 46][0], 1, &otp_gamma_addr_value[ 46][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 47][0], 1, &otp_gamma_addr_value[ 47][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 48][0], 1, &otp_gamma_addr_value[ 48][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 49][0], 1, &otp_gamma_addr_value[ 49][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 50][0], 1, &otp_gamma_addr_value[ 50][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 51][0], 1, &otp_gamma_addr_value[ 51][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 52][0], 1, &otp_gamma_addr_value[ 52][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 53][0], 1, &otp_gamma_addr_value[ 53][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 54][0], 1, &otp_gamma_addr_value[ 54][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 55][0], 1, &otp_gamma_addr_value[ 55][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 56][0], 1, &otp_gamma_addr_value[ 56][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 57][0], 1, &otp_gamma_addr_value[ 57][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 58][0], 1, &otp_gamma_addr_value[ 58][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 59][0], 1, &otp_gamma_addr_value[ 59][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 60][0], 1, &otp_gamma_addr_value[ 60][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 61][0], 1, &otp_gamma_addr_value[ 61][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 62][0], 1, &otp_gamma_addr_value[ 62][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 63][0], 1, &otp_gamma_addr_value[ 63][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 64][0], 1, &otp_gamma_addr_value[ 64][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 65][0], 1, &otp_gamma_addr_value[ 65][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 66][0], 1, &otp_gamma_addr_value[ 66][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 67][0], 1, &otp_gamma_addr_value[ 67][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 68][0], 1, &otp_gamma_addr_value[ 68][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 69][0], 1, &otp_gamma_addr_value[ 69][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 70][0], 1, &otp_gamma_addr_value[ 70][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 71][0], 1, &otp_gamma_addr_value[ 71][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 72][0], 1, &otp_gamma_addr_value[ 72][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 73][0], 1, &otp_gamma_addr_value[ 73][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 74][0], 1, &otp_gamma_addr_value[ 74][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 75][0], 1, &otp_gamma_addr_value[ 75][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 76][0], 1, &otp_gamma_addr_value[ 76][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 77][0], 1, &otp_gamma_addr_value[ 77][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 78][0], 1, &otp_gamma_addr_value[ 78][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 79][0], 1, &otp_gamma_addr_value[ 79][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 80][0], 1, &otp_gamma_addr_value[ 80][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 81][0], 1, &otp_gamma_addr_value[ 81][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 82][0], 1, &otp_gamma_addr_value[ 82][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 83][0], 1, &otp_gamma_addr_value[ 83][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 84][0], 1, &otp_gamma_addr_value[ 84][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 85][0], 1, &otp_gamma_addr_value[ 85][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 86][0], 1, &otp_gamma_addr_value[ 86][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 87][0], 1, &otp_gamma_addr_value[ 87][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 88][0], 1, &otp_gamma_addr_value[ 88][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 89][0], 1, &otp_gamma_addr_value[ 89][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 90][0], 1, &otp_gamma_addr_value[ 90][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 91][0], 1, &otp_gamma_addr_value[ 91][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 92][0], 1, &otp_gamma_addr_value[ 92][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 93][0], 1, &otp_gamma_addr_value[ 93][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 94][0], 1, &otp_gamma_addr_value[ 94][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 95][0], 1, &otp_gamma_addr_value[ 95][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 96][0], 1, &otp_gamma_addr_value[ 96][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 97][0], 1, &otp_gamma_addr_value[ 97][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 98][0], 1, &otp_gamma_addr_value[ 98][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[ 99][0], 1, &otp_gamma_addr_value[ 99][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[100][0], 1, &otp_gamma_addr_value[100][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[101][0], 1, &otp_gamma_addr_value[101][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[102][0], 1, &otp_gamma_addr_value[102][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[103][0], 1, &otp_gamma_addr_value[103][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 1, &otp_gamma_addr_value[104][0], 1, &otp_gamma_addr_value[104][1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE,           MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &page_addr_value[0][0]       , 0, NULL                         }, 1, 0},
};

static struct dsi_cmd_desc write_cmd[] = {
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &write_prev_addr_value[0][0] , 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &write_prev_addr_value[1][0] , 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &write_prev_addr_value[2][0] , 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &write_prev_addr_value[3][0] , 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &write_prev_addr_value[4][0] , 0, NULL}, 1, 0},

	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &page_addr_value[0][0]       , 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 3, &gmm_band_addr_value[1][0]   , 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &page_addr_value[1][0]       , 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[  0][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[  1][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[  2][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[  3][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[  4][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[  5][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[  6][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[  7][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[  8][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[  9][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 10][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 11][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 12][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 13][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 14][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 15][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 16][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 17][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 18][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 19][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 20][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 21][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 22][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 23][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 24][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 25][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 26][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 27][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 28][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 29][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 30][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 31][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 32][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 33][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 34][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 35][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 36][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 37][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 38][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 39][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 40][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 41][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 42][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 43][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 44][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 45][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 46][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 47][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 48][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 49][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 50][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 51][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 52][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 53][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 54][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 55][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 56][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 57][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 58][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 59][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 60][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 61][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 62][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 63][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 64][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 65][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 66][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 67][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 68][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 69][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 70][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 71][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 72][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 73][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 74][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 75][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 76][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 77][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 78][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 79][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 80][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 81][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 82][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 83][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 84][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 85][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 86][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 87][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 88][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 89][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 90][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 91][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 92][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 93][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 94][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 95][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 96][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 97][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 98][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[ 99][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[100][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[101][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[102][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[103][0], 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &otp_gamma_addr_value[104][0], 0, NULL}, 1, 0},

	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &write_ram_addr_value[0][0]  , 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &write_ram_addr_value[1][0]  , 0, NULL}, 1, 0},

	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &write_post_addr_value[0][0] , 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &write_post_addr_value[1][0] , 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &write_post_addr_value[2][0] , 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &write_post_addr_value[3][0] , 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &write_post_addr_value[4][0] , 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &write_post_addr_value[5][0] , 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, &page_addr_value[0][0]       , 0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 3, &gmm_band_addr_value[2][0]   , 0, NULL}, 1, 0},
};

#endif /* DRM_GMM_SVEN_H */
