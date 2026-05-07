/* ----------------------------------------------------------------------
 * Project:      CMSIS DSP Library
 * Title:        arm_const_structs.c
 * Description:  Constant structs that are initialized for user convenience.
 *               For example, some can be given as arguments to the arm_cfft_f32() or arm_rfft_f32() functions.
 *
 * $Date:        27. January 2017
 * $Revision:    V.1.5.1
 *
 * Target Processor: Cortex-M cores
 * -------------------------------------------------------------------- */
/*
 * Copyright (C) 2010-2017 ARM Limited or its affiliates. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "arm_const_structs.h"

/* Floating-point structs */
const arm_cfft_instance_f32 arm_cfft_sR_f32_len16 = {
	16, twiddleCoef_16, armBitRevIndexTable16, ARMBITREVINDEXTABLE_16_TABLE_LENGTH
};
const arm_cfft_instance_f32 arm_cfft_sR_f32_len32 = {
	32, twiddleCoef_32, armBitRevIndexTable32, ARMBITREVINDEXTABLE_32_TABLE_LENGTH
};

const arm_cfft_instance_f32 arm_cfft_sR_f32_len64 = {
	64, twiddleCoef_64, armBitRevIndexTable64, ARMBITREVINDEXTABLE_64_TABLE_LENGTH
};

const arm_cfft_instance_f32 arm_cfft_sR_f32_len128 = {
	128, twiddleCoef_128, armBitRevIndexTable128, ARMBITREVINDEXTABLE_128_TABLE_LENGTH
};



/* Structure for real-value inputs */
/* Floating-point structs */
const arm_rfft_fast_instance_f32 arm_rfft_fast_sR_f32_len32 = {
	{ 16, twiddleCoef_32, armBitRevIndexTable32, ARMBITREVINDEXTABLE_16_TABLE_LENGTH },
	32U,
	(float32_t *)twiddleCoef_rfft_32
};

const arm_rfft_fast_instance_f32 arm_rfft_fast_sR_f32_len64 = {
	 { 32, twiddleCoef_32, armBitRevIndexTable32, ARMBITREVINDEXTABLE_32_TABLE_LENGTH },
	64U,
	(float32_t *)twiddleCoef_rfft_64
};

const arm_rfft_fast_instance_f32 arm_rfft_fast_sR_f32_len128 = {
	{ 64, twiddleCoef_64, armBitRevIndexTable64, ARMBITREVINDEXTABLE_64_TABLE_LENGTH },
	128U,
	(float32_t *)twiddleCoef_rfft_128
};

const arm_rfft_fast_instance_f32 arm_rfft_fast_sR_f32_len256 = {
	{ 128, twiddleCoef_128, armBitRevIndexTable128, ARMBITREVINDEXTABLE_128_TABLE_LENGTH },
	256U,
	(float32_t *)twiddleCoef_rfft_256
};
