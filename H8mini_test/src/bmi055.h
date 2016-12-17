/*
The MIT License (MIT)

Copyright (c) 2016 BillyBag2

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

// These defines are taken from the data sheet.

// ACC ident.
#define BMI055_BGW_CHIPID          0x0
#define BMI055_BGW_CHIPID_VAL      0xFA

// ACC Data, XL,XH,YL,YH,ZL,ZH
#define BMI055_ACCD_X_LSB         0x02

//GYR data  XL,XH,YL,YH,ZL,ZH
#define BMI055_RATE_X_LSB         0x02

// GYR Ident
#define BMI055_CHIP_ID            0x0
#define BMI055_CHIP_ID_VAL        0xF

// ACC range
#define BMI055_PMU_RANGE           0x0f
#define BMI055_PMU_RANGE_16G				0x0c

// GYR range
#define BMI055_RANGE               0xF
#define BMI055_RANGE_2000DPS       0x0

// ACC filter
#define BMI055_PMU_BW              0x10
#define BMI055_PMU_BW_8HZ					0x8
#define BMI055_PMU_BW_16HZ					0x9
#define BMI055_PMU_BW_31HZ					0xa
#define BMI055_PMU_BW_63HZ					0xB
#define BMI055_PMU_BW_125HZ				0xC
#define BMI055_PMU_BW_250HZ				0xD
#define BMI055_PMU_BW_500HZ				0xE
#define BMI055_PMU_BW_1000HZ				0xF

//GYR filter
#define BMI055_BW                  0x10
#define BMI055_BW_32HZ             0x7
#define BMI055_BW_64HZ             0x6
#define BMI055_BW_12HZ             0x5
#define BMI055_BW_23HZ             0x4
#define BMI055_BW_47HZ             0x3
#define BMI055_BW_116HZ            0x2
#define BMI055_BW_230HZ            0x1
#define BMI055_BW_523HZ            0x0

#define BMI055_BGW_SOFTRESET       0x14
#define BMI055_BGW_SOFTRESET_RESET 0xB6 // Magic soft reset code.
