#define __PROG_TYPES_COMPAT__
#include <avr/pgmspace.h>
#include "Arduino.h"
#include "tables.h"

// KLS adjustments for KVS=0, starting fron KLS=1
const prog_uchar PROGMEM kls_kvs0[99][32] =
{
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 3, 3},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 3, 3, 4, 4},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 3, 3, 4, 5, 5},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 8},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 8, 9},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 8, 10, 11},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 9, 10, 12},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 8, 10, 12, 13},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 9, 11, 13, 14},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 8, 10, 12, 14, 16},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 3, 4, 4, 5, 6, 7, 9, 11, 13, 15, 17},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 3, 4, 4, 5, 7, 8, 9, 11, 14, 16, 18},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 9, 10, 12, 15, 17, 20},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 9, 11, 13, 15, 18, 21},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 4, 5, 6, 7, 8, 10, 12, 14, 17, 20, 22},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 8, 10, 12, 15, 17, 21, 23},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 9, 11, 13, 15, 18, 22, 25},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 4, 4, 6, 7, 8, 10, 11, 14, 16, 20, 23, 26},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 8, 10, 12, 14, 17, 20, 24, 27},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 9, 10, 13, 15, 18, 21, 25, 29},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 9, 11, 13, 15, 19, 22, 26, 30},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 3, 4, 4, 5, 7, 8, 10, 11, 14, 16, 20, 23, 28, 31},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 8, 10, 12, 14, 17, 20, 24, 29, 33},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 9, 10, 12, 15, 18, 21, 25, 30, 34},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 3, 3, 4, 5, 6, 8, 9, 11, 13, 15, 18, 22, 26, 31, 35},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 3, 4, 4, 5, 6, 8, 9, 11, 13, 16, 19, 23, 27, 32, 36},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 3, 4, 4, 5, 7, 8, 10, 12, 14, 16, 20, 24, 28, 33, 38},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 3, 4, 5, 6, 7, 8, 10, 12, 14, 17, 20, 24, 29, 34, 39},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 9, 10, 12, 15, 18, 21, 25, 30, 35, 40},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 9, 11, 13, 15, 18, 22, 26, 31, 37, 42},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 9, 11, 13, 16, 19, 22, 27, 32, 38, 43},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 3, 4, 4, 5, 6, 8, 10, 11, 14, 16, 19, 23, 28, 33, 39, 44},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 3, 4, 5, 5, 7, 8, 10, 12, 14, 17, 20, 24, 28, 34, 40, 45},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 3, 4, 5, 6, 7, 8, 10, 12, 14, 17, 20, 24, 29, 35, 41, 47},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 8, 10, 12, 15, 17, 21, 25, 30, 35, 42, 48},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 9, 11, 13, 15, 18, 22, 26, 31, 37, 43, 49},
	{0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 9, 11, 13, 16, 19, 22, 26, 32, 38, 45, 51},
	{0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 9, 11, 13, 16, 19, 23, 27, 32, 38, 45, 52},
	{0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 3, 4, 4, 5, 6, 8, 9, 12, 14, 16, 20, 23, 28, 33, 40, 47, 53},
	{0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 3, 3, 4, 4, 6, 7, 8, 10, 12, 14, 17, 20, 24, 28, 34, 40, 48, 54},
	{0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 3, 3, 4, 5, 6, 7, 8, 10, 12, 14, 17, 20, 24, 29, 35, 41, 49, 56},
	{0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 8, 10, 12, 15, 18, 21, 25, 30, 36, 43, 50, 57},
	{0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 8, 10, 13, 15, 18, 21, 26, 30, 36, 43, 51, 58},
	{0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 9, 11, 13, 15, 18, 22, 26, 31, 37, 44, 52, 60},
	{0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 9, 11, 13, 16, 19, 22, 27, 32, 38, 45, 53, 61},
	{0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 3, 3, 4, 5, 6, 8, 9, 11, 14, 16, 19, 23, 27, 32, 39, 46, 55, 62},
	{0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 3, 4, 4, 5, 7, 8, 9, 11, 14, 16, 20, 23, 28, 33, 40, 47, 56, 63},
	{0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 3, 4, 4, 5, 7, 8, 9, 12, 14, 17, 20, 24, 28, 34, 40, 48, 57, 65},
	{0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 3, 4, 4, 6, 7, 8, 10, 12, 14, 17, 20, 24, 29, 35, 41, 49, 58, 66},
	{0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 3, 4, 5, 6, 7, 8, 10, 12, 15, 17, 21, 25, 30, 35, 42, 50, 59, 67},
	{0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 3, 4, 5, 6, 7, 8, 10, 12, 15, 18, 21, 25, 30, 36, 43, 51, 60, 69},
	{0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 9, 10, 12, 15, 18, 22, 26, 31, 36, 44, 52, 61, 70},
	{0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 9, 10, 13, 16, 18, 22, 26, 31, 37, 45, 53, 63, 71},
	{0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 9, 11, 13, 16, 19, 22, 27, 32, 38, 45, 54, 63, 72},
	{0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 4, 4, 5, 6, 8, 9, 11, 13, 16, 19, 23, 27, 32, 38, 46, 55, 65, 74},
	{0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 4, 4, 5, 6, 8, 9, 11, 13, 16, 19, 23, 28, 33, 39, 47, 56, 66, 75},
	{0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 4, 4, 5, 6, 8, 9, 11, 14, 17, 20, 24, 28, 33, 40, 48, 57, 67, 76},
	{0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 4, 4, 5, 7, 8, 10, 11, 14, 17, 20, 24, 29, 34, 41, 49, 58, 68, 78},
	{0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 4, 4, 5, 7, 8, 10, 12, 14, 17, 20, 24, 29, 35, 41, 49, 59, 69, 79},
	{0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 4, 5, 6, 7, 8, 10, 12, 14, 18, 21, 25, 30, 35, 42, 50, 60, 70, 80},
	{0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 4, 5, 6, 7, 9, 10, 12, 15, 18, 21, 25, 30, 36, 43, 51, 61, 72, 82},
	{0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 4, 5, 6, 7, 9, 10, 12, 15, 18, 21, 26, 30, 36, 43, 52, 61, 73, 83},
	{0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 4, 5, 6, 7, 9, 10, 12, 15, 18, 22, 26, 31, 37, 44, 53, 63, 74, 84},
	{0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 9, 11, 13, 15, 19, 22, 26, 31, 37, 45, 53, 63, 75, 85},
	{0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 9, 11, 13, 16, 19, 22, 27, 32, 38, 45, 54, 64, 76, 87},
	{0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 9, 11, 13, 16, 19, 23, 27, 32, 39, 46, 55, 65, 77, 88},
	{0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5, 6, 8, 9, 11, 13, 16, 20, 23, 28, 33, 39, 47, 56, 66, 78, 89},
	{0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 5, 5, 6, 8, 10, 11, 13, 16, 20, 23, 28, 33, 40, 47, 57, 67, 80, 91},
	{0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 5, 5, 6, 8, 10, 11, 14, 16, 20, 24, 28, 34, 40, 48, 57, 68, 80, 92},
	{0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 4, 5, 5, 6, 8, 10, 12, 14, 17, 20, 24, 29, 34, 41, 49, 58, 69, 82, 93},
	{0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 4, 5, 5, 7, 8, 10, 12, 14, 17, 21, 24, 29, 35, 41, 49, 59, 70, 83, 94},
	{0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 8, 10, 12, 14, 17, 21, 25, 30, 35, 42, 50, 60, 71, 84, 96},
	{0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 8, 10, 12, 14, 17, 21, 25, 30, 36, 43, 51, 60, 72, 85, 97},
	{0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 8, 10, 12, 15, 18, 22, 25, 30, 36, 43, 51, 61, 73, 86, 98},
	{0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 9, 10, 12, 15, 18, 22, 26, 31, 37, 44, 52, 62, 74, 87, 100},
	{0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 9, 11, 13, 15, 18, 22, 26, 31, 37, 44, 53, 63, 75, 88, 101},
	{0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 9, 11, 13, 15, 18, 22, 26, 32, 38, 45, 53, 64, 76, 90, 102},
	{0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 9, 11, 13, 15, 19, 23, 27, 32, 38, 45, 54, 65, 77, 91, 103},
	{0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 9, 11, 13, 16, 19, 23, 27, 32, 39, 46, 55, 65, 78, 92, 105},
	{0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 9, 11, 13, 16, 19, 23, 27, 33, 39, 47, 55, 66, 79, 93, 106},
	{0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 2, 3, 3, 4, 5, 6, 8, 9, 11, 13, 16, 19, 24, 28, 33, 40, 47, 56, 67, 80, 94, 107},
	{0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 2, 3, 3, 4, 5, 6, 8, 9, 11, 14, 16, 20, 24, 28, 34, 40, 48, 57, 68, 81, 95, 109},
	{0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 3, 4, 6, 6, 8, 9, 12, 14, 16, 20, 24, 28, 34, 41, 48, 57, 69, 82, 96, 110},
	{0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 3, 4, 6, 7, 8, 10, 12, 14, 17, 20, 24, 29, 35, 41, 49, 58, 70, 83, 98, 111},
	{0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 3, 4, 6, 7, 8, 10, 12, 14, 17, 20, 25, 29, 35, 41, 49, 59, 70, 83, 98, 112},
	{0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 6, 7, 8, 10, 12, 14, 17, 21, 25, 29, 35, 42, 50, 59, 71, 84, 100, 114},
	{0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 6, 7, 8, 10, 12, 14, 17, 21, 25, 30, 36, 43, 51, 60, 72, 86, 101, 115},
	{0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 8, 10, 12, 15, 17, 21, 26, 30, 36, 43, 51, 61, 73, 86, 102, 116},
	{0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 8, 10, 12, 15, 18, 21, 26, 31, 37, 43, 52, 62, 74, 87, 103, 118},
	{0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 8, 10, 13, 15, 18, 21, 26, 31, 37, 44, 52, 62, 74, 88, 104, 119},
	{0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 8, 10, 13, 15, 18, 22, 26, 31, 37, 44, 53, 63, 75, 89, 105, 120},
	{0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 9, 10, 13, 15, 18, 22, 27, 31, 38, 45, 53, 63, 76, 90, 106, 121},
	{0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 9, 11, 13, 15, 18, 22, 27, 32, 38, 45, 54, 64, 77, 91, 108, 123},
	{0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 9, 11, 13, 16, 19, 22, 27, 32, 39, 46, 55, 65, 78, 92, 109, 124},
	{0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 9, 11, 13, 16, 19, 23, 28, 32, 39, 46, 55, 65, 78, 93, 110, 125},
	{0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 9, 11, 13, 16, 19, 23, 28, 33, 39, 47, 56, 66, 79, 94, 111, 127}
};

// KLS TL adjustments for KVS=1..7, starting fron KLS=1
const prog_uchar PROGMEM kls_kvs17[99][32] =
{
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 4, 5},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 4, 5, 6},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 7},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 8, 9},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 4, 5, 6, 7, 9, 10},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 3, 3, 4, 4, 5, 6, 7, 9, 10, 11},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 8, 9, 11, 12},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 4, 5, 6, 7, 9, 10, 12, 14},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 8, 9, 11, 13, 15},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 3, 3, 4, 4, 5, 6, 7, 9, 10, 12, 14, 16},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 8, 9, 11, 13, 16, 18},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 8, 10, 12, 14, 17, 19},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5, 5, 6, 8, 9, 11, 13, 15, 18, 20},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 8, 9, 11, 13, 16, 19, 21},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 3, 3, 4, 4, 5, 6, 7, 9, 10, 12, 14, 17, 20, 23},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 4, 5, 6, 8, 9, 11, 13, 15, 18, 21, 24},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 8, 9, 11, 13, 16, 19, 22, 25},
	{0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 8, 10, 12, 14, 17, 20, 24, 27},
	{0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 4, 5, 6, 7, 9, 10, 12, 15, 18, 21, 25, 28},
	{0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 4, 5, 7, 8, 9, 11, 13, 15, 18, 22, 26, 29},
	{0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 8, 10, 11, 13, 16, 19, 23, 27, 30},
	{0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 4, 4, 5, 6, 7, 8, 10, 12, 14, 17, 20, 24, 28, 32},
	{0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 4, 4, 5, 6, 7, 9, 10, 12, 15, 18, 21, 25, 29, 33},
	{0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 3, 3, 4, 4, 5, 6, 8, 9, 11, 13, 15, 18, 22, 26, 30, 34},
	{0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5, 5, 7, 8, 9, 11, 13, 16, 19, 23, 27, 32, 36},
	{0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 8, 10, 12, 14, 16, 19, 23, 27, 32, 37},
	{0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 9, 10, 12, 14, 17, 20, 24, 29, 34, 38},
	{0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 4, 4, 5, 6, 7, 9, 10, 12, 15, 17, 21, 25, 29, 35, 39},
	{0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 3, 3, 4, 4, 5, 6, 8, 9, 11, 13, 15, 18, 21, 26, 30, 36, 41},
	{0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 3, 3, 4, 5, 5, 6, 8, 9, 11, 13, 16, 19, 22, 27, 32, 37, 42},
	{0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 8, 10, 11, 14, 16, 19, 23, 27, 32, 38, 43},
	{0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 8, 10, 12, 14, 17, 20, 24, 28, 33, 39, 45},
	{0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 3, 3, 3, 4, 5, 6, 7, 8, 10, 12, 14, 17, 20, 24, 29, 34, 40, 46},
	{0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 3, 3, 4, 4, 5, 6, 7, 9, 11, 12, 15, 18, 21, 25, 30, 35, 42, 47},
	{0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 4, 5, 6, 7, 9, 11, 13, 15, 18, 21, 25, 30, 36, 42, 48},
	{0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 4, 5, 6, 8, 9, 11, 13, 16, 19, 22, 26, 31, 37, 44, 50},
	{0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 8, 9, 11, 13, 16, 19, 23, 27, 32, 38, 45, 51},
	{0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 8, 10, 12, 14, 16, 19, 23, 27, 33, 39, 46, 52},
	{0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 8, 10, 12, 14, 17, 20, 24, 28, 34, 40, 47, 54},	// CHECK THIS !!!
	{0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 8, 10, 12, 14, 17, 20, 24, 28, 34, 40, 47, 54},	// CHECK THIS !!!
	{0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 4, 4, 5, 6, 7, 9, 10, 13, 15, 18, 21, 25, 30, 35, 42, 49, 56},
	{0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 4, 4, 5, 6, 7, 9, 11, 13, 15, 18, 22, 26, 30, 36, 43, 51, 58},
	{0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 4, 5, 6, 8, 9, 11, 13, 15, 18, 22, 26, 31, 37, 44, 52, 59},
	{0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 4, 5, 7, 8, 9, 11, 13, 16, 19, 22, 27, 32, 38, 45, 53, 60},
	{0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 8, 9, 11, 14, 16, 19, 23, 27, 32, 38, 46, 54, 61},
	{0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 8, 10, 12, 14, 16, 20, 23, 28, 33, 39, 47, 55, 63},
	{0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 4, 4, 5, 6, 7, 8, 10, 12, 14, 17, 20, 24, 28, 34, 40, 48, 56, 64},
	{0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 4, 4, 5, 6, 7, 8, 10, 12, 15, 17, 20, 24, 29, 34, 41, 49, 57, 65},
	{0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 4, 4, 5, 6, 7, 9, 10, 12, 15, 18, 21, 25, 30, 35, 42, 50, 59, 67},
	{0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 4, 4, 5, 6, 7, 9, 10, 12, 15, 18, 21, 25, 30, 36, 43, 50, 60, 68},
	{0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 4, 4, 5, 6, 8, 9, 11, 13, 15, 18, 22, 26, 31, 36, 43, 52, 61, 69},
	{0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 4, 4, 5, 6, 8, 9, 11, 13, 16, 18, 22, 26, 31, 37, 44, 52, 62, 70},
	{0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5, 5, 6, 8, 9, 11, 13, 16, 19, 23, 27, 32, 38, 45, 53, 63, 72},
	{0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5, 5, 7, 8, 9, 11, 13, 16, 19, 23, 27, 32, 38, 46, 54, 64, 73},
	{0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 8, 10, 11, 14, 17, 19, 23, 28, 33, 39, 47, 55, 65, 74},
	{0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 8, 10, 12, 14, 17, 20, 24, 28, 34, 40, 48, 56, 67, 76},
	{0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 2, 3, 3, 4, 5, 6, 7, 8, 10, 12, 14, 17, 20, 24, 29, 34, 40, 48, 57, 67, 77},
	{0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 2, 3, 3, 4, 5, 6, 7, 9, 10, 12, 14, 17, 21, 25, 29, 35, 41, 49, 58, 69, 78},
	{0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 2, 3, 3, 4, 5, 6, 7, 9, 10, 12, 15, 18, 21, 25, 30, 35, 42, 50, 59, 70, 79},
	{0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 5, 6, 7, 9, 10, 12, 15, 18, 21, 25, 30, 36, 42, 51, 60, 71, 81},
	{0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5, 5, 6, 7, 9, 11, 13, 15, 18, 22, 26, 31, 36, 43, 52, 61, 72, 82},
	{0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5, 5, 6, 8, 9, 11, 13, 15, 19, 22, 26, 31, 37, 44, 52, 62, 73, 83},
	{0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5, 5, 6, 8, 9, 11, 13, 16, 19, 22, 27, 32, 38, 44, 53, 63, 74, 85},
	{0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5, 5, 6, 8, 9, 11, 13, 16, 19, 23, 27, 32, 38, 45, 54, 64, 75, 86},
	{0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5, 5, 6, 8, 10, 11, 13, 16, 19, 23, 27, 32, 39, 46, 55, 65, 77, 87},
	{0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 8, 10, 11, 13, 16, 20, 23, 28, 33, 39, 46, 55, 66, 77, 88},
	{0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 8, 10, 12, 14, 17, 20, 24, 28, 33, 40, 47, 56, 67, 79, 90},
	{0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 3, 3, 4, 5, 6, 7, 8, 10, 12, 14, 17, 20, 24, 29, 34, 40, 48, 57, 68, 80, 91},
	{0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 3, 3, 4, 5, 6, 7, 8, 10, 12, 14, 17, 21, 24, 29, 34, 41, 48, 58, 69, 81, 92},
	{0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 3, 3, 4, 5, 6, 7, 8, 10, 12, 14, 17, 21, 25, 29, 35, 41, 49, 59, 70, 82, 94},
	{0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 3, 3, 4, 5, 6, 7, 9, 10, 12, 14, 17, 21, 25, 30, 35, 42, 50, 59, 71, 83, 95},
	{0, 0, 0, 0, 0, 1, 1, 2, 2, 2, 3, 3, 3, 4, 5, 6, 7, 9, 11, 12, 15, 18, 21, 25, 30, 36, 43, 51, 60, 72, 84, 96},
	{0, 0, 0, 0, 0, 1, 1, 2, 2, 2, 3, 3, 3, 4, 5, 6, 7, 9, 11, 13, 15, 18, 22, 26, 30, 36, 43, 51, 61, 72, 85, 97},
	{0, 0, 0, 0, 0, 1, 1, 2, 2, 2, 3, 3, 3, 4, 5, 6, 7, 9, 11, 13, 15, 18, 22, 26, 31, 37, 44, 52, 62, 73, 87, 99},
	{0, 0, 0, 0, 0, 1, 1, 2, 2, 2, 3, 3, 4, 4, 5, 6, 7, 9, 11, 13, 15, 18, 22, 26, 31, 37, 44, 53, 63, 75, 88, 100},
	{0, 0, 0, 0, 0, 1, 1, 2, 2, 2, 3, 3, 4, 4, 6, 6, 8, 9, 11, 13, 15, 19, 23, 27, 32, 38, 45, 53, 63, 75, 89, 101},
	{0, 0, 0, 0, 0, 1, 1, 2, 2, 2, 3, 3, 4, 4, 6, 6, 8, 9, 11, 13, 16, 19, 23, 27, 32, 38, 45, 54, 64, 76, 90, 103},
	{0, 0, 0, 0, 0, 1, 1, 2, 2, 2, 3, 3, 4, 4, 6, 7, 8, 9, 11, 13, 16, 19, 23, 27, 33, 39, 46, 54, 65, 77, 91, 104},
	{0, 0, 0, 0, 0, 1, 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 8, 9, 12, 14, 16, 19, 23, 28, 33, 39, 47, 55, 66, 78, 92, 105},
	{0, 0, 0, 0, 0, 1, 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 8, 10, 12, 14, 16, 20, 24, 28, 33, 40, 47, 56, 67, 79, 93, 106},
	{0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 3, 4, 5, 6, 7, 8, 10, 12, 14, 16, 20, 24, 28, 34, 40, 48, 57, 68, 80, 95, 108},
	{0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 3, 4, 5, 6, 7, 8, 10, 12, 14, 17, 20, 24, 29, 34, 41, 48, 57, 68, 81, 96, 109},
	{0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 3, 4, 5, 6, 7, 8, 10, 12, 14, 17, 20, 25, 29, 35, 41, 49, 58, 69, 82, 97, 110},
	{0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 6, 7, 8, 10, 12, 14, 17, 21, 25, 29, 35, 42, 49, 59, 70, 83, 98, 112},
	{0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 6, 7, 8, 10, 12, 15, 17, 21, 25, 30, 35, 42, 50, 59, 71, 84, 99, 113},
	{0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 6, 7, 8, 10, 13, 15, 17, 21, 25, 30, 36, 42, 51, 60, 72, 85, 100, 114},
	{0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 6, 7, 9, 10, 13, 15, 18, 21, 26, 30, 36, 43, 51, 61, 73, 86, 102, 116},
	{0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 6, 7, 9, 11, 13, 15, 18, 21, 26, 31, 37, 43, 52, 61, 73, 87, 102, 117},
	{0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 6, 7, 9, 11, 13, 15, 18, 22, 26, 31, 37, 44, 52, 62, 74, 88, 104, 118},
	{0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 7, 7, 9, 11, 13, 15, 18, 22, 27, 31, 37, 44, 53, 63, 75, 89, 105, 119},
	{0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 7, 8, 9, 11, 13, 16, 18, 22, 27, 32, 38, 45, 53, 63, 76, 90, 106, 121},
	{0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 7, 8, 9, 11, 13, 16, 19, 22, 27, 32, 38, 45, 54, 64, 76, 91, 107, 122},
	{0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 7, 8, 9, 11, 14, 16, 19, 23, 27, 32, 39, 46, 55, 65, 77, 92, 108, 123},
	{0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 7, 8, 9, 11, 14, 16, 19, 23, 28, 33, 39, 46, 55, 65, 78, 93, 109, 125},
	{0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 7, 8, 9, 11, 14, 16, 19, 23, 28, 33, 39, 47, 56, 66, 79, 94, 110, 126},
	{0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 7, 8, 9, 11, 14, 16, 19, 23, 28, 33, 40, 47, 56, 67, 80, 95, 112, 127}
};

// KVS adjustments, starting from KVS=1
const prog_uchar PROGMEM kvs[7][128] =
{
	{
		6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
		7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
		7, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8,
		8, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9,
		9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10, 10,
		10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
		12, 12, 12, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13,
		14, 14, 14, 14, 14, 15, 15, 15, 15, 16, 16, 16, 17, 17, 17, 18
	},
	{
		5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6,
		6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
		7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9,
		9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11,
		11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13,
		13, 13, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 16, 16,
		16, 16, 17, 17, 17, 17, 17, 18, 18, 18, 18, 19, 19, 19, 20, 20,
		20, 21, 21, 21, 22, 22, 23, 23, 24, 24, 25, 25, 26, 27, 28, 29
	},
	{
		4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
		6, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7,
		7, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 10, 10,
		10, 10, 10, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 13, 13,
		13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15, 15, 16, 16, 16, 16,
		16, 17, 17, 17, 17, 18, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20,
		21, 21, 21, 21, 22, 22, 23, 23, 23, 24, 24, 24, 25, 25, 26, 26,
		27, 27, 28, 28, 29, 30, 30, 31, 32, 33, 34, 34, 36, 37, 38, 40
	},
	{
		3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5,
		5, 5, 5, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 8,
		8, 8, 8, 8, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 11, 11,
		11, 11, 12, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 15, 15,
		15, 15, 16, 16, 16, 17, 17, 17, 17, 18, 18, 18, 19, 19, 19, 19,
		20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 24, 25,
		25, 26, 26, 26, 27, 27, 28, 28, 29, 29, 30, 30, 31, 32, 32, 33,
		34, 34, 35, 36, 37, 38, 38, 39, 40, 41, 43, 44, 45, 47, 48, 51
	},
	{
		2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4,
		5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 7, 7, 7, 7, 8,
		8, 8, 8, 9, 9, 9, 9, 10, 10, 10, 10, 10, 11, 11, 11, 12,
		12, 12, 13, 13, 13, 14, 14, 14, 15, 15, 15, 15, 16, 16, 16, 17,
		17, 17, 18, 18, 19, 19, 19, 20, 20, 20, 20, 21, 21, 22, 22, 22,
		23, 23, 23, 24, 24, 25, 25, 25, 26, 26, 27, 27, 28, 28, 29, 29,
		30, 30, 31, 31, 32, 32, 33, 34, 34, 35, 35, 36, 37, 38, 38, 39,
		40, 41, 42, 43, 44, 45, 46, 47, 48, 50, 51, 53, 55, 56, 58, 62
	},
	{
		1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 4, 4,
		4, 4, 4, 5, 5, 5, 6, 6, 6, 6, 6, 7, 7, 7, 7, 8,
		8, 8, 9, 9, 9, 9, 10, 10, 10, 11, 11, 11, 12, 12, 12, 13,
		13, 13, 14, 14, 15, 15, 15, 16, 16, 16, 17, 17, 18, 18, 18, 19,
		19, 19, 20, 21, 21, 21, 22, 22, 22, 23, 23, 24, 24, 25, 25, 25,
		26, 26, 27, 27, 28, 28, 29, 29, 30, 30, 31, 31, 32, 32, 33, 33,
		34, 35, 36, 36, 37, 37, 38, 39, 40, 40, 41, 42, 43, 44, 45, 46,
		47, 48, 49, 50, 51, 53, 54, 55, 57, 58, 60, 62, 64, 66, 69, 73
	},
	{
		0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3,
		4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 7, 7, 7, 7, 8,
		8, 8, 9, 9, 10, 10, 10, 11, 11, 11, 11, 12, 12, 13, 13, 14,
		14, 14, 15, 15, 16, 16, 17, 17, 18, 18, 18, 19, 19, 20, 20, 21,
		21, 21, 22, 23, 23, 24, 24, 25, 25, 25, 26, 26, 27, 28, 28, 28,
		29, 29, 30, 31, 31, 32, 32, 33, 33, 34, 35, 35, 36, 36, 37, 38,
		39, 39, 40, 41, 42, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52,
		53, 55, 56, 57, 59, 60, 62, 63, 65, 67, 69, 71, 74, 76, 79, 84
	}
};

const prog_uchar PROGMEM lfo_other[100] =
{
	0, 31, 74, 89, 106, 114, 125, 133, 138, 144,
	148, 153, 157, 161, 165, 167, 171, 173, 176, 178,
	181, 184, 185, 188, 189, 191, 194, 195, 197, 198,
	200, 201, 203, 205, 206, 207, 208, 210, 211, 212,
	213, 214, 216, 217, 218, 219, 220, 221, 222, 223,
	223, 225, 226, 226, 227, 228, 229, 230, 231, 231,
	232, 233, 234, 234, 235, 236, 237, 237, 238, 238,
	239, 240, 240, 241, 242, 242, 243, 244, 244, 245,
	246, 246, 247, 247, 248, 249, 249, 250, 250, 251,
	251, 252, 252, 253, 253, 253, 254, 255, 255, 255
};

const prog_uchar PROGMEM lfo_sh[100] =
{
	0, 2, 5, 7, 10, 12, 15, 18, 20, 23,
	25, 28, 30, 33, 36, 38, 41, 43, 46, 48,
	51, 54, 56, 59, 61, 64, 67, 69, 72, 74,
	77, 79, 82, 85, 87, 90, 92, 95, 97, 100,
	103, 105, 108, 110, 113, 116, 118, 121, 123, 126,
	128, 131, 134, 136, 139, 141, 144, 146, 149, 152,
	154, 157, 159, 162, 165, 167, 170, 172, 175, 177,
	180, 183, 185, 188, 190, 193, 195, 198, 201, 203,
	206, 208, 211, 213, 216, 219, 221, 224, 226, 229,
	232, 234, 237, 239, 242, 244, 247, 250, 252, 255
};

const uint8_t basic_tl[21] =
{
	127, 122, 118, 114, 110, 107, 104, 102, 100, 98, 96, 94, 92, 90, 88, 86, 85, 84, 82, 81, 79
};

const uint8_t volume_tl[100] =
{
	127, 90, 83, 77, 68, 65, 62, 60, 55, 53,
	52, 48, 47, 45, 44, 42, 41, 40, 38, 37,
	36, 35, 34, 33, 32, 32, 30, 30, 29, 28,
	27, 27, 26, 25,	24, 24, 23, 22, 22, 21,
	21, 20, 20, 19, 18, 18, 17, 17, 17, 16,
	16, 15, 15, 14, 14, 13, 13, 13, 12, 12,
	12,	11, 11, 10, 10, 10, 9, 9, 9, 8,
	8, 8, 7, 7, 7, 7, 6, 6, 6, 5,
	5, 5, 4, 4, 4, 3, 3, 3, 3, 2,
	2, 2, 2, 1, 1, 1, 1, 0, 0, 0
}

/*
* ALG OP1 OP2 OP3 OP4
* -------------------
* 0   0*  0   0   0
* 1   0*  0   0   0
* 2   0*  0   0   0
* 3   0*  0   0   0
* 4   8*  0   8*  0
* 5   13* 13* 13* 0
* 6   13* 13* 13* 0
* 7   16* 16* 16* 16*
*--------------------
* '*' indicates that operator is affected by volume change
*
* mind the operator order in VMEM (4-2-3-1)!
*/

const uint8_t tl_alg[8][4] =
{
	{0, 0, 0, 0},
	{0, 0, 0, 0},
	{0, 0, 0, 0},
	{0, 0, 0, 0},
	{8, 0, 8, 0},
	{13, 13, 13, 0},
	{13, 13, 13, 0},
	{16, 16, 16, 16}
};

const uint8_t tl_vol[8][4] =
{
	{1, 0, 0, 0},
	{1, 0, 0, 0},
	{1, 0, 0, 0},
	{1, 0, 0, 0},
	{1, 0, 1, 0},
	{1, 1, 1, 0},
	{1, 1, 1, 0},
	{1, 1, 1, 1}
};


const prog_uchar PROGMEM amd[100] =
{
	0,  0,  0,  0,  0,  1,  1,  1,  1,  2,
	2,  2,  2,  3,  3,  3,  3,  4,  4,  4,
	5,  5,  5,  6,  6,  6,  7,  7,  7,  7,
	8,  8,  8,  9,  9,  9, 10, 10, 10, 11,
	11, 12, 12, 12, 13, 13, 14, 14, 15, 15,
	16, 16, 17, 17, 18, 18, 19, 19, 20, 20,
	21, 21, 22, 23, 23, 24, 25, 25, 26, 27,
	28, 28, 29, 30, 31, 32, 33, 34, 35, 36,
	37, 38, 40, 41, 42, 44, 45, 48, 49, 51,
	54, 56, 60, 62, 67, 70, 77, 86, 96, 127
};

const prog_uchar PROGMEM pmd[100] =
{
	0,  0,  2,  3,  4,  5,  7,  8,  9, 11,
	12, 13, 14, 16, 17, 18, 20, 21, 22, 23,
	25, 26, 27, 29, 30, 31, 33, 34, 35, 36,
	38, 39, 40, 42, 43, 44, 45, 47, 48, 49,
	51, 52, 53, 54, 56, 57, 58, 60, 61, 62,
	63, 65, 66, 67, 69, 70, 71, 72, 74, 75,
	76, 78, 79, 80, 82, 83, 84, 85, 87, 88,
	89, 91, 92, 93, 94, 96, 97, 98, 100, 101,
	102, 103, 105, 106, 107, 109, 110, 111, 112, 114,
	115, 116, 118, 119, 120, 121, 123, 124, 125, 127
};


const prog_uchar PROGMEM amd_mw_99_amd_0[128] =
{
	0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2,
	2, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 6, 6,
	6, 6, 7, 7, 7, 7, 8, 8, 8, 8, 9, 9, 9, 9, 10, 10,
	10, 11, 11, 11, 12, 12, 12, 12, 13, 13, 13, 14, 14, 14, 15, 15,
	16, 16, 16, 17, 17, 17, 18, 18, 19, 19, 19, 20, 20, 21, 21, 22,
	22, 23, 23, 24, 24, 25, 25, 26, 26, 27, 28, 28, 29, 29, 30, 31,
	32, 32, 33, 34, 35, 35, 36, 37, 38, 39, 40, 41, 42, 44, 45, 46,
	48, 49, 51, 52, 54, 56, 58, 61, 64, 67, 70, 74, 80, 86, 96, 112
};

const prog_uchar PROGMEM pmd_mw_99_pmd_0[128] =
{
	0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,
	15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30,
	31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46,
	47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62,
	63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78,
	79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94,
	95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110,
	111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126
};

const prog_uchar PROGMEM amd_mw_98_amd_0[128] =
{
	0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2,
	2, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 6, 6,
	6, 6, 7, 7, 7, 7, 8, 8, 8, 8, 9, 9, 9, 9, 10, 10,
	10, 11, 11, 11, 12, 12, 12, 12, 13, 13, 13, 14, 14, 14, 15, 15,
	15, 16, 16, 16, 17, 17, 17, 18, 18, 19, 19, 19, 20, 20, 21, 21,
	22, 22, 23, 23, 24, 24, 25, 25, 26, 26, 27, 28, 28, 29, 29, 30,
	31, 31, 32, 33, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44,
	45, 47, 48, 50, 51, 53, 55, 57, 60, 62, 65, 68, 72, 77, 83, 90 // 102, 127
};

const prog_uchar PROGMEM pmd_mw_98_pmd_0[128] =
{
	0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,
	15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30,
	31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46,
	47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62,
	63, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77,
	78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93,
	94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109,
	110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125
};
/*
дальше при уменьшении настройки 97-90 максимальные значения уменьшаются следующим образом:

amd: 83, 74, 68, 65, 61, 58, 55, 53
pmd: 124, 122, 121, 120, 118, 117, 116, 115
*/
const prog_uchar PROGMEM mul[64] =
{
	0, 0, 0, 0, 1, 1, 1, 1,
	2, 2, 3, 2, 2, 4, 3, 3,
	5, 3, 4, 6, 4, 4, 7, 5,
	5, 8, 6, 5, 9, 6, 7, 10,
	6, 7, 11, 8, 12, 7, 8, 9,
	13,	8, 14, 10, 9, 15, 9, 11,
	10,	12, 11, 10, 13, 12, 11, 14,
	13,	12, 15, 14, 13, 15, 14, 15
};

const prog_uchar PROGMEM dt2[64] =
{
	0, 1, 2, 3, 0, 1, 2, 3,
	0, 1, 0, 2, 3, 0, 1, 2,
	0, 3, 1, 0, 2, 3, 0, 1,
	2, 0, 1, 3, 0, 2, 1, 0,
	3, 2, 0, 1, 0, 3, 2, 1,
	0, 3, 0, 1, 2, 0, 3, 1,
	2, 1, 2, 3, 1, 2, 3, 1,
	2, 3, 1, 2, 3, 2, 3, 3
};

