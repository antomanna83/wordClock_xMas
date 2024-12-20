#define NO_ROW 0
#define _ITA_
// #define _ENG_

#ifdef _ITA_
uint8_t ORE[] = {32, 11, 10}; // Ore
uint8_t MINUTI[] = {110, 109, 88, 87, 66, 65}; // Minuti
uint8_t HOURS[][9] = {
    {120,  99, 98, 77, 55, 54, 32, 11, 10},  // [0] Sono le ore
    {119,  97, 78, 75, 56, 56, 56, 56, 56},  // [1] E' l'una
    { 34,  31, 12, 12, 12, 12, 12, 12, 12},  // [2] Due
    {118, 101, 96, 96, 96, 96, 96, 96, 96},  // [3] Tre
    {115, 104, 93, 82, 71, 60, 49, 49, 49},  // [4] Quattro
    {114, 105, 92, 83, 70, 61, 61, 61, 61},  // [5] Cinque
    { 27,   5, 16, 16, 16, 16, 16, 16, 16},  // [6] Sei
    { 50,  37, 28, 15,  6,  6,  6,  6,  6},  // [7] Sette
    {79,   74, 57, 52, 52, 52, 52, 52, 52},  // [8] Otto
    {35,   30, 13,  8,  8,  8,  8,  8,  8},  // [9] Nove
    {117, 102, 95, 80, 73, 73, 73, 73, 73},  // [10] Dieci
    {58,   51, 36, 29, 14,  7,  7,  7,  7},  // [11] Undici
    {116, 103, 94, 81, 72, 59, 59, 59, 59}   // [12] Dodici
};

uint8_t MINUTES[][13] = {
    {125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125},  // [0] Empty
    {113,  64,  45,  42,  23,  20,   1,   1,   1,   1,   1,   1,   1},  // [1] e cinque
    {113,  46,  41,  24,  19,   2,   2,   2,   2,   2,   2,   2,   2},  // [2] e dieci
    {113,  91,  84,  62,  47,  40,  25,  18,   3,   3,   3,   3,   3},  // [3] e un quarto
    {113, 111, 108,  89,  86,  67,  67,  67,  67,  67,  67,  67,  67},  // [4] e venti
    {113, 111, 108,  89,  86,  67,  64,  45,  42,  23,  20,   1,   1},  // [5] e venticinque
    {113,  44,  43,  22,  21,   0,   0,   0,   0,   0,   0,   0,   0},  // [6] e mezza
    {113, 112, 107,  90,  85,  68,  63,  64,  45,  42,  23,  20,   1},  // [7] e trentacinque
    { 39,  26,  17,   4, 111, 108,  89,  86,  67,  67,  67,  67,  67},  // [8] meno venti
    { 39,  26,  17,   4,  91,  84,  62,  47,  40,  25,  18,   3,   3},  // [9] meno un quarto
    { 39,  26,  17,   4,  46,  41,  24,  19,   2,   2,   2,   2,   2},  // [10] meno dieci
    { 39,  26,  17,   4,  64,  45,  42,  23,  20,   1,   1,   1,   1}   // [11] meno cinque
};
#else
#ifdef _ENG_
uint8_t ORE[] = {32, 11, 10}; // Ore
uint8_t MINUTI[] = {110, 109, 88, 87, 66, 65}; // Minuti
uint8_t HOURS[][9] = {
    {120,  99, 98, 77, 55, 54, 32, 11, 10},  // [0] Sono le ore
    {119,  97, 78, 75, 56, 56, 56, 56, 56},  // [1] E' l'una
    { 34,  31, 12, 12, 12, 12, 12, 12, 12},  // [2] Due
    {118, 101, 96, 96, 96, 96, 96, 96, 96},  // [3] Tre
    {115, 104, 93, 82, 71, 60, 49, 49, 49},  // [4] Quattro
    {114, 105, 92, 83, 70, 61, 61, 61, 61},  // [5] Cinque
    { 27,   5, 16, 16, 16, 16, 16, 16, 16},  // [6] Sei
    { 50,  37, 28, 15,  6,  6,  6,  6,  6},  // [7] Sette
    {79,   74, 57, 52, 52, 52, 52, 52, 52},  // [8] Otto
    {35,   30, 13,  8,  8,  8,  8,  8,  8},  // [9] Nove
    {117, 102, 95, 80, 73, 73, 73, 73, 73},  // [10] Dieci
    {58,   51, 36, 29, 14,  7,  7,  7,  7},  // [11] Undici
    {116, 103, 94, 81, 72, 59, 59, 59, 59}   // [12] Dodici
};

uint8_t MINUTES[][13] = {
    {125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125},  // [0] Empty
    {113,  64,  45,  42,  23,  20,   1,   1,   1,   1,   1,   1,   1},  // [1] e cinque
    {113,  46,  41,  24,  19,   2,   2,   2,   2,   2,   2,   2,   2},  // [2] e dieci
    {113,  91,  84,  62,  47,  40,  25,  18,   3,   3,   3,   3,   3},  // [3] e un quarto
    {113, 111, 108,  89,  86,  67,  67,  67,  67,  67,  67,  67,  67},  // [4] e venti
    {113, 111, 108,  89,  86,  67,  64,  45,  42,  23,  20,   1,   1},  // [5] e venticinque
    {113,  44,  43,  22,  21,   0,   0,   0,   0,   0,   0,   0,   0},  // [6] e mezza
    {113, 112, 107,  90,  85,  68,  63,  64,  45,  42,  23,  20,   1},  // [7] e trentacinque
    { 39,  26,  17,   4, 111, 108,  89,  86,  67,  67,  67,  67,  67},  // [8] meno venti
    { 39,  26,  17,   4,  91,  84,  62,  47,  40,  25,  18,   3,   3},  // [9] meno un quarto
    { 39,  26,  17,   4,  46,  41,  24,  19,   2,   2,   2,   2,   2},  // [10] meno dieci
    { 39,  26,  17,   4,  64,  45,  42,  23,  20,   1,   1,   1,   1}   // [11] meno cinque
};
#endif //_ENG_
#endif //_ITA_


uint8_t DOTS[][4] = {
    {125, 125, 125, 125},  // [0] Empty
    {121, 121, 121, 121},  // [1] One dot
    {121, 122, 122, 122},  // [2] Two dots
    {121, 122, 123, 123},  // [3] Three dots
    {121, 122, 123, 124}   // [4] Four dots
};

uint8_t NUMBERS[][19] = {
  {35, 30, 13, 51, 7, 50, 15, 6, 49, 27, 5, 48, 39, 4, 47, 3, 41, 24, 19},      // 0
  {24, 25, 26, 27, 28, 29, 30, 36, 19, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41}, // 1
  {35, 30, 13, 51, 7, 6, 16, 26, 40, 46, 41, 24, 19, 2, 2, 2, 2, 2, 2},         // 2
  {52, 35, 30, 13, 8, 14, 28, 16, 4, 3, 19, 24, 41, 47, 47, 47, 47, 47, 47},    // 3
  {13, 14, 15, 16, 17, 18, 19, 4, 26, 39, 48, 49, 37, 29, 29, 29, 29, 29, 29},  // 4
  {52, 35, 30, 13, 8, 51, 50, 37, 28, 15, 5, 4, 3, 19, 24, 41, 47, 47, 47},     // 5
  {30, 13, 8, 36, 50, 49, 48, 47, 38, 27, 16, 4, 3, 19, 24, 41, 41, 41, 41},    // 6
  {52, 35, 30, 13, 8, 7, 15, 27, 39, 40, 41, 41, 41, 41, 41, 41, 41, 41, 41},   // 7
  {35, 30, 13, 50, 51, 6, 7, 38, 27, 16, 3, 4, 47, 48, 41, 24, 19, 19, 19},     // 8
  {35, 30, 13, 4, 5, 6, 7, 50, 51, 38, 27, 16, 18, 24, 41, 41, 41, 41, 41}      // 9
};