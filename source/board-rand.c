#include <board-rand.h>
#include <fsl_rnga.h>

uint32_t board_rand(void) {
    uint32_t result;

    RNGA_SetMode(RNG, kRNGA_ModeNormal);
    RNGA_GetRandomData(RNG, &result, sizeof(uint32_t));
    RNGA_SetMode(RNG, kRNGA_ModeSleep);

    return result;
}

void board_rand_init(void) {
	RNGA_Init(RNG);
	RNGA_SetMode(RNG, kRNGA_ModeSleep);
}
