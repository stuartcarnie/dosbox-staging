//
// Created by Stuart Carnie on 8/1/20.
//

#include "timer.h"
#include "config.h"
#include <SDL_timer.h>
#include <errno.h>
#include <time.h>

static uint64_t epoch_ns;
static uint64_t delayed_ticks_ns = 0;

void SDLCALL SDL_Delay(Uint32 ms)
{
    delayed_ticks_ns += ms * 1000000;
}

Uint32 SDLCALL SDL_GetTicks(void)
{
    __auto_type tick_now = clock_gettime_nsec_np(CLOCK_MONOTONIC_RAW) - epoch_ns;
    return (Uint32)((tick_now + delayed_ticks_ns) / 1000000);
}

void initialize_timer()
{
    epoch_ns = clock_gettime_nsec_np(CLOCK_MONOTONIC_RAW);
}

void reset_timer()
{
    delayed_ticks_ns = 0;
}