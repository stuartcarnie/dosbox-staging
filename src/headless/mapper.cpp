//
// Created by Stuart Carnie on 8/1/20.
//

#include "mapper.h"

bool autofire = false;

// These used to be defined in sdl_mapper.cpp, which we no longer include in
// Boxer.
void MAPPER_AddHandler(MAPPER_Handler *handler, MapKeys key, Bitu mods,
                       char const *const eventname,
                       char const *const buttonname)
{}
void MAPPER_Init(void) {}
void MAPPER_StartUp(Section *sec) {}
void MAPPER_Run(bool pressed) {}
void MAPPER_RunInternal() {}
void MAPPER_LosingFocus(void) {}
void MAPPER_AutoType(std::vector<std::string> &sequence, const uint32_t wait_ms,
                     const uint32_t pacing_ms)
{}
std::vector<std::string> MAPPER_GetEventNames(const std::string &prefix)
{
    return std::vector<std::string>();
}
