//
// Created by Stuart Carnie on 8/1/20.
//

#include <stdlib.h>

void *SDL_malloc(size_t size) { return malloc(size); }
void *SDL_realloc(void *mem, size_t size) { return realloc(mem, size); }
void SDL_free(void *mem) { free(mem); }