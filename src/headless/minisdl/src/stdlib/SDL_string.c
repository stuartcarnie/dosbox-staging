//
// Created by Stuart Carnie on 8/1/20.
//

#include "SDL_stdinc.h"
#include <string.h>

int SDL_memcmp(const void *s1, const void *s2, size_t len)
{
    return memcmp(s1, s2, len);
}

void *SDL_memcpy(void *dst, const void *src, size_t len)
{
    return memcpy(dst, src, len);
}

void *SDL_memset(void *dst, int c, size_t len)
{
    return memset(dst, c, len);
}

size_t SDL_strlcpy(char *dst, const char *src, size_t len)
{
    return strlcpy(dst, src, len);
}

int SDL_vsnprintf(char *text, size_t maxlen, const char *fmt, va_list ap)
{
    if (!fmt) {
        fmt = "";
    }
    return vsnprintf(text, maxlen, fmt, ap);
}