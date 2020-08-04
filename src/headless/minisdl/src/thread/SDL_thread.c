//
// Created by Stuart Carnie on 8/1/20.
//

#include "../SDL_error_c.h"
#include <SDL_thread.h>
#include <pthread.h>

SDL_threadID SDLCALL SDL_ThreadID(void)
{
    return (SDL_threadID)pthread_mach_thread_np(pthread_self());
}

/* Routine to get the thread-specific error variable */
SDL_error *SDL_GetErrBuf(void)
{
    /* Non-thread-safe global error variable */
    static SDL_error SDL_global_error;
    return &SDL_global_error;
}

// mutex implementation


struct SDL_mutex
{
};

static struct SDL_mutex g_mutex = {};

/* Create a mutex */
SDL_mutex *
SDL_CreateMutex(void)
{
    return &g_mutex;
}

/* Free the mutex */
void
SDL_DestroyMutex(SDL_mutex * mutex)
{
}

/* Lock the mutex */
int
SDL_LockMutex(SDL_mutex * mutex)
{
    return 0;
}

/* Unlock the mutex */
int
SDL_mutexV(SDL_mutex * mutex)
{
    return 0;
}
