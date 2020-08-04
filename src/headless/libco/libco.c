/*
  libco
  auto-selection module
  license: public domain
*/

#if defined __amd64__
#include "amd64.c"
#elif defined(__aarch64__)
#include "aarch64.c"
#else
#error "libco: unsupported processor, compiler or operating system"
#endif
