/*
 * brisc_shim.c — HOST BUILD ONLY: libc-backed implementations of the briscits
 * heap/string helpers the pamodbus-disco library links against. Used instead
 * of the Pike briscits RTOS allocator when building the host test tools.
 */

#include <stdlib.h>
#include <string.h>

#include "briscits-heap.h"
#include "briscits-string.h"

void *brisc_heap_malloc(size_t size)        { return malloc(size); }
void  brisc_heap_free(void *ptr)            { free(ptr); }
void *brisc_heap_realloc(void *ptr, size_t s){ return realloc(ptr, s); }

void  brisc_string_free(void *ptr)          { free(ptr); }
void *brisc_string_memset(void *s, int c, size_t n) { return memset(s, c, n); }
void *brisc_string_memcpy(void *dest, const void *src, size_t n)
{
    return memcpy(dest, src, n);
}
void *brisc_string_memmove(void *dest, const void *src, size_t n)
{
    return memmove(dest, src, n);
}