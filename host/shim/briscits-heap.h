/*
 * briscits-heap.h — HOST BUILD ONLY shim.
 *
 * pamodbus-disco/src uses <briscits-heap.h> for brisc_heap_*() heap helpers.
 * On an embedded build that resolves to the Pike briscits RTOS allocator
 * (and drags in brisc_thread.h). For the host test tools we shadow it with
 * this header and provide the brisc_* symbols via libc in brisc_shim.c.
 */

#ifndef BRISC_HEAP_H
#define BRISC_HEAP_H

#include <stddef.h>
#include <stdlib.h>   /* plain malloc/free/realloc used elsewhere in library */

#ifdef __cplusplus
extern "C" {
#endif

void *brisc_heap_malloc(size_t size);
void  brisc_heap_free(void *ptr);
void *brisc_heap_realloc(void *ptr, size_t size);

#ifdef __cplusplus
}
#endif

#endif /* BRISC_HEAP_H */