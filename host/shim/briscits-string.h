/*
 * briscits-string.h — HOST BUILD ONLY shim.
 *
 * pamodbus-disco/src uses <briscits-string.h> for brisc_string_*() helpers.
 * On an embedded build this is the Pike briscits string shim; on the host we
 * shadow it with bare declarations and implement via libc in brisc_shim.c.
 */

#ifndef BRISC_STRING_H
#define BRISC_STRING_H

#include <stddef.h>
#include <string.h>   /* plain memset/memcpy used elsewhere in library */

#ifdef __cplusplus
extern "C" {
#endif

void  brisc_string_free(void *ptr);
void *brisc_string_memset(void *s, int c, size_t n);
void *brisc_string_memcpy(void *dest, const void *src, size_t n);
void *brisc_string_memmove(void *dest, const void *src, size_t n);

#ifdef __cplusplus
}
#endif

#endif /* BRISC_STRING_H */