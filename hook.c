#define _LARGEFILE64_SOURCE /* See feature_test_macros(7) */
#define _GNU_SOURCE
#include <errno.h>
#include <unistd.h>
#include <dlfcn.h>
#include <stdio.h>
#include <sys/types.h>
#include <limits.h>
#include <stdlib.h>
#include <sys/time.h>
#include <time.h>
#include <pthread.h>

#define FUNC_RELOAD(ret, name, typeArgs, args, hackFunc) \
typedef ret (*MY##name) typeArgs; \
\
ret name typeArgs \
{ \
    void *handle; \
	MY##name old_name; \
\
    handle = dlopen("libc.so.6", RTLD_LAZY); \
	old_name = (MY##name)dlsym(handle, #name); \
\
    ret r; \
	r = old_name args; \
	hackFunc; \
	return r; \
}

//lseek64
FUNC_RELOAD(off64_t, lseek64, \
(int fd, off64_t offset, int whence), (fd, offset, whence), \
r = (off64_t)LONG_MAX)
