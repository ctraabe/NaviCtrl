#include <errno.h>
#include <sys/types.h>

caddr_t _sbrk (int incr)
{
  register char * stack_ptr asm ("sp");
  extern char __bss_end__;  // Defined by the linker
  static char * heap_end = NULL;
  char * heap_end_pv;

  if (heap_end == NULL)
  heap_end = &__bss_end__;

  heap_end_pv = heap_end;

  if (heap_end + incr > stack_ptr)
  {
    errno = ENOMEM;
    return (caddr_t)0;
  }

  heap_end += incr;

  return (caddr_t)heap_end_pv;
}
