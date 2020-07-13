/*
 * platform-native.h
 *
 *  Created on: 9/04/2020.
 *      Author: alexrayne <alexraynepe196@gmail.com>
 */

#ifndef ARCH_PLATFORM_NATIVE_PLATFORM_NATIVE_H_
#define ARCH_PLATFORM_NATIVE_PLATFORM_NATIVE_H_

#include <unistd.h>
#include <sys/types.h>


// crappy way of remembering and accessing argc/v
extern int contiki_argc;
extern char **contiki_argv;



struct select_callback {
  // @return - file descriptor with highest id, that assigned to  fdr or frw.
  int  (* set_fd)(fd_set *fdr, fd_set *fdw);
  void (* handle_fd)(fd_set *fdr, fd_set *fdw);
};

/**
 *  @brief - provide callbacks for file descriptors selectinon, that polls in main loop.
 *
 *  main loop monitors app-provided non-blocking filedescriptors with select.
 *  to establish this filedescriptors use select_callback structure.
 *
 *  @param fd >= 0 append callback for mainloop monitoring
 *            < 0  - remove callback handling
 */
int select_set_callback(int fd, const struct select_callback *callback);



#ifndef __NOINLINE
#define __NOINLINE __attribute__((noinline))
#endif
/*
 * @brief - board depenent startup
 * These functions must be provided externally
 */
__NOINLINE
void board_init(void);



#endif /* ARCH_PLATFORM_NATIVE_PLATFORM_NATIVE_H_ */
