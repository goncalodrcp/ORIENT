/* Wrapper file for time.h that ensures the proper POSIX defines for MATLAB Coder */
#ifndef CODER_POSIX_TIME_H
#define CODER_POSIX_TIME_H

#if !defined(_POSIX_C_SOURCE) || _POSIX_C_SOURCE < 199309L
#error Value of _POSIX_C_SOURCE must be at least: 199309L
#endif

#include <time.h>
#endif /* CODER_POSIX_TIME_H */
