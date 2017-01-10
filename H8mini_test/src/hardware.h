// The hardware defines are set in the build settings (uVision or Makefile)
// Example to create a new hardware target(let's call it mytarget):
// 1. copy hardware_h8mini.h file to hardware_mytarget.h
// 2. modify the hardware_mytarget.h
// 3. in the Makefile, add mytarget to the TARGETS variable
// 4. build using gcc by running "make mytarget"
// 5. flash by running "make flash_mytarget"

// HARDWARE_HEADER is a preprocessor define and is defined
// in the Makefile or uVision project settings.
#include HARDWARE_HEADER

