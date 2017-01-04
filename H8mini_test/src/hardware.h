// The hardware defines are set in the build settings (uVision or Makefile)
// Example to create a new hardware target(let's call it mytarget):
// 1. copy hardware_h8mini.h file to hardware_mytarget.h
// 2. modify the hardware_mytarget.h
// 3. in hardware_mytarget.h, change the #ifdef HARDWARE_H8MINI to #ifdef HARDWARE_MYTARGET
// 4. in this file add the following to the statement below 
//    #elif defined HARDWARE_MYTARGET 
//    #include "hardware_mytarget.h"
// 6. in the Makefile, add mytarget to the TARGETS variable
// 7. build using gcc by running "make mytarget"
// 8. flash by running "make flash_mytarget"

#if defined HARDWARE_H8MINI
#include "hardware_h8mini.h"
#elif defined HARDWARE_FQ777_124
#include "hardware_fq777-124.h"
#endif

