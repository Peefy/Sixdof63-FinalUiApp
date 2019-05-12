
#ifndef __APP_CONFIG_H_
#define __APP_CONFIG_H_

#define PROGRAM_NAME "Sixdof63Dialog"

#define IS_63_PLATFORM
//#define IS_81_PLATFORM
//#define IS_84_PLATFORM

#ifdef IS_63_PLATFORM

#define IS_BIG_MOTION 0
#define IS_PULSE_CARD_REVERSE 1
#define IS_REVERSE_CONTROL 0

#endif

#ifdef IS_81_PLATFORM

#define IS_BIG_MOTION 1
#define IS_PULSE_CARD_REVERSE 1
#define IS_REVERSE_CONTROL 0

#endif

#ifdef IS_84_PLATFORM

#define IS_BIG_MOTION 0
#define IS_PULSE_CARD_REVERSE 1
#define IS_REVERSE_CONTROL 1

#endif

#endif
