/*! @file config.h
    @brief A configuration file that controls the options for the project and the target robot
    
    This file is automatically generated by CMake. Do NOT modify this file. Seriously, don't modify
    this file. If you really need to put something here, then you want to modify ./walkconfig.in.

    @author Jason Kulk
 */
#ifndef WALKCONFIG_H
#define WALKCONFIG_H

#define USE_MOTION_WALK_ON
#ifdef USE_MOTION_WALK_ON
    #define USE_WALK                                           //!< this will be defined when the build is configured to include motion
#else
    #undef USE_WALK
#endif

#ifdef USE_WALK
    #define USE_WALKOPTIMISER_OFF 
	#ifdef USE_WALKOPTIMISER_ON
        #define USE_WALKOPTIMISER
    #else
        #undef USE_WALKOPTIMISER
    #endif
    #define USE_WALK_JWALK_OFF 
    #ifdef USE_WALK_JWALK_ON
        #define USE_JWALK
    #else
        #undef USE_JWALK
    #endif
    #define USE_WALK_JUPPWALK_OFF 
    #ifdef USE_WALK_JUPPWALK_ON
        #define USE_JUPPWALK
    #else
        #undef USE_JUPPWALK
    #endif
    #define USE_WALK_NBWALK_OFF 
    #ifdef USE_WALK_NBWALK_ON
        #define USE_NBWALK
    #else
        #undef USE_NBWALK
    #endif
    #define USE_WALK_VSCWALK_OFF 
    #ifdef USE_WALK_VSCWALK_ON
        #define USE_VSCWALK
    #else
        #undef USE_VSCWALK
    #endif
    #define USE_WALK_ALWALK_ON 
    #ifdef USE_WALK_ALWALK_ON
        #define USE_ALWALK
    #else
        #undef USE_ALWALK
    #endif
#else
    #undef USE_JWALK
    #undef USE_JUPPWALK
    #undef USE_NBWALK
    #undef USE_VSCWALK
    #undef USE_ALWALK
#endif

#endif // !WALKCONFIG_H

