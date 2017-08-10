#pragma once

#define IMAGE_WIDTH                                                                                                    \
    320  // This is taken from NUbots/module/input/LinuxCamera/data/config/LinuxCamera.yaml  (rUNSWift is using
         // 1280!!!!!!!!!)
#define IMAGE_HEIGHT 240
#define SURF_SUBSAMPLE 1       // Reduced from 8, since we are using 1/4 of the resolution that rUNSWift is using
#define INIT_SAMPLE 1          // The initial subsample of the integral (which itself has already been subsampled)
#define SURF_HORIZON_WIDTH 10  // Reduced by 1/4 from 60

/**
 * The size of the SURF feature point descriptor in floats, must be even number
 **/
#define SURF_DESCRIPTOR_LENGTH 6

/**
 * The number of scale space octaves used when searching for features
 **/
#define OCTAVES 4

/**
 * The threshold value for interest points
 **/
#define THRESH 325.125f

/**
 * The number of samples taken from within each descriptor window.
 **/
#define SURF_DESCRIPTOR_SAMPLES 5

/**
 * The possible states of the robot
 **/
#define STATE_INITIAL 0
#define STATE_READY 1
#define STATE_PLAYING 2

/** Field line dimensions */
#define FIELD_WIDTH 6000
#define FIELD_LENGTH 9000

#define FIELD_LENGTH_OFFSET 755
#define FIELD_WIDTH_OFFSET 675

/** Field dimensions including edge offsets */
#define FULL_FIELD_WIDTH (FIELD_WIDTH + (FIELD_WIDTH_OFFSET * 2))
#define FULL_FIELD_LENGTH (FIELD_LENGTH + (FIELD_LENGTH_OFFSET * 2))

/**
 * The number of RANSAC iterations to use when matching feature points between images
 **/
#define MATCH_ITERATIONS 20

/**
 * The error margin to use when RANSAC matching feature points on a line,
 * In terms of the original image the allowed error is really PIXEL_ERROR_MARGIN * SURF_SUBSAMPLE
 **/
#define PIXEL_ERROR_MARGIN 5.0f
