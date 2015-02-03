/*
 * Random.hpp
 *
 *  Created on: Jan 23, 2015
 *      Author: Joost Huizinga
 *
 * A small collection of functions for creating random doubles.
 *
 * You do not need to modify this file.
 */

#ifndef RANDOM_HPP_
#define RANDOM_HPP_

#include <stdlib.h>
#include <time.h>

/**
 * Seeds the random number generator with the indicated seed.
 */
inline void seed(unsigned int seed){
    srand(seed);
}

/**
 * Seeds the random number generator based on the current time.
 */
inline void seed(){
    srand(time(NULL));
}

/**
 * Returns a random double in the interval [0,1].
 */
inline double randDouble(){
    return (double)rand()/(double)RAND_MAX;
}

/**
 * Returns a random double in the interval [0,max]
 */
inline double randDouble(double max){
    return randDouble() * max;
}

/**
 * Returns a random double in the interval [min, max]
 */
inline double randDouble(double min, double max){
    return randDouble() * (max - min) + min;
}


#endif /* RANDOM_HPP_ */
