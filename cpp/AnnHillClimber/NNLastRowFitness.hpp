/*
 * NNLastRowFitness.hpp
 *
 *  Created on: Jan 23, 2015
 *      Author: Joost Huizinga
 */

#ifndef NNLASTROWFITNESS_HPP_
#define NNLASTROWFITNESS_HPP_

#include <math.h>

class NNLastRowFitness
{
public:
    /**
     * Function that evaluates and individual and returns its fitness.
     *
     * This fitness function should calculate the average absolute difference
     * between the final activation values of the network after 10 steps, when
     * all neurons are initialized to 0.5, and the vector:
     * {0, 1, 0, 1, 0, 1, 0, 1, 0, 1}
     * (vector of length 10 with a 0 at every even index and a 1 at every odd index)
     *
     * The fitness of the individual should then be:
     * 1 - average_absolute_difference
     */
    void evaluate(NeuralNetwork& individual){
        /* TODO: YOUR CODE HERE */

    }

    /* TODO: MORE CODE HERE IF YOU WANT*/
};



#endif /* NNLASTROWFITNESS_HPP_ */
