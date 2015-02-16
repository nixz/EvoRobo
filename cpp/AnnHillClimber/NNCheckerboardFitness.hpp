/*
 * NNCheckerboardFitness.hpp
 *
 *  Created on: Jan 23, 2015
 *      Author: Joost Huizinga
 */

#ifndef NNCHECKERBOARDFITNESS_HPP_
#define NNCHECKERBOARDFITNESS_HPP_

#include <math.h>

class NNCheckerboardFitness
{
public:

    /**
     * Function that evaluates an individual and returns its fitness.
     *
     * Implement this function such that it maximizes the difference in the activation
     * values of neurons, both between adjacent neurons, and between current activation
     * values and the activation values at the previous time-step.
     *
     * Once again, initialize all neurons with an activation value of 0.5, and run the network
     * for 10 time-steps. Do note that this fitness function has to be calculated for each time-step
     * not just after 10 steps.
     *
     * To compare current activation values against previous activation values, store
     * the previous activation values in a separate vector.
     *
     * For each neuron at each time-step, sum the following differences (if possible):
     *   fabs(neurons[i].getValue() - neurons[i+1].getValue())
     *   fabs(neurons[i].getValue() - previousActivation[i])
     *
     * Normalize the result by dividing it by the maximum possible summed difference (should be 180).
     */
    void evaluate(NeuralNetwork& individual){
        /* TODO: YOUR CODE HERE */

    }

    /* TODO: MORE CODE HERE IF YOU WANT*/

};



#endif /* NNCHECKERBOARDFITNESS_HPP_ */
