/*
 * VectorFitnessFunction.hpp
 *
 *  Created on: Jan 22, 2015
 *      Author: Joost Huizinga
 *
 * This will be the fitness function for the vector individual.
 */

#ifndef VECTORFITNESSFUNCTION_HPP_
#define VECTORFITNESSFUNCTION_HPP_

#include "VectorIndividual.hpp"

class VectorFitnessFunction
{
public:
    /**
     * Function that evaluates and individual and returns its fitness.
     *
     * Implement such that the fitness of the vector is the average of its elements.
     *
     * Use individual.setFitness(fitness) to assign the fitness to the individual once calculated.
     */
    void evaluate(VectorIndividual& individual){
        double fitness = 0;
        for (int i = 0; i < individual.size(); i++) {
            fitness += individual[i];
        }
        fitness = fitness / individual.size ();
        individual.setFitness( fitness );
    }
};



#endif /* VECTORFITNESSFUNCTION_HPP_ */
