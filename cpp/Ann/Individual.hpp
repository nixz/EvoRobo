/*
 * Individual.hpp
 *
 *  Created on: Jan 22, 2015
 *      Author: Joost Huizinga
 *
 * The base class for every individual so you can assign a fitness to it.
 *
 * You do not need to modify this file.
 */

#ifndef INDIVIDUAL_HPP_
#define INDIVIDUAL_HPP_

class Individual
{
public:
    Individual(): _fitness(0){
        //nix
    }

    /**
     * Sets the fitness of this individual.
     */
    void setFitness(double fitness){
        _fitness = fitness;
    }

    /**
     * Gets this fitness of this individual.
     */
    double getFitness(){
        return _fitness;
    }

private:
    double _fitness;
};

#endif /* INDIVIDUAL_HPP_ */
