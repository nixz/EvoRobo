/*
 * HillClimber.hpp
 *
 *  Created on: Jan 22, 2015
 *      Author: Joost Huizinga
 *
 * This will be your implementation of a simple hill-climber.
 */

#ifndef HILLCLIMBER_HPP_
#define HILLCLIMBER_HPP_

#include <fstream>
#include <iostream>

template<typename Indiv, typename Fit>
class HillClimber
{
public:
    HillClimber(){
        //nix
    }

    /**
     * Initializes the population (parent) of the hill climber.
     *
     * This function should do 4 things:
     * - assign the fitness function to be the provided fitness function.
     * - assign the parent to be the provided initial individual.
     * - randomize the parent (remember to use the randomize() function you implemented).
     * - evaluate the parent (if implemented correctly, you should be able to call
     *   _fitnessFunction.evaluate(_parent); to do so).
     */
    void init(Indiv initialIndividual, Fit fitnessFunction){
        /* TODO: YOUR CODE HERE */
    }

    /**
     * Runs the hill climber for the provided number of generations.
     *
     * Implement such that it will run the hill climbing algorithm for the indicated
     * number of generations.
     * It should also write the _parent to a file with the individualFileName,
     * and it should write the fitness of the parent at each generation to a file with
     * the fitnessFileName, if either of them are provided.
     *
     * At each generation you should perform the following steps:
     * - Create a copy of the parent and assign it to _child.
     * - Mutate the child (remember to use the mutate function you implemented)
     * - Evaluate the child (you can use _fitnessFunction.evaluate(_child); to do so)
     * - If the fitness of the child is greater than the fitness of the parent,
     *   replace the parent with the child. Note that, if your individuals have been evaluated,
     *   you can retrieve the fitness of an individual by calling _indiv.getFitness().
     * - Log the parent and its fitness. Note that you can directly write the parent to a
     *   stream with: myFile << _parent << "\n"; (provided myFile is defined as an output stream,
     *   see below.) Note that, in order to use the provided plotting functions, the fitness file
     *   should contain one fitness value per line, and the individual file should contain one
     *   individual per line.
     *
     * To write a file to disk, first create an output stream:
     * std::ofstream myFile;
     * then open the stream if a filename is provided:
     * if(fileName != "") myFile.open(fileName);
     * and then, if opened, write to the stream like this:
     * if(myFile.is_open()) myFile << myValue << "\n";
     */
    void run(size_t numberOfGenerations, std::string fitnessFileName = "", std::string individualFileName = ""){
        /* TODO: YOUR CODE HERE */
    }

    /**
     * Returns a reference to the current parent.
     */
    Indiv &getParent(){
        return _parent;
    }

    /**
     * Returns a reference to the current child.
     */
    Indiv &getChild(){
        return _child;
    }

private:
    Indiv _parent;
    Indiv _child;
    Fit _fitnessFunction;

    /* TODO: YOUR CODE HERE IF YOU LIKE */
};



#endif /* HILLCLIMBER_HPP_ */
