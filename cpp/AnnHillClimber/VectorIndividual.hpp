/*
 * VectorIndividual.hpp
 *
 *  Created on: Jan 22, 2015
 *      Author: Joost Huizinga
 *
 * An individual that consists of a vector of doubles.
 *
 * Note that the minimum, maximum and mutation rate are all stored with the individual.
 */

#ifndef VECTORINDIVIDUAL_HPP_
#define VECTORINDIVIDUAL_HPP_

#include <vector>

#include "Individual.hpp"
#include "Random.hpp"

class VectorIndividual: public Individual
{
public:
    VectorIndividual(size_t size = 0, double min = 0.0, double max = 1.0, double mutationRate = 0.05):
        _valueVector(size), _min(min), _max(max), _mutationRate(mutationRate){
        //nix
    }

    /**
     * Randomizes the vector.
     *
     * Implement such that each element in the vector is assigned a random value
     * between _min and _mix.
     */
    void randomize(){
        for (int i = 0; i < this->size(); i++) {
            _valueVector[i] = randDouble(_min, _max );
        }
    }

    /**
     * Mutates the vector.
     *
     * Implement such that each element in the vector has a _mutationRate chance
     * of being re-assigned a value between _min and _max.
     */
    void mutate(){
        for (int i = 0; i < this->size() ; i++) {
            if(randDouble() < _mutationRate){
                _valueVector[ i ] = randDouble( _min, _max );
            }
        }
    }

    /**
     * Returns the size of the vector.
     */
    size_t size() const{
        return _valueVector.size();
    }

    /**
     * Returns a reference to an element of the vector.
     */
    double& operator[] (const int nIndex)
    {
        return _valueVector[nIndex];
    }

private:
    std::vector<double> _valueVector;
    double _min;
    double _max;
    double _mutationRate;
};

/**
 * Convenience operator for writing a vector individual.
 */
std::ostream& operator<<(std::ostream& is, VectorIndividual& obj){
    if(obj.size() > 0){
        for(size_t i=0; i<obj.size()-1; ++i){
            is << obj[i] << " ";
        }
        is << obj[obj.size()-1];
    }
    return is;
}

#endif /* VECTORINDIVIDUAL_HPP_ */
