/*
 * NeuralNetwork.hpp
 *
 *  Created on: Jan 22, 2015
 *      Author: Joost Huizinga
 */

#ifndef NEURALNETWORK_HPP_
#define NEURALNETWORK_HPP_

#include <vector>
#include <fstream>

#include "VectorIndividual.hpp"
#include "Random.hpp"

/**
 * A simple neuron class without activation function.
 *
 * Note that this class is mostly appropriate for fully connected networks;
 * sparsely connected networks should consider a graph like network instead.
 *
 * Also note that this class supports recurrent connections.
 * If the network is feed-forward, a simpler neuron model can be used.
 */
class Neuron
{
public:
    /**
     * Creates a new neuron with the indicated number of incoming connection.
     *
     * The current and new activations are initialized to zero.
     */
    Neuron(size_t numberOfConnections = 0): _value(0), _newValue(0){
        _incommingConnections = VectorIndividual(numberOfConnections, -1.0, 1.0);
    }

    /**
     * Return the current activation value of this neuron.
     */
    double getValue(){
        return _value;
    }

    /**
     * Sets the current value.
     */
    void setValue(double value){
        _value = value;
    }

    /**
     * Returns the new value.
     */
    double getNewValue(){
        return _newValue;
    }

    /**
     * Sets the new value.
     */
    void setNewValue(double newValue){
        _newValue = newValue;
    }

    /**
     * Returns the weight of the connection with the supplied index.
     */
    double getConnection(size_t connectionIndex){
        return _incommingConnections[connectionIndex];
    }

    /**
     * Sets the weight of the connection with the supplied index.
     */
    void setConnection(size_t connectionIndex, double weight){
        _incommingConnections[connectionIndex] = weight;
    }

    /**
     * Returns the vector of incoming connections.
     */
    VectorIndividual &getConnections(){
        return _incommingConnections;
    }

    /* Optional */
    //TODO: MORE CODE HERE IF YOU WANT
private:
    double _value;
    double _newValue;
    VectorIndividual _incommingConnections;
};


/**
 * The basis for the neural network you have to implement.
 *
 * Note that the network extends the Individual class,
 * so it can be used with the HillClimber next homework.
 */
class NeuralNetwork: public Individual
{
public:
    /**
     * Initializes the network with the supplied number of neurons.
     *
     * Implement this function such that the network is fully connected,
     * meaning that every neuron has a number of incoming connections equal to
     * the number of neurons in the network.
     */
    NeuralNetwork(size_t numberOfNeurons = 0){
        for (int i = 0; i < numberOfNeurons; i++) {
            _neurons.push_back(Neuron( numberOfNeurons ));
        }
    }


    /**
     * Performs one step of network activation.
     *
     * Implement this function such that, at each step, each neuron is updated
     * according to its incoming connections and the current activation values
     * of the neurons it is connected to.
     *
     * For a neuron called 'thisNeuron', the contribution
     * of a single neuron should be calculated as:
     * thisNeuron.getConnection(j) * _neurons[j].getValue();
     * where j indicates the index of the incoming neuron.
     * Sum the contribution of each incomming neuron and set
     * this sum as the new neuron value (do not set it as the current value yet!).
     * If the sum is greater than 1, set it to 1,
     * if the sum is smaller than 0, set it to 0.
     *
     * Do not forget to, after each the new value of each neuron is updated,
     * update the current value.
     */
    void step(){
        for (int i =0 ; i < _neurons.size(); i++) {
            double sum = 0.0;
            for (int j=0; j < _neurons.size(); j++) {
                sum+=_neurons[i].getConnection( j )*_neurons[j].getValue();
            }
            _neurons[i].setNewValue( sum );
            if ( sum > 1.0 ) {
                _neurons[i].setNewValue( 1.0 );
            }
            if ( sum < 0.0 ) {
                _neurons[i].setNewValue( 0.0 );
            }
        }
        // Update new value
        for (int i =0 ; i < _neurons.size(); i++) {
            _neurons[i].setValue( _neurons[i].getNewValue() );
        }
    }


    /**
     * Writes the current activation of the network to the output stream.
     *
     * Implement this function such that it writes the current activation of each
     * neuron to the supplied output stream. Separate each value with a space,
     * and write an end-of-line ("\n") once all values have been written.
     */
    void logActivation(std::ofstream& activationFile){
        for (int i =0 ; i < _neurons.size(); i++) {
            activationFile << _neurons[i].getValue() << " ";
        }
        activationFile << "\n";
    }


    /**
     * Assigns random weights to each connection.
     *
     * Implement this function such that each connection is assigned a random number
     * in [-1, 1]. Note that you can reuse the randomize function of your vectorIndividual.
     */
    void randomize(){
        for (int i =0 ; i < _neurons.size(); i++) {
            for (int j = 0; j < _neurons.size(); j++) {
                _neurons[i].setConnection( j, randDouble( -1, 1 ) );
            }
        }
    }


    /**
     * Returns a reference to the vector of neurons of this network.
     */
    std::vector<Neuron>& getNeurons(){
        return _neurons;
    }

    //TODO: MORE CODE HERE IF YOU WANT

private:
    std::vector<Neuron> _neurons;
};


/**
 * Convenience function for writing network connections to a file-stream.
 */
std::ostream& operator<<(std::ostream& is, NeuralNetwork& obj){
    std::vector<Neuron> neurons = obj.getNeurons();
    for(size_t i=0; i<neurons.size(); i++){
        is << neurons[i].getConnections() << " ";
    }
    return is;
}



#endif /* NEURALNETWORK_HPP_ */
