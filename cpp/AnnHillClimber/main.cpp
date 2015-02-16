/*
 * Main.cpp
 *
 *  Created on: Jan 22, 2015
 *      Author: Joost Huizinga
 *
 * This is the main file for homework 1.
 */

#define _USE_MATH_DEFINES
#include <iostream>
#include <fstream>
#include <math.h>

/* Assignment 2 */
#include "NeuralNetwork.hpp"

int main(int argc, char* argv[]) {
    NeuralNetwork network( 10 );
    network.randomize();
    std::ofstream networkFile("hw2_network.csv");
    networkFile << network << "\n";
    networkFile.close();

    std::ofstream activationFile("hw2_activation.csv");
    // Initiate the network with random values
    for (int i=0; i < network.getNeurons().size(); i++) {
        network.getNeurons()[i].setValue( randDouble());
    }
    // Run the network for 50 time steps and log the activation
    for (int i=0; i < 50; i++) {
        network.step();
        network.logActivation( activationFile );
    }


    return 0;
}
