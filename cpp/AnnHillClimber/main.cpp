/*
 * Main.cpp
 *
 *  Created on: Jan 22, 2015
 *      Author: Joost Huizinga
 *
 * This is the main file for homework 1.
 */

#define _CHECKER
// #undef  _CHECKER

#define _USE_MATH_DEFINES
#include <iostream>
#include <fstream>
#include <math.h>

/* Assignment 3 */
#include "NeuralNetwork.hpp"
#include "HillClimber.hpp"

#ifdef _CHECKER
#include "NNCheckerboardFitness.hpp"
#else
#include "NNLastRowFitness.hpp"
#endif


int main(int argc, char* argv[]) {
    seed( 9 );
    NeuralNetwork network( 10 , 0.05 );
#ifdef _CHECKER
    NNCheckerboardFitness fitnessFunction;
    HillClimber<NeuralNetwork, NNCheckerboardFitness> hillClimberNN;
#else
    NNLastRowFitness fitnessFunction;
    HillClimber<NeuralNetwork, NNLastRowFitness> hillClimberNN;
#endif

    hillClimberNN.init( network, fitnessFunction );

    // Pre Activation
    hillClimberNN.getParent().init( 0.5 );
    std::ofstream preActivationFile("hw3_preActivation.csv");
    for (int i=0; i < 10; i++) {
        hillClimberNN.getParent().step();
        hillClimberNN.getParent().logActivation( preActivationFile );
    }
    preActivationFile.close();

    // Evolve
    hillClimberNN.run( 1000, "hw3.fitness.csv", "hw3_individual.csv" );

    // Post activation
    hillClimberNN.getParent().init( 0.5 );
    std::ofstream postActivationFile("hw3_postActivation.csv");
    for (int i=0; i < 10; i++) {
        hillClimberNN.getParent().step();
        hillClimberNN.getParent().logActivation( postActivationFile );
    }
    postActivationFile.close();

    return 0;
}
