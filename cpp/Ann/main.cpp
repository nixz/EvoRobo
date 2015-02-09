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

    return 0;
}
