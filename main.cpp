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
#include <math.h>

/* Assignment 1 */
#include "HillClimber.hpp"
#include "VectorIndividual.hpp"
#include "VectorFitnessFunction.hpp"
#include "Random.hpp"

void zero()
{
    seed();
    VectorFitnessFunction fitnessFunction;
    VectorIndividual initialIndividual(50, 0.0, 1.0, 0.05);
    HillClimber<VectorIndividual, VectorFitnessFunction> hillClimber;
    hillClimber.init(initialIndividual, fitnessFunction);
    hillClimber.run(5000, "hw1_fitness1.csv", "hw1_individual1.csv");
}

void one()
{
    seed(1);
    VectorFitnessFunction fitnessFunction;
    VectorIndividual initialIndividual(50, 0.0, 1.0, 0.05);
    HillClimber<VectorIndividual, VectorFitnessFunction> hillClimber;
    hillClimber.init(initialIndividual, fitnessFunction);
    hillClimber.run(5000, "fit1.csv", "ind1.csv");
}

void two()
{
    seed(2);
    VectorFitnessFunction fitnessFunction;
    VectorIndividual initialIndividual(50, 0.0, 1.0, 0.05);
    HillClimber<VectorIndividual, VectorFitnessFunction> hillClimber;
    hillClimber.init(initialIndividual, fitnessFunction);
    hillClimber.run(5000, "fit2.csv", "ind2.csv");
}

void three()
{
    seed(3);
    VectorFitnessFunction fitnessFunction;
    VectorIndividual initialIndividual(50, 0.0, 1.0, 0.05);
    HillClimber<VectorIndividual, VectorFitnessFunction> hillClimber;
    hillClimber.init(initialIndividual, fitnessFunction);
    hillClimber.run(5000, "fit3.csv", "ind3.csv");
}

void four()
{
    seed(4);
    VectorFitnessFunction fitnessFunction;
    VectorIndividual initialIndividual(50, 0.0, 1.0, 0.05);
    HillClimber<VectorIndividual, VectorFitnessFunction> hillClimber;
    hillClimber.init(initialIndividual, fitnessFunction);
    hillClimber.run(5000, "fit4.csv", "ind4.csv");
}

void five()
{
    seed(5);
    VectorFitnessFunction fitnessFunction;
    VectorIndividual initialIndividual(50, 0.0, 1.0, 0.05);
    HillClimber<VectorIndividual, VectorFitnessFunction> hillClimber;
    hillClimber.init(initialIndividual, fitnessFunction);
    hillClimber.run(5000, "fit5.csv", "ind5.csv");
}

int main(int argc, char* argv[]) {
    zero();
    one();
    two();
    three();
    four();
    five();

    return 0;
}
