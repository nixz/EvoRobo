#!/usr/bin/python
import sys
import numpy as np 
import matplotlib.pyplot as plt
import math

def readFile(filename):
   inputFile = open(filename, 'r')
   matrix = []

   inputFile.seek(0)
   for line in inputFile:
      line = line.strip()
      numbers  = line.split()
      row = []
      for number in numbers:
         row.append(float(number))
      matrix.append(row)
   inputFile.close()

   return matrix

def plotNetwork(row):
   numNeurons = int(math.sqrt(len(row)))
   neuronPositions = []
   angle = 0.0
   angleUpdate = 2 * np.pi / numNeurons
   for i in range(0,numNeurons):
      x = np.sin(angle)
      y = np.cos(angle)
      angle = angle + angleUpdate
      neuronPositions.append((x,y))

   for neuronPos in neuronPositions:
      print neuronPos
      x, y = neuronPos
      plt.plot(x, y,'ko',markerfacecolor=[1,1,1], markersize=18)

   for neuron1 in xrange(numNeurons):
      for neuron2 in xrange(numNeurons):
         x1, y1 = neuronPositions[neuron1]
         x2, y2 = neuronPositions[neuron2]
         weight = row[neuron1+neuron2*numNeurons]
         w = int(10*abs(weight))+1
         if weight < 0.0:
            plt.plot((x1,x2), (y1,y2), color=[0.8,0.8,0.8], linewidth=w)
         else:
            plt.plot((x1,x2), (y1,y2), color=[0,0,0], linewidth=w)

for arg in sys.argv[1:]:
    matrix = readFile(arg)
    for i in xrange(len(matrix)):
       plotNetwork(matrix[i])
       plt.savefig(arg +"_" + str(i) + ".pdf")
       plt.clf()

