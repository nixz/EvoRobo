#!/usr/bin/python
import sys
import matplotlib.pyplot as plt
   
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

def getColumn(matrix, column_nr=0):
   column = []
   for i in xrange(len(matrix)):
      column.append(matrix[i][column_nr])
   return column

for arg in sys.argv[1:]:
   matrix = readFile(arg)
   column = getColumn(matrix)
   plt.plot(range(len(column)), column)
   
plt.xlabel("Generations")
plt.ylabel("Fitness")
plt.savefig("lineplot.pdf")

