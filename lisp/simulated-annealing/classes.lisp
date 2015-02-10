;;;; -*- Mode: Lisp; indent-tabs-mode: nil -*-
;;;; ==========================================================================
;;;; classes.lisp --- The file contains all the classes
;;;;
;;;; Copyright (c) 2013, Nikhil Shetty <nikhil.j.shetty@gmail.com>
;;;;   All rights reserved.
;;;;
;;;; Redistribution and use in source and binary forms, with or without
;;;; modification, are permitted provided that the following conditions
;;;; are met:
;;;;
;;;;  o Redistributions of source code must retain the above copyright
;;;;    notice, this list of conditions and the following disclaimer.
;;;;  o Redistributions in binary form must reproduce the above copyright
;;;;    notice, this list of conditions and the following disclaimer in the
;;;;    documentation and/or other materials provided with the distribution.
;;;;  o Neither the name of the author nor the names of the contributors may
;;;;    be used to endorse or promote products derived from this software
;;;;    without specific prior written permission.
;;;;
;;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
;;;; "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
;;;; LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
;;;; A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
;;;; OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
;;;; SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
;;;; LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
;;;; DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
;;;; THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
;;;; (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
;;;; OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
;;;; ==========================================================================

(in-package #:sa)

;;; ---------------------------------------------------------------------------
(defclass individual ( )
  ((fitness :initarg :fitness
         :initform 0
         :accessor fitness
         :allocation :instance
         :documentation "fitness value of the individual"))
  (:documentation "The base class for every individual so you can assign a
  fitness to it"))

;;; ---------------------------------------------------------------------------
(defclass vector-individual (individual)
  ((value-vector :initarg :value-vector
         :initform (error ":value-vector must be specified")
         :accessor value-vector
         :allocation :instance
         :documentation "holds the array of genes")
   (min :initarg :min
         :initform (error ":min must be specified")
         :accessor range-min
         :allocation :instance
         :documentation "the min value")
   (max :initarg :max
         :initform (error ":max must be specified")
         :accessor range-max
         :allocation :instance
         :documentation "the max value")
   (mutation-rate :initarg :mutation-rate
         :initform (error ":mutation-rate must be specified")
         :accessor mutation-rate
         :allocation :instance
         :documentation "the rate of mutation"))
  (:documentation "An individual that consists of a vector of doubles. Note
  that hte minimum, maximum and mutation rate are all stored with the
  individual"))

;;; ---------------------------------------------------------------------------
(defun make-vector-individual (&key size (min 0.0) (max 1.0) (mutation-rate 0.05))
  "A constructor to construct a vetor individual. The size has to be specified
with other optional keys like min,max and mutation-rate"
  (initialize-instance 'vector-individual
                       :value-vector (make-array size)
                       :min min
                       :max max
                       :mutation-rate mutation-rate))

;;; ---------------------------------------------------------------------------
(defclass neuron ( )
  ((value :initarg :value
         :initform 0
         :allocation :instance
         :documentation "the value of the neuron")
   (new-value :initarg :new-value
         :initform 0
         :allocation :instance
         :documentation "where whe new value is stored")
   (incomming-connections :initarg :incomming-connections
         :type 'vector-individual
         :allocation :instance
         :documentation "all the incoming connection into the neuron"))
  (:documentation "
A simple neuron class without activation function.

Note that this class is mostly appropriate for fully connected networks;
sparsely connected networks should consider a graph like network instead.

Also note that this class supports recurrent connections. If the network is
feed-forward, a simpler neuron model can be used."))

;;; ---------------------------------------------------------------------------
(defun make-neuron (&key incomming-connections-size)
  (initialize-instance 'neuron
                       :value 0
                       :new-value 0
                       :incomming-connections
                       (make-vector-individual :size incomming-connections-size)))

;;; ---------------------------------------------------------------------------
(defclass neural-network (individual)
  ((neurons :initarg :neurons
            :initform nil
            :accessor neurons
         :allocation :instance
         :documentation "The set of neurons making up the neural network"))
  (:documentation "the neural network"))

;;; ---------------------------------------------------------------------------
(defun make-neural-network (&key number-of-connections)
  (initialize-instance 'neural-network
                       :neurons (make-array number-of-connections
                                            :element-type 'neuron)))

;;; ---------------------------------------------------------------------------
(defclass hill-climber ()
  ((parent :initarg :parent
         :initform (error ":parent must be specified")
         :type 'individal
         :allocation :instance
         :documentation "The parent individual"))
  (:documentation "Implements the hillclimber algorithm"))

;;; ---------------------------------------------------------------------------
(defgeneric size (individual))
(defgeneric randomize (individual))
(defgeneric mutate (individial))
(defgeneric fitness (individual))

;;; ---------------------------------------------------------------------------
(defmethod size ((entity vector-individual))
  "Returns the size of the vector"
  (with-slots (value-vector) entity
    (length value-vector)))

;;; ---------------------------------------------------------------------------
(defmethod randomize ((entity vector-individual) )
  "
Randomizes the vector
**********************

Implement such that each element in the vector is assigned a random value
between min and max"
  (with-slots (value-vector range-min range-max) entity
    (dotimes (i (size entity))
      (setf (elt value-vector i) (rand :min range-min
                                       :max range-max)))))

;;; ---------------------------------------------------------------------------
(defmethod mutate ((entity vector-individual))
  "
Mutates the vector
*******************

Implement such that each element in the vector has a _mutationRate chance of
being re-assigned a value between _min and _max.
"
  (with-slots (value-vector range-min range-max mutation-rate) entity
    (dotimes (i (* mutation-rate (array-total-size value-vector)))
      (setf (elt value-vector (rand :max (size entity)))
            (rand :min range-min
                  :max range-max)))))

;;; ---------------------------------------------------------------------------
(defmethod fitness ((entity vector-individual))
  " 
Calculates the fitness funtion of the individual
************************************************

Function that evaluates and individual and returns its fitness. Implement
such that the fitness of the vector is the average of its elements.
"
  (with-slots (value-vector fitness) entity
    (setf fitness (/ (reduce #'+ value-vector)
                     (length value-vector)))))

