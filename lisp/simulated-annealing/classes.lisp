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
         :accessor min
         :allocation :instance
         :documentation "the min value")
   (max :initarg :max
         :initform (error ":max must be specified")
         :accessor max
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
