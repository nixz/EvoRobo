/*
 * TouchSensor.hpp
 *
 *  Created on: Jan 23, 2015
 *      Author: Joost Huizinga
 */

#ifndef TOUCHSENSOR_HPP_
#define TOUCHSENSOR_HPP_

/**
 * A simple touch sensor based on the bullet collision callback function.
 */
class TouchSensor
{
public:
    /**
     * Creates a touch sensor
     */
    TouchSensor(): _value(false){

    }

    /**
     * Sets the value of the touch sensor.
     */
    void setValue(bool value){
        _value = value;
    }

    /**
     * Returns the value of the touch sensor.
     *
     * True indicates this touch sensor is touching an object, false otherwise.
     */
    bool getValue(){
        return _value;
    }

    /**
     * Resets the value of the touch sensor to false.
     *
     * Should be called after every time-step to get accurate readings.
     */
    void reset(){
        _value=false;
    }

private:
    bool _value;
};



#endif /* TOUCHSENSOR_HPP_ */
