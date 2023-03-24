/*
  drv8871.h - Library for interfacing the DRV8871 DC motor driver.
  Created by robotto, January 9, 2022.
  Released into the public domain.

DRV8871 has two input pins, and two output pins:

|------------------------------------|
| IN1 | IN2 || OUT1 | OUT2 | Note    |
|------------------------------------|
|  0  |  0  || HI-Z | HI-Z | Coast   |
|------------------------------------|
|  0  |  1  ||  L   |  H   | Reverse |
|------------------------------------|
|  1  |  0  ||  H   |  L   | Forward |
|------------------------------------|
|  1  |  1  ||  L   |  L   | Brake   |
|------------------------------------|
*/

#ifndef DRV8871_h
#define DRV8871_h

#include "Arduino.h"

class DRV8871
{
  public:
    DRV8871(int pin1, int pin2);
    void brake();
    void forward();
    void reverse();
    void coast();
  private:
    int _pin1;
    int _pin2;
};

#endif