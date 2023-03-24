/*
  drv8871.cpp - Library for interfacing the DRV8871 DC motor driver.
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
#include "Arduino.h"
#include "drv8871.h"

DRV8871::DRV8871(int pin1, int pin2){
  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  coast();

  _pin1 = pin1;
  _pin2 = pin2;
}

void DRV8871::brake(){
  digitalWrite(_pin1, HIGH);
  digitalWrite(_pin2, HIGH);
}

void DRV8871::forward(){
  digitalWrite(_pin1, HIGH);
  digitalWrite(_pin2, LOW);
}

void DRV8871::reverse(){
  digitalWrite(_pin1, LOW);
  digitalWrite(_pin2, HIGH);
}

void DRV8871::coast(){
  digitalWrite(_pin1, LOW);
  digitalWrite(_pin2, LOW);
}
