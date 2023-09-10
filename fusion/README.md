# Sensor Fusion example

This example uses the following MicroPython sensor fusion library to get the
pitch and roll of the Starlight board: https://github.com/micropython-IMU/micropython-fusion

## Example

Install ``starlight.py``, ``fusion.py``, ``deltat.py``, and ``example.py`` on your
Starlight board, then run ``example.py`` to print the pitch and roll.

## Usage

To incorporate into your own program, make sure that you are calling ``update_nomag``
often in your control loop for the library to be able to perform the calculations.
