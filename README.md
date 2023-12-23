# STARLIGHT
STARLIGHT is an all-in-one flight computer with all sorts of features useful to have in a model rocket flight computer.
You can read more about the board [here.](https://shop.circuitwizardry.com/products/starlight)

This repository contains all the code necessary to interface with the sensors on STARLIGHT.

Check out `example.py` for a quick start on reading sensors on the board, and `starlight.py` for the raw code that talks to the two sensors!

# Getting Started
Now, you may be asking – How do I actually write code for this board? There are loads of tutorials for other boards on the market, but STARLIGHT is somewhat unique in a sense. The act of programming it will be the same, but the features that come pre-packaged with the board trump many other current boards on the market.

[Download the STARLIGHT UF2 for this board. This board uses a custom UF2 file, you can download it here.
](https://circuitwizardry.com/wp-content/uploads/2023/09/starlight_micropython_9_10_2022_revB.uf2)

Install Thonny IDE. This is how you’re going to write and upload code to the board. You can find a link to download it here.
Plug in your board: Drag the .uf2 file into the USB mass storage device that the board boots into by default, give it some time to copy over. If this operation succeeds, the board will reboot and you will no longer see it as a mass storage device.

Select the MicroPython interpreter. This can be done in the bottom right of Thonny IDE.

If you followed the above steps, you should see a MicroPython console in Thonny IDE!

Print “Hello World”! In your MicroPython console, type `print(“Hello World!”)` and press ENTER. If everything is working, you should get a response!

Now turn on the on-board LED. Paste this code into the console, and press ENTER. You should see the LED on the STARLIGHT board light up!
```python
from machine import Pin
led = Pin(24, Pin.OUT)
led.value(1)
```
In order to run code from a file, simply paste the code into the text editor and click the green ▶️ run button at the top of the IDE. You can also save files directly to the RP2040 by selecting “Raspberry Pi Pico” when saving.

# Reading Sensors
In order to read sensors and begin using STARLIGHT to its fullest, check out `example.py`. If you're interested in sensor fusion, check out the `fusion` directory.
