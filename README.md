# mite-servo

This is an 8-channel hobby servo controller using CH32V003. It generates 8
separate 50Hz PWM signals, with pulse width between 0 and about 3000µs, with
16-bit precision.  The controller communicates over I2C, and exposes 8 16-bit
registers, where you can write the desired pulse width for the given channel.
To lessen the power requirements a little, the pulses are staggered, four at a
time.

The output pins are as follows: PD2, PA1, PC3, PC4, PC1, PC7, PD6, PD5.

For a PCB that goes with this code, see [the project page](https://deshipu.art/projects/project-mite-servo/).
