# ECE387
My ongoing ECE 387 project

I had to export this from the online Mbed complier suite to get my source files and the library source files. I chose to export it for the 
GCC(ARM Embedded) toolchain as I thought this would give me less annoying toolchain specific files and more of the straight source code. The 
code probably does not function completely as indended in this form.

==================== DIR description ====================

Added Value - Contains my personal source code, all of which was written by me with some functions being adapted from my referenences

Library with Configuration Files for Mbed OS - Contains all libraries associated with the Mbed OS

Makefile - The toolchain export included a makefile which would need some editing to make sure it reflects the new directories

Main library File - Here I solely copied the I2C.cpp and I2C.h files over as it was the main library I directly used 
