# Mbed OS 5 library for CryptoAuthLib

**[Example program](https://github.com/janjongboom/mbed-os-example-cryptoauthlib)**

Library to talk to Microchip CryptoAuth devices from Mbed OS. This library currently only supports I2C devices, but modifying it to also deal with SWI or UART should be relatively simple.

Compatible with:

* ATSHA204A
* ATECC108A
* ATECC508A
* ATECC608A

Right-now it only searches for the CryptoAuth device on the *main* I2C bus. If you want to change this (or help me patch it), see `source/hal_mbed.cpp`, in the `hal_i2c_init` function.

## How to add this library to your project

1. In Mbed CLI, run:

    ```
    $ mbed add https://github.com/janjongboom/mbed-cryptoauthlib
    ```

1. Or, in the Online Compiler:
    * Right-click on your project.
    * Select *Import library > From URL...*.
    * Enter: `https://github.com/janjongboom/mbed-cryptoauthlib`.

## Running unit tests

To run all the unit tests, see the [example program](https://github.com/janjongboom/mbed-os-example-cryptoauthlib).
