# XC Quantum correlators driver library [![DOI](https://zenodo.org/badge/295015500.svg)](https://zenodo.org/badge/latestdoi/295015500)

Documentation: https://ahp-electronics.github.io/libahp-xc

#### This repository contains the driver library for the XC series correlators

![AHP XC8 cross-correlator](https://iliaplatone.com/Pictures/XC8_small.png "XC8")

To use the correlator try the GUI: https://github.com/ahp-electronics/xc-gui

More informations at https://www.iliaplatone.com/xc8

The library uses 57600 baud/second UART communication with the correlator

Here below is defined the communication protocol:

    0x0d: Set integration parameters: bit 4 => enable capture, bit 5 => use external clock, bit 6 => reset timestamp when enabling capture, bit 7 => extra commands
    0x01: select active line: first byte: bits [7:4] => length of command LSN, second byte: length of command MSN, next bytes: bits [7:4] => active line LSN first
    0x02: led lines: extra commands => nibble indexer, bits [7:4] => power voltage value
    0x03: if extra commands = 1 first byte: bits [7:4] => length of command LSN, second byte: length of command MSN, next bytes: bits [7:4] => cross-correlator order LSN first, if extra commands = 0 baudrate 57600 left-shifted by bits [7:4] 
    0x04: Set voltage: extra commands => nibble indexer, bits [7:4] => power voltage value
    0x08: sampling clock tau =  first two byte: bits [7:4] => length of command LSN first, next bytes: bits [7:4] => value LSN first - if test[5:4] is 0 scan increment, if test[5:4] is 1 scan length if test[5:4] scan start channel if extra commands = 0 autocorrelator, if extra commands = 1 crosscorrelator.
    0x0c: Set tests: extra commands => nibble indexer, bits [7:4] => test lines value - bit 1 => enable autocorrelation scan, bit 2 => enable crosscorrelation scan, bit 3 => BCM encoder on led 0 (pulse XOR by sampling clock), bits [5:4] => indexer of sampling clock tau

The count of pulses and correlation comes with an ASCII packet string ended with a 0x0d character

Each packet starts with a header with payload length indication, it is possible to change some parameters from the code

header

    char 0: Nibble length of input lines hex digits [n0]
    chars +n0: hexadecimal inputs quantity
    char +1: Nibble length of sample size value hex digits [n1]
    chars +n1: hexadecimal sample size value
    char +1: Nibble length of delay channels hex digits [n2]
    chars +n2: hexadecimal delay channels
    char +1: Nibble length of live autocorrelator delay channels hex digits [n3]
    chars +n3: hexadecimal live autocorrelator delay channels
    char +1: Nibble length of live crosscorrelator delay channels hex digits [n4]
    chars +n4: hexadecimal live crosscorrelator delay channels
    chars +2 hexadecimal flags [bits: 0=live autocorrelator, 1=live crosscorrelator, 2=leds available, 3=cross-correlator, 4=Nyquist sampling, 5=Shannon sampling]
    chars +4: hexadecimal value of the clock tau in picoseconds

payload

    chars +lines#: pulses count of each line within the integration time
    chars +lines#: autocorrelations real/imaginary count of each line within the packet time
    chars +baselines#: crosscorrelations real/imaginary count of pulses of each line with others ones within the packet time
    chars +16: 8-byte timestamp of current packet

checksum

    chars +2: 1-byte CRC of packet payload

EOP

    chars +1: carriage return indicates end of packet

The packet time is determined by the baud rate and the packet size, the sampling rate is determined by the clock tau multiplied by the number of mux lines, raised by the clock tau power of two exponent.
