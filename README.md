# XC Quantum correlators driver library [![DOI](https://zenodo.org/badge/295015500.svg)](https://zenodo.org/badge/latestdoi/295015500)

Documentation: https://ahp-electronics.github.io/libahp-xc

#### This repository contains the driver library for the XC series correlators

![AHP XC8 cross-correlator](https://iliaplatone.com/Pictures/XC8_small.png "XC8")

To use the correlator try the GUI: https://github.com/ahp-electronics/xc-gui

More informations at https://www.iliaplatone.com/xc8

The library uses 57600 baud/second UART communication with the correlator

There is a set of commands to start integrations:

    0x0d: Set integration parameters: bit 4 => enable capture, bit 5 => use external clock, bit 6 => reset timestamp when enabling capture, bit 7 => enable extra commands
    0x01: select active line: bits [7:6] => indexer, bits [5:4] => value (if extra commands bits [5:4] are routed to 4 extra lines)
    0x02: activate leds or power lines using bits [5:4], invert pulse reading with bit 6, single clock cycle pulse width with bit 7
    0x03: baudrate 57600 left-shifted by bits [7:4]
    0x04: bits [1:0] => indexer, bits [6:4] => delay value. If bit 7 is 0, then start delay for cross-correlations is set, if bit 4 is 1, then start delay for autocorrelations is set, if extra commands these values define the size of the correlation scan).
    0x08: sampling clock tau = clock tau left-shifted by bits [7:4]
    0x09: power voltage = bits [7:4]
    0x0c: Set tests: bit 5 => enable pll oscillator signal on led 0, bit 5 => enable autocorrelation scan, bit 6 => enable crosscorrelation scan, bit 7 => BCM encoder on led 0 (pulse XOR by sampling clock) (if extra commands bits [5:4] are routed to 4 extra tests flags)

The count of pulses and correlation comes with an ASCII packet string ended with a 0x0d character

Each packet starts with a header with payload length indication, it is possible to change some parameters from the code

header

    bytes 0-1: hexadecimal sample size value
    bytes 2-3: hexadecimal inputs quantity
    bytes 4-6: hexadecimal delay channels quantity
    bytes 7-10: hexadecimal live delay channels quantity
    bytes 11: hexadecimal flags [bits: 0=live autocorrelator, 1=live crosscorrelator, 2=leds available, 3=cross-correlator]
    bytes 12-15: hexadecimal value of the clock tau in picoseconds

payload

    bytes +lines#: pulses count of each line within the integration time
    bytes +lines#: autocorrelations count of each line by the selected autocorrelation line delayed by crosscorrelation lag zero
    bytes +baselines#: crosscorrelations count of pulses of each line with others by the selected delay amount
    bytes +16: 8-byte timestamp of current packet

checksum

    bytes +1: 1-byte CRC of packet payload

The packet rate is determined by the baud rate and the packet size, the sampling rate is determined by the clock tau multiplied by the number of mux lines, divided by the clock tau multiplier plus one power of two.
