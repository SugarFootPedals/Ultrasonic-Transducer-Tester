# Ultrasonic-Transducer-Tester
tests for the Resonant and Harmonic Frequencies of Ultrasonic Transducers

Using a SEEED RP2040 Zero and a 9vdc input, user selects the frequency range to scan
an Ultrasonic Cleaning Transducer to determin its exact Resonant Frequency and best Harmonic
Frequency. 

I split the it into three frequency ranges, since scanning takes some time. I recommend scan each transducer
through each ranges and write down the best output based on the graph and max current draw from the OLED display.

I built this small circuit board to test transducers after discovering that transducers I bought
from China for my line of Vibrato Ultrasonic Cleaners that the listed resonant frequency was NOT
it's actual resonant frequency, but a harmonic that caused excessive heat.

With this devise, I can determin the actual Resonant Frequency, adjust my circuit accordingly and even
create a Dual-Frequency Ultrasonic Cleaner that generates more powerful cavitation with less heat
and fewer failures!
