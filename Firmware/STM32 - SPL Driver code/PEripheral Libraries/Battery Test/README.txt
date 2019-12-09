Gives you a rough idea of battery levels between an ADC value of about 1800 to 137. Should be enough resolution to get an idea of the battery level
However the board i tested it on had the capacitor removed so that may affect things.
There also appears to be some leakage current when the pin is high but it appears to be extremely minor and not a problem.

ADC and GPIO pins have been initialised in this code. Just need to set and reset the pin to turn on and off the transistor while the ADC pins reads constant (SSet a breakpoint somewhere in there).
PA2 = 1 (Transistor off and cant read)
PA2 = 0 (Transistor on and can properly read)