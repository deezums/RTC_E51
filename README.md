# RTC_E51
Library for using onboard RTC on SAME/D51 CPUs

Using CRYSTALLESS defines to automatically switch to the ULP32k osc when no external crystal is avaialble. 

Needs some work...

Loading time from backup needs work, not very clean at the moment. Checking reset cause needs to be reworked to include BOD1.2? Need to configure PM, Vbat mux?
Currently allows overwriting of the "DO NOT MODIFY" ULP32K calibration register values. Mine is 2x slow without loading 0x23. The default value is hidden in userpage, and mine loads 0x00 on it's own. Maybe use the RTC PPM calibration instead?
Crystal untested, I don't have a crystal board just yet.

