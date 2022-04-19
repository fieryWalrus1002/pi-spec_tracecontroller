# pi-spec_tracecontroller

This is a component of a custom benchtop spectrophotometer for measuring of photosynthesis paramters in vivo. Previous devices did not offer the time resolution and reliability required for new types of experiments, so the pi-spec was developed to satisfy these needs. It is capable of difference absorbance measurements of cytb6f and p700 in plant leaves, as well as PAM fluorescence measurements.

This software runs on a teensy 4.1 microcontroller, and will execute a measurement trace according to parameters sent from a python program running on a Raspberry Pi. It collects data from a Si PIN photodiode with a 16-bit ADC. 
