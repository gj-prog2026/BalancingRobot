### This is a log of observations drawn from the db2.txt
**Methodology**: The commands `v0.5  t5  g1` were sent to the balancing bot via BLE through the PuTTY terminal. It's original purpose was to see if reducing v (scale factor of states: distance moved from starting point and speed at which it moves across the distance).

**Result**: In db2.txt, lines 22-38 in the first run's log and lines 197-207 in the second run's log showed enc_d was 0 i.e. the motor shaft was not spinning at all despite rel deviating from the equilibrium position

**Hypothesis**: This could be the result of deadband - a factor considered in the simulation model but assumed to be ideal. A deadband compensation with live knob sent through PuTTY will be added to test this hypothesis.