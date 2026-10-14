### Sign Test
## Purpose
This test is to check the conventions of the robot i.e. whether the tilting angle and other measurables align with what the modal assumes as there may be conflicts due to the MATLAB model built entirely from formulae I looked up online.

## Result
The log of the result can be found in [Result_Log][./result_log.txt]. 
Several observations were made:
1. When robot tilted forward, rel was negative which was opposite to what the modal assumed
2. While rel fell from about 95° toward 0 (lines 38-51), gyro read about -50 to -86 °/s - also opposite to modal assumption

**However**, both the gyro and rel had the same type of sign change as they moved forward, which made it manageable and the simulation not completely invalid.