## Introduction
This folder constains several subfolder and a file as follows:

| Subfolder name | Purpose |
|--------------|---------|
|[development_stages](software/development_stages)| In the early stage of development, I uploaded firmware by parts based on the different components such as MPU6050 and HC05 so that I can guarantee each part is functioning before proceeding, assiting with debug|
|[known_good_firmware](software/known_good_firmware)| This folder stores the most recent successful firmware upload. Currently, it has only one entry: the one used to achieve 40s of staying upright|
|[tests_LQR](software/tests_LQR)| This contains the test result logs from after I changed the algorithm to LQR|
|[tests_PID](software/tests_PID)| This contains the readings printed on PuTTY from when I was using the PID algorithm|

> **Who did what.** The development_stages firmware was written by Claude, as I was new to C.
> In the LQR stage I wrote and integrated the firmware and ran every hardware test.
> Claude reviewed the code, helped analyse the logs (for example, finding the IMU lag mismatch) and suggested specific changes, which I implemented and tested.
