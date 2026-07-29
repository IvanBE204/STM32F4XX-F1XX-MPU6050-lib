MPU6050 Library for STM32!
So for this has been tested for the NUCLEO F411RET6 and the Blupill (F103C6T).

The library has a set of functions for seemless implementation and ease of use!
To use it just create an MPU6050 object like this
MPU6050_t myMPU;

Then initialize it with either the default_ini or the custom_init.

Once everything is done you can start reading values!

UPDATE JULY/15: Kalman filter added to the library!
It comes as a separate object KalmanFilter_t. Same thing as the MPU6050_t, initialize the object and just use it!

UPDATE JULY/28: Mahony filter added to the library!
As always, you have a mahonyfilter type which you pass to the init and compute functions. Also you can use toEule function
to convert the quaternion orientation to euler angles orientation.
