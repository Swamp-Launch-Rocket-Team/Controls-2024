default:main.cpp IMU/imu.cpp
	c++ -std=c++11 -g main.cpp IMU/imu.cpp Dynamics_Model_Controller/controller.cpp Dynamics_Model_Controller/dynamics_model.cpp  Dynamics_Model_Controller/drag.cpp Dynamics_Model_Controller/pi.cpp busynano/busynano.cpp bitbang/bitbang.cpp -o main -lwiringPi -O3

sensor_test: sensor_test.cpp IMU/imu.cpp bitbang/bitbang.cpp busynano/busynano.cpp
	c++ -std=c++11 -g sensor_test.cpp IMU/imu.cpp bitbang/bitbang.cpp busynano/busynano.cpp -o sensor_test -lwiringPi