default:
	echo "NOT DONE YET"

sensor_test:
	c++ -std=c++11 -g sensor_test.cpp IMU/imu.cpp bitbang/bitbang.cpp busynano/busynano.cpp -o sensor_test -lwiringPi