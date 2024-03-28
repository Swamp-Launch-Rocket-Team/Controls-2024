default:main.cpp IMU/imu.cpp bitbang/bitbang.cpp busynano/busynano.cpp Dynamics_Model_Controller/controller.cpp Dynamics_Model_Controller/drag.cpp Dynamics_Model_Controller/dynamics_model.cpp Dynamics_Model_Controller/pi.cpp Logger/logger.cpp
	c++ -std=c++20 -g main.cpp IMU/imu.cpp Dynamics_Model_Controller/controller.cpp Dynamics_Model_Controller/dynamics_model.cpp  Dynamics_Model_Controller/drag.cpp Dynamics_Model_Controller/pi.cpp busynano/busynano.cpp bitbang/bitbang.cpp Logger/logger.cpp -o main -lwiringPi -O3

sensor_test: sensor_test.cpp IMU/imu.cpp bitbang/bitbang.cpp busynano/busynano.cpp
	c++ -std=c++20 -g sensor_test.cpp IMU/imu.cpp bitbang/bitbang.cpp busynano/busynano.cpp -o sensor_test -lwiringPi

servo_test: Servo/Servo_test.cpp 
	c++ -std=c++20 -g Servo/Servo_test.cpp -o servo_test -lwiringPi

clean:
	rm -f msg-*.csv
	rm -f state-*.csv
	echo 0 > metadata