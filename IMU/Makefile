default: imu_test

imu_test: imu.cpp imu.h imu_test.cpp ../busynano/busynano.cpp ../busynano/busynano.h
	c++ -std=c++11 -g -o imu_test imu_test.cpp imu.cpp ../busynano/busynano.cpp

drift_test: imu.cpp imu.h imu_drift_test.cpp
	c++ -std=c++11 -g -o drift_test imu_drift_test.cpp imu.cpp

calibration_test: imu.cpp imu.h calibration_test.cpp
	c++ -std=c++11 -g -o calibration_test calibration_test.cpp imu.cpp

config_imu: imu.cpp imu.h config_imu.cpp
	c++ -std=c++11 -g -o config_imu config_imu.cpp imu.cpp ../busynano/busynano.cpp

plog: imu.cpp imu.h log_imu.cpp
	c++ -std=c++11 -g -o plog log_imu.cpp imu.cpp ../busynano/busynano.cpp -lwiringPi

clean:
	rm -f imu_test calibration_test drift_test config_imu
