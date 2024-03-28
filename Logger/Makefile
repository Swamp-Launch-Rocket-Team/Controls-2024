default:logger.cpp logger_test.cpp
	c++ -std=c++20 -g logger_test.cpp logger.cpp -o logger_test -lpthread

clean:
	rm -f msg-*.csv
	rm -f state-*.csv
	echo 0 > metadata
	