default: busynano_test

busynano_test: busynano.cpp busynano_test.cpp
	c++ -std=c++11 -g -O2 -o busynano_test busynano_test.cpp busynano.cpp -lwiringPi

clean:
	rm -f busynano_test
