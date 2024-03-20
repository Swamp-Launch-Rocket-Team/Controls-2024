main:
	echo "Not done"

test:
	c++ altimiter_test.cpp ../busynano/busynano.cpp -o altimiter_test -Wall -Wextra -lwiringPi
	c++ bitbang.cpp ../busynano/busynano.cpp -o bitbang_test -Wall -Wextra -lwiringPi

clean:
	rm -f altimiter_test