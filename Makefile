default:
	mkdir -p out
	c++ -g src/main.cpp src/spi/spi.cpp -o out/main

run:
	./out/main