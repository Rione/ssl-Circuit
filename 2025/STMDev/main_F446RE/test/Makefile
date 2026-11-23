# PC (Linux・WSL・macOS) の g++ で、HAL に依存しない部分の試験を動かす。ファームには入らない。
#   cd test && make
CXX ?= g++
CXXFLAGS = -std=c++17 -Wall -Wextra -Werror -O1 -I../src/unit -I../src/mode

ras_protocol_test: ras_protocol_test.cpp ../src/unit/RasProtocol.cpp ../src/unit/RasProtocol.hpp
	$(CXX) $(CXXFLAGS) -o $@ ras_protocol_test.cpp ../src/unit/RasProtocol.cpp

mode_policy_test: mode_policy_test.cpp ../src/mode/ModePolicy.hpp
	$(CXX) $(CXXFLAGS) -o $@ mode_policy_test.cpp

.PHONY: test clean
test: ras_protocol_test mode_policy_test
	./ras_protocol_test
	./mode_policy_test

clean:
	rm -f ras_protocol_test mode_policy_test

.DEFAULT_GOAL := test
