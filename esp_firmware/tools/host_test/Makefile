# Тесты логики прошивки на ПК (без платы). Нужен g++ и библиотека uPID:
#   make UPID_DIR=/путь/к/uPID test
UPID_DIR ?= $(HOME)/Arduino/libraries/uPID
FW = ../../esp32_firmware
CXX ?= g++
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra -Wno-unused-parameter
STUBS = -Istubs -I$(UPID_DIR)

all: sim_floor sim_real sim_new sim_old sim_sweep test_estimator test_encoders_equiv test_glue

sim_new: sim_new.cpp new_chain.h sim_run.h sim_common.h $(FW)/motor_controller.cpp $(FW)/speed_estimator.h stubs/stubs.cpp
	$(CXX) $(CXXFLAGS) $(STUBS) -I$(FW) -I. sim_new.cpp $(FW)/motor_controller.cpp stubs/stubs.cpp -o $@

sim_real: sim_real.cpp new_chain.h sim_run.h sim_common.h $(FW)/motor_controller.cpp $(FW)/speed_estimator.h stubs/stubs.cpp
	$(CXX) $(CXXFLAGS) $(STUBS) -I$(FW) -I. sim_real.cpp $(FW)/motor_controller.cpp stubs/stubs.cpp -o $@

sim_floor: sim_floor.cpp new_chain.h sim_run.h sim_common.h $(FW)/motor_controller.cpp $(FW)/speed_estimator.h stubs/stubs.cpp
	$(CXX) $(CXXFLAGS) $(STUBS) -I$(FW) -I. sim_floor.cpp $(FW)/motor_controller.cpp stubs/stubs.cpp -o $@

sim_sweep: sim_sweep.cpp new_chain.h sim_run.h sim_common.h $(FW)/motor_controller.cpp $(FW)/speed_estimator.h stubs/stubs.cpp
	$(CXX) $(CXXFLAGS) $(STUBS) -I$(FW) -I. sim_sweep.cpp $(FW)/motor_controller.cpp stubs/stubs.cpp -o $@

sim_old: sim_old.cpp sim_run.h sim_common.h orig/motor_controller.cpp stubs/stubs.cpp
	$(CXX) $(CXXFLAGS) $(STUBS) -Iorig -I. sim_old.cpp orig/motor_controller.cpp stubs/stubs.cpp -o $@

test_estimator: test_estimator.cpp sim_common.h $(FW)/speed_estimator.h stubs/stubs.cpp
	$(CXX) $(CXXFLAGS) $(STUBS) -I$(FW) -I. test_estimator.cpp stubs/stubs.cpp -o $@

enc_new.o: $(FW)/encoders.cpp $(FW)/encoders.h
	$(CXX) $(CXXFLAGS) $(STUBS) -I$(FW) -c $(FW)/encoders.cpp -o $@

enc_old.o: orig/encoders.cpp orig/encoders.h
	$(CXX) $(CXXFLAGS) $(STUBS) -Iorig -DEncoders=OldEncoders -c orig/encoders.cpp -o $@

test_encoders_equiv: test_encoders_equiv.cpp enc_new.o enc_old.o stubs/stubs.cpp
	$(CXX) $(CXXFLAGS) $(STUBS) -I$(FW) -I. test_encoders_equiv.cpp enc_new.o enc_old.o stubs/stubs.cpp -o $@

FWCPP = $(filter-out $(FW)/esp32_firmware.ino,$(wildcard $(FW)/*.cpp))

test_glue: test_glue.cpp $(FWCPP) $(FW)/esp32_firmware.ino $(wildcard $(FW)/*.h) stubs/stubs.cpp
	$(CXX) $(CXXFLAGS) $(STUBS) -I$(FW) -I. -x c++ $(FW)/esp32_firmware.ino -x c++ test_glue.cpp $(FWCPP) stubs/stubs.cpp -o $@

test: all
	./test_glue
	./test_estimator
	./test_encoders_equiv
	./sim_old
	./sim_new

clean:
	rm -f test_glue sim_floor sim_real sim_new sim_old sim_sweep test_estimator test_encoders_equiv *.o trace_new.csv
