# Тесты логики прошивки на ПК (без платы). Нужен g++ и библиотека uPID:
#   make UPID_DIR=/путь/к/uPID test
UPID_DIR ?= $(HOME)/Arduino/libraries/uPID
FW = ../../esp32_firmware
TESTS = ../../tests
BRIDGE = ../../../serial_bridge_cpp/include
CXX ?= g++
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra -Wno-unused-parameter
STUBS = -Istubs -I$(UPID_DIR)

all: test_estimator_aligned sim_ladrc sim_sync sim_floor sim_real sim_new sim_old sim_sweep test_estimator test_encoders_equiv test_glue test_protocol_v2 test_bridge_v2 test_end_to_end_v2 test_frame_limits

sim_new: sim_new.cpp new_chain.h sim_run.h sim_common.h $(FW)/motor_controller.cpp $(FW)/speed_estimator.h stubs/stubs.cpp
	$(CXX) $(CXXFLAGS) $(STUBS) -I$(FW) -I. sim_new.cpp $(FW)/motor_controller.cpp stubs/stubs.cpp -o $@

sim_real: sim_real.cpp new_chain.h sim_run.h sim_common.h $(FW)/motor_controller.cpp $(FW)/speed_estimator.h stubs/stubs.cpp
	$(CXX) $(CXXFLAGS) $(STUBS) -I$(FW) -I. sim_real.cpp $(FW)/motor_controller.cpp stubs/stubs.cpp -o $@

sim_floor: sim_floor.cpp new_chain.h sim_run.h sim_common.h $(FW)/motor_controller.cpp $(FW)/speed_estimator.h stubs/stubs.cpp
	$(CXX) $(CXXFLAGS) $(STUBS) -I$(FW) -I. sim_floor.cpp $(FW)/motor_controller.cpp stubs/stubs.cpp -o $@

sim_sync: sim_sync.cpp new_chain.h sim_run.h sim_common.h $(FW)/motor_controller.cpp $(FW)/speed_estimator.h stubs/stubs.cpp
	$(CXX) $(CXXFLAGS) $(STUBS) -I$(FW) -I. sim_sync.cpp $(FW)/motor_controller.cpp stubs/stubs.cpp -o $@

sim_ladrc: sim_ladrc.cpp new_chain.h sim_run.h sim_common.h $(FW)/motor_controller.cpp $(FW)/speed_estimator.h stubs/stubs.cpp
	$(CXX) $(CXXFLAGS) $(STUBS) -I$(FW) -I. sim_ladrc.cpp $(FW)/motor_controller.cpp stubs/stubs.cpp -o $@

sim_sweep: sim_sweep.cpp new_chain.h sim_run.h sim_common.h $(FW)/motor_controller.cpp $(FW)/speed_estimator.h stubs/stubs.cpp
	$(CXX) $(CXXFLAGS) $(STUBS) -I$(FW) -I. sim_sweep.cpp $(FW)/motor_controller.cpp stubs/stubs.cpp -o $@

sim_old: sim_old.cpp sim_run.h sim_common.h orig/motor_controller.cpp stubs/stubs.cpp
	$(CXX) $(CXXFLAGS) $(STUBS) -Iorig -I. sim_old.cpp orig/motor_controller.cpp stubs/stubs.cpp -o $@

test_estimator_aligned: test_estimator_aligned.cpp sim_common.h $(FW)/speed_estimator.h
	$(CXX) $(CXXFLAGS) $(STUBS) -I$(FW) -I. test_estimator_aligned.cpp -o $@

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

# --- Тесты протокола v2 (не требуют uPID) -----------------------------------
# Проверка контракта «мост ↔ прошивка»: команда моста разбирается парсером
# прошивки, кадр прошивки — парсером моста.
test_protocol_v2: $(TESTS)/test_protocol_v2.cpp $(FW)/command_parser.cpp $(FW)/command_parser.h $(FW)/command_policy.h
	$(CXX) $(CXXFLAGS) -I$(TESTS)/stubs -I$(FW) $(TESTS)/test_protocol_v2.cpp $(FW)/command_parser.cpp -o $@

test_bridge_v2: $(TESTS)/test_bridge_v2.cpp $(BRIDGE)/serial_bridge_cpp/frame_parser.hpp
	$(CXX) $(CXXFLAGS) -I$(BRIDGE) $(TESTS)/test_bridge_v2.cpp -o $@

test_end_to_end_v2: $(TESTS)/test_end_to_end_v2.cpp $(FW)/command_parser.cpp $(FW)/command_parser.h $(FW)/command_policy.h $(BRIDGE)/serial_bridge_cpp/frame_parser.hpp $(BRIDGE)/serial_bridge_cpp/protocol.hpp
	$(CXX) $(CXXFLAGS) -I$(TESTS)/stubs -I$(FW) -I$(BRIDGE) $(TESTS)/test_end_to_end_v2.cpp $(FW)/command_parser.cpp -o $@

# Границы длины кадра: приёмный буфер SerialReceiver против контракта
# команд из PROTOCOL.md (риск R-11). uPID не нужна.
test_frame_limits: $(TESTS)/test_frame_limits.cpp $(FW)/serial_receiver.cpp $(FW)/serial_receiver.h $(FW)/command_parser.cpp $(FW)/command_parser.h $(FW)/command_policy.h
	$(CXX) $(CXXFLAGS) -I$(TESTS)/stubs -I$(FW) $(TESTS)/test_frame_limits.cpp $(FW)/serial_receiver.cpp $(FW)/command_parser.cpp -o $@

# Только тесты протокола: библиотека uPID не нужна.
test_protocol: test_protocol_v2 test_bridge_v2 test_end_to_end_v2 test_frame_limits
	./test_protocol_v2
	./test_bridge_v2
	./test_end_to_end_v2
	./test_frame_limits

test: all
	./test_glue
	./test_estimator
	./test_estimator_aligned
	./test_encoders_equiv
	./sim_old
	./sim_new
	./test_protocol_v2
	./test_bridge_v2
	./test_end_to_end_v2
	./test_frame_limits

clean:
	rm -f test_glue test_estimator_aligned sim_ladrc sim_sync sim_floor sim_real sim_new sim_old sim_sweep test_estimator test_encoders_equiv test_protocol_v2 test_bridge_v2 test_end_to_end_v2 test_frame_limits trace_new.csv
	# Объектные файлы — обычные артефакты сборки, они пересобираются
	# автоматически (uPID для encoders.cpp не требуется) и в git не хранятся.
	rm -f enc_new.o enc_old.o
