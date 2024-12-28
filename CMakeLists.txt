# CMake build script for stm32-cpp.
# Intended to be integrated into a larger project, not built standalone.

add_library(stm32-cpp STATIC
	src/cpu/cpu.S
	src/cpu/cache.S
	src/cpu/Faults.cpp
	src/cpu/NVIC.cpp

	src/helpers/I2CServer.cpp

	src/newlib-stubs/stubs.cpp

	src/peripheral/ADC.cpp
	src/peripheral/CRC.cpp
	src/peripheral/DTS.cpp
	src/peripheral/EXTI.cpp
	src/peripheral/Flash.cpp
	src/peripheral/GPIO.cpp
	src/peripheral/I2C.cpp
	src/peripheral/MDMA.cpp
	src/peripheral/OctoSPI.cpp
	src/peripheral/OctoSPIManager.cpp
	src/peripheral/Power.cpp
	src/peripheral/QuadSPI.cpp
	src/peripheral/RCC.cpp
	src/peripheral/RTC.cpp
	src/peripheral/SPI.cpp
	src/peripheral/Timer.cpp

	src/helpers/I2CServer.cpp
	src/helpers/SPIServer.cpp

	devices/src/${TARGET_MCU}.cpp
	)

target_include_directories(stm32-cpp
	PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/devices/inc
	PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src
	)
