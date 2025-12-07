########################################################################################################################
# Definitions for this target come first
# TODO: KVS definitions need to be updated since no internal flash??

add_compile_definitions(STM32MP257)
add_compile_definitions(STM32MP2_CPU1)
add_compile_definitions(MICROKVS_WRITE_BLOCK_SIZE=32)
add_compile_definitions(KVS_NAMELEN=32)

# CEP multicore framework: we are one of two secondaries
add_compile_definitions(NUM_SECONDARY_CORES=2)
add_compile_definitions(NUM_IPC_CHANNELS=16)

# Tell stm32-cpp that we have multiple cores which changes the initialization model to not have a main function anymore
add_compile_definitions(MULTICORE)

# We're targeting aarch64 (if not specified defaults to arm-m)
set(ARCHITECTURE "aarch64")

# Tag locally generated log messages with our core ID
add_compile_definitions(LOGGER_CPU_TAG="a35-%d")

# Platform flags
set(ARCHFLAGS "-mcpu=cortex-a35 -fno-exceptions -fno-rtti -static -nostartfiles -fno-threadsafe-statics -no-pie -fno-use-cxa-atexit")

########################################################################################################################
# Common build flags at the end since they pull in ARCHFLAGS

include("${CMAKE_CURRENT_LIST_DIR}/common.cmake")
