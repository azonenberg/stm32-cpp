########################################################################################################################
# Definitions for this target come first
# TODO: KVS definitions need to be updated since no internal flash??

add_compile_definitions(STM32MP257)
add_compile_definitions(STM32MP2_CPU2)
add_compile_definitions(MICROKVS_WRITE_BLOCK_SIZE=32)
add_compile_definitions(KVS_NAMELEN=32)

# CEP multicore framework: we are the primary core and have two secondaries
add_compile_definitions(PRIMARY_CORE)
add_compile_definitions(NUM_SECONDARY_CORES=2)
add_compile_definitions(NUM_IPC_CHANNELS=16)

# Tag locally generated log messages with our core ID
add_compile_definitions(LOGGER_CPU_TAG="m33  ")

# Platform flags
set(ARCHFLAGS "-mcpu=cortex-m33 -march=armv8-m.main+dsp+fp -mfloat-abi=hard")
set(ARCHFLAGS "${ARCHFLAGS} -fno-exceptions -fno-rtti --specs=nano.specs -fno-threadsafe-statics")

########################################################################################################################
# Common build flags at the end since they pull in ARCHFLAGS

include("${CMAKE_CURRENT_LIST_DIR}/common.cmake")
