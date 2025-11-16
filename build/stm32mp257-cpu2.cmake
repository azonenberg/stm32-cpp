########################################################################################################################
# Definitions for this target come first
# TODO: KVS definitions need to be updated since no internal flash??

add_compile_definitions(STM32MP257)
add_compile_definitions(STM32MP257_CPU2)
add_compile_definitions(MICROKVS_WRITE_BLOCK_SIZE=32)
add_compile_definitions(KVS_NAMELEN=32)

# Platform flags
set(ARCHFLAGS "-mcpu=cortex-m33 -march=armv8-m.main+dsp+fp -mfloat-abi=hard")
set(ARCHFLAGS "${ARCHFLAGS} -fno-exceptions -fno-rtti --specs=nano.specs -fno-threadsafe-statics")

########################################################################################################################
# Common build flags at the end since they pull in ARCHFLAGS

include("${CMAKE_CURRENT_LIST_DIR}/common.cmake")
