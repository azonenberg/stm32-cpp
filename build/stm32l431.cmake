########################################################################################################################
# Definitions for this target come first

add_compile_definitions(STM32L431)
add_compile_definitions(MICROKVS_WRITE_BLOCK_SIZE=8)
add_compile_definitions(KVS_NAMELEN=32)

# Platform flags
set(ARCHFLAGS "-mcpu=cortex-m4 -march=armv7e-m+fp -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(ARCHFLAGS "${ARCHFLAGS} -fno-exceptions -fno-rtti --specs=nano.specs -fno-threadsafe-statics")

########################################################################################################################
# Common build flags at the end since they pull in ARCHFLAGS

include("${CMAKE_CURRENT_LIST_DIR}/common.cmake")
