########################################################################################################################
# Definitions for this target come first

add_compile_definitions(STM32L031)
add_compile_definitions(MICROKVS_WRITE_BLOCK_SIZE=8)
add_compile_definitions(KVS_NAMELEN=32)

# Platform flags
set(ARCHFLAGS "-mcpu=cortex-m0plus")
set(ARCHFLAGS "${ARCHFLAGS} -fno-exceptions -fno-rtti --specs=nano.specs -fno-threadsafe-statics")

########################################################################################################################
# Common build flags at the end since they pull in ARCHFLAGS

include("${CMAKE_CURRENT_LIST_DIR}/common.cmake")
