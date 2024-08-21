########################################################################################################################
# Generic build configuration that's common to all users of stm32-cpp

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)

set(CMAKE_CXX_STANDARD 20)

########################################################################################################################
# Warning configurations

# Warning config
set(WARNINGS "-Wall -Wextra -Wuninitialized -Wshadow -Wunsafe-loop-optimizations ")
set(WARNINGS "${WARNINGS} -Wpedantic -Wwrite-strings -Wmissing-declarations -Wvla")
# -Wcast-align

########################################################################################################################
# Other common flags

# No-malloc flags
set(NOMALLOCFLAGS "-Wl,--wrap=malloc -Wl,--wrap=calloc -Wl,--wrap=realloc -Wl,--wrap=alloca")
set(NOMALLOCFLAGS "${NOMALLOCFLAGS} -Wl,--wrap=sbrk_aligned -Wl,--wrap=free")

# Final compiler flag setup
set(CMAKE_ASM_FLAGS "${ARCHFLAGS}")
set(CMAKE_CXX_FLAGS "${WARNINGS} ${ARCHFLAGS} ${NOMALLOCFLAGS} -ffunction-sections -Wl,--build-id")
