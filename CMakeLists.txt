cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

# Project name
project(lr11xx-updater-tool)

zephyr_compile_options(-fdiagnostics-color=always)

# add_subdirectory(application/src)

# Define source files
set(C_SOURCES
    application/src/main.c
    application/src/lr11xx_firmware_update.c
    lr1121_modem_driver/src/lr1121_modem_modem.c

)

include_directories(
    application/inc
    lr1110_modem_driver/src
    lr1121_modem_driver/src
)

target_sources(app PRIVATE ${C_SOURCES})
