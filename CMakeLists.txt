cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(blescroll)

target_sources(app PRIVATE src/main.c)

# NCS 3.2.4 generates a UF2 whose blocks are relocated to the application
# partition, but whose reset vector is still linked at address zero.  Convert
# the correctly linked Intel HEX ourselves, preserving its physical addresses.
if(CONFIG_FLASH_BASE_ADDRESS STREQUAL "0x1000")
  set(UF2_OUTPUT_NAME blescroll_nosd)
else()
  set(UF2_OUTPUT_NAME blescroll)
endif()

add_custom_target(blescroll_uf2 ALL
  COMMAND ${Python3_EXECUTABLE} ${ZEPHYR_BASE}/scripts/build/uf2conv.py
    -c
    -f 0xada52840
    -o ${CMAKE_CURRENT_BINARY_DIR}/${UF2_OUTPUT_NAME}.uf2
    ${CMAKE_CURRENT_BINARY_DIR}/zephyr/zephyr.hex
  DEPENDS zephyr_final
  COMMENT "Generating correctly addressed ${UF2_OUTPUT_NAME}.uf2"
)
