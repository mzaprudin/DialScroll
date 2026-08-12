cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(blescroll_scroll_settings)

target_include_directories(app PRIVATE ../../include)
target_sources(app PRIVATE
  ../../src/scroll_resolution.c
  src/main.c
)
