cmake_minimum_required(VERSION 3.20)

include($ENV{IDF_PATH}/tools/cmake/idf.cmake)
project(libespidf C)

# Process extra components
foreach(component_dir IN ITEMS $ENV{EXTRA_COMPONENT_DIRS})
    idf_build_component(${component_dir})
endforeach()

if(DEFINED ENV{COMPONENTS})
    idf_build_process($ENV{IDF_TARGET} SDKCONFIG $ENV{SDKCONFIG} SDKCONFIG_DEFAULTS $ENV{SDKCONFIG_DEFAULTS} COMPONENTS $ENV{COMPONENTS})
else()
    idf_build_process($ENV{IDF_TARGET} SDKCONFIG $ENV{SDKCONFIG} SDKCONFIG_DEFAULTS $ENV{SDKCONFIG_DEFAULTS})
endif()

idf_build_get_property(aliases BUILD_COMPONENT_ALIASES)

add_executable(libespidf.elf main.c)
target_link_libraries(libespidf.elf PUBLIC "${aliases}")
idf_build_executable(libespidf.elf)

# used to query the components in the build script
idf_build_get_property(comps BUILD_COMPONENTS)
set(BUILD_COMPONENTS ${comps} CACHE STRING "all esp-idf components" FORCE)
