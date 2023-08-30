cmake_minimum_required(VERSION 3.20)

set(IDF_TARGET $ENV{IDF_TARGET} CACHE STRING "IDF Build Target" FORCE)

set(EXTRA_COMPONENT_DIRS $ENV{EXTRA_COMPONENT_DIRS})
set(COMPONENTS $ENV{ESP_IDF_COMPONENTS})
set(SDKCONFIG $ENV{SDKCONFIG})
set(SDKCONFIG_DEFAULTS $ENV{SDKCONFIG_DEFAULTS})

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(dependencies_lock "${CMAKE_CURRENT_LIST_DIR}/dependencies.lock")
set(components_lock "$ENV{PROJECT_DIR}/components_${IDF_TARGET}.lock")

# The DEPENDENCIES_LOCK attribute does not exist for esp-idf version less than 5.1,
# so we'll keep the file in the cargo project dir ($components_lock) and the 
# file in the out dir ($dependencies_lock) up to date with each other.
if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "5.1")
    idf_build_set_property(DEPENDENCIES_LOCK ${components_lock})
else()
    if(EXISTS ${components_lock})
        file(COPY_FILE ${components_lock} ${dependencies_lock} ONLY_IF_DIFFERENT)
    else()
        file(REMOVE ${dependencies_lock})
    endif()
endif()

project(libespidf)

# Copy the components.lock to the cargo project dir
if(EXISTS ${dependencies_lock})
    file(COPY_FILE ${dependencies_lock} ${components_lock} ONLY_IF_DIFFERENT)
endif()

# used to query the components in the build script
idf_build_get_property(comps BUILD_COMPONENTS)
set(BUILD_COMPONENTS ${comps} CACHE STRING "all esp-idf components" FORCE)
