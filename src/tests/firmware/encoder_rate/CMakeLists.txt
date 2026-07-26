# Standalone encoder-refresh-rate bench firmware (encoder characterization,
# motion-planner issue §7.3). A completely separate CODAL project from the
# main firmware: its own codal.json points "application" at ./app (one
# main.cpp), and the `src` symlink beside this file re-exposes the repo's
# src/ so the shared CODAL machinery (src/utils/cmake/*) and the ALREADY-
# INSTALLED library checkouts (src/libraries/*) are reused without a
# re-clone. Adapted from the root CMakeLists.txt with the sample-copy
# conveniences removed and the toolchain path made absolute (this project
# is configured from its own build/ directory).
#
# Build:
#   cd src/tests/firmware/encoder_rate && mkdir -p build && cd build && cmake .. && make -j
# Flash:
#   pyocd flash -t nrf52833 src/tests/firmware/encoder_rate/build/MICROBIT.hex
cmake_minimum_required(VERSION 3.6)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(${CMAKE_CURRENT_LIST_DIR}/src/utils/cmake/JSONParser.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/src/utils/cmake/util.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/src/utils/cmake/colours.cmake)

if (NOT "${BUILD_TOOL}" STRGREATER "")
    set(BUILD_TOOL "CODAL")
endif()

set(CMAKE_C_COMPILER_WORKS 1)
set(CMAKE_CXX_COMPILER_WORKS 1)

file(READ "${CMAKE_CURRENT_LIST_DIR}/codal.json" codal_json)
sbeParseJson(codal codal_json)

set(CODAL_APP_OUTPUT_DIR ".")
set(CODAL_APP_SOURCE_DIR "app")

if("${codal.application}" STRGREATER "")
    set(CODAL_APP_SOURCE_DIR "${codal.application}")
endif()

if(NOT "${codal.target.name}" STRGREATER "")
    message(FATAL_ERROR "${BoldRed}INVALID TARGET.${ColourReset}")
endif()

set(CODAL_DEPS "")
set(LIB_DEST "src/libraries")

INSTALL_DEPENDENCY(${LIB_DEST} ${codal.target.name} ${codal.target.url} ${codal.target.branch} ${codal.target.type})
list(APPEND CODAL_DEPS ${codal.target.name})

if("${codal.target.dev}" STRGREATER "")
    file(READ "${CMAKE_CURRENT_LIST_DIR}/${LIB_DEST}/${codal.target.name}/target.json" device_json)
else()
    file(READ "${CMAKE_CURRENT_LIST_DIR}/${LIB_DEST}/${codal.target.name}/target-locked.json" device_json)
endif()

message("${BoldBlue}Targeting ${codal.target.name} (encoder_rate bench)${ColourReset}")

sbeParseJson(device device_json)

SET(CODAL_TARGET_NAME ${device.target.name})
SET(CODAL_OUTPUT_NAME ${device.device})
SET(CODAL_TARGET_PROCESSOR ${device.processor})
SET(CODAL_TARGET_CPU_ARCHITECTURE ${device.architecture})

SET(TOOLCHAIN ${device.toolchain})
SET(TOOLCHAIN_FOLDER "${CMAKE_CURRENT_LIST_DIR}/src/utils/cmake/toolchains/${device.toolchain}")

set(CMAKE_TOOLCHAIN_FILE "${TOOLCHAIN_FOLDER}/toolchain.cmake" CACHE PATH "toolchain file")

project(codal)
enable_language(ASM)

include(${TOOLCHAIN_FOLDER}/compiler-flags.cmake)
set(PLATFORM_INCLUDES_PATH "${TOOLCHAIN_FOLDER}")

file(MAKE_DIRECTORY "${PROJECT_SOURCE_DIR}/build")

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY  "${PROJECT_SOURCE_DIR}/build")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/build")

SET(CODAL_DEFINITIONS "")

EXTRACT_JSON_ARRAY(codal "codal\.config\." CODAL_FIELDS CODAL_VALUES)
EXTRACT_JSON_ARRAY(device "device\.config\." DEVICE_FIELDS DEVICE_VALUES)
UNIQUE_JSON_KEYS(CODAL_FIELDS CODAL_VALUES DEVICE_FIELDS DEVICE_VALUES FINAL_FIELDS FINAL_VALUES)
FORM_DEFINITIONS(FINAL_FIELDS FINAL_VALUES CODAL_DEFINITIONS)

foreach(var ${device})
    if(NOT "${var}" MATCHES "device\.cmake_definitions\.")
        continue()
    endif()
    string(REGEX MATCH "[^device\.cmake_definitions\.]([A-Z,a-z,0-9,_,]+)" CODAL_CMAKE_DEFINITION "${var}")
    set(${CODAL_CMAKE_DEFINITION} ${${var}})
endforeach()

if("${device.definitions}" STRGREATER "")
    add_definitions("${device.definitions}")
endif()

if("${device.cpu_opts}" STRGREATER "")
    set(_CPU_COMPILATION_OPTIONS "${device.cpu_opts}")
    set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} ${device.cpu_opts}")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${device.cpu_opts}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${device.cpu_opts}")
    set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} ${device.cpu_opts}")
endif()

set(_C_FAMILY_FLAGS_INIT "-fno-exceptions -fno-unwind-tables -ffunction-sections -fdata-sections -Wall -Wextra -Wno-unused-parameter")

if("${device.asm_flags}" STRGREATER "")
    set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} ${device.asm_flags}")
endif()
if("${device.c_flags}" STRGREATER "")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${_C_FAMILY_FLAGS_INIT} ${device.c_flags}")
    set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} ${device.c_flags}")
endif()
if("${device.cpp_flags}" STRGREATER "")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${_C_FAMILY_FLAGS_INIT} ${device.cpp_flags}")
    set(CMAKE_CXX_LINK_FLAGS "${device.cpp_flags}")
endif()

# Same C++20 override as the root build (last -std wins over the target's
# pinned c++11).
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++20")

if("${device.linker_flags}" STRGREATER "")
    set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} ${device.linker_flags}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${device.linker_flags}")
endif()

if("${CODAL_DEFINITIONS}" STRGREATER "")
    set(EXTRA_INCLUDES_NEW_PATH "${PROJECT_SOURCE_DIR}/build/codal_extra_definitions_new.h")
    set(EXTRA_INCLUDES_PATH "${PROJECT_SOURCE_DIR}/build/codal_extra_definitions.h")
    file(WRITE "${EXTRA_INCLUDES_NEW_PATH}" ${CODAL_DEFINITIONS})
    configure_file(${EXTRA_INCLUDES_NEW_PATH} ${EXTRA_INCLUDES_PATH} COPYONLY)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -include \"${EXTRA_INCLUDES_PATH}\"")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -include \"${EXTRA_INCLUDES_PATH}\"")
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -I\"${PLATFORM_INCLUDES_PATH}\"")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I\"${PLATFORM_INCLUDES_PATH}\"")

set(CODAL_BUILD_SYSTEM TRUE)
set(CODAL_UTILS_LOCATION "${PROJECT_SOURCE_DIR}/src/utils/cmake/util.cmake")

set(LIB_FOLDERS "")

include_directories(${PROJECT_SOURCE_DIR}/${LIB_DEST})

if("${device.libraries}" STRGREATER "")
    message("Installing dependencies...")
    set(DEVICE_LIBS ${device.libraries})
    foreach(i ${DEVICE_LIBS})
        SET(BRANCH "NONE")
        SET(URL "${device.libraries_${i}.url}")
        if("${device.libraries_${i}.branch}" STRGREATER "")
            SET(BRANCH "${device.libraries_${i}.branch}")
        endif()
        if("${codal.target.branches.${URL}}" STRGREATER "")
            SET(BRANCH "${codal.target.branches.${URL}}")
        endif()
        INSTALL_DEPENDENCY(${LIB_DEST} ${device.libraries_${i}.name} ${URL} ${BRANCH} ${device.libraries_${i}.type})
        list(APPEND CODAL_DEPS "${device.libraries_${i}.name}")
    endforeach()
    foreach(dep ${CODAL_DEPS})
        message("${BoldGreen}Using library: ${dep}${ColourReset}")
        add_subdirectory("${PROJECT_SOURCE_DIR}/${LIB_DEST}/${dep}" "${CMAKE_BINARY_DIR}/libs/${dep}")
    endforeach()
endif()

RECURSIVE_FIND_DIR(INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/${CODAL_APP_SOURCE_DIR}" "*.h")
list(APPEND INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/${CODAL_APP_SOURCE_DIR}")

RECURSIVE_FIND_FILE(SOURCE_FILES "${PROJECT_SOURCE_DIR}/${CODAL_APP_SOURCE_DIR}" "*.cpp")

if("${SOURCE_FILES}" STREQUAL "")
    message(FATAL_ERROR "${BoldRed}No user application to build at: ${PROJECT_SOURCE_DIR}/${CODAL_APP_SOURCE_DIR}${ColourReset}")
endif()

string(COMPARE EQUAL "${BUILD_TOOL}" "CODAL" CODAL_BUILD)
if (${CODAL_BUILD})
    include("${PROJECT_SOURCE_DIR}/src/utils/cmake/buildtools/codal.cmake")
endif()

set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")
set(CMAKE_EXE_EXPORTS_C_FLAG "")
set(CMAKE_EXE_EXPORTS_CXX_FLAG "")
