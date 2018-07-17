cmake_minimum_required(VERSION 2.8)

set(VERSION_MAJOR "1")
set(VERSION_MINOR "0")
set(VERSION_PATCH "2")

project(matrix CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "MinSizeRel" CACHE STRING "Build type" FORCE)
	message(STATUS "set build type to ${CMAKE_BUILD_TYPE}")
endif()

set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug;Release;RelWithDebInfo;MinSizeRel;Coverage")

option(SUPPORT_STDIOSTREAM "If enabled provides support for << operator (as used with std::cout)" OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(${CMAKE_SOURCE_DIR})

if(SUPPORT_STDIOSTREAM)
	add_definitions(-DSUPPORT_STDIOSTREAM)
endif()

set(CMAKE_CXX_FLAGS_COVERAGE
	"--coverage -fprofile-arcs -ftest-coverage -fno-default-inline -fno-inline -fno-inline-small-functions -fno-elide-constructors"
	CACHE STRING "Flags used by the C++ compiler during coverage builds" FORCE)
set(CMAKE_EXE_LINKER_FLAGS_COVERAGE
	"--coverage -ftest-coverage -lgcov"
	CACHE STRING "Flags used for linking binaries during coverage builds" FORCE)
mark_as_advanced(CMAKE_CXX_FLAGS_COVERAGE CMAKE_C_FLAGS_COVERAGE CMAKE_EXE_LINKER_FLAGS_COVERAGE)

add_compile_options(
	-pedantic
	-Wall
	-Warray-bounds
	-Wcast-align
	-Wcast-qual
	-Wconversion
	-Wctor-dtor-privacy
	-Wdisabled-optimization
	-Werror
	-Wextra
	-Wfloat-equal
	-Wformat-security
	-Wformat=2
	-Winit-self
	-Wlogical-op
	-Wmissing-declarations
	-Wmissing-include-dirs
	-Wno-sign-compare
	-Wno-unused
	-Wno-unused-parameter
	-Wnoexcept
	-Wold-style-cast
	-Woverloaded-virtual
	-Wpointer-arith
	-Wredundant-decls
	-Wreorder
	-Wshadow
	-Wsign-conversion
	-Wsign-promo
	-Wstrict-null-sentinel
	-Wswitch-default
	-Wundef
	-Wuninitialized
	-Wunused-variable
	)

# clang tolerate unknown gcc options
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
	add_compile_options(
		-Wno-error=unused-command-line-argument-hard-error-in-future
		-Wno-unknown-warning-option
		)
else()
	add_compile_options(
		-Wstrict-overflow=5
		)
endif()
