cmake_minimum_required(VERSION 3.13)
project(micron_source)

file(GLOB SRC "src/*.cpp")

add_library(micron_source SHARED ${SRC})
target_link_libraries(micron_source PRIVATE sdrpp_core ftd2xx fmt)
set_target_properties(micron_source PROPERTIES PREFIX "")

target_include_directories(micron_source PRIVATE "src/")
target_include_directories(micron_source PRIVATE "src/" "/usr/include/sdrpp_core/" "/usr/include/sdrpp_core/imgui")

if (MSVC)
    target_compile_options(micron_source PRIVATE /O2 /Ob2 /std:c++17 /EHsc)
elseif (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(micron_source PRIVATE -O3 -std=c++17 -Wno-unused-command-line-argument -undefined dynamic_lookup)
else ()
    target_compile_options(micron_source PRIVATE -O3 -std=c++17)
endif ()


# Install directives
install(TARGETS micron_source DESTINATION /usr/lib/sdrpp/plugins)
