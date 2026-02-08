set(ENGINE_SOURCES
    src/main.cpp
    src/satellite.cpp
    src/constellation.cpp
    src/thermal_model.cpp
    src/orbital_mechanics.cpp
    src/mission_manager.cpp
    src/telemetry.cpp
)

add_executable(orbital_pursuit ${ENGINE_SOURCES})

target_include_directories(orbital_pursuit PRIVATE include)

# Platform-specific linking
if(UNIX)
    target_link_libraries(orbital_pursuit pthread)
endif()

# Compiler warnings
if(MSVC)
    target_compile_options(orbital_pursuit PRIVATE /W4)
else()
    target_compile_options(orbital_pursuit PRIVATE -Wall -Wextra -Wpedantic)
endif()
