cmake_minimum_required(VERSION 3.10)
project(nkcemu C)

add_executable(nkcemu sim1.c sim2.c sim3.c sim4.c sim5.c sim6.c sim7.c simctl.c simfun.c simglb.c simint.c gdp64.c iosim.c disas.c cas.c vdip.c nkcemu.c)

# Prefer pkg-config: on Homebrew/Linux it links the SDL2 dylib by its absolute
# install name, so no rpath fixups are needed.  Fall back to the SDL2 CMake
# package (e.g. the macOS SDL2.framework), adding its location to the rpath.
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(SDL2 QUIET sdl2)
endif()

if(SDL2_FOUND)
    target_include_directories(nkcemu PRIVATE ${SDL2_INCLUDE_DIRS})
    target_link_directories(nkcemu PRIVATE ${SDL2_LIBRARY_DIRS})
    target_link_libraries(nkcemu ${SDL2_LIBRARIES})
else()
    find_package(SDL2 REQUIRED)
    target_include_directories(nkcemu PRIVATE ${SDL2_INCLUDE_DIRS})
    target_link_libraries(nkcemu ${SDL2_LIBRARIES})
    set_target_properties(nkcemu PROPERTIES
        BUILD_RPATH   "/Library/Frameworks"
        INSTALL_RPATH "/Library/Frameworks")
endif()

install(TARGETS nkcemu RUNTIME DESTINATION bin)
