cmake_minimum_required(VERSION 3.14)   # FetchContent_MakeAvailable
project(nkcemu C)

add_executable(nkcemu sim1.c sim2.c sim3.c sim4.c sim5.c sim6.c sim7.c simctl.c simfun.c simglb.c simint.c gdp64.c iosim.c disas.c cas.c vdip.c flo3.c timer.c ser2.c nkcemu.c)

# Locate SDL2.  Three tiers, most-preferred first:
#   1. pkg-config (Homebrew/Linux): links the dylib by its absolute install
#      name, so no rpath fixups are needed.
#   2. the SDL2 CMake package (e.g. the macOS SDL2.framework), adding its
#      location to the rpath.
#   3. nothing on the system -> fetch and build a pinned SDL2 from source, so a
#      fresh checkout or CI runner builds with no manual SDL2 install.
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(SDL2 QUIET sdl2)
endif()

if(SDL2_FOUND)
    target_include_directories(nkcemu PRIVATE ${SDL2_INCLUDE_DIRS})
    target_link_directories(nkcemu PRIVATE ${SDL2_LIBRARY_DIRS})
    target_link_libraries(nkcemu ${SDL2_LIBRARIES})
else()
    find_package(SDL2 QUIET)
    if(SDL2_FOUND)
        target_include_directories(nkcemu PRIVATE ${SDL2_INCLUDE_DIRS})
        target_link_libraries(nkcemu ${SDL2_LIBRARIES})
        set_target_properties(nkcemu PROPERTIES
            BUILD_RPATH   "/Library/Frameworks"
            INSTALL_RPATH "/Library/Frameworks")
    else()
        # No system SDL2 at all -- build a known-good one from source and link it
        # statically, so the nkcemu binary is self-contained (no rpath fixups).
        message(STATUS "SDL2 not found on the system; fetching and building it from source")
        include(FetchContent)
        set(SDL_SHARED OFF CACHE BOOL "" FORCE)
        set(SDL_STATIC ON  CACHE BOOL "" FORCE)
        set(SDL_TEST   OFF CACHE BOOL "" FORCE)
        FetchContent_Declare(SDL2
            GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
            GIT_TAG        release-2.32.10
            GIT_SHALLOW    TRUE)
        FetchContent_MakeAvailable(SDL2)
        target_link_libraries(nkcemu SDL2::SDL2-static)
    endif()
endif()

install(TARGETS nkcemu RUNTIME DESTINATION bin)
