cmake_minimum_required(VERSION 3.20)
project(showcase-castle LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(APPLE)
  # Enable high optimization by default for a faster render
  if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
  endif()
endif()

# Main rendering executable
add_executable(showcase
  src/main.cpp
  src/cg_math.h
  src/image.h
  src/image.cpp
  src/camera.h
  src/camera.cpp
  src/ray.h
  src/obj_loader.h
  src/obj_loader.cpp
  src/bvh.h
  src/bvh.cpp
  src/scene.h
  src/scene.cpp
  src/stb_image.h
)

target_include_directories(showcase PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

if(MSVC)
  target_compile_options(showcase PRIVATE /W4)
else()
  target_compile_options(showcase PRIVATE -Wall -Wextra -Wshadow -Wconversion -Wno-sign-conversion)
endif()

if(APPLE)
  find_library(COREFOUNDATION_FRAMEWORK CoreFoundation)
  find_library(COREGRAPHICS_FRAMEWORK CoreGraphics)
  find_library(IMAGEIO_FRAMEWORK ImageIO)
  target_link_libraries(showcase PRIVATE
    ${COREFOUNDATION_FRAMEWORK}
    ${COREGRAPHICS_FRAMEWORK}
    ${IMAGEIO_FRAMEWORK}
  )
endif()

# Interactive preview tool with GLFW
find_package(glfw3 QUIET)
if(glfw3_FOUND)
  add_executable(preview
    preview.cpp
    src/cg_math.h
    src/image.h
    src/image.cpp
    src/camera.h
    src/camera.cpp
    src/ray.h
    src/obj_loader.h
    src/obj_loader.cpp
    src/bvh.h
    src/bvh.cpp
    src/scene.h
    src/scene.cpp
    src/stb_image.h
  )
  
  target_include_directories(preview PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(preview PRIVATE glfw)
  
  if(APPLE)
    find_library(OPENGL_FRAMEWORK OpenGL)
    target_link_libraries(preview PRIVATE
      ${OPENGL_FRAMEWORK}
      ${COREFOUNDATION_FRAMEWORK}
      ${COREGRAPHICS_FRAMEWORK}
      ${IMAGEIO_FRAMEWORK}
    )
  elseif(UNIX)
    find_package(OpenGL REQUIRED)
    target_link_libraries(preview PRIVATE OpenGL::GL)
  endif()
  
  if(MSVC)
    target_compile_options(preview PRIVATE /W4)
  else()
    target_compile_options(preview PRIVATE -Wall -Wextra -Wshadow -Wconversion -Wno-sign-conversion)
  endif()
  
  message(STATUS "Building interactive preview tool (preview)")
else()
  message(STATUS "GLFW not found - skipping preview tool. Install with: brew install glfw")
endif()


