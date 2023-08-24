cmake_minimum_required (VERSION 3.20)

add_library(K1804BC1 SHARED K1804BC1.cpp)
target_include_directories(K1804BC1 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

include(FetchContent)
FetchContent_Declare(
  model_utils
  GIT_REPOSITORY https://github.com/MT-PROTEUS-PROJECT/model_utils
  GIT_TAG        e9febc0ee740e430d902356103021b935fd1e380
)
FetchContent_MakeAvailable(model_utils)

target_link_libraries(K1804BC1 PRIVATE model_utils)

set_property(TARGET K1804BC1 PROPERTY CXX_STANDARD 23)
