cmake_minimum_required(VERSION 3.10)

project(KukaMirrorBaseGenerator VERSION 1.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC OFF)
set(CMAKE_AUTORCC OFF)

find_package(Qt5 5.12 REQUIRED COMPONENTS Widgets)

add_executable(KukaMirrorBaseGenerator WIN32
    src/main.cpp
    src/mainwindow.cpp
    src/mainwindow.h
    src/pathpreview.cpp
    src/pathpreview.h
    src/method1tutorial.cpp
    src/method1tutorial.h
    src/mirrorcore.cpp
    src/mirrorcore.h
)

target_link_libraries(KukaMirrorBaseGenerator PRIVATE Qt5::Widgets)

if(MSVC)
    target_compile_options(KukaMirrorBaseGenerator PRIVATE /W4 /permissive- /utf-8)
elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(KukaMirrorBaseGenerator PRIVATE
        -Wall -Wextra -Wpedantic
        -finput-charset=UTF-8
        -fexec-charset=UTF-8
    )
else()
    target_compile_options(KukaMirrorBaseGenerator PRIVATE -Wall -Wextra -Wpedantic)
endif()

target_compile_definitions(KukaMirrorBaseGenerator PRIVATE UNICODE _UNICODE)

include(GNUInstallDirs)
install(TARGETS KukaMirrorBaseGenerator
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)