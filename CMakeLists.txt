cmake_minimum_required(VERSION 3.27)
project(homework_graph)

set(CMAKE_CXX_STANDARD 17)

# Configurar la ruta de SFML (ajusta esta ruta según donde instalaste SFML)
set(SFML_DIR "C:/SFML-2.6.1/lib/cmake/SFML")

add_executable(${PROJECT_NAME} main.cpp
        graph.h
        gui.h
        node.h
        edge.h
        window_manager.h
        path_finding_manager.h
)

find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)
if(SFML_FOUND)
    target_link_libraries(${PROJECT_NAME} PRIVATE sfml-graphics sfml-window sfml-system)
else()
    message("SFML not found")
endif()
