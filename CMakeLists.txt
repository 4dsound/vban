project(vban)

set(sources
        src/vban/vbanstreamencoder.cpp
)

set(headers
        src/vban/dirtyflag.h
        src/vban/vban.h
        src/vban/vbanstreamencoder.h
)

add_library(${PROJECT_NAME} ${sources} ${headers})
target_include_directories(${PROJECT_NAME} PUBLIC src)
