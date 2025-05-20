set(SOURCES
    # Put the intended source file here
    pub_benchmark.c # Comment out if other test is used
    transport/uart.c # Comment out if other transport is used
)

idf_component_register(SRCS ${SOURCES}
    INCLUDE_DIRS "."
)