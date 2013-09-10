FUNCTION(ADD_HAT)
    SET(options, "")
    SET(oneValueArgs "NAME")
    SET(multiValueArgs, "MODULES")
    CMAKE_PARSE_ARGUMENTS(HAT. "", ${oneValueArgs}, ${multiValueArgs} ${ARGN})

    ADD_CUSTOM_COMMAND(TARGET ${HAT_NAME} PRE_BUILD
        COMMAND ${NUBOTS_SCRIPTS_DIR}/generate.sh "${CMAKE_BINARY_DIR}/hats/${HAT_NAME}.cpp" ${HAT_MODULES})

    ADD_EXECUTABLE(${HAT_NAME} "${CMAKE_BINARY_DIR}/hats/${HAT_NAME}.cpp")
    TARGET_LINK_LIBRARIES(${HAT_NAME} ${HAT_MODULES} ${NUBOTS_SHARED_LIBRARIES})
ENDFUNCTION(ADD_HAT)
