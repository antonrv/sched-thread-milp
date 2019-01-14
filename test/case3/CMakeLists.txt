set(THIS_CMD "--n=2 --h=2 --c=2 --delay=${CMAKE_CURRENT_SOURCE_DIR}/delay.dat --capacity=${CMAKE_CURRENT_SOURCE_DIR}/capacity.dat --preced=${CMAKE_CURRENT_SOURCE_DIR}/prec.dat --idlepower=1 --maxpower=4 --power=${CMAKE_CURRENT_SOURCE_DIR}/power.dat")
add_test(NAME power_small_makespan COMMAND ${CORE_BINARY} ${THIS_CMD} WORKING_DIRECTORY ${CMAKE_TEST_OUTPUT_DIRECTORY})
