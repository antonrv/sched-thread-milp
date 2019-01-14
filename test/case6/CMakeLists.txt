set(THIS_CMD "--n=20 --h=4 --c=8 --delay=${CMAKE_CURRENT_SOURCE_DIR}/delay.dat --capacity=${CMAKE_CURRENT_SOURCE_DIR}/capacity.dat --preced=${CMAKE_CURRENT_SOURCE_DIR}/prec.dat --idlepower=5 --maxpower=100 --power=${CMAKE_CURRENT_SOURCE_DIR}/power.dat")
add_test(NAME big_energy COMMAND ${CORE_BINARY} ${THIS_CMD} WORKING_DIRECTORY ${CMAKE_TEST_OUTPUT_DIRECTORY})
