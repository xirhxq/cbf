# CMake generated Testfile for 
# Source directory: /tmp/cbf2026-route1-dynamic
# Build directory: /tmp/cbf2026-route1-dynamic/build-diagnostic
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(testSwarmFailureExit "/opt/homebrew/Cellar/cmake/3.28.3/bin/cmake" "-E" "env" "CBF_SWARM_BINARY=/tmp/cbf2026-route1-dynamic/build-diagnostic/Swarm" "/opt/homebrew/Frameworks/Python.framework/Versions/3.13/bin/python3.13" "-m" "unittest" "tests.test_swarm_failure_exit" "-v")
set_tests_properties(testSwarmFailureExit PROPERTIES  WORKING_DIRECTORY "/tmp/cbf2026-route1-dynamic" _BACKTRACE_TRIPLES "/tmp/cbf2026-route1-dynamic/CMakeLists.txt;113;add_test;/tmp/cbf2026-route1-dynamic/CMakeLists.txt;0;")
