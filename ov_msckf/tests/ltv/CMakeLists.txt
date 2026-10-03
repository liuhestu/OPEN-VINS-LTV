# Phase 0 core is deliberately absent from ov_msckf_lib.
add_library(ltv_phase0_core STATIC ../../src/ltv/ltv_observer.cpp)
ament_target_dependencies(ltv_phase0_core ov_core)
target_include_directories(ltv_phase0_core PUBLIC ../../src)

add_executable(test_ltv_core_parity ../../../docs/ltv_handover/parity_fixture/harness.cpp)
target_link_libraries(test_ltv_core_parity ltv_phase0_core)
add_executable(test_ltv_alias test_ltv_alias.cpp)
add_executable(test_ltv_alias_fixed test_ltv_alias_fixed.cpp)
target_link_libraries(test_ltv_alias_fixed ltv_phase0_core)
install(TARGETS test_ltv_core_parity test_ltv_alias test_ltv_alias_fixed DESTINATION lib/${PROJECT_NAME})

add_executable(test_ltv_core_lifecycle test_ltv_core_lifecycle.cpp)
target_link_libraries(test_ltv_core_lifecycle ltv_phase0_core)
install(TARGETS test_ltv_core_lifecycle DESTINATION lib/${PROJECT_NAME})
