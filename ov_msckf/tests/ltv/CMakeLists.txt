# Phase 0 core is deliberately absent from ov_msckf_lib.
add_library(ltv_phase0_core STATIC ../../src/ltv/ltv_observer.cpp)
ament_target_dependencies(ltv_phase0_core ov_core)
target_include_directories(ltv_phase0_core PUBLIC ../../src)

add_executable(test_ltv_core_parity ../../../docs/ltv_handover/parity_fixture/harness.cpp)
target_link_libraries(test_ltv_core_parity ltv_phase0_core)
add_executable(test_ltv_alias test_ltv_alias.cpp)
install(TARGETS test_ltv_core_parity test_ltv_alias DESTINATION lib/${PROJECT_NAME})

# Native T1/T2/T3/T5 targets are deferred at the alias stop gate.
