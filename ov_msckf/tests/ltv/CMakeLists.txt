# compact_v2 contracts; historical Euler/parity probes are in prechange/source.tar.gz.
foreach(test_name test_ltv_compact_math test_ltv_compact_native test_ltv_adapter_hardened test_ltv_gv_jacobian test_ltv_fej test_ltv_options test_ltv_feature_manager test_ltv_active_lifecycle test_ltv_active_consistency test_ltv_readiness test_bounded_history test_identity_guard test_ltv_history_competition)
    add_executable(${test_name} ${test_name}.cpp)
    ament_target_dependencies(${test_name} ${ament_libraries})
    target_link_libraries(${test_name} ov_msckf_lib ${thirdparty_libraries})
    target_compile_options(${test_name} PRIVATE -UNDEBUG)
    install(TARGETS ${test_name} DESTINATION lib/${PROJECT_NAME})
endforeach()
