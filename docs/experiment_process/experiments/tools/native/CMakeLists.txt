# The final driver retains the frozen CLI; research cache/shadow drivers are archived.
add_executable(run_ltv_gv_production run_ltv_gv.cpp)
ament_target_dependencies(run_ltv_gv_production ${ament_libraries})
target_link_libraries(run_ltv_gv_production ov_msckf_lib ${thirdparty_libraries})
install(TARGETS run_ltv_gv_production DESTINATION lib/${PROJECT_NAME})

add_executable(test_ltv_experiment_modes test_experiment_modes.cpp)
target_compile_options(test_ltv_experiment_modes PRIVATE -UNDEBUG)
install(TARGETS test_ltv_experiment_modes DESTINATION lib/${PROJECT_NAME})
