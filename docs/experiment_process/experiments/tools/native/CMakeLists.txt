# Current ASL driver uses the same mode/weight profile as the native subscriber.
add_executable(run_ltv_gv_production run_ltv_gv.cpp)
ament_target_dependencies(run_ltv_gv_production ${ament_libraries})
target_link_libraries(run_ltv_gv_production ov_msckf_lib ${thirdparty_libraries})
install(TARGETS run_ltv_gv_production DESTINATION lib/${PROJECT_NAME})
