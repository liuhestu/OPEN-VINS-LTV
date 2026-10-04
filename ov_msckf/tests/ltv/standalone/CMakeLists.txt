cmake_minimum_required(VERSION 3.16)
project(ltv_passive_boundary_checks LANGUAGES CXX)
# Build the exact production sources without ROS; no alternate observer implementation.
set(LTV_SOURCE_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../.." CACHE PATH "OpenVINS source tree (also accepts the frozen baseline checkout)")
get_filename_component(LTV_SOURCE_ROOT "${LTV_SOURCE_ROOT}" ABSOLUTE)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
enable_testing()
set(ltv_dir "${LTV_SOURCE_ROOT}/ov_msckf/src/ltv")
if(EXISTS "${ltv_dir}/observer/ltv_observer.cpp")
    include("${ltv_dir}/sources.cmake")
    set(passive_sources ${LTV_PRODUCTION_SOURCES})
    list(FILTER passive_sources EXCLUDE REGEX "/(UpdaterLTV|ValueDiagnostics|LtvOpenVinsInput)\\.cpp$")
    list(TRANSFORM passive_sources PREPEND "${LTV_SOURCE_ROOT}/ov_msckf/")
    add_library(ltv_passive STATIC
        ${LTV_SOURCE_ROOT}/ov_core/src/utils/print.cpp ${passive_sources})
else()
add_library(ltv_passive STATIC
    ${LTV_SOURCE_ROOT}/ov_core/src/utils/print.cpp
    ${ltv_dir}/ltv_observer.cpp
    ${ltv_dir}/LtvFeatureHistory.cpp
    ${ltv_dir}/LtvFeatureQuality.cpp
    ${ltv_dir}/LtvSeedEstimator.cpp
    ${ltv_dir}/LtvSeedUncertainty.cpp
    ${ltv_dir}/LtvLandmarkManager.cpp
    ${ltv_dir}/LtvActiveConsistency.cpp
    ${ltv_dir}/LtvFeaturePipeline.cpp
    ${ltv_dir}/LtvReadiness.cpp
    ${ltv_dir}/LtvAdapter.cpp
    ${ltv_dir}/LtvAdapterHardened.cpp
    ${ltv_dir}/LtvPassiveCache.cpp)
endif()
set_target_properties(ltv_passive PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(ltv_passive PUBLIC
    "${LTV_SOURCE_ROOT}/ov_msckf/src" "${LTV_SOURCE_ROOT}/ov_core/src" ${OpenCV_INCLUDE_DIRS})
target_link_libraries(ltv_passive PUBLIC Eigen3::Eigen Boost::filesystem Boost::system ${OpenCV_LIBS})
target_compile_options(ltv_passive PRIVATE -Wall -Wextra)
foreach(name
    test_ltv_active_consistency test_ltv_active_lifecycle test_ltv_feature_pipeline
    test_ltv_manager_bounded test_ltv_adapter_hardened test_ltv_controlled
    test_ltv_readiness test_ltv_history_competition test_bounded_history
    test_identity_guard test_ltv_feature_manager test_ltv_core_lifecycle
    test_ltv_alias_fixed test_ltv_pose_convention)
    add_executable(${name} "${LTV_SOURCE_ROOT}/ov_msckf/tests/ltv/${name}.cpp")
    target_link_libraries(${name} PRIVATE ltv_passive)
    target_compile_options(${name} PRIVATE -UNDEBUG)
    add_test(NAME ${name} COMMAND ${name})
endforeach()
add_executable(replay_ltv_feature_cache "${LTV_SOURCE_ROOT}/ov_msckf/tests/ltv/tools/replay_ltv_feature_cache.cpp")
target_link_libraries(replay_ltv_feature_cache PRIVATE ltv_passive)
add_executable(compare_cache_numerical "${LTV_SOURCE_ROOT}/scripts/ltv_passive_hardening/tests/compare_cache_numerical.cpp")
target_link_libraries(compare_cache_numerical PRIVATE ltv_passive)
add_library(ltv_synthetic_wrapper SHARED "${LTV_SOURCE_ROOT}/scripts/ltv_active_consistency_r4/synthetic_wrapper.cpp")
target_link_libraries(ltv_synthetic_wrapper PRIVATE ltv_passive)

# Optional native main-estimator checks against an independently built full library.
set(LTV_NATIVE_LIBRARY "" CACHE FILEPATH "Full ov_msckf_lib shared library to validate")
if(LTV_NATIVE_LIBRARY)
    add_library(ltv_native SHARED IMPORTED)
    set_target_properties(ltv_native PROPERTIES IMPORTED_LOCATION "${LTV_NATIVE_LIBRARY}")
    foreach(name test_ltv_timing test_ltv_native_default_off)
        # The regression harness is shared between baseline and cleanup builds.
        if(name STREQUAL "test_ltv_native_default_off")
            set(test_source "${CMAKE_CURRENT_LIST_DIR}/../${name}.cpp")
        else()
            set(test_source "${LTV_SOURCE_ROOT}/ov_msckf/tests/ltv/${name}.cpp")
        endif()
        add_executable(${name} "${test_source}")
        target_include_directories(${name} PRIVATE "${LTV_SOURCE_ROOT}/ov_msckf/src"
            "${LTV_SOURCE_ROOT}/ov_core/src" "${LTV_SOURCE_ROOT}/ov_init/src" ${OpenCV_INCLUDE_DIRS})
        target_compile_definitions(${name} PRIVATE ROS_AVAILABLE=0)
        target_compile_options(${name} PRIVATE -UNDEBUG)
        target_link_libraries(${name} PRIVATE ltv_native Eigen3::Eigen Boost::filesystem Boost::system ${OpenCV_LIBS})
    endforeach()
    add_library(ltv_native_synthetic_wrapper SHARED
        "${LTV_SOURCE_ROOT}/scripts/ltv_active_consistency_r4/synthetic_wrapper.cpp")
    target_include_directories(ltv_native_synthetic_wrapper PRIVATE "${LTV_SOURCE_ROOT}/ov_msckf/src"
        "${LTV_SOURCE_ROOT}/ov_core/src" ${OpenCV_INCLUDE_DIRS})
    target_compile_definitions(ltv_native_synthetic_wrapper PRIVATE ROS_AVAILABLE=0)
    target_link_libraries(ltv_native_synthetic_wrapper PRIVATE ltv_native Eigen3::Eigen
        Boost::filesystem Boost::system ${OpenCV_LIBS})
    add_test(NAME test_ltv_timing COMMAND test_ltv_timing)
endif()
