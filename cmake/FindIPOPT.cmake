find_package(PkgConfig)

pkg_check_modules(PC_IPOPT REQUIRED ipopt>=3.12.4)

set(IPOPT_DEFINITIONS ${PC_IPOPT_CFLAGS_OTHER})
set(IPOPT_INCLUDE_DIRS ${PC_IPOPT_INCLUDE_DIRS})
set(IPOPT_LIBRARIES ${PC_IPOPT_LIBRARIES})
set(IPOPT_VERSION ${PC_IPOPT_VERSION})
include(FindPackageHandleStandardArgs)
# if all listed variables are TRUE
find_package_handle_standard_args(IPOPT DEFAULT_MSG
        IPOPT_LIBRARIES IPOPT_INCLUDE_DIRS)
mark_as_advanced(IPOPT_INCLUDE_DIRS IPOPT_LIBRARIES)

if (${IPOPT_FOUND})
    message(STATUS "Found IPOPT version: " ${IPOPT_VERSION} " installed in: " ${PC_IPOPT_PREFIX})
    message(STATUS "IPOPT INCLUDE: " ${PC_IPOPT_INCLUDE_DIRS})
    message(STATUS "IPOPT LIB: " ${PC_IPOPT_LIBRARIES})
else ()
    message(SEND_ERROR "Could not find IPOPT")
endif ()
