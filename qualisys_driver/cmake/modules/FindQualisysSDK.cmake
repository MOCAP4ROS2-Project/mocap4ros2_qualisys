string(REPLACE install/${PROJECT_NAME} install COLCON_INSTALL_DIR ${CMAKE_INSTALL_PREFIX})

SET(QualisysSDK_SEARCH_PATHS
  /usr/include
  ${COLCON_INSTALL_DIR}/qualisys_cpp_sdk
	${QualisysSDK_PATH}
)

FIND_PATH(QualisysSDK_INCLUDE_DIR RTProtocol.h
  HINTS $ENV{QualisysSDK_PATH}
	PATH_SUFFIXES include/qualisys_cpp_sdk
	PATHS ${QualisysSDK_SEARCH_PATHS}
)

FIND_LIBRARY(QualisysSDK_LIBRARY_TEMP
	NAMES qualisys_cpp_sdk
	HINTS
	$ENV{QualisysSDKDIR}
	PATH_SUFFIXES lib64 lib
	PATHS ${QualisysSDK_SEARCH_PATHS}
)

string(REPLACE -lpthread "" QualisysSDK_LIBRARY ${QualisysSDK_LIBRARY_TEMP})

INCLUDE(FindPackageHandleStandardArgs)

FIND_PACKAGE_HANDLE_STANDARD_ARGS(QualisysSDK REQUIRED_VARS QualisysSDK_LIBRARY QualisysSDK_INCLUDE_DIR)
