# Copyright 2020 National Institute of Advanced Industrial
# Science and Technology, Japan
# Copyright 2019 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Floris Erich <floris.erich@aist.go.jp>

string(REPLACE install/${PROJECT_NAME}
  install COLCON_INSTALL_DIR ${CMAKE_INSTALL_PREFIX})

set(QUALISYSSDK_SEARCH_PATHS
  /usr/include
  ${COLCON_INSTALL_DIR}/qualisys_cpp_sdk
  ${QUALISYSSDK_PATH}
)

find_path(QUALISYSSDK_INCLUDE_DIR RTProtocol.h
  HINTS $ENV{QUALISYSSDK_PATH}
  PATH_SUFFIXES include/qualisys_cpp_sdk
  PATHS ${QUALISYSSDK_SEARCH_PATHS}
)

find_library(QUALISYSSDK_LIBRARY_TEMP
  NAMES qualisys_cpp_sdk
  HINTS
  $ENV{QUALISYSSDKDIR}
  PATH_SUFFIXES lib64 lib
  PATHS ${QUALISYSSDK_SEARCH_PATHS}
)

string(REPLACE -lpthread "" QUALISYSSDK_LIBRARY ${QUALISYSSDK_LIBRARY_TEMP})

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(
  QUALISYSSDK REQUIRED_VARS QUALISYSSDK_LIBRARY QUALISYSSDK_INCLUDE_DIR)
