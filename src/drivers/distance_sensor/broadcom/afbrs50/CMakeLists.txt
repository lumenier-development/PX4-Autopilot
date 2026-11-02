############################################################################
#
#   Copyright (c) 2021 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

# The vendored Broadcom SDK (Inc/, Lib/) is the public release. A newer SDK that
# is not public yet, such as the one the AFBR-S50EX85D needs, is supplied at build
# time instead of being added to the tree: point AFBR_SDK_DIR at the SDK's
# AFBR-S50 directory (the one holding Include/ and Lib/), either in the
# environment when the build directory is first configured or as a CMake cache
# variable. The driver's optional features are selected by ARGUS_API_VERSION.
set(AFBR_SDK_DIR "$ENV{AFBR_SDK_DIR}" CACHE PATH "External Broadcom AFBR-S50 SDK directory (optional)")

if(AFBR_SDK_DIR)
	set(AFBR_INCLUDE_DIR ${AFBR_SDK_DIR}/Include)
	set(AFBR_LIBRARY ${AFBR_SDK_DIR}/Lib/libafbrs50_m4_fpu_os_nolog.a)
	message(STATUS "afbrs50: using external Broadcom SDK at ${AFBR_SDK_DIR}")
else()
	set(AFBR_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Inc)
	set(AFBR_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/Lib/libafbrs50_m4_fpu.a)
endif()

add_library(afbrs50_m4_fpu STATIC IMPORTED GLOBAL)
set_property(TARGET afbrs50_m4_fpu PROPERTY IMPORTED_LOCATION ${AFBR_LIBRARY})

px4_add_module(
	MODULE drivers__distance_sensor__afbrs50
	MAIN afbrs50
	COMPILE_FLAGS
	STACK_MAIN
		4096
	INCLUDES
		API/Inc
		${AFBR_INCLUDE_DIR}
	SRCS
		AFBRS50.cpp
		AFBRS50.hpp
		API/Src/irq.c
		API/Src/s2pi.cpp
		API/Src/timer.c
		argus_hal_test.c
	MODULE_CONFIG
		parameters.yaml
	DEPENDS
		px4_work_queue
		drivers_rangefinder
		afbrs50_m4_fpu
	)

target_link_libraries(afbrs50_m4_fpu INTERFACE drivers__distance_sensor__afbrs50)
