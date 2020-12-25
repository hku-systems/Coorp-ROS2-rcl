// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifdef __cplusplus
extern "C"
{
#endif

#include "rcutils/logging_macros.h"

#include "./collector.h"

rcl_collector_t
rcl_get_zero_initialized_collector()
{
    static rcl_collector_t null_collector = {0};
    return null_collector;
}

rcl_ret_t
rcl_collector_init(
    rcl_collector_t * collector)
{
    return RCL_RET_OK;
}

rcl_ret_t
rcl_collector_fini(
    rcl_collector_t * collector)
{
    return RCL_RET_OK;
}

rcl_ret_t
rcl_collector_on_message(
    rcl_collector_t * collector,
    size_t size)
{
    RCUTILS_LOG_DEBUG_NAMED(
        ROS_PACKAGE_NAME, "On message with size %zu", size);
    return RCL_RET_OK;
}

#ifdef __cplusplus
}
#endif
