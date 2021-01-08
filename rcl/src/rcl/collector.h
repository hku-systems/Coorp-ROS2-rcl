// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#ifndef RCL__COLLECTOR_PROXY_H_
#define RCL__COLLECTOR_PROXY_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "rcl/allocator.h"
#include "rcl/types.h"
#include "rcl/visibility_control.h"
#include "rcl/time.h"
#include "rcl/publisher.h"

typedef struct
{
    // traffic model parameter
    double a, b, sigma_t;
    double s;
    double sigma_s;
    bool initialized;
} traffic_model_t;

typedef struct
{
    // the name of the topic being collected
    const char *topic_name;

    // associated publisher for reporting the model
    rcl_publisher_t publisher;

    //
    rcl_clock_t clock;

    // type support corresponding to the message of this publisher
    const rosidl_message_type_support_t * ts;

    // time history
    double *times;
    // size history
    double *sizes;
    size_t head, tail;
    unsigned int count;  // the total number of samples

    traffic_model_t traffic_model;

    uint64_t id;
} rcl_collector_t;

RCL_LOCAL
rcl_collector_t
rcl_get_zero_initialized_collector();

RCL_LOCAL
rcl_ret_t
rcl_collector_init(
    rcl_collector_t * collector,
    const rcl_node_t * node,
    const rosidl_message_type_support_t *ts,
    const char *topic_name
);

RCL_LOCAL
rcl_ret_t
rcl_collector_fini(
    rcl_collector_t * collector,
    rcl_node_t * node
);

RCL_LOCAL
rcl_ret_t
rcl_collector_on_message(
    rcl_collector_t * collector,
    size_t size
);

#ifdef __cplusplus
}
#endif

#endif  // RCL__COLLECTOR_PROXY_H_
