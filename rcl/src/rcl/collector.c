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

#include <math.h>

#include "rcutils/logging_macros.h"
#include "rcutils/time.h"
#include "rcl_interfaces/msg/traffic_model.h"

#include "./collector.h"

#define HISTORY_LENGTH 100
#define UNSTABLE 2
#define WARMUP 10

rcl_collector_t
rcl_get_zero_initialized_collector()
{
    static rcl_collector_t null_collector = {0};
    return null_collector;
}

rcl_ret_t
rcl_collector_init(
    rcl_collector_t *collector, const rcl_node_t *node)
{
    rcutils_allocator_t allocator = rcutils_get_default_allocator();

    if (RCL_RET_OK !=
            rcl_steady_clock_init(&collector->clock, &allocator)) {
        RCUTILS_SET_ERROR_MSG("Failed to init clock for collector");
        return RCL_RET_ERROR;
    }

    collector->times = allocator.allocate(
        sizeof(double)*(HISTORY_LENGTH+1),
        allocator.state);
    collector->sizes = allocator.allocate(
        sizeof(double)*(HISTORY_LENGTH+1),
        allocator.state);

    collector->publisher = rcl_get_zero_initialized_publisher();
    rcl_publisher_options_t options = rcl_publisher_get_default_options();  // TODO: we may need non-default options
    const rosidl_message_type_support_t * ts = ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, TrafficModel);
    if(RCL_RET_OK != rcl_publisher_init_internal(
            &collector->publisher, node, ts, "ros2_traffic_model", &options, false)) {
        RCUTILS_SET_ERROR_MSG("Failed to create publisher on topic 'ros2_traffic_model'");
        return RCL_RET_ERROR;
    }

    RCUTILS_LOG_DEBUG_NAMED(
        ROS_PACKAGE_NAME "_collector", "Collector initialized");

    return RCL_RET_OK;
}

rcl_ret_t
rcl_collector_fini(
    rcl_collector_t * collector, rcl_node_t *node)
{
    rcutils_allocator_t allocator = rcutils_get_default_allocator();

    if (RCL_RET_OK
            != rcl_publisher_fini(&collector->publisher, node)) {
        RCUTILS_SET_ERROR_MSG("Failed to finalize publisher");
        return RCL_RET_ERROR;
    }

    if (RCL_RET_OK
            != rcl_publisher_fini(&collector->publisher, node)) {
        RCUTILS_SET_ERROR_MSG("Failed to finalize publisher");
        return RCL_RET_ERROR;
    }

    if (RCL_RET_OK
            != rcl_clock_fini(&collector->clock)) {
        RCUTILS_SET_ERROR_MSG("Failed to finalize clock");
        return RCL_RET_ERROR;
    }

    allocator.deallocate(collector->times, allocator.state);
    allocator.deallocate(collector->sizes, allocator.state);

    return RCL_RET_OK;
}

rcl_ret_t
rcl_collector_on_message(
    rcl_collector_t * collector,
    size_t param_size)
{
    ++collector->count;
    if (collector->count < UNSTABLE)
        return RCL_RET_OK;

    rcl_time_point_value_t param_time;

    if (RCL_RET_OK != rcl_clock_get_now(&collector->clock, &param_time)) {
        RCUTILS_SET_ERROR_MSG("Failed to get current time");
        return RCL_RET_ERROR;
    }

    double time = ((double)param_time)*1e-9;
    double size = param_size;

    RCUTILS_LOG_DEBUG_NAMED(
        ROS_PACKAGE_NAME "_collector", "On message with size %zu time %f", param_size, time);

    // append time log
    if ((collector->tail+1)%(HISTORY_LENGTH+1) == collector->head)
        collector->head = (collector->head+1)%(HISTORY_LENGTH+1);
    collector->times[collector->tail] = time;
    collector->sizes[collector->tail] = size;
    collector->tail = (collector->tail+1)%(HISTORY_LENGTH+1);

    if ((collector->tail+(HISTORY_LENGTH+1)-collector->head)%(HISTORY_LENGTH+1) < WARMUP)
        return RCL_RET_OK;

    // recompute time model if the prediction is inaccurate
    double time_pred = NAN;
    if (collector->traffic_model.initialized) {
        time_pred = collector->traffic_model.a*round((time - collector->traffic_model.b) / collector->traffic_model.a)
            + collector->traffic_model.b;
        RCUTILS_LOG_DEBUG_NAMED(
            ROS_PACKAGE_NAME "_collector", "Predicted time %f", time_pred);
    }
    if (!collector->traffic_model.initialized || true) {
        double n = (collector->tail+(HISTORY_LENGTH+1)-collector->head)%(HISTORY_LENGTH+1);
        double sum_x = (n-1)*n/2,  // sum of {0, 1, 2, ...}
              sum_x2 = (n-1)*n*(2*n-1)/6;  // sum of {0^2, 1^2, 2^2, ...}
        double sum_y = 0, sum_xy = 0;
        for (size_t cur = collector->head, k=0; cur != collector->tail; cur=(cur+1)%(HISTORY_LENGTH+1), ++k) {
            sum_y += collector->times[cur];
            sum_xy += k*collector->times[cur];
        }

        double a = (n*sum_xy - sum_x * sum_y) / (n*sum_x2 - sum_x*sum_x);
        double b = (sum_y - a*sum_x) / n;

        // sigma
        double tmp_sum = 0;
        for (size_t cur = collector->head, k=0; cur != collector->tail; cur=(cur+1)%(HISTORY_LENGTH+1), ++k) {
            double tmp = (a*k+b-collector->times[cur]);
            tmp_sum += tmp*tmp;
        }
        double sigma = sqrt(tmp_sum/(n-2));

        // update model
        collector->traffic_model.a = a;
        collector->traffic_model.b = b;
        collector->traffic_model.sigma_t = sigma;

        RCUTILS_LOG_DEBUG_NAMED(
            ROS_PACKAGE_NAME "_collector", "New time model a=%f b=%f sigma=%f", a, b, sigma);
        rcl_interfaces__msg__TrafficModel *msg = rcl_interfaces__msg__TrafficModel__create();
        msg->a = collector->traffic_model.a;
        msg->b = collector->traffic_model.b;
        msg->sigma_t = collector->traffic_model.sigma_t;
        msg->s = collector->traffic_model.s;
        msg->sigma_s = collector->traffic_model.sigma_s;
        rcl_ret_t ret_pub = rcl_publish(&collector->publisher, msg, NULL);
        if (RCL_RET_OK == ret_pub) {
            RCUTILS_LOG_DEBUG_NAMED(
                ROS_PACKAGE_NAME "_collector", "Successfully published new model");
        } else {
            RCUTILS_LOG_ERROR_NAMED(
                ROS_PACKAGE_NAME "_collector", "Failed publishing new model, '%s'", rcutils_get_error_string().str);
            rcutils_reset_error();
        }
        rcl_interfaces__msg__TrafficModel__destroy(msg);
    }

    // recompute size model if the probability is rare

    collector->traffic_model.initialized = true;

    return RCL_RET_OK;
}

#ifdef __cplusplus
}
#endif
