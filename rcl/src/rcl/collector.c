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

#include "./collector.h"

#define HISTORY_LENGTH 100
#define WARMUP 10

rcl_collector_t
rcl_get_zero_initialized_collector()
{
    static rcl_collector_t null_collector = {0};
    return null_collector;
}

rcl_ret_t
rcl_collector_init(
    rcl_collector_t * collector, rcl_allocator_t *allocator)
{
    rcl_steady_clock_init(&collector->clock, allocator);
    collector->times = allocator->allocate(
        sizeof(double)*(HISTORY_LENGTH+1),
        allocator->state);
    collector->sizes = allocator->allocate(
        sizeof(double)*(HISTORY_LENGTH+1),
        allocator->state);
    RCUTILS_LOG_DEBUG_NAMED(
        ROS_PACKAGE_NAME "_collector", "Collector initialized");
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
    rcl_time_point_value_t time;
    rcl_clock_get_now(&collector->clock, &time);
    RCUTILS_LOG_DEBUG_NAMED(
        ROS_PACKAGE_NAME "_collector", "On message with size %zu time %f", size, time*1e-9);

    // append time log
    if ((collector->tail+1)%(HISTORY_LENGTH+1) == collector->head)
        collector->head = (collector->head+1)%(HISTORY_LENGTH+1);
    collector->times[collector->tail] = time*1e-9;
    collector->sizes[collector->tail] = size;
    collector->tail = (collector->tail+1)%(HISTORY_LENGTH+1);
    ++collector->count;

    if (collector->count < WARMUP)
        return RCL_RET_OK;

    // recompute time model if the prediction is inaccurate
    double time_pred =
        collector->traffic_model.a * (collector->count-1)
        + collector->traffic_model.b;
    RCUTILS_LOG_DEBUG_NAMED(
        ROS_PACKAGE_NAME "_collector", "Prediction %f*%d+%f=%f",
            collector->traffic_model.a, collector->count-1, collector->traffic_model.b,
            time_pred);
    if (true) {
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
        double sigma = sqrtf(tmp_sum/(n-2));

        // update model
        collector->traffic_model.a = a;
        collector->traffic_model.b = b;
        collector->traffic_model.sigma_t = sigma;

        RCUTILS_LOG_DEBUG_NAMED(
            ROS_PACKAGE_NAME "_collector", "New time model a=%f b=%f sigma=%f", a, b, sigma);
    }

    // recompute size model if the probability is rare

    // estimate g

    return RCL_RET_OK;
}

#ifdef __cplusplus
}
#endif
