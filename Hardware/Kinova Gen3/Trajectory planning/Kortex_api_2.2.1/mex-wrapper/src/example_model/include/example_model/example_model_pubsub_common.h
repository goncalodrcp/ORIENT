// Copyright 2019 The MathWorks, Inc.
// Generated 13-Apr-2021 16:36:26
#ifndef _EXAMPLE_MODEL_PUBSUB_COMMON_
#define _EXAMPLE_MODEL_PUBSUB_COMMON_
#include "example_model_types.h"
#ifndef SET_QOS_VALUES
#define SET_QOS_VALUES(qosStruct, hist, dep, dur, rel)  \
    {                                                   \
        qosStruct.history = hist;                       \
        qosStruct.depth = dep;                          \
        qosStruct.durability = dur;                     \
        qosStruct.reliability = rel;                    \
    }
#endif
namespace ros2 {
    namespace matlab {
    }
}
#endif // _EXAMPLE_MODEL_PUBSUB_COMMON_
