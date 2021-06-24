/*
* KINOVA (R) KORTEX (TM)
*
* Copyright (c) 2019 Kinova inc. All rights reserved.
*
* This software may be modified and distributed
* under the terms of the BSD 3-Clause license.
*
* Refer to the LICENSE file for details.
*/

#ifndef _KORTEX_WRAPPER_DATA_
#define _KORTEX_WRAPPER_DATA_

#include <stdint.h>
#include <stdbool.h>

#define MAX_JOINTS 7
#define GRIPPER_MAX_MOTOR_COUNT 10
#define MAX_ERROR_INFO_DESCRIPTION_SIZE 512
#define MAX_PRE_COMPUTED_TRAJECTORY_ELEMENTS 30000
#define MAX_INSANITY_INFO_SIZE 512
#define MAX_ENUM_NAME_SIZE 1024

enum VisionEvent 
{
    UNSPECIFIED_VISION_EVENT = 0,   // Unspecified vision event
    SENSOR_SETTINGS_CHANGED = 1,    // Sensor setting changed event
    OPTION_VALUE_CHANGED = 2        // Option value changed event
};

enum FocusAction
{
    FOCUSACTION_UNSPECIFIED             = 0, // Unspecified focus action
    FOCUSACTION_START_CONTINUOUS_FOCUS  = 1, // Start continuous focus
    FOCUSACTION_PAUSE_CONTINUOUS_FOCUS  = 2, // Pause continuous focus 
    FOCUSACTION_FOCUS_NOW               = 3, // Focus now (single-shot)
    FOCUSACTION_DISABLE_FOCUS           = 4, // Disable focus
    FOCUSACTION_SET_FOCUS_POINT         = 5, // Set a focus point
    FOCUSACTION_SET_MANUAL_FOCUS        = 6 // Set the manual focus distance
};

enum KortexErrorCodes {
    SUB_ERROR_NONE = 0,                           // No sub error
    ERROR_METHOD_FAILED = 1,                      // Method returned a failure status (generic error)
    ERROR_UNIMPLEMENTED = 2,                      // Unimplemented method
    ERROR_INVALID_PARAM = 3,                      // Invalid parameter
    
    ERROR_UNSUPPORTED_SERVICE = 4,                // Service not recognized
    ERROR_UNSUPPORTED_METHOD = 5,                 // Method not recognized
    ERROR_TOO_LARGE_ENCODED_FRAME_BUFFER = 6,     // Encoded frame bigger than what transport permits
    ERROR_FRAME_ENCODING_ERR = 7,                 // Unable to encode frame
    ERROR_FRAME_DECODING_ERR = 8,                 // Unable to decode frame
    ERROR_INCOMPATIBLE_HEADER_VERSION = 9,        // Frame header version differs from what is expected and is considered incompatible
    ERROR_UNSUPPORTED_FRAME_TYPE = 10,            // Unrecognized frame type
    ERROR_UNREGISTERED_NOTIFICATION_RECEIVED = 11,// Server receiving unregistered notification
    ERROR_INVALID_SESSION = 12,                   // Session not recognized
    ERROR_PAYLOAD_DECODING_ERR = 13,              // Unable to decode payload
    ERROR_UNREGISTERED_FRAME_RECEIVED = 14,       // Client received a response for which it did not send an RPC call
    
    ERROR_PASSWORD_INVALID = 15,                  // Password does not match specified user
    ERROR_USER_NOT_FOUND = 16,                    // Unrecognized user
    
    ERROR_ENTITY_NOT_FOUND = 17,                  // Cannot find entity
    
    ERROR_ROBOT_MOVEMENT_IN_PROGRESS = 18,        // Robot refuses new control command because robot movement in progress
    ERROR_ROBOT_NOT_MOVING = 19,                  // Robot refuses stop command because robot is not moving
    
    ERROR_NO_MORE_STORAGE_SPACE = 20,             // Unable to execute because no more storage
    
    ERROR_ROBOT_NOT_READY = 21,                   // Robot initialization is not complete
    ERROR_ROBOT_IN_FAULT = 22,                    // Robot in fault
    ERROR_ROBOT_IN_MAINTENANCE = 23,              // Robot in maintenance
    ERROR_ROBOT_IN_UPDATE_MODE = 24,              // Robot in update
    ERROR_ROBOT_IN_EMERGENCY_STOP = 25,           // Robot in emergency stop state
    
    ERROR_SINGLE_LEVEL_SERVOING = 26,             // Robot is in single-level servoing mode
    ERROR_LOW_LEVEL_SERVOING = 27,                // Robot is in low-level servoing mode
    
    ERROR_MAPPING_GROUP_NON_ROOT = 28,            // Trying to add a non-root MapGroup to Mapping
    ERROR_MAPPING_INVALID_GROUP = 29,             // Trying to add an invalid or non-existent MapGroup to Mapping
    ERROR_MAPPING_INVALID_MAP = 30,               // Trying to add an invalid or non-existent Map to Mapping
    ERROR_MAP_GROUP_INVALID_MAP = 31,             // Trying to add an invalid or non-existent Map to MapGroup
    ERROR_MAP_GROUP_INVALID_PARENT = 32,          // Trying to add a MapGroup under an invalid parent
    ERROR_MAP_GROUP_INVALID_CHILD = 33,           // Trying to add an invalid or non-existent to MapGroup
    ERROR_MAP_GROUP_INVALID_MOVE = 34,            // Trying to change a MapGroup's parent: move not supported
    ERROR_MAP_IN_USE = 35,                        // Deleting a Map used in a Mapping or MapGroup
    
    ERROR_WIFI_CONNECT_ERROR = 36,                // Unable to connect to specified Wifi network
    ERROR_UNSUPPORTED_NETWORK_TYPE = 37,          // Unsupported network type
    ERROR_TOO_LARGE_ENCODED_PAYLOAD_BUFFER = 38,  // Encoded payload bigger than what transport permits
    
    ERROR_UPDATE_PERMISSION_DENIED = 39,          // Attempting update command on non-updatable entity
    ERROR_DELETE_PERMISSION_DENIED = 40,          // Attempting delete command on non-deletable entity
    ERROR_DATABASE_ERROR = 41,                    // Internal DB error
    
    ERROR_UNSUPPORTED_OPTION = 42,                // Option not supported
    ERROR_UNSUPPORTED_RESOLUTION = 43,            // Resolution not supported
    ERROR_UNSUPPORTED_FRAME_RATE = 44,            // Frame rate not supported
    ERROR_UNSUPPORTED_BIT_RATE = 45,              // Bit rate not supported
    ERROR_UNSUPPORTED_ACTION = 46,                // Action not supported (generic, when an action is not supported for a particular item)
    ERROR_UNSUPPORTED_FOCUS_ACTION = 47,          // Focus action not supported
    ERROR_VALUE_IS_ABOVE_MAXIMUM = 48,            // Specified value is above the supported maximum
    ERROR_VALUE_IS_BELOW_MINIMUM = 49,            // Specified value is below the supported minimum
    
    ERROR_DEVICE_DISCONNECTED = 50,               // Device is not connected
    ERROR_DEVICE_NOT_READY = 51,                  // Device is not ready
    
    ERROR_INVALID_DEVICE = 52,                    // Device id is invalid during bridging
    
    ERROR_SAFETY_THRESHOLD_REACHED = 53,          // Safety threshold is reached therefore safety is on
    
    ERROR_INVALID_USER_SESSION_ACCESS = 54,       // Service or function access not allowed: out of session or level access
    
    ERROR_CONTROL_MANUAL_STOP = 55,               // Manually stopped sequence or action
    ERROR_CONTROL_OUTSIDE_WORKSPACE = 56,         // Commanded Cartesian position is outside of robot workspace
    ERROR_CONTROL_ACTUATOR_COUNT_MISMATCH = 57,   // Number of constraint sent does not correspond to number of actuator (ex: joint speed)
    ERROR_CONTROL_INVALID_DURATION = 58,          // Duration constraint is too short. The robot would need out of limit speeds/accelerations to reach this duration.
    ERROR_CONTROL_INVALID_SPEED = 59,             // Speed constraint is negative
    ERROR_CONTROL_LARGE_SPEED = 60,               // Speed constraint is too high (exceed speed limit of leads to high acceleration)
    ERROR_CONTROL_INVALID_ACCELERATION = 61,      // Speed constraint is too high or duration constraint too short and leads to high acceleration
    ERROR_CONTROL_INVALID_TIME_STEP = 62,         // Refresh rate is smaller than the duration of the trajectory
    ERROR_CONTROL_LARGE_SIZE = 63,                // Duration of the trajectory is more than 100s. The length of the trajectory is limited to 100000 points to avoid saturating the base memory.
    ERROR_CONTROL_WRONG_MODE = 64,                // Control mode is not a trajectory mode
    ERROR_CONTROL_JOINT_POSITION_LIMIT = 65,      // Commanded configuration contains at least one actuator which is out of its physical limits
    ERROR_CONTROL_NO_FILE_IN_MEMORY = 66,         // Trajectory is not computed and try to be started
    ERROR_CONTROL_INDEX_OUT_OF_TRAJECTORY = 67,   // Attempting to read a point of the trajectory with an index higher than the number of point in trajectory point list.
    ERROR_CONTROL_ALREADY_RUNNING = 68,           // Trajectory is already running
    ERROR_CONTROL_WRONG_STARTING_POINT = 69,      // Robot is not on the first point of the trajectory when we try to start the trajectory. This can happen if there is a motion between the moment when trajectory is computed and when it is started.
    ERROR_CONTROL_CARTESIAN_CANNOT_START =  70,   // Cannot start
    ERROR_CONTROL_UNDEFINED_CONSTRAINT = 71,      // Kontrol library is not initialized
    ERROR_CONTROL_UNINITIALIZED = 72,             // Contraint sent is not defined
    ERROR_CONTROL_NO_ACTION = 73,                 // Action does not exist
    ERROR_CONTROL_UNDEFINED = 74,                 // Undefined error
    
    ERROR_WRONG_SERVOING_MODE = 75,               // Robot is in not in the right servoing mode
    
    ERROR_CONTROL_WRONG_STARTING_SPEED = 76,      // Robot is not at the right speed when starting a new trajectory.
    
    ERROR_USERNAME_LENGTH_EXCEEDED = 100,         // User profile username length exceeds maximum allowed length
    ERROR_FIRSTNAME_LENGTH_EXCEEDED = 101,        // User profile first name length exceeds maximum allowed length
    ERROR_LASTNAME_LENGTH_EXCEEDED = 102,         // User profile last name length exceeds maximum allowed length
    ERROR_PASSWORD_LENGTH_EXCEEDED = 103,         // User profile password length exceeds maximum allowed length
    ERROR_USERNAME_ALREADY_EXISTS = 104,          // User profile username already in use by another profile
    ERROR_USERNAME_EMPTY = 105,                   // User profile empty username not allowed
    ERROR_PASSWORD_NOT_CHANGED = 106,             // Change password both passwords are the same
    ERROR_MAXIMUM_USER_PROFILES_USED = 107,       // Maximum number of user profiles in use
    ERROR_ROUTER_UNVAILABLE = 108,                // The client router is currently unavailable. This can happen if an API method is called after the router has been deactivated via the method SetActivationStatus.
    
    ERROR_ADDRESS_NOT_IN_VALID_RANGE = 120,       // IP Address not valid against netmask
    ERROR_ADDRESS_NOT_CONFIGURABLE = 121,         // IP Address not configurable on specified interface
    
    ERROR_SESSION_NOT_IN_CONTROL = 130,           // Trying to perform command from a non-controlling session in single-level mode
    
    ERROR_METHOD_TIMEOUT = 131,                   // Timeout occured during method execution
    
    ERROR_UNSUPPORTED_ROBOT_CONFIGURATION = 132,  // Product Configuration setter method failed because changing this parameter is unsupported on your robot model
    ERROR_NVRAM_READ_FAIL = 133,                  // Failed to read in NVRAM.
    ERROR_NVRAM_WRITE_FAIL = 134,                 // Failed to write in NVRAM.
    
    ERROR_NETWORK_NO_ADDRESS_ASSIGNED = 135,      // The specified interface has no assigned IP 
};


enum Option 
{
    OPTION_UNSPECIFIED                     = 0, // Unspecifed Option 
    OPTION_BACKLIGHT_COMPENSATION          = 1, // Enable / disable color backlight compensation (unsupported)
    OPTION_BRIGHTNESS                      = 2, // Color image brightness (supported on color sensor only: -4.0 to 4.0, step 1.0)
    OPTION_CONTRAST                        = 3, // Color image contrast (supported on color sensor only: -4.0 to 4.0, step 1.0)
    OPTION_EXPOSURE                        = 4, // Controls exposure time of color camera. Setting any value will disable auto exposure (supported on depth sensor only: 20.0 to 166000.0, step 20.0)
    OPTION_GAIN                            = 5, // Color image gain (supported on depth sensor only: 16.0 to 248.0, step 1.0)
    OPTION_GAMMA                           = 6, // Color image gamma setting (unsupported)
    OPTION_HUE                             = 7, // Color image hue (unsupported)
    OPTION_SATURATION                      = 8, // Color image saturation setting (supported on color sensor only: -4.0 to 4.0, step 1.0)
    OPTION_SHARPNESS                       = 9, // Color image sharpness setting (unsupported)
    OPTION_WHITE_BALANCE                   = 10, // Controls white balance of color image. Setting any value will disable auto white balance (unsupported)
    OPTION_ENABLE_AUTO_EXPOSURE            = 11, // Enable / disable color image auto-exposure (supported on depth sensor only: 0.0 to 1.0, step 1.0)
    OPTION_ENABLE_AUTO_WHITE_BALANCE       = 12, // Enable / disable color image auto-white-balance (unsupported)
    OPTION_VISUAL_PRESET                   = 13, // Provide access to several recommend sets of option presets for the depth camera (supported on depth sensor only:  0.0 to 5.0, step 1.0)
    OPTION_LASER_POWER                     = 14, // Power of the projector, with 0 meaning projector off (unsupported)
    OPTION_ACCURACY                        = 15, // Sets the number of patterns projected per frame. The higher the accuracy value the more patterns projected (unsupported)
    OPTION_MOTION_RANGE                    = 16, // Motion vs. Range trade-off, with lower values allowing for better motion sensitivity and higher values allowing for better depth range (unsupported)
    OPTION_FILTER_OPTION                   = 17, // Sets the filter to apply to each depth frame. Each one of the filter is optimized per the application requirements (unsupported)
    OPTION_CONFIDENCE_THRESHOLD            = 18, // The confidence level threshold used by the Depth algorithm pipe to set whether a pixel will get a valid range or will be marked with invalid range (unsupported)
    OPTION_EMITTER_ENABLED                 = 19, // Laser Emitter enabled (unsupported)
    OPTION_FRAMES_QUEUE_SIZE               = 20, // Number of frames the user is allowed to keep per stream. Trying to hold-on to more frames will cause frame-drops (supported on depth sensor only: 0.0 to 32.0, step 1.0)
    OPTION_TOTAL_FRAME_DROPS               = 21, // Total number of detected frame drops from all streams (unsupported)
    OPTION_AUTO_EXPOSURE_MODE              = 22, // Auto-Exposure modes: Static, Anti-Flicker and Hybrid (unsupported)
    OPTION_POWER_LINE_FREQUENCY            = 23, // Power Line Frequency control for anti-flickering Off/50Hz/60Hz/Auto (unsupported)
    OPTION_ASIC_TEMPERATURE                = 24, // Current Asic Temperature (supported on depth sensor only: Read Only -40.0 to 125.0)
    OPTION_ERROR_POLLING_ENABLED           = 25, // Disable error handling (supported on depth sensor only: 0.0 to 1.0, step 1.0)
    OPTION_PROJECTOR_TEMPERATURE           = 26, // Current Projector Temperature (unsupported)
    OPTION_OUTPUT_TRIGGER_ENABLED          = 27, // Enable / disable trigger to be outputed from the camera to any external device on every depth frame (supported on depth sensor only: 0.0 to 1.0, step 1.0)
    OPTION_MOTION_MODULE_TEMPERATURE       = 28, // Current Motion-Module Temperature (unsupported)
    OPTION_DEPTH_UNITS                     = 29, // Number of meters represented by a single depth unit (supported on depth sensor only: 0.0001 to 0.0100, step 0.000001)
    OPTION_ENABLE_MOTION_CORRECTION        = 30, // Enable/Disable automatic correction of the motion data (unsupported)
    OPTION_AUTO_EXPOSURE_PRIORITY          = 31, // Allows sensor to dynamically ajust the frame rate depending on lighting conditions (unsupported)
    OPTION_COLOR_SCHEME                    = 32, // Color scheme for data visualization (unsupported)
    OPTION_HISTOGRAM_EQUALIZATION_ENABLED  = 33, // Perform histogram equalization post-processing on the depth data (unsupported)
    OPTION_MIN_DISTANCE                    = 34, // Minimal distance to the target (unsupported)
    OPTION_MAX_DISTANCE                    = 35, // Maximum distance to the target (unsupported)
    OPTION_TEXTURE_SOURCE                  = 36, // Texture mapping stream unique ID (unsupported)
    OPTION_FILTER_MAGNITUDE                = 37, // The 2D-filter effect. The specific interpretation is given within the context of the filter (unsupported)
    OPTION_FILTER_SMOOTH_ALPHA             = 38, // 2D-filter parameter controls the weight/radius for smoothing (unsupported)
    OPTION_FILTER_SMOOTH_DELTA             = 39, // 2D-filter range/validity threshold (unsupported)
    OPTION_HOLES_FILL                      = 40, // Enhance depth data post-processing with holes filling where appropriate (unsupported)
    OPTION_STEREO_BASELINE                 = 41, // The distance in mm between the first and the second imagers in stereo-based depth cameras (supported on depth sensor only: 55.241055 to 55.241055, step 0.0)
    OPTION_AUTO_EXPOSURE_CONVERGE_STEP     = 42 // Allows dynamically ajust the converge step value of the target exposure in Auto-Exposure algorithm (unsupported)
};

enum Sensor 
{
    SENSOR_UNSPECIFIED  = 0, // Unspecified Sensor
    SENSOR_COLOR        = 1, // Select the Vision module color sensor
    SENSOR_DEPTH        = 2 // Select the Vision module depth sensor
};

enum Resolution 
{
    RESOLUTION_UNSPECIFIED  = 0, // Unspecified resolution
    RESOLUTION_320x240      = 1, // 320 x 240 pixels (supported on color sensor only)
    RESOLUTION_424x240      = 2, // 424 x 240 pixels (supported on depth sensor only)
    RESOLUTION_480x270      = 3, // 480 x 270 pixels (supported on depth sensor only)
    RESOLUTION_640x480      = 4, // 640 x 480 pixels (supported on color sensor only)
    RESOLUTION_1280x720     = 5, // 1280 x 720 pixels (HD) (supported on color sensor only)
    RESOLUTION_1920x1080    = 6 // 1920 x 1080 pixels (full HD) (supported on color sensor only)
};

enum FrameRate 
{
    FRAMERATE_UNSPECIFIED  = 0, // Unspecified frame rate
    FRAMERATE_6_FPS        = 1, // 6 frames per second (supported on depth sensor only)
    FRAMERATE_15_FPS       = 2, // 15 frames per second
    FRAMERATE_30_FPS       = 3 // 30 frame per second
};

enum BitRate 
{
    BITRATE_UNSPECIFIED  = 0, // Unspecified bit rate (supported on depth sensor only)
    BITRATE_10_MBPS      = 1, // 10 Mbps maximum bit rate (supported on color sensor only)
    BITRATE_15_MBPS      = 2, // 15 Mbps maximum bit rate (supported on color sensor only)
    BITRATE_20_MBPS      = 3, // 20 Mbps maximum bit rate (supported on color sensor only)
    BITRATE_25_MBPS      = 4  // 25 Mbps maximum bit rate (supported on color sensor only)
};

enum CartesianConstraintType
{
    CARTESIAN_NO_CONSTRAINT = 0,
    CARTESIAN_CONSTRAINT_DURATION = 1,  // (we only support CARTESIAN_CONSTRAINT_SPEED for now)
    CARTESIAN_CONSTRAINT_SPEED = 2
};

enum JointConstraintType
{
    UNSPECIFIED_JOINT_CONSTRAINT = 0,   // Unspecified joint constraint
    JOINT_CONSTRAINT_DURATION = 1,      // Duration constraint (in second)
    JOINT_CONSTRAINT_SPEED = 2,         // Speed constraint (in meters per second)
};

enum TwistMode
{
    TWIST_TOOL_JOYSTICK = 1,
    TWIST_BASE_FRAME_JOYSTICK = 2,
    TWIST_CARTESIAN_JOYSTICK = 3
};

enum ToolMode
{
    UNSPECIFIED_GRIPPER_MODE = 0,   // Unspecified gripper mode
    GRIPPER_FORCE = 1,              // Force control (in Newton) (not implemented yet)
    GRIPPER_SPEED = 2,              // Speed control (in meters per second)
    GRIPPER_POSITION = 3,           // Position control (in meters)
};

enum TrajectoryContinuityMode
{
    TRAJECTORY_CONTINUITY_MODE_UNDEFINED     = 0,    // undefined
    TRAJECTORY_CONTINUITY_MODE_POSITION      = 1,    // Position continuity only
    TRAJECTORY_CONTINUITY_MODE_SPEED         = 2,    // Position and speed continuity
    TRAJECTORY_CONTINUITY_MODE_ACCELERATION  = 3     // Position, speed and acceleration continuity
};

enum CartesianReferenceFrame
{
    UNSPECIFIED = 0,  // Unspecified Cartesian reference frame
    MIXED = 1,        // Mixed reference frame where translation reference = base and  orientation reference = tool
    TOOL = 2,         // Tool reference frame where translation reference = tool and orientation reference = tool
    BASE = 3          // World reference frame where translation reference = base and orientation reference = tool (v2.0)
};

enum AdmittanceMode
{
    UNSPECIFIED_ADMITTANCE_MODE = 0,    // Unspecified admittance mode
    CARTESIAN = 1,                      // Cartesian admittance mode
    JOINT = 2,                          // Joint admittance mode
    NULL_SPACE = 3,                     // Null space admittance mode
    DISABLED = 4                        // No admittance
};

enum ServoingMode
{
    UNSPECIFIED_SERVOING_MODE = 0,  // Unspecified servoing mode
    SINGLE_LEVEL_SERVOING = 2,      // Single-level servoing
    LOW_LEVEL_SERVOING = 3,         // Low-level servoing
    BYPASS_SERVOING = 4             // Bypass mode
};

enum MovementStatus
{
    MOVEMENT_STATUS_IDLE = 0,
    MOVEMENT_STATUS_RUNNING = 1,
    MOVEMENT_STATUS_PAUSED = 2,
    MOVEMENT_STATUS_ABORTED = 3,
    MOVEMENT_STATUS_COMPLETED = 4,
    PRE_PROCESSING = 5
};

typedef struct
{
    uint32_t code;
    uint32_t sub_code;
    char     description[MAX_ERROR_INFO_DESCRIPTION_SIZE];
} ErrorInfo;

typedef struct
{
    uint32_t cmdHandle;
    uint32_t status;
    bool hasBeenCompleted;
    uint32_t reason;
    bool sanityStateStatus;
    char insanityInfo[MAX_INSANITY_INFO_SIZE];
} MovementStatusInfo;

typedef struct
{
    uint32_t flags[MAX_JOINTS];
    double position[MAX_JOINTS];
    double velocity[MAX_JOINTS];
    double torque_joint[MAX_JOINTS];
    double current_motor[MAX_JOINTS];
} ActuatorsCommand;

typedef struct
{
    uint32_t command_id;
    uint32_t flags;
    double position;
    double velocity;
    double force;
} InterconnectCommand;

typedef struct
{
    uint32_t arm_state;                       // Active state of the arm
    double arm_voltage;                       // Arm voltage (in Volts)
    double arm_current;                       // Arm current (in Amperes)
    double temperature_cpu;                   // CPU temperature (in degree Celcius)
    double temperature_ambient;               // Ambient temperature (in degree Celcius)
    double imu_acceleration[3];               // IMU Measured acceleration (X(1)-Y(2)-Z(3)-Axis) of the base (in meters per second squared)
    double imu_angular_velocity[3];           // IMU Measured angular velocity (X(1)-Y(2)-Z(3)-Axis) of the base (in degrees per second)
    double tool_pose[6];                      // Measured Cartesian position (X(1)-Y(2)-Z(3)-tX(4)-tY(5)-tZ(6)) of the End Effector (EE) (in meters and degrees)
    double tool_twist[6];                     // Measured Cartesian velocity (X(1)-Y(2)-Z(3)-tX(4)-tY(5)-tZ(6)) of the End Effector (EE) (in meters per second and degress per second)
    double tool_external_wrench_force[3];     // Calculated force in (X(1)-Y(2)-Z(3)) from external wrench (in Newton)
    double tool_external_wrench_torque[3];    // Calculated torque about (X(1)-Y(2)-Z(3)) from external wrench (in Newton * meters)
    uint32_t fault_bank_a;                    // The arm fault flags bank A (0 if no fault)
    uint32_t fault_bank_b;                    // The arm fault flags bank B (0 if no fault)
    uint32_t warning_bank_a;                  // The arm warning flags bank A (0 if no warning)
    uint32_t warning_bank_b;                  // The arm warning flags bank B (0 if no warning)
} BaseFeedback;

typedef struct
{
    uint32_t status_flags[MAX_JOINTS];    // Status flags
    uint32_t jitter_comm[MAX_JOINTS];     // Jitter from the communication (in microseconds)
    double position[MAX_JOINTS];          // Position of the actuator (in degrees)
    double velocity[MAX_JOINTS];          // Velocity of the actuator (in degrees per second)
    double torque[MAX_JOINTS];            // Torque of the actuator (in Newton * meters)
    double current_motor[MAX_JOINTS];     // Current of the motor (in Amperes)
    double voltage[MAX_JOINTS];           // Voltage of the main board (in Volts)
    double temperature_motor[MAX_JOINTS]; // Motor temperature (maximum of the three (3) phase temperatures in °C)
    double temperature_core[MAX_JOINTS];  // Microcontroller temperature (in degrees Celsius)
    uint32_t fault_bank_a[MAX_JOINTS];    // Fault bank A
    uint32_t fault_bank_b[MAX_JOINTS];    // Fault bank B
    uint32_t warning_bank_a[MAX_JOINTS];  // Warning bank A
    uint32_t warning_bank_b[MAX_JOINTS];  // Warning bank B
} ActuatorsFeedback;

typedef struct
{
    uint32_t motor_id;        // Motor ID (1, nb_motor)
    uint32_t status_flags;
    uint32_t jitter_comm;
    double position;          // Position of the gripper fingers in percentage (0-100%)
    double velocity;          // Velocity of the gripper fingers in percentage (0-100%)
    double force;
    double current_motor;     // Current comsumed by the gripper motor (mA)
    double voltage;           // Motor Voltage (V)
    double temperature_core;
    double temperature_motor; // Motor temperature. (degree celsius)
} MotorFeedback;

typedef struct
{
    uint32_t feedback_id;    // MessageId
    uint32_t status_flags;   // Status flags (see GripperConfig.RobotiqGripperStatusFlags)
    uint32_t fault_bank_a;   // Fault bank A (see GripperConfig.SafetyIdentifier)
    uint32_t fault_bank_b;   // Fault bank B (see GripperConfig.SafetyIdentifier)
    uint32_t warning_bank_a; // Warning bank A (see GripperConfig.SafetyIdentifier)
    uint32_t warning_bank_b; // Warning bank B (see GripperConfig.SafetyIdentifier)
    uint32_t motor_count;    // Motor count on the gripper.
    MotorFeedback motor[GRIPPER_MAX_MOTOR_COUNT];
} GripperFeedback;

typedef struct
{
    uint32_t feedback_id;          // MessageId
    uint32_t status_flags;         // Status flags
    uint32_t jitter_comm;          // Jitter from the communication (in microsecond)
    double imu_acceleration_x;     // IMU Measured acceleration (X-Axis) of the interconnect (in meters per second ^ squared)
    double imu_acceleration_y;     // IMU Measured acceleration (Y-Axis) of the interconnect (in meters per second ^ squared)
    double imu_acceleration_z;     // IMU Measured acceleration (Z-Axis) of the interconnect (in meters per second ^ squared)
    double imu_angular_velocity_x; // IMU Measured angular velocity (X-Axis) of the interconnect (in degrees per second)
    double imu_angular_velocity_y; // IMU Measured angular velocity (Y-Axis) of the interconnect (in degrees per second)
    double imu_angular_velocity_z; // IMU Measured angular velocity (Z-Axis) of the interconnect (in degrees per second)
    double voltage;                // Voltage of the main board (in Volt)
    double temperature_core;       // Microcontroller temperature (in degrees Celsius)
    uint32_t fault_bank_a;         // Fault bank A (see InterconnectConfig.SafetyIdentifier)
    uint32_t fault_bank_b;         // Fault bank B (see InterconnectConfig.SafetyIdentifier)
    uint32_t warning_bank_a;       // Warning bank A (see InterconnectConfig.SafetyIdentifier)
    uint32_t warning_bank_b;       // Warning bank B (see InterconnectConfig.SafetyIdentifier)
    double difference_count_a;
    double difference_count_b;
    GripperFeedback gripper_feedback;
} InterconnectFeedback;

typedef struct 
{
    uint32_t sensor;         // The sensor (color or depth)
    uint32_t resolution; // The resolution setting
    uint32_t frame_rate;  // Frame rate setting
    uint32_t bit_rate;      // Maximum encoded bit rate
} SensorSettings;

typedef struct
{
    uint32_t sensor; // The sensor to configure
    uint32_t option; // The option to configure on the sensor
} OptionIdentifier;

typedef struct
{
    uint32_t sensor; // The sensor to configure (color or depth)
    uint32_t option; // The option to configure
    double  value;  // The desired value for the option
} OptionValue;

typedef struct
{
    uint32_t sensor;        // The sensor (color or depth)
    uint32_t option;        // The option
    bool   supported;       // True if the option is supported by the chosen sensor, false otherwise
    bool   read_only;       // True if the option is read-only, false if it can be changed
    double  minimum;        // Minimum value for the option
    double  maximum;        // Maximum value for the option
    double  step;           // Step size for the option value (if it takes on discrete values)
    double  default_value;  // Default value for the option
} OptionInformation;

typedef struct
{
    uint32_t sensor;             // The sensor on which to perform the focus action
    uint32_t focus_action ; // The focus action to perform on the sensor
} SensorFocusAction;

typedef struct
{
    uint32_t event;  // Vision event
    uint32_t sensor;      // The sensor that caused the notification (if applicable)
    uint32_t option;      // The option that caused the notification (if applicable)
} VisionNotification;

typedef struct
{
    double k1; // First radial distortion coefficient
    double k2; // Second radial distortion coefficient
    double k3; // Third radial distortion coefficient
    double p1; // First tangential distortion coefficient
    double p2; // Second tangential distortion coefficient
} DistortionCoefficients;

typedef struct
{
    uint32_t sensor;           // The sensor on which to perform the focus action
    uint32_t resolution;       // The resolution for which the parameters apply to

    double principal_point_x;  // Horizontal coordinate of the principal point of the image, as a pixel offset from the left edge
    double principal_point_y;  // Vertical coordinate of the principal point of the image, as a pixel offset from the top edge
    double focal_length_x;     // Focal length of the image plane, as a multiple of pixel width
    double focal_length_y;     // Focal length of the image plane, as a multiple of pixel height

    DistortionCoefficients distortion_coeffs;
} IntrinsicParameters;

typedef struct
{
    float column1; // Value between -1.0 and 1.0
    float column2; // Value between -1.0 and 1.0
    float column3; // Value between -1.0 and 1.0
} RotationMatrixRow;

typedef struct
{
    RotationMatrixRow row1; // First rotation matrix row
    RotationMatrixRow row2; // Second rotation matrix row
    RotationMatrixRow row3; // Third rotation matrix row
} RotationMatrix;

typedef struct
{
    float x; // Translation in meters in the x axis
    float y; // Translation in meters in the y axis
    float z; // Translation in meters in the z axis
} TranslationVector;

typedef struct
{
    RotationMatrix rotation;       // The rotation matrix from depth to color camera
    TranslationVector translation; // The translation vector from depth to color camera
} ExtrinsicParameters;


#endif