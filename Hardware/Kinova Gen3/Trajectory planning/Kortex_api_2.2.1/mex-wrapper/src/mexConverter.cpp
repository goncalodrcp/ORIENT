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

// Header file
#include <mexConverter.h>

// For mex functions
#include <mex.h>

// For std::runtime_error
#include <stdexcept>

namespace KortexMatlab
{
   
mxArray* createDistortionCoefficientsStructMatrix(const DistortionCoefficients* coefficients)
{
    static const char* fieldnames[] = {
        "k1",
        "k2",
        "k3",
        "p1",
        "p2"
    };
    
    static const int nfields = sizeof(fieldnames) / sizeof(*fieldnames);
    mxArray* myStructMatrix = mxCreateStructMatrix(1, 1, nfields, fieldnames);
    
    mxArray* k1 = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    mxArray* k2 = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    mxArray* k3 = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    mxArray* p1 = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    mxArray* p2 = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    
    if(myStructMatrix == nullptr || 
       k1 == nullptr || 
       k2 == nullptr ||
       k3 == nullptr ||
       p1 == nullptr ||
       p2 == nullptr)
    {
        throw std::runtime_error("createErrorStructMatrix: missing memory space !!");
    }
    
    if(coefficients)
    {
         *mxGetDoubles(k1) = coefficients->k1;
         *mxGetDoubles(k2) = coefficients->k2;
         *mxGetDoubles(k3) = coefficients->k3;
         *mxGetDoubles(p1) = coefficients->p1;
         *mxGetDoubles(p2) = coefficients->p2;
    }
    
    mxSetField(myStructMatrix, 0, "k1", k1);
    mxSetField(myStructMatrix, 0, "k2", k2);
    mxSetField(myStructMatrix, 0, "k3", k3);
    mxSetField(myStructMatrix, 0, "p1", p1);
    mxSetField(myStructMatrix, 0, "p2", p2);
    
    return myStructMatrix;
}
    
mxArray* createIntrinsicStructMatrix(const IntrinsicParameters *Intrinsic)
{
    // Structure field names
    static const char* fieldnames[] = {
        "sensor",
        "resolution",
        "principal_point_x",
        "principal_point_y",
        "focal_length_x",
        "focal_length_y",
        "distortion_coeffs",
    };

    static const int nfields = sizeof(fieldnames) / sizeof(*fieldnames);
    mxArray* myStructMatrix = mxCreateStructMatrix(1, 1, nfields, fieldnames);

    mxArray* sensor = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    mxArray* resolution = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    mxArray* principal_point_x = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    mxArray* principal_point_y = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    mxArray* focal_length_x = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    mxArray* focal_length_y = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    mxArray* distortion_coeffs = KortexMatlab::createDistortionCoefficientsStructMatrix(&(Intrinsic->distortion_coeffs));
    
    if ( myStructMatrix == nullptr || 
            sensor == nullptr || 
            resolution == nullptr ||
            principal_point_x == nullptr ||
            principal_point_y == nullptr ||
            focal_length_x == nullptr ||
            focal_length_y == nullptr ||
            distortion_coeffs == nullptr)
    {
        throw std::runtime_error("createErrorStructMatrix: missing memory space !!");
    }

    *mxGetUint32s(sensor) = Intrinsic->sensor;
    *mxGetUint32s(resolution) = Intrinsic->resolution;
    *mxGetDoubles(principal_point_x) = Intrinsic->principal_point_x;
    *mxGetDoubles(principal_point_y) = Intrinsic->principal_point_y;
    *mxGetDoubles(focal_length_x) = Intrinsic->focal_length_x;
    *mxGetDoubles(focal_length_y) = Intrinsic->focal_length_y;

    mxSetField(myStructMatrix, 0, "sensor",        sensor);
    mxSetField(myStructMatrix, 0, "resolution",    resolution);
    mxSetField(myStructMatrix, 0, "principal_point_x",    principal_point_x);
    mxSetField(myStructMatrix, 0, "principal_point_y",    principal_point_y);
    mxSetField(myStructMatrix, 0, "focal_length_x",    focal_length_x);
    mxSetField(myStructMatrix, 0, "focal_length_y",    focal_length_y);
    mxSetField(myStructMatrix, 0, "focal_length_y",    focal_length_y);
    mxSetField(myStructMatrix, 0, "distortion_coeffs",    distortion_coeffs);

    return myStructMatrix;
}

mxArray* createRotationMatrixRowStructMatrix(const RotationMatrixRow* row)
{
    static const char* fieldnames[] = {
        "column1",
        "column2",
        "column3"
    };
    
    static const int nfields = sizeof(fieldnames) / sizeof(*fieldnames);
    mxArray* myStructMatrix = mxCreateStructMatrix(1, 1, nfields, fieldnames);
    
    mxArray* column1 = mxCreateNumericMatrix(1, 1, mxSINGLE_CLASS, mxREAL);
    mxArray* column2 = mxCreateNumericMatrix(1, 1, mxSINGLE_CLASS, mxREAL);
    mxArray* column3 = mxCreateNumericMatrix(1, 1, mxSINGLE_CLASS, mxREAL);
    
    if(row)
    {
        *mxGetSingles(column1) = row->column1;
        *mxGetSingles(column2) = row->column2;
        *mxGetSingles(column3) = row->column3;
    }
    
    mxSetField(myStructMatrix, 0, "column1", column1);
    mxSetField(myStructMatrix, 0, "column2", column2);
    mxSetField(myStructMatrix, 0, "column3", column3);
    
    return myStructMatrix;
}

mxArray* createRotationMatrixStructMatrix(const RotationMatrix* rotation)
{
    static const char* fieldnames[] = {
        "row1",
        "row2",
        "row3"
    };
    
    static const int nfields = sizeof(fieldnames) / sizeof(*fieldnames);
    mxArray* myStructMatrix = mxCreateStructMatrix(1, 1, nfields, fieldnames);
    
    mxArray* row1 = KortexMatlab::createRotationMatrixRowStructMatrix(&(rotation->row1));
    mxArray* row2 = KortexMatlab::createRotationMatrixRowStructMatrix(&(rotation->row2));
    mxArray* row3 = KortexMatlab::createRotationMatrixRowStructMatrix(&(rotation->row3));
    
    mxSetField(myStructMatrix, 0, "row1", row1);
    mxSetField(myStructMatrix, 0, "row2", row2);
    mxSetField(myStructMatrix, 0, "row3", row3);
    
    return myStructMatrix;
}

mxArray* createTranslationVectorStructMatrix(const TranslationVector* translationVector)
{
    static const char* fieldnames[] = {
        "x",
        "y",
        "z"
    };
    
    static const int nfields = sizeof(fieldnames) / sizeof(*fieldnames);
    mxArray* myStructMatrix = mxCreateStructMatrix(1, 1, nfields, fieldnames);
    
    mxArray* x = mxCreateNumericMatrix(1, 1, mxSINGLE_CLASS, mxREAL);
    mxArray* y = mxCreateNumericMatrix(1, 1, mxSINGLE_CLASS, mxREAL);
    mxArray* z = mxCreateNumericMatrix(1, 1, mxSINGLE_CLASS, mxREAL);
    
    if(translationVector)
    {
        *mxGetSingles(x) = translationVector->x;
        *mxGetSingles(y) = translationVector->y;
        *mxGetSingles(z) = translationVector->z;
    }
            
    mxSetField(myStructMatrix, 0, "x", x);
    mxSetField(myStructMatrix, 0, "y", y);
    mxSetField(myStructMatrix, 0, "z", z);
    
    return myStructMatrix;
}

mxArray* createExtrinsicStructMatrix(const ExtrinsicParameters *Extrinsic)
{
    // Structure field names
    static const char* fieldnames[] = {
        "rotation",
        "translation"
    };

    static const int nfields = sizeof(fieldnames) / sizeof(*fieldnames);
    mxArray* myStructMatrix = mxCreateStructMatrix(1, 1, nfields, fieldnames);

    mxArray* rotation = KortexMatlab::createRotationMatrixStructMatrix(&(Extrinsic->rotation));
    mxArray* translation = KortexMatlab::createTranslationVectorStructMatrix(&(Extrinsic->translation));
    
    if ( myStructMatrix == nullptr || 
         rotation == nullptr ||
         translation == nullptr)
    {
        throw std::runtime_error("createErrorStructMatrix: missing memory space !!");
    }

    mxSetField(myStructMatrix, 0, "rotation", rotation);
    mxSetField(myStructMatrix, 0, "translation", translation);

    return myStructMatrix;
}


mxArray* createOptionValueStructMatrix(const OptionValue* optionValue)
{
    // Structure field names
    static const char* fieldnames[] = {
        "sensor",
        "option",
        "value"
    };

    static const int nfields = sizeof(fieldnames) / sizeof(*fieldnames);
    mxArray* myStructMatrix = mxCreateStructMatrix(1, 1, nfields, fieldnames);
    
    mxArray* sensor = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    mxArray* option = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    mxArray* value = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    
    if(optionValue)
    {
        *mxGetUint32s(sensor) = optionValue->sensor;
        *mxGetUint32s(option) = optionValue->option;
        *mxGetDoubles(value) = optionValue->value;
    }
    
    mxSetField(myStructMatrix, 0, "sensor", sensor);
    mxSetField(myStructMatrix, 0, "option", option);
    mxSetField(myStructMatrix, 0, "value", value);
    
    return myStructMatrix;
}

mxArray* createSensorSettingsStructMatrix(const SensorSettings* settings)
{    
    // Structure field names
    static const char* fieldnames[] = {
        "sensor",
        "resolution",
        "frame_rate",
        "bit_rate"
    };

    static const int nfields = sizeof(fieldnames) / sizeof(*fieldnames);
    mxArray* myStructMatrix = mxCreateStructMatrix(1, 1, nfields, fieldnames);
    
    mxArray* sensor = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    mxArray* resolution = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    mxArray* frame_rate = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    mxArray* bit_rate = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    
    if(settings)
    {
        *mxGetUint32s(sensor) = settings->sensor;
        *mxGetUint32s(resolution) = settings->resolution;
        *mxGetUint32s(frame_rate) = settings->frame_rate;
        *mxGetUint32s(bit_rate) = settings->bit_rate;
    }
    
    mxSetField(myStructMatrix, 0, "sensor", sensor);
    mxSetField(myStructMatrix, 0, "resolution", resolution);
    mxSetField(myStructMatrix, 0, "frame_rate", frame_rate);
    mxSetField(myStructMatrix, 0, "bit_rate", bit_rate);
    
    return myStructMatrix;
}

mxArray* createErrorStructMatrix(const ErrorInfo *ErrorInfo)
{
    // Structure field names
    static const char* fieldnames[] = {
        "code",
        "sub_code"
    };

    static const int nfields = sizeof(fieldnames) / sizeof(*fieldnames);
    mxArray* myStructMatrix = mxCreateStructMatrix(1, 1, nfields, fieldnames);

    mxArray* code = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    mxArray* sub_code = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    
    if ( myStructMatrix == nullptr || 
            code == nullptr || 
            sub_code == nullptr)
    {
        throw std::runtime_error("createErrorStructMatrix: missing memory space !!");
    }

    *mxGetUint32s(code) = ErrorInfo->code;
    *mxGetUint32s(sub_code) = ErrorInfo->sub_code;

    mxSetField(myStructMatrix, 0, "code",        code);
    mxSetField(myStructMatrix, 0, "sub_code",    sub_code);

    return myStructMatrix;
}

mxArray* createBasicFeedbackStructMatrix(uint32_t nbrJoints, const BaseFeedback *pBaseFeedback, const ActuatorsFeedback *pActuatorsFeedback)
{
    // Structure field names
    static const char* fieldnames[] = {
        "arm_state",
        "tool_pose", 
        "position", 
        "velocity", 
        "torque", 
    };

    // Allocate memory for the structure
    // -- Signature: mxArray *mxCreateStructMatrix(mwSize m, mwSize n, int nfields, const char **fieldnames);
    static const int nfields = sizeof(fieldnames) / sizeof(*fieldnames);
    mxArray* myStructMatrix = mxCreateStructMatrix(1, 1, nfields, fieldnames);

    //
    // Create mxArray data structures to hold the data to be assigned for the structure.
    //
    mxArray* arm_state = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    mxArray* tool_pose = mxCreateNumericMatrix(1, 6, mxDOUBLE_CLASS, mxREAL);
    
    mxArray* position = mxCreateNumericMatrix(1, nbrJoints, mxDOUBLE_CLASS, mxREAL);
    mxArray* velocity = mxCreateNumericMatrix(1, nbrJoints, mxDOUBLE_CLASS, mxREAL);
    mxArray* torque = mxCreateNumericMatrix(1, nbrJoints, mxDOUBLE_CLASS, mxREAL);

    if ( myStructMatrix == nullptr || 
            arm_state == nullptr || 
            tool_pose == nullptr || 
            position == nullptr || 
            velocity == nullptr || 
            torque == nullptr )
    {
        throw std::runtime_error("createBasicFeedbackStructMatrix: missing memory space !!");
    }
            
     //
    // Initialize all fields
    //
    *mxGetUint32s(arm_state) = pBaseFeedback->arm_state;

    for(int i = 0; i < 6; ++i)
    {
        mxGetDoubles(tool_pose)[i] = pBaseFeedback->tool_pose[i];
    }

    for (int joint=0; joint < MAX_JOINTS; joint++)
    {
        mxGetDoubles(position)  [joint] = pActuatorsFeedback->position         [joint];
        mxGetDoubles(velocity)  [joint] = pActuatorsFeedback->velocity         [joint];
        mxGetDoubles(torque)    [joint] = pActuatorsFeedback->torque           [joint];
    }
    
    mxSetField(myStructMatrix, 0, "arm_state",          arm_state);
    mxSetField(myStructMatrix, 0, "tool_pose",          tool_pose);
    
    mxSetField(myStructMatrix, 0, "position",           position);
    mxSetField(myStructMatrix, 0, "velocity",           velocity);
    mxSetField(myStructMatrix, 0, "torque",             torque);

    return myStructMatrix;
}

mxArray* createBaseFeedbackStructMatrix(const BaseFeedback *pBaseFeedback)
{
    // Structure field names
    static const char* fieldnames[] = {
        "arm_state",
        "arm_voltage", 
        "arm_current", 
        "temperature_cpu", 
        "temperature_ambient", 
        "imu_acceleration", 
        "imu_angular_velocity", 
        "tool_pose", 
        "tool_twist", 
        "tool_external_wrench_force", 
        "tool_external_wrench_torque", 
        "fault_bank_a",
        "fault_bank_b",
        "warning_bank_a",
        "warning_bank_b"
    };

    // Allocate memory for the structure
    // -- Signature: mxArray *mxCreateStructMatrix(mwSize m, mwSize n, int nfields, const char **fieldnames);
    static const int nfields = sizeof(fieldnames) / sizeof(*fieldnames);
    mxArray* myStructMatrix = mxCreateStructMatrix(1, 1, nfields, fieldnames);

    //
    // Create mxArray data structures to hold the data to be assigned for the structure.
    //
    mxArray* arm_state = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    mxArray* arm_voltage = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    mxArray* arm_current = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    mxArray* temperature_cpu = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    mxArray* temperature_ambient = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    mxArray* imu_acceleration = mxCreateNumericMatrix(1, 3, mxDOUBLE_CLASS, mxREAL);
    mxArray* imu_angular_velocity = mxCreateNumericMatrix(1, 3, mxDOUBLE_CLASS, mxREAL);
    mxArray* tool_pose = mxCreateNumericMatrix(1, 6, mxDOUBLE_CLASS, mxREAL);
    mxArray* tool_twist = mxCreateNumericMatrix(1, 6, mxDOUBLE_CLASS, mxREAL);
    mxArray* tool_external_wrench_force = mxCreateNumericMatrix(1, 3, mxDOUBLE_CLASS, mxREAL);
    mxArray* tool_external_wrench_torque = mxCreateNumericMatrix(1, 3, mxDOUBLE_CLASS, mxREAL);
    
    mxArray* fault_bank_a = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    mxArray* fault_bank_b = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);

    mxArray* warning_bank_a = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    mxArray* warning_bank_b = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);

    if(nullptr == myStructMatrix
            || nullptr == arm_state
            || nullptr == arm_voltage
            || nullptr == arm_current
            || nullptr == temperature_cpu
            || nullptr == temperature_ambient
            || nullptr == imu_acceleration
            || nullptr == imu_angular_velocity
            || nullptr == tool_pose
            || nullptr == tool_twist
            || nullptr == tool_external_wrench_force
            || nullptr == tool_external_wrench_torque
            || nullptr == fault_bank_a
            || nullptr == fault_bank_b
            || nullptr == warning_bank_a
            || nullptr == warning_bank_b
        )
    {
        throw std::runtime_error("Not enough memory available. Exiting");
    }

    //
    // Initialize all fields
    //
    if (pBaseFeedback)
    {
        *mxGetUint32s(arm_state)              = pBaseFeedback->arm_state;

        *mxGetDoubles(arm_voltage)            = pBaseFeedback->arm_voltage;
        *mxGetDoubles(arm_current)            = pBaseFeedback->arm_current;
        *mxGetDoubles(temperature_cpu)        = pBaseFeedback->temperature_cpu;
        *mxGetDoubles(temperature_ambient)    = pBaseFeedback->temperature_ambient;

        *mxGetUint32s(fault_bank_a)           = pBaseFeedback->fault_bank_a;
        *mxGetUint32s(fault_bank_b)           = pBaseFeedback->fault_bank_b;
        *mxGetUint32s(warning_bank_a)         = pBaseFeedback->warning_bank_a;
        *mxGetUint32s(warning_bank_b)         = pBaseFeedback->warning_bank_b;

        for(int i = 0; i < 3; ++i)
        {
            mxGetDoubles(imu_acceleration)[i]               = pBaseFeedback->imu_acceleration[i];
            mxGetDoubles(imu_angular_velocity)[i]           = pBaseFeedback->imu_angular_velocity[i];
            mxGetDoubles(tool_external_wrench_force)[i]     = pBaseFeedback->tool_external_wrench_force[i];
            mxGetDoubles(tool_external_wrench_torque)[i]    = pBaseFeedback->tool_external_wrench_torque[i];
        }

        for(int i = 0; i < 6; ++i)
        {
            mxGetDoubles(tool_pose)[i]                      = pBaseFeedback->tool_pose[i];
            mxGetDoubles(tool_twist)[i]                     = pBaseFeedback->tool_twist[i];
        }
    }
    else
    {
        *mxGetUint32s(arm_state)            = 0;

        *mxGetDoubles(arm_voltage)          = 0;
        *mxGetDoubles(arm_current)          = 0;
        *mxGetDoubles(temperature_cpu)      = 0;
        *mxGetDoubles(temperature_ambient)  = 0;

        *mxGetUint32s(fault_bank_a)         = 0;
        *mxGetUint32s(fault_bank_b)         = 0;
        *mxGetUint32s(warning_bank_a)       = 0;
        *mxGetUint32s(warning_bank_b)       = 0;

        for(int i = 0; i < 3; ++i)
        {
            mxGetDoubles(imu_acceleration)[i]               = 0;
            mxGetDoubles(imu_angular_velocity)[i]           = 0;
            mxGetDoubles(tool_external_wrench_force)[i]     = 0;
            mxGetDoubles(tool_external_wrench_torque)[i]    = 0;
        }

        for(int i = 0; i < 6; ++i)
        {
            mxGetDoubles(tool_pose)[i]  = 0;
            mxGetDoubles(tool_twist)[i] = 0;
        }
    }

    //
    // Assign mxArray data structures to fields (by field NAMES)
    //
    mxSetField(myStructMatrix, 0, "arm_state",                  arm_state);
    mxSetField(myStructMatrix, 0, "arm_voltage",                arm_voltage);
    mxSetField(myStructMatrix, 0, "arm_current",                arm_current);
    mxSetField(myStructMatrix, 0, "temperature_cpu",            temperature_cpu);
    mxSetField(myStructMatrix, 0, "temperature_ambient",        temperature_ambient);
    mxSetField(myStructMatrix, 0, "imu_acceleration",           imu_acceleration);
    mxSetField(myStructMatrix, 0, "imu_angular_velocity",       imu_angular_velocity);
    mxSetField(myStructMatrix, 0, "tool_pose",                  tool_pose);
    mxSetField(myStructMatrix, 0, "tool_twist",                 tool_twist);
    mxSetField(myStructMatrix, 0, "tool_external_wrench_force", tool_external_wrench_force);
    mxSetField(myStructMatrix, 0, "tool_external_wrench_torque",tool_external_wrench_torque);
    mxSetField(myStructMatrix, 0, "fault_bank_a",               fault_bank_a);
    mxSetField(myStructMatrix, 0, "fault_bank_b",               fault_bank_b);
    mxSetField(myStructMatrix, 0, "warning_bank_a",             warning_bank_a);
    mxSetField(myStructMatrix, 0, "warning_bank_b",             warning_bank_b);

    return myStructMatrix;
}

mxArray* createActuatorsFeedbackStructMatrix(uint32_t nbrJoints, const ActuatorsFeedback *pActuatorsFeedback )
{
    // todo we should use the std::array<const char*> instead
    // Structure field names
    static const char* fieldnames[] = {
        "status_flags", 
        "jitter_comm", 
        "position", 
        "velocity", 
        "torque", 
        "current_motor", 
        "voltage", 
        "temperature_motor", 
        "temperature_core", 
        "fault_bank_a",
        "fault_bank_b",
        "warning_bank_a",
        "warning_bank_b"
    };

    // Allocate memory for the structure
    // -- Signature: mxArray *mxCreateStructMatrix(mwSize m, mwSize n, int nfields, const char **fieldnames);
    static const int nfields = sizeof(fieldnames) / sizeof(*fieldnames);
    mxArray* myStructMatrix = mxCreateStructMatrix(1, 1, nfields, fieldnames);

    //
    // Create mxArray data structures to hold the data to be assigned for the structure.
    //
    mxArray* status_flags = mxCreateNumericMatrix(1, nbrJoints, mxUINT32_CLASS, mxREAL);
    mxArray* jitter_comm = mxCreateNumericMatrix(1, nbrJoints, mxUINT32_CLASS, mxREAL);
    mxArray* position = mxCreateNumericMatrix(1, nbrJoints, mxDOUBLE_CLASS, mxREAL);
    mxArray* velocity = mxCreateNumericMatrix(1, nbrJoints, mxDOUBLE_CLASS, mxREAL);
    mxArray* torque = mxCreateNumericMatrix(1, nbrJoints, mxDOUBLE_CLASS, mxREAL);
    mxArray* current_motor = mxCreateNumericMatrix(1, nbrJoints, mxDOUBLE_CLASS, mxREAL);
    mxArray* voltage = mxCreateNumericMatrix(1, nbrJoints, mxDOUBLE_CLASS, mxREAL);
    mxArray* temperature_motor = mxCreateNumericMatrix(1, nbrJoints, mxDOUBLE_CLASS, mxREAL);
    mxArray* temperature_core = mxCreateNumericMatrix(1, nbrJoints, mxDOUBLE_CLASS, mxREAL);

    mxArray* fault_bank_a = mxCreateNumericMatrix(1, nbrJoints, mxUINT32_CLASS, mxREAL);
    mxArray* fault_bank_b = mxCreateNumericMatrix(1, nbrJoints, mxUINT32_CLASS, mxREAL);

    mxArray* warning_bank_a = mxCreateNumericMatrix(1, nbrJoints, mxUINT32_CLASS, mxREAL);
    mxArray* warning_bank_b = mxCreateNumericMatrix(1, nbrJoints, mxUINT32_CLASS, mxREAL);

    if( nullptr == myStructMatrix
            || nullptr == status_flags
            || nullptr == jitter_comm
            || nullptr == position
            || nullptr == velocity
            || nullptr == torque
            || nullptr == current_motor
            || nullptr == voltage
            || nullptr == temperature_motor
            || nullptr == temperature_core
            || nullptr == fault_bank_a
            || nullptr == fault_bank_b
            || nullptr == warning_bank_a
            || nullptr == warning_bank_b
        )
    {
        throw std::runtime_error("Not enough memory to allocate actuator feedback, exiting.");
    }
    
    //
    // Assign mxArray data structures to fields (by field NAMES)
    //
    for (int joint=0; joint < MAX_JOINTS; joint++)
    {
        if (pActuatorsFeedback)
        {
            mxGetUint32s(status_flags)      [joint] = pActuatorsFeedback->status_flags     [joint];

            mxGetUint32s(jitter_comm)       [joint] = pActuatorsFeedback->jitter_comm      [joint];
            mxGetDoubles(position)          [joint] = pActuatorsFeedback->position         [joint];
            mxGetDoubles(velocity)          [joint] = pActuatorsFeedback->velocity         [joint];
            mxGetDoubles(torque)            [joint] = pActuatorsFeedback->torque           [joint];
            mxGetDoubles(current_motor)     [joint] = pActuatorsFeedback->current_motor    [joint];
            mxGetDoubles(voltage)           [joint] = pActuatorsFeedback->voltage          [joint];
            mxGetDoubles(temperature_motor) [joint] = pActuatorsFeedback->temperature_motor[joint];
            mxGetDoubles(temperature_core)  [joint] = pActuatorsFeedback->temperature_core [joint];

            mxGetUint32s(fault_bank_a)      [joint] = pActuatorsFeedback->fault_bank_a     [joint];
            mxGetUint32s(fault_bank_b)      [joint] = pActuatorsFeedback->fault_bank_b     [joint];
            mxGetUint32s(warning_bank_a)    [joint] = pActuatorsFeedback->warning_bank_a   [joint];
            mxGetUint32s(warning_bank_b)    [joint] = pActuatorsFeedback->warning_bank_b   [joint];
        }
        else
        {
            mxGetUint32s(status_flags)      [joint] = 0;

            mxGetUint32s(jitter_comm)       [joint] = 0;
            mxGetDoubles(position)          [joint] = 0;
            mxGetDoubles(velocity)          [joint] = 0;
            mxGetDoubles(torque)            [joint] = 0;
            mxGetDoubles(current_motor)     [joint] = 0;
            mxGetDoubles(voltage)           [joint] = 0;
            mxGetDoubles(temperature_motor) [joint] = 0;
            mxGetDoubles(temperature_core)  [joint] = 0;

            mxGetUint32s(fault_bank_a)      [joint] = 0;
            mxGetUint32s(fault_bank_b)      [joint] = 0;
            mxGetUint32s(warning_bank_a)    [joint] = 0;
            mxGetUint32s(warning_bank_b)    [joint] = 0;
        }
    }
    mxSetField(myStructMatrix, 0, "status_flags",       status_flags);
    mxSetField(myStructMatrix, 0, "jitter_comm",        jitter_comm);
    mxSetField(myStructMatrix, 0, "position",           position);
    mxSetField(myStructMatrix, 0, "velocity",           velocity);
    mxSetField(myStructMatrix, 0, "torque",             torque);
    mxSetField(myStructMatrix, 0, "current_motor",      current_motor);
    mxSetField(myStructMatrix, 0, "voltage",            voltage);
    mxSetField(myStructMatrix, 0, "temperature_motor",  temperature_motor);
    mxSetField(myStructMatrix, 0, "temperature_core",   temperature_core);
    mxSetField(myStructMatrix, 0, "fault_bank_a",       fault_bank_a);
    mxSetField(myStructMatrix, 0, "fault_bank_b",       fault_bank_b);
    mxSetField(myStructMatrix, 0, "warning_bank_a",     warning_bank_a);
    mxSetField(myStructMatrix, 0, "warning_bank_b",     warning_bank_b);

    return myStructMatrix;
}
static const char* MotorFeedbackNames[] = {
    "motor_id",
    "status_flags",
    "jitter_comm",
    "position",
    "velocity",
    "force",
    "current_motor",
    "voltage",
    "temperature_core",
    "temperature_motor"
};

mxArray* createMotorFeedbackStructMatrix(uint32_t length)
{
    static const int nfields = sizeof(MotorFeedbackNames) / sizeof(*MotorFeedbackNames);
    mxArray* motorFeedback = mxCreateStructMatrix(1, length, nfields, MotorFeedbackNames);
    return motorFeedback;
}

void PopulateMotorFeedbackStructMatrix(mxArray* array, uint32_t index, const MotorFeedback* feedback)
{
    mxArray* motor_id = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    mxArray* status_flags = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    mxArray* jitter_comm = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    mxArray* position = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    mxArray* velocity = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    mxArray* force = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    mxArray* current_motor = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    mxArray* voltage = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    mxArray* temperature_core = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    mxArray* temperature_motor = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    if(feedback)
    {
        *mxGetUint32s(motor_id) = feedback->motor_id;
        *mxGetUint32s(status_flags) = feedback->status_flags;
        *mxGetUint32s(jitter_comm) = feedback->jitter_comm;
        *mxGetDoubles(position) = feedback->position;
        *mxGetDoubles(velocity)= feedback->velocity;
        *mxGetDoubles(force) = feedback->force;
        *mxGetDoubles(current_motor) = feedback->current_motor;
        *mxGetDoubles(voltage) = feedback->voltage;
        *mxGetDoubles(temperature_core) = feedback->temperature_core;
        *mxGetDoubles(temperature_motor) = feedback->temperature_motor;
    }
    
    mxSetField(array, index, "motor_id", motor_id);
    mxSetField(array, index, "status_flags", status_flags);
    mxSetField(array, index, "jitter_comm", jitter_comm);
    mxSetField(array, index, "position", position);
    mxSetField(array, index, "velocity", velocity);
    mxSetField(array, index, "force", force);
    mxSetField(array, index, "current_motor", current_motor);
    mxSetField(array, index, "voltage", voltage);
    mxSetField(array, index, "temperature_core", temperature_core);
    mxSetField(array, index, "temperature_motor", temperature_motor);
}

mxArray* createInterconnectFeedbackStructMatrix(const InterconnectFeedback *pInterconnectFeedback)
{
    static const char* fieldnames[] = {
        "feedback_id", 
        "status_flags", 
        "jitter_comm", 
        "imu_acceleration_x",
        "imu_acceleration_y",
        "imu_acceleration_z",
        "imu_angular_velocity_x",
        "imu_angular_velocity_y",
        "imu_angular_velocity_z",
        "voltage",
        "temperature_core", 
        "fault_bank_a",
        "fault_bank_b",
        "warning_bank_a",
        "warning_bank_b",
        "difference_count_a",
        "difference_count_b",
        "gripper_feedback"
    };

    static const char* GripperFeedbacknames[] = {
        "feedback_id", 
        "status_flags",
        "fault_bank_a",
        "fault_bank_b",
        "warning_bank_a",
        "warning_bank_b",
        "motor_count",
        "motor"
    };

    static const int nfields = sizeof(fieldnames) / sizeof(*fieldnames);
    mxArray* myStructMatrix = mxCreateStructMatrix(1, 1, nfields, fieldnames);

    static const int nGripperFeedbackfields = sizeof(GripperFeedbacknames) / sizeof(*GripperFeedbacknames);
    mxArray* myStructGripperFeedback = mxCreateStructMatrix(1, 1 , nGripperFeedbackfields, GripperFeedbacknames);

    mxArray* feedback_id = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    mxArray* status_flags = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    mxArray* jitter_comm = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    mxArray* imu_acceleration_x = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    mxArray* imu_acceleration_y = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    mxArray* imu_acceleration_z = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    mxArray* imu_angular_velocity_x = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    mxArray* imu_angular_velocity_y = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    mxArray* imu_angular_velocity_z = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    mxArray* voltage = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    mxArray* temperature_core = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    
    mxArray* fault_bank_a = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    mxArray* fault_bank_b = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);

    mxArray* warning_bank_a = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    mxArray* warning_bank_b = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    
    mxArray* difference_count_a = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    mxArray* difference_count_b = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);

    mxArray* gripper_feedback_id = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    mxArray* gripper_status_flags = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    mxArray* gripper_fault_bank_a = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    mxArray* gripper_fault_bank_b = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    mxArray* gripper_warning_bank_a = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    mxArray* gripper_warning_bank_b = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    mxArray* gripper_motor_count = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);

    mxArray* motor = createMotorFeedbackStructMatrix(GRIPPER_MAX_MOTOR_COUNT);

    if(nullptr == myStructMatrix
            || nullptr == feedback_id
            || nullptr == status_flags
            || nullptr == jitter_comm
            || nullptr == imu_acceleration_x
            || nullptr == imu_acceleration_y
            || nullptr == imu_acceleration_z
            || nullptr == imu_angular_velocity_x
            || nullptr == imu_angular_velocity_y
            || nullptr == imu_angular_velocity_z
            || nullptr == voltage
            || nullptr == temperature_core
            || nullptr == fault_bank_a
            || nullptr == fault_bank_b
            || nullptr == warning_bank_a
            || nullptr == warning_bank_b
            || nullptr == difference_count_a
            || nullptr == difference_count_b
        )
    {
        throw std::runtime_error("Not enough memory to allocate interconnect feedback, exiting.");
    }

    if(nullptr == myStructGripperFeedback
            || nullptr == gripper_feedback_id
            || nullptr == gripper_status_flags
            || nullptr == gripper_fault_bank_a
            || nullptr == gripper_fault_bank_b
            || nullptr == gripper_warning_bank_a
            || nullptr == gripper_warning_bank_b
            || nullptr == gripper_motor_count
        )
    {
        throw std::runtime_error("Not enough memory to allocate gripper feedback, exiting.");
    }

    if (pInterconnectFeedback)
    {
        *mxGetUint32s(feedback_id)     = pInterconnectFeedback->feedback_id;
        *mxGetUint32s(status_flags)     = pInterconnectFeedback->status_flags;
        *mxGetUint32s(jitter_comm)      = pInterconnectFeedback->jitter_comm;
        *mxGetDoubles(imu_acceleration_x) = pInterconnectFeedback->imu_acceleration_x;
        *mxGetDoubles(imu_acceleration_y) = pInterconnectFeedback->imu_acceleration_y;
        *mxGetDoubles(imu_acceleration_z) = pInterconnectFeedback->imu_acceleration_z;
        *mxGetDoubles(imu_angular_velocity_x) = pInterconnectFeedback->imu_angular_velocity_x;
        *mxGetDoubles(imu_angular_velocity_y) = pInterconnectFeedback->imu_angular_velocity_y;
        *mxGetDoubles(imu_angular_velocity_z) = pInterconnectFeedback->imu_angular_velocity_z;
        *mxGetDoubles(voltage) = pInterconnectFeedback->voltage;
        *mxGetDoubles(temperature_core) = pInterconnectFeedback->temperature_core;
        *mxGetUint32s(fault_bank_a)     = pInterconnectFeedback->fault_bank_a;
        *mxGetUint32s(fault_bank_b)     = pInterconnectFeedback->fault_bank_b;
        *mxGetUint32s(warning_bank_a)   = pInterconnectFeedback->warning_bank_a;
        *mxGetUint32s(warning_bank_b)   = pInterconnectFeedback->warning_bank_b;

        *mxGetUint32s(gripper_feedback_id)   = pInterconnectFeedback->gripper_feedback.feedback_id;
        *mxGetUint32s(gripper_status_flags)   = pInterconnectFeedback->gripper_feedback.status_flags;
        *mxGetUint32s(gripper_fault_bank_a)   = pInterconnectFeedback->gripper_feedback.fault_bank_a;
        *mxGetUint32s(gripper_fault_bank_b)   = pInterconnectFeedback->gripper_feedback.fault_bank_b;
        *mxGetUint32s(gripper_warning_bank_a)   = pInterconnectFeedback->gripper_feedback.warning_bank_a;
        *mxGetUint32s(gripper_warning_bank_b)   = pInterconnectFeedback->gripper_feedback.warning_bank_b;
        *mxGetUint32s(gripper_motor_count)   = pInterconnectFeedback->gripper_feedback.motor_count;

        for(int i = 0; i < GRIPPER_MAX_MOTOR_COUNT; i++)
        {
            PopulateMotorFeedbackStructMatrix(motor, i, nullptr);
        }

        for(int i = 0; i < pInterconnectFeedback->gripper_feedback.motor_count; i++)
        {
            PopulateMotorFeedbackStructMatrix(motor, i, &pInterconnectFeedback->gripper_feedback.motor[i]);
        }

        mxSetField(myStructMatrix, 0, "feedback_id",          feedback_id);
        mxSetField(myStructMatrix, 0, "status_flags",         status_flags);
        mxSetField(myStructMatrix, 0, "jitter_comm",          jitter_comm);
        mxSetField(myStructMatrix, 0, "imu_acceleration_x",   imu_acceleration_x);
        mxSetField(myStructMatrix, 0, "imu_acceleration_y",   imu_acceleration_y);
        mxSetField(myStructMatrix, 0, "imu_acceleration_z",   imu_acceleration_z);
        mxSetField(myStructMatrix, 0, "imu_angular_velocity_x",   imu_angular_velocity_x);
        mxSetField(myStructMatrix, 0, "imu_angular_velocity_y",   imu_angular_velocity_y);
        mxSetField(myStructMatrix, 0, "imu_angular_velocity_z",   imu_angular_velocity_z);
        mxSetField(myStructMatrix, 0, "voltage",              voltage);
        mxSetField(myStructMatrix, 0, "temperature_core",     temperature_core);
        mxSetField(myStructMatrix, 0, "fault_bank_a",       fault_bank_a);
        mxSetField(myStructMatrix, 0, "fault_bank_b",       fault_bank_b);
        mxSetField(myStructMatrix, 0, "warning_bank_a",     warning_bank_a);
        mxSetField(myStructMatrix, 0, "warning_bank_b",     warning_bank_b);
        mxSetField(myStructMatrix, 0, "difference_count_a",     difference_count_a);
        mxSetField(myStructMatrix, 0, "difference_count_b",     difference_count_b);
        
        mxSetField(myStructGripperFeedback, 0, "feedback_id",      gripper_feedback_id);
        mxSetField(myStructGripperFeedback, 0, "status_flags",     gripper_status_flags);
        mxSetField(myStructGripperFeedback, 0, "fault_bank_a",     gripper_fault_bank_a);
        mxSetField(myStructGripperFeedback, 0, "fault_bank_b",     gripper_fault_bank_b);
        mxSetField(myStructGripperFeedback, 0, "warning_bank_a",   gripper_warning_bank_a);
        mxSetField(myStructGripperFeedback, 0, "warning_bank_b",   gripper_warning_bank_b);
        mxSetField(myStructGripperFeedback, 0, "motor_count",      gripper_motor_count);
        mxSetField(myStructGripperFeedback, 0, "motor",            motor);
        
        mxSetField(myStructMatrix, 0, "gripper_feedback",     myStructGripperFeedback);
    }

    return myStructMatrix;
}
     
    
}