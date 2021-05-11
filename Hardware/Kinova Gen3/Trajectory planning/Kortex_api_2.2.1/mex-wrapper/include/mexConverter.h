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

#ifndef MEX_CONVERTER_H_
#define MEX_CONVERTER_H_

// For structs to convert
#include <kortex_wrapper_data.h>  

// For mxArray declaration
#include <mex.h>  

namespace KortexMatlab
{
    
// Create an Distortion Coefficients mxArray from a C++ struct 
// \param[in] coefficients DistortionCoefficients C++ struct
// \return mxArray representing coefficients
mxArray* createDistortionCoefficientsStructMatrix(const DistortionCoefficients* coefficients);
    
// Create an Intrinsic Parameters mxArray from a C++ struct 
// \param[in] Intrinsic IntrinsicParameters C++ struct
// \return mxArray representing Intrinsic
mxArray* createIntrinsicStructMatrix(const IntrinsicParameters *Intrinsic = nullptr);

// Create an Rotation Matrix Row mxArray from a C++ struct 
// \param[in] row RotationMatrixRow C++ struct
// \return mxArray representing row
mxArray* createRotationMatrixRowStructMatrix(const RotationMatrixRow* row = nullptr);

// Create a Rotation Matrix Parameters mxArray from a C++ struct 
// \param[in] rotation RotationMatrix C++ struct
// \return mxArray representing rotation
mxArray* createRotationMatrixStructMatrix(const RotationMatrix* rotation = nullptr);

// Create a Translation Vector mxArray from a C++ struct 
// \param[in] translationVector TranslationVector C++ struct
// \return mxArray representing translationVector
mxArray* createTranslationVectorStructMatrix(const TranslationVector* translationVector = nullptr);

// Create an Extrinsic Parameters mxArray from a C++ struct 
// \param[in] Extrinsic ExtrinsicParameters C++ struct
// \return mxArray representing Extrinsic
mxArray* createExtrinsicStructMatrix(const ExtrinsicParameters *Extrinsic = nullptr);

// Create an Option Value mxArray from a C++ struct 
// \param[in] optionValue OptionValue C++ struct
// \return mxArray representing optionValue
mxArray* createOptionValueStructMatrix(const OptionValue* optionValue = nullptr);

// Create an Sensor Settings mxArray from a C++ struct 
// \param[in] settings SensorSettings C++ struct
// \return mxArray representing settings
mxArray* createSensorSettingsStructMatrix(const SensorSettings* settings = nullptr);
    
// Create an ErrorInfo mxArray from a C++ struct 
// \param[in] ErrorInfo ErrorInfo C++ struct
// \return mxArray representing ErrorInfo
mxArray* createErrorStructMatrix(const ErrorInfo *ErrorInfo = nullptr);

// Create a Base Feedback mxArray from a C++ struct 
// \param[in] pBaseFeedback BaseFeedback C++ struct
// \return mxArray representing pBaseFeedback
mxArray* createBaseFeedbackStructMatrix(const BaseFeedback *pBaseFeedback = nullptr);
    
// Create an Actuators Feedback mxArray from a C++ struct 
// \param[in] pActuatorsFeedback ActuatorsFeedback C++ struct
// \return mxArray representing pActuatorsFeedback
mxArray* createActuatorsFeedbackStructMatrix(uint32_t nbrJoints, const ActuatorsFeedback *pActuatorsFeedback = nullptr);

// Create an empty Motor Feedback mxArray 
// \param[in] length numbers of Motor Feedback instances to allocate
// \return mxArray representing length numbers of empty Motor Feedback instances
mxArray* createMotorFeedbackStructMatrix(uint32_t length = 1);

// Populate a Motor Feedback mxArray from a C++ struct 
// \param[in] array mxArray to populate
// \param[in] index Index fo the instance to populate inside the array
// \param[in] feedback MotorFeedback C++ struct
void PopulateMotorFeedbackStructMatrix(mxArray* array, uint32_t index, const MotorFeedback* feedback);

// Create an Interconnect Feedback mxArray from a C++ struct 
// \param[in] pInterconnectFeedback InterconnectFeedback C++ struct
// \return mxArray representing pInterconnectFeedback
mxArray* createInterconnectFeedbackStructMatrix(const InterconnectFeedback *pInterconnectFeedback = nullptr);

}

#endif //include guard