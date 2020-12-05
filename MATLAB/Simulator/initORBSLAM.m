function [initialState,imds] = initORBSLAM(ParamGlobal,ParamFilter,GT_data,CAM_data,IMU_data)

%Load Ground truth trajectory
trajReal = GT_data.trajReal;

%Load camera paramters (intrinsics and extrinsics)
cameraParams = ParamFilter.cameraParams;

%Initialise orientation, position and velocity of the body with ground-truth
%The default order for Euler angle rotations is "ZYX"
Rot0 = eul2rotm([trajReal.psi(1),trajReal.theta(1),trajReal.phi(1)]);
x0 = trajReal.x(:,1);
v0 = trajReal.v(:,1);

%Create image data store object
imds = imageDatastore(ParamGlobal.dirImage);

%Read first image
currFrameID = 1;
[currI, imInfo] = readimage(imds,currFrameID);
currI = undistortImage(currI,cameraParams);
himage = imshow(currI);

%% Map initialisation

end

