robot = loadrobot('kinovaGen3','DataFormat','row','Gravity',[0 0 -9.81]);
currentRobotJConfig = homeConfiguration(robot);
show(robot);
axis auto;
view([60,10]);


ik = inverseKinematics('RigidBodyTree', robot);

ik.SolverParameters.AllowRandomRestart = false;

weights = [1, 1, 1, 1, 1, 1];

numJoints = numel(currentRobotJConfig);
endEffector = "EndEffector_Link";

jointInit = currentRobotJConfig;
taskInit = getTransform(robot,jointInit,endEffector);