function mainExperiment_plot(trajs, i, trajReal, t)

trajR = trajs.trajR;
trajL = trajs.trajL;
trajU = trajs.trajU;
trajRef = trajs.trajRef;
trajI = trajs.trajI;

errorR = computeError(trajR,trajReal,i);
errorL = computeError(trajL,trajReal,i);
errorU = computeError(trajU,trajReal,i);
errorRef = computeError(trajRef,trajReal,i);
errorI = computeError(trajI,trajReal,i);

figure;hold on;
plot(t(2:i-1)-t(2),errorR.errorR);
plot(t(2:i-1)-t(2),errorL.errorR);
plot(t(2:i-1)-t(2),errorRef.errorR);
plot(t(2:i-1)-t(2),errorU.errorR);
plot(t(2:i-1)-t(2),errorI.errorR);
disp(sqrt(mean([errorR.errorR errorL.errorR  errorRef.errorR errorU.errorR errorI.errorR].^2)));
legend('R-UKF-LG','L-UKF-LG','SE(3)-UKF','UKF','IEKF')
xlabel('t (s)')
ylabel('RMSE attitdude (°)')
title('RMSE on attitude as function of time')
figure
hold on;
plot(t(2:i-1)-t(2),errorR.errorX);
plot(t(2:i-1)-t(2),errorL.errorX);
plot(t(2:i-1)-t(2),errorRef.errorX);
plot(t(2:i-1)-t(2),errorU.errorX);
plot(t(2:i-1)-t(2),errorI.errorX);
disp(sqrt(mean([errorR.errorX  errorL.errorX errorRef.errorX errorU.errorX errorI.errorX].^2)));
legend('R-UKF-LG','L-UKF-LG','SE(3)-UKF','UKF','IEKF')
xlabel('t (s)')
ylabel('RMSE position (m)')
title('RMSE on position as function of time')