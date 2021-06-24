%% Pre computed trajectory to send to Kinova Gen3
%
% The objective is to follow a simple sigmoid trajectory - for now, only
% one joint is necessary.
% Generalized logistic function:
% https://en.wikipedia.org/wiki/Generalised_logistic_function
% 
% Constraints:
% Go to Kinova Gen3 MATLAB github repository:
% https://github.com/Kinovarobotics/matlab_kortex/blob/master/simplified_api/documentation/precomputed_joint_trajectories.md
% 
%
% Gonçalo Pereira, nº81602
% ORIENT
%
%% Example 

clc
clear;
close all;

% Create time vector
ts = 0.001; %Sampling period (s)
duration = 5; %(s)
t = 0:ts:duration;

%Sigmoid parameters
%
% f(t) = L/(1+Ae^(-k(t-m)))
m = 2; %the time value of the sigmoid's midpoint;  f(m) = L/2;
L = pi\4; %target angle - upper asymptote
k = 3; %slope/steepness of the curve
A = 1; %additional parameter, change if necessary

%Joint position
theta=L./(1 + A*exp(-k.*(t-m)));

%Velocity
omega=diff(theta)/ts;

%Acceleration
alpha=diff(omega)/ts;

example=figure;
plot(t,theta);
hold on;
plot(t(:,1:length(omega)),omega);
plot(t(:,1:length(alpha)),alpha);

%% Generate different profiles

% Create time vector
ts = 0.001; %Sampling period (s)
duration = 5; %(s)
t = 0:ts:duration;

%Sigmoid parameters
%
% f(t) = L/(1+Ae^(-k(t-m)))
m = [1 2 3]; %the time value of the sigmoid's midpoint;  f(m) = L/2;
L = pi\8; %target angle - upper asymptote
k = [2 3 3.5 4]; %slope/steepness of the curve
A = 1; %additional parameter, change if necessary

%Create figures;
plotTheta=figure;
plotOmega=figure;
plotAlpha=figure;

for i=1:length(k)
    %Keep the midpoint constant at t = 2;
    %Joint position
    theta=L./(1 + A*exp(-k(i).*(t-m(2))));
    %Velocity
    omega=diff(theta)/ts;
    %Acceleration
    alpha=diff(omega)/ts;
    
    %plot results
    figure(plotTheta);
    plot(t,theta);
    hold on;
    xlabel('t (s)');
    ylabel('$\theta$ (rad)', 'Interpreter', 'latex');
    
    figure(plotOmega);
    plot(t(:,1:length(omega)),omega);
    hold on;
    xlabel('t (s)');
    ylabel('$\dot{\theta} = \omega$ (rad/$s$)', 'Interpreter', 'latex');
    
    figure(plotAlpha);
    plot(t(:,1:length(alpha)),alpha);
    hold on;
    xlabel('t (s)');
    ylabel('$\ddot{\theta} = \alpha$ (rad/$s^2$)', 'Interpreter', 'latex');
       
end

figure(plotTheta);
legend(sprintf('k = %f', k(1)), ...
       sprintf('k = %f', k(2)), ...
       sprintf('k = %f', k(3)), ...
       sprintf('k = %f', k(4)));
   
figure(plotOmega);
legend(sprintf('k = %.1f', k(1)), ...
       sprintf('k = %.1f', k(2)), ...
       sprintf('k = %.1f', k(3)), ...
       sprintf('k = %f', k(4)));

figure(plotAlpha);
legend(sprintf('k = %.1f', k(1)), ...
       sprintf('k = %.1f', k(2)), ...
       sprintf('k = %.1f', k(3)), ...
       sprintf('k = %.1f', k(4)));

