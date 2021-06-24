%
%  evaluate the wx(Iw) term with respect to Ia
%
%  do this for a saccades of given amplitudes in a particular direction in LP
%  from primary position
%
function rotationEuler(R, phi)
if nargin<2 phi = 0; end
if nargin<1 R = [2, 5, 10, 20, 30, 40, 50, 60]; end
dt = 0.01;
time = [0:dt:2.0];
T = length(time);
N = length(R);
Frac = zeros(N,1);
Frac2 = zeros(N,1);
Frac3=Frac2;
Wmax = zeros(N,1);
F = zeros(T,1);
Wt = F; At = F;

figure(1); clf; figure(3); clf;
for n=1:N
alpha = 150/(1+0.1*R(n))   % to get a sort of main sequence

%
% generate the saccade position signal as a sigmoid 
%
rt = 0.5*R(n)*tanh(alpha*(time-1.0))+0.5*R(n);   % in deg
figure(1); 
subplot(3,1,1); hold on
plot(time,rt, 'r-', 'linewidth',1.5)
axis([0.85 1.15 -10 70]);
%xlabel('time (s)', 'fontsize',15);
set(gca, 'xtick', []);
ylabel('\Delta R(t) (deg)', 'fontsize', 14)
set(gca, 'fontsize',14);

ry = rt*sin(2*pi*phi)*pi/180; % in radians
rz = rt*cos(2*pi*phi)*pi/180;

wx = 0;
wy = diff(ry)/dt;    % in rad/s
wz = diff(rz)/dt;
subplot(3,1,2); hold on
plot(time(1:end-1), wz, 'r-', 'linewidth',1.5);
axis([0.85 1.15 0 15]);
% xlabel('time (s)', 'fontsize',15);
set(gca, 'xtick', []);
ylabel('\omega (t) (rad/s)', 'fontsize', 14)
set(gca, 'fontsize',14);

ax = 0;
ay = diff(wy)/dt;   % in rad/s^2
az = diff(wz)/dt;
subplot(3,1,3); hold on
plot(time(2:end-1), az, 'r-', 'linewidth',1.5);
axis([0.85 1.15 -400 400]);
xlabel('Time (s)', 'fontsize',15);
ylabel('\alpha (t) (rad/s^2)', 'fontsize', 14)
set(gca, 'fontsize',14);

a = [ax; max(ay); max(az)];
w = [wx; max(wy); max(wz)];
Wmax(n) = sqrt(w(2)^2+w(3)^2);

I = [4.759 -0.01 0.111;
     -0.01 4.316 0;
      0.111 0 3.956] * 1e-4;
  
A = I*a;
W = cross(w, I*w);

Frac(n) = sqrt(W(1)^2+W(2)^2+W(3)^2)/sqrt(A(1)^2+A(2)^2+A(3)^2);

for t=1:T-2
    a = [ax; ay(t); az(t)];
    w = [wx; wy(t); wz(t)];
    A = I*a;
    W = cross(w, I*w);
    F(t) = sqrt(W(1)^2+W(2)^2+W(3)^2)/sqrt(A(1)^2+A(2)^2+A(3)^2+1e-5);
    At(t) = sqrt(A(1)^2+A(2)^2+A(3)^2); 
    Wt(t) = sqrt(W(1)^2+W(2)^2+W(3)^2);
end
Frac2(n) = max(F);

n1=0.85/dt; n2=1.15/dt;

Frac3(n) = rms(Wt(n1:n2))^2/rms(At(n1:n2))^2;   % the signals' root-mean square powers over 300 ms 

figure(3); hold on;
%plot(time, F, 'r-', 'linewidth',1.5);
plot(time, 100*Wt, 'k-', 'linewidth',1.5);
plot(time, At, 'r-', 'linewidth',1.5);
legend('|\omega \times (I\cdot \omega)|', '|I\cdot \alpha|');
axis([0.85 1.15 0 0.2]);
xlabel('time (s)', 'fontsize',15);
ylabel('100 \cdot |\omega \times (I \cdot \omega)|,  | I\cdot \alpha|', 'fontsize', 14)
set(gca, 'fontsize',14);
nicegraph

end

Frac
Frac2
Frac3

figure(2); clf;  % main sequence
plot(R, Wmax, 'ko-','linewidth',1.5,'markersize', 10,'markerfacecolor','r');
ylabel('\omega_{max} (rad/s)','fontsize',16);
xlabel('\Delta R (deg)','fontsize',16);
set(gca, 'fontsize',14);
nicegraph

figure(4); clf; hold on
plot(R, Frac3, 'ko-', 'linewidth',1.5,'markersize', 10,'markerfacecolor','r');
axis([0 60 0 0.00025]);
ylabel('P(\omega \times (I\omega))/P(I\alpha)','fontsize',16);
xlabel('Saccade amplitude (deg)','fontsize',16);
set(gca, 'fontsize',14);
plot([0 60],[0 0], 'k--');
nicegraph

figure(1); print -deps2c EulerEqnA;
figure(2); print -deps2c EulerEqnB;
figure(3); print -deps2c EulerEqnC;
figure(4); print -deps2c EulerEqnD;

