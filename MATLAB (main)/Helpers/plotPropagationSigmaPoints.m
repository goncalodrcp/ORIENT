clear;
close all;

figure

%% Plot a sphere
%Create unit sphere
[X,Y,Z] = sphere;

%Plot surface
sph = surf(X,Y,Z);
view(60,18);
axis equal;

%Define colors,lighting, etc
sph.FaceColor = [0.53,0.81,0.93]; %Face color
sph.FaceAlpha = 0.4; %Face transparency
sph.EdgeAlpha = 0.3; %Edge transparency

%Define axis' limits
xlim([-1.5 1.5]);
ylim([-1.5 1.5]);

%Label axis
xlabel('x');
ylabel('y');
zlabel('z');

set(gca,'visible','off')

hold on;
%% Plot a plane on top of the sphere

%Define vertices
v = [-1.5 1.5 1; -1.5 -1.5 1; 1.5 -1.5 1; 1.5 1.5 1];
%Define faces
f = [1 2 3 4];

%Plot surface
pl = patch('Faces',f,'Vertices',v);

%Define colors,lighting,etc
pl.FaceColor = [0.46,0.96,0.62];
pl.FaceAlpha = 0.6;
pl.EdgeAlpha = 0;

text(-1.2,0.8,1.4,'$T_{\hat{\mathbf{x}}_{k-1}}\mathcal{S}$','interpreter','latex','FontSize',14,'FontWeight','bold');

%% Plot points on tangent space and projection on the sphere

%Define point at identity (Mean)
startPoint = [0 0 1]; %Mean
%in this case the point corresponds to the intersection between the sphere
%and the tangent plane
%Plot 3d points and surface line
%Plot identity point
plot3(startPoint(1),startPoint(2),startPoint(3),'X','Color','r','LineWidth',2);
text(0,0,1.2,'$\hat{\mathbf{x}}_{k-1}$','interpreter','latex','FontSize',14,'FontWeight','bold');
text(0.2,0.5,1.3,'$\hat{\mathbf{S}}_{k-1}$','interpreter','latex','FontSize',14,'FontWeight','bold');



%% Propagate mean through a simple rotation

%Define rotation - To keep it simple, we consider only a rotation around
%the X-Axis
theta = 90; %deg
%R = rotx(angle);
dtheta = 2;
v = 2:dtheta:theta;
points = zeros(3,length(v)+1);
points(:,1) = startPoint';
for i=1:length(v)
      angle = v(i);
      R = rotx(angle);
      points(:,i+1) = R*startPoint';
end

endPoint = points(:,end);

plot3(endPoint(1),endPoint(2),endPoint(3),'.','Color','r','MarkerSize',10);
text(endPoint(1),endPoint(2)-0.5,endPoint(3),'$\check{\mathbf{x}}_{k}$','interpreter','latex','FontSize',14,'FontWeight','bold');
plot3(points(1,2:end-1),points(2,2:end-1),points(3,2:end-1),'-','Color','k','LineWidth',1.5);
text(0.5,endPoint(2)-0.5,-0.5,'$\check{\mathbf{S}}_{k}$','interpreter','latex','FontSize',14,'FontWeight','bold');

%% Plot sigma points
rho = ones(1,20);
elevation = repmat(pi/2,1,20)+normrnd(0,0.3,1,20);
azimuth = repmat(pi/6,1,20)+normrnd(0,0.5,1,20);

[x,y,z] = sph2cart(azimuth,elevation,rho); 
plot3(x',y',z','.b','MarkerSize',10)

%Plot sigmapoints on tangent space (z=1)
plot3(x,y,ones(1,20),'.y','MarkerSize',10);


%Create uncertainty region
CovTan = [0.05 0.02; 0.02 0.1];
meanTan = [0 0];
n = 300;
[x_out y_out] =  CreateUncertaintyRegion(CovTan,meanTan,n);
ellipse = [x_out y_out ones(300,1)];
% r = [r  zeros(10,1)];
% sigPointsTangent = repmat(startPoint,10,1);
% sigPointsTangent = sigPointsTangent(:,1:end)+r;
% plot3(sigPointsTangent(1:end,1),sigPointsTangent(1:end,2),sigPointsTangent(1:end,3),'*','Color','y','LineWidth',1);
plot3(ellipse(1:end,1),ellipse(1:end,2),ellipse(1:end,3),'Color','red','LineWidth',1);


%% Draw tangent plane to propagated mean

%Define vertices
v = [endPoint(1)-1 endPoint(2) endPoint(3)+1; endPoint(1)-1 endPoint(2) endPoint(3)-1; ...
    endPoint(1)+1 endPoint(2) endPoint(3)-1; endPoint(1)+1 endPoint(2) endPoint(3)+1];
%Define faces
f = [1 2 3 4];
%Plot surface
pl = patch('Faces',f,'Vertices',v);
%Define colors,lighting,etc
pl.FaceColor = [0.99,0.79,0.32];
pl.FaceAlpha = 0.6;
pl.EdgeAlpha = 0;
text(-1.2,-1.4,-1,'$T_{\check{\mathbf{x}}_{k}}\mathcal{S}$','interpreter','latex','FontSize',14,'FontWeight','bold');

%% Propagate sigma points

for i=1:length(x)
      R = rotx(theta);
      propagatedSigmaPoints(:,i) = R*[x(i) ; y(i) ; z(i)];
      rotatedEllipse = R*ellipse';
end

plot3(propagatedSigmaPoints(1,1:end),propagatedSigmaPoints(2,1:end),propagatedSigmaPoints(3,1:end), ...
      '.b','MarkerSize',10);
  
plot3(propagatedSigmaPoints(1,1:end),-ones(1,length(propagatedSigmaPoints)),propagatedSigmaPoints(3,1:end), ...
      '.y','MarkerSize',10);
  
plot3(rotatedEllipse(1,1:end),rotatedEllipse(2,1:end),rotatedEllipse(3,1:end),'Color','red','LineWidth',1);


%%

legend('','','','','','Transformed sigma-points', 'Tangent space sigma-points','Uncertainty region','Location','southeast');

%Plot another point in the Lie group
%Plot arbitrary point in tangent space
%plot3([startPoint(1) endPoint(1)],[startPoint(2) endPoint(2)],[1 1],'-','Color','#0072BD','LineWidth',1.5);
%plot3(endPoint(1),endPoint(2),1,'.','Color','#0072BD','MarkerSize',10);
%text(endPoint(1),endPoint(2),1.2,'$\frak{g}$','interpreter','latex','FontSize',14,'FontWeight','bold');

%print(gcf,'propagation_legend.png','-dpng','-r300');
