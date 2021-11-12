
figure

%% Plot a sphere
%Create unit sphere
[X,Y,Z] = sphere;

%Plot surface
sph = surf(X,Y,Z);
view(50,20);
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

%% Plot points on tangent space and projection on the sphere

%Define point at identity
startPoint = [0 0 1]; 
%in this case the point corresponds to the intersection between the sphere
%and the tangent plane

%Define rotation - To keep it simple, we consider only a rotation around
%the X-Axis
theta = 60; %deg
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

%Plot 3d points and surface line
%Plot identity point
plot3(startPoint(1),startPoint(2),startPoint(3),'X','Color','r','LineWidth',2);
text(0,0,1.2,'$g$','interpreter','latex','FontSize',14,'FontWeight','bold');
%Plot another point in the Lie group
plot3(endPoint(1),endPoint(2),endPoint(3),'.','Color','r','MarkerSize',10);
text(endPoint(1),endPoint(2)-0.2,endPoint(3),'$g^{\prime}$','interpreter','latex','FontSize',14,'FontWeight','bold');
plot3(points(1,2:end-1),points(2,2:end-1),points(3,2:end-1),'-','Color','r','LineWidth',1.5);
%Plot arbitrary point in tangent space
plot3([startPoint(1) endPoint(1)],[startPoint(2) endPoint(2)],[1 1],'-','Color','#0072BD','LineWidth',1.5);
plot3(endPoint(1),endPoint(2),1,'.','Color','#0072BD','MarkerSize',10);
%text(endPoint(1),endPoint(2),1.2,'$\frak{g}$','interpreter','latex','FontSize',14,'FontWeight','bold');
text(-0.8,0.8,1.2,'$T_{g}\mathcal{G}$','interpreter','latex','FontSize',14,'FontWeight','bold');
set(gca,'visible','off')
%print(gcf,'foo.png','-dpng','-r300');


