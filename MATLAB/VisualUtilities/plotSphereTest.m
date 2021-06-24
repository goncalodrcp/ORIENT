
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
sph.EdgeAlpha = 0.6; %Edge transparency

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
v = [-1 1 1; -1 -1 1; 1 -1 1; 1 1 1];
%Define faces
f = [1 2 3 4];

%Plot surface
pl = patch('Faces',f,'Vertices',v);

%Define colors,lighting,etc
pl.FaceColor = [0.46,0.96,0.62];
pl.FaceAlpha = 0.6;
pl.EdgeAlpha = 0;

