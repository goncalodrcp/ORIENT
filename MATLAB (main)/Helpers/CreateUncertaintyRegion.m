function [x_out,y_out] = CreateUncertaintyRegion(Cx,ux,n)

%Function that determines the eigenvalues and eigenvectors of Cx
%and draws the associated uncertainty region

width= 2;

%Eigenvalues and eigenvectors of Cx
[v,lamb] = eig(Cx);

%a. Generate a set of points on a circle with unit radius
%Outline of the circle
teta_out = linspace(0,2*pi,n)';
x_out = cos(teta_out);
y_out = sin(teta_out);
%plot(x_out,y_out);

%b.Scale the x and y coordinates of these points
a0 = width*sqrt(lamb(1));
a1 = width*sqrt(lamb(4));
x_out = x_out*a0;
y_out = y_out*a1;

%c. Rotate the set of points in accordance with the direction of the
%principal axes.
x_out1 = v(1,1)*x_out+v(1,2)*y_out;
y_out1 = v(2,1)*x_out+v(2,2)*y_out;

%d.Shift the whole set to the position determined by ux
x_out = ux(1)+x_out1;
y_out = ux(2)+y_out1;

%plot(x_out,y_out); %plot outline of the region
%hold on;
%xlabel('$$x$$','interpreter','latex');
%ylabel('$$y$$','interpreter','latex');
%Draw eigenvectors
%quiver([ux(1) ux(1)],[ux(2) ux(2)],v(1,:),v(2,:),4,'filled');
%xL = xlim;
%yL = ylim;
%line([ux(1) ux(1)], yL,'Color','black','LineStyle','--');  %x-axis
%line(xL,[ux(2),ux(2)],'Color','black','LineStyle','--');  %y-axis

end

