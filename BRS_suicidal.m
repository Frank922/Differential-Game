%for the case that the translational velocity of the evader is faster than
%the translational velocity of the pursuer
clear
clf
tic
T = 0.1; %sample time
%initialize the parameters between the evader and the pursuer
% w = pi/10; %contrain on turning for the robot
% Ve = 2;
% Vp = 1.5;
% %R = (Ve/2)*(1/w); %the minimum turning radius
% R = Ve*(1/w); %the minimum turning radius
% l = 0.5 + 1.5*T; %the target set radius  

%specifications of the simulation
alpha = 1; %the max turn acceleration 
w = 1;
Ve = 1;
Vp = 0.6;
R = 0.7;
% R_min = 0.7;
% R = R_min/(alpha*T);
l = 0.5;

c = Ve/R;
s_bar = acos(-Vp/Ve);

syms eta_max
LHS = (l+Vp*eta_max)*sin(s_bar-c*eta_max);
RHS = R-R*cos(c*eta_max);
h = fplot(LHS);
h.Color = 'r';
hold on
fplot(RHS)
grid on
xlim([-30 30]);
ylim([-20 20]);
root1 = vpasolve(RHS-LHS,eta_max,[0 2]);
%root2 = vpasolve(RHS-LHS,eta_max,[3 4]);


time_max = root1;


clf

[x_e,y_e] = create_circle(0,0,l);
fill(x_e,y_e,'blue');
hold on

t = 0:0.001:time_max;
if t(end) < root1
    t = 0:0.001:time_max+0.001;
end
x = [];
y = [];
v1_unit = [];
v2_unit = [];
x_add = [];
y_add = [];
for i =1:length(t)
    x = [x -R + R*cos(c*t(i))+(l+Vp*t(i))*sin(s_bar-c*t(i))];
    y = [y R*sin(c*t(i))+(l+Vp*t(i))*cos(s_bar-c*t(i))];
    v1_unit = [v1_unit sin(s_bar-c*t(i))];
    v2_unit = [v2_unit cos(s_bar-c*t(i))];
end

plot(x,y,'b')
hold on
plot(-x,y,'b')
hold on

toc
x_max = double(max(x));
y_max = double(max(y));
x_min = double(min(x));
y_min = double(min(y));

function [x,y] = create_circle(xCenter,yCenter,radius)
    theta = 0 : 0.01 : 2*pi;
    x = radius * cos(theta) + xCenter;
    y = radius * sin(theta) + yCenter;
end