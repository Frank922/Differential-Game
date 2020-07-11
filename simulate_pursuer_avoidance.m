%{
The ellipse is on the outside of the backwards reachable set in the relative frame. The position of the obstacle and the robot in the 
inertial frame will be first converted to the relative frame. 

1) the ellipse is made based on the obstacle's speed and the sampling time
so that it can make sure that in each time step the obstacle will not go
straight into the BRS from a point outside the ellipse

2) In the relative frame, if the obstacle is inside the proximity,
represented by the ellipse then the robot is to activate the obstacle
avoidance controller, which is to derive its control strategy from the unit
vector of the closest point on the barrier of the BRS to the obstacle.

%}
clear;
clf;

load('x_brs1.mat'); 
load('y_brs2.mat');
load('v1_unit.mat');
load('v2_unit.mat');


x_brs = x;
y_brs = y;

clear('x');
clear('y');

% Below is to initialize the BRS and the outer layer of the BRS
T = 0.1; %simulation time for each time step (100Hz)

%specifications of the simulation
w_1 = 1;
Ve = 1;
Vp = 0.6;
R = 0.7;
l = 0.5;
w_max = Ve/R;

c = Ve/R;
s_bar = acos(-Vp/Ve);


subplot(2,1,1)
plot(x_brs, y_brs, 'b');
hold on 
subplot(2,1,1)
plot(-x_brs, y_brs, 'b');
hold on

[x_e,y_e] = create_circle(0,0,l);
subplot(2,1,1)
fill(x_e,y_e,'blue');
hold on

syms eta_max
LHS = (l+Vp*eta_max)*sin(s_bar-c*eta_max);
RHS = R-R*cos(c*eta_max);
root1 = vpasolve(RHS-LHS,eta_max,[0 3]);
%root2 = vpasolve(RHS-LHS,eta_max,[3 4]);

time_max = root1;

%determine the outer layer of the BRS
y_axis_max = R*sin(c*time_max)+(l+Vp*time_max)*cos(s_bar-c*time_max);
y_axis_min = -l;
syms t
dx = -c*R*sin(c*t) + Vp*sin(s_bar - c*t) - c*(l+Vp*t)*cos(s_bar - c*t); %the derivative of x in respect to t
t_max_x = vpasolve(dx,t,[0 time_max]); %the t value where x is maximized
x_axis_max = -R + R*cos(c*t_max_x)+(l+Vp*t_max_x)*sin(s_bar-c*t_max_x);
x_axis_min = -x_axis_max;
y_axis_radius = (y_axis_max - y_axis_min)/2 + 0.2; %find the vertical radius for the outer ellipse
x_axis_radius = (x_axis_max - x_axis_min)/2 + 0.2; %find the horizontal radius for the outer ellipse
x0 = 0;
y0 = y_axis_radius/2;
t_plot = -pi:0.01:pi;
x_ellipse = x0 + x_axis_radius*cos(t_plot);
y_ellipse = y0 + y_axis_radius*sin(t_plot);
subplot(2,1,1)
plot(x_ellipse,y_ellipse,'b');
hold on

%equation of the ellipse with center at the origin: (x - x0)^2/(x_axis_radius)^2 +
%(y - y0)^2/(y_axis_radius)^2 <= 1 within the ellipse

% The main program where the simulation takes place

%use the pure pursuit controller for path following
controller = robotics.PurePursuit('DesiredLinearVelocity', 1);
path = [0 5;
        10 5];
controller.Waypoints = path;
x_robot = [0;5];
theta_robot = wrapToPi(0);

%the pursuer's initial position and specification
x_pur = [5; 5];
radius_pur = 0.2;

subplot(2,1,2)
plot([path(1,1) path(end,1)],[path(1,2) path(end,end)],'blue'); hold on %plot the reference path
xlim([-5 20]);
ylim([-5 10]);

%initially plotting the robot and obstacle
subplot(2,1,2)
[vx,vy] = pmark(x_robot(1),x_robot(2),theta_robot,0.2);
fill(vx,vy,'r');
hold on

%draw the pursuer
subplot(2,1,2)
[x_ob,y_ob] = create_circle(x_pur(1),x_pur(2),0.2);
fill(x_ob,y_ob,'b');
hold on


robot_pos = [x_robot;theta_robot]; %the matrix used to store the pose of the robot at each moment
obstacle_pos = x_pur; %the matrix used to store the pose of the obstacle at each moment 
iter_while = 1; %counter used in the simulation
while pdist([x_robot(1),x_robot(2);controller.Waypoints(end,1),controller.Waypoints(end,2)]) > 0.2
    
    if iter_while >= 52 && iter_while < 60
%         x_pur(1) = x_pur(1) + Vp*cos((2/3)*pi)*T;
%         x_pur(2) = x_pur(2) + Vp*sin((2/3)*pi)*T;
        x_pur(1) = x_pur(1) + Vp*cos(pi/4)*T;
        x_pur(2) = x_pur(2) + Vp*sin(pi/4)*T;
    end
    
    if iter_while >= 60 && iter_while < 80
%         x_pur(1) = x_pur(1) + Vp*cos((2/3)*pi)*T;
%         x_pur(2) = x_pur(2) + Vp*sin((2/3)*pi)*T;
        x_pur(1) = x_pur(1) + Vp*cos(pi/7)*T;
        x_pur(2) = x_pur(2) + Vp*sin(pi/7)*T;
    end
    
    %find the relative position of the pursuer to the robot
    [x_pur_rel] = obtain_ob_relative(x_robot,theta_robot,x_pur);
    x_pur_rel = round(x_pur_rel,4);
    subplot(2,1,1)
    scatter(x_pur_rel(1),x_pur_rel(2),[],[1 0 0],'filled');
    hold on
    
    %decide when to activate the pursuer evasion strategy
    if ((x_pur_rel(1) - x0)^2/(x_axis_radius)^2 + (x_pur_rel(2) - y0)^2/(y_axis_radius)^2) <= 1
        %check whether the robot is in the relative frame of reference
        if x_pur_rel(1) == 0 %the pursuer is right along the y-axis in the relative frame
            x_bound = 0;
            x_inter = x_bound; 
            y_max_rel =  R*sin(c*time_max)+(l+Vp*time_max)*cos(s_bar-c*time_max);
            y_inter = y_max_rel;
            previous_t = [];
            if x_pur_rel(2) > -l && x_pur_rel(2) <  y_max_rel
                break;
            end
        elseif x_pur_rel(1) < 0 %the pursuer is on the left half in the relative frame
            t = minimizer_left(x_pur_rel,c,l,Vp,R,s_bar,time_max);
            if t >= 0.01 ^ 2
                previous_t = [];
            else
                previous_t = t;
            end
            x_bound = -(-R + R*cos(c*t)+(l+Vp*t)*sin(s_bar-c*t));
            x_inter = x_bound;
            y_inter = R*sin(c*t)+(l+Vp*t)*cos(s_bar-c*t);
            if x_pur_rel(1) > x_bound && isempty(previous_t)
                break;
            end
        elseif x_pur_rel(1) > 0 %the pursuer is on the right half in the relative frame
            t = minimizer_right(x_pur_rel,c,l,Vp,R,s_bar,time_max);
            if t >= 0.01^2
                previous_t = [];
            else
                previous_t = t;
            end
            x_bound = -R + R*cos(c*t)+(l+Vp*t)*sin(s_bar-c*t);
            x_inter = x_bound;
            y_inter = R*sin(c*t)+(l+Vp*t)*cos(s_bar-c*t);
            if x_pur_rel(1) < x_bound && isempty(previous_t)
                break;
            end
        end
        
        if isempty(previous_t)
            %find the intercept point on the boundary 
            %below the unit_v is obtained by the intercept of LOS and the barrier boundary 
            %[unit_v] = obtain_norm_unit_vec(x_brs,y_brs,x_pur_rel,R,c,Vp,s_bar,l,time_max); %find the intercept on the boundary of and its unit vector on BRS

            %below the unit_v is obtained from the paper 
            if x_pur_rel(1) > 0 
                unit_v = [sin(s_bar - c*t); cos(s_bar - c*t)];
            elseif x_pur_rel(1) < 0 
                unit_v = [-sin(s_bar - c*t); cos(s_bar - c*t)];
            elseif x_pur_rel(1) == 0
                unit_v = [sin(s_bar - c*time_max); cos(s_bar - c*time_max)];
            end
%             subplot(2,1,1);
%             quiver(x_inter,y_inter,unit_v(1),unit_v(2));
%             hold on
            
            %find the optimal control for the robot
            A = x_pur_rel(2)*unit_v(1) - x_pur_rel(1)*unit_v(2); %
            u_in_rel = w_1*(-sign(A)); %the optimal control for the robot 

            %change the relative input to input in the inertial frame
            u_in = -(Ve/R)*u_in_rel;
        else
            %check whether the pursuer is within the target set
            if x_pur_rel(1)^2 + x_pur_rel(2)^2 <= l^2
                break;
            end

            u_in = 0; %if the pursuer is behind the evader, then just run straight ahead for the evader
        end
        w = double(u_in);
    else
        %the pure pursuit controller
        [v,w] = controller([x_robot(1) x_robot(2) theta_robot]);
    end
    
    %maximum allowable angular speed
    if w > w_max 
        w = w_max;
    elseif w < -w_max
        w = -w_max;
    end
    
    %with the control for the robot and obtain the new states of
    %the robot
    [~,robot_state] = ode45(@(t, x) [Ve*cos(x(3)); Ve*sin(x(3)); w], [0 T], [x_robot;theta_robot]);
    x_robot(1) = robot_state(end,1);
    x_robot(2) = robot_state(end,2);
    theta_robot = robot_state(end,3); 
    robot_pos = [robot_pos [x_robot;theta_robot]]; %store the newest pose of the robot for display later
    obstacle_pos = [obstacle_pos x_pur]; %store the newest pose of the obstacle for display later
    %draw the robot
    subplot(2,1,2)
    [vx,vy] = pmark(x_robot(1),x_robot(2),theta_robot,0.2);
    fill(vx,vy,'r');
    hold on
    %draw the pursuer
    subplot(2,1,2)
    [x_ob,y_ob] = create_circle(x_pur(1),x_pur(2),0.2);
    fill(x_ob,y_ob,'b');
    hold on
    iter_while = iter_while + 1;
end

Display_simulation(robot_pos,obstacle_pos,path);

%functions below are used for the simulation

%objective function for finding whether the obstacle is in the BRS with the 
%obstacle is in the right half 
function t = minimizer_right(x_pur_rel,c,l,Vp,R,s_bar,time_max)
    t = fmincon(@objfun1,0,[],[],[],[],0,double(time_max));
    function f = objfun1(t)
        f = (x_pur_rel(1) - ( -R + R*cos(c*t)+(l+Vp*t)*sin(s_bar-c*t)))^2 + (x_pur_rel(2) - (R*sin(c*t)+(l+Vp*t)*cos(s_bar-c*t)))^2;
    end
end

%objective function for finding whether the obstacle is in the BRS with the
%obstacle is in the left half
function t = minimizer_left(x_pur_rel,c,l,Vp,R,s_bar,time_max)
    t = fmincon(@objfun1,0,[],[],[],[],0,double(time_max));
    function f = objfun1(t)
        f = (x_pur_rel(1) + ( -R + R*cos(c*t)+(l+Vp*t)*sin(s_bar-c*t)))^2 + (x_pur_rel(2) - (R*sin(c*t)+(l+Vp*t)*cos(s_bar-c*t)))^2;
    end
end

%function to obtain the relative position of the pursuer to the robot
function [x_pur_rel] = obtain_ob_relative(x_robot,theta_robot,x_pur)
    theta_transform = pi/2 - theta_robot;
    Rmat = [cos(theta_transform) -sin(theta_transform); sin(theta_transform) cos(theta_transform)];
    x_trans = x_robot(1); %translation in x-axis
    y_trans = x_robot(2); %translation in y-axis
    x_pur0 = [x_pur(1) - x_trans; x_pur(2) - y_trans]; %the transformation applied before the translation
    x_pur_rel = Rmat*x_pur0; 
end

%function used to draw the point mass 
function [x,y] = create_circle(xCenter,yCenter,radius)
    theta = 0 : 0.01 : 2*pi;
    x = radius * cos(theta) + xCenter;
    y = radius * sin(theta) + yCenter;
end

%function used to draw the mobile robot
function [vx,vy] = pmark(x,y,theta,s)

    R = [cos(theta) -sin(theta);sin(theta) cos(theta)];

    v(:,1)= [x;y] +s*R*[1; 0];
    v(:,2)= [x;y] +s*R*[-1.5; -1];
    v(:,3)= [x;y] +s*R*[-1; 0];
    v(:,4)= [x;y] +s*R*[-1.5; 1];

    vx=v(1,:);
    vy=v(2,:);
end

%function used to display the simulation of the robot 
function Display_simulation(robot_pos,obstacle_pos,path)
    figure(500)
    line_width = 1.5;
    x_r_1 = []; %store the points that have been travelled by the robot
    y_r_1 = [];
    x_o_1 = []; %store the points that have been travelled by the obstacle
    y_o_1 = [];
    for i = 1:size(robot_pos,2)
        plot([path(1,1) path(end,1)],[path(1,2) path(end,end)],'blue'); hold on %plot the reference path 
        xlim([-5 20]);
        ylim([-5 10]);
        xRobot = robot_pos(1,i); yRobot = robot_pos(2,i); thetaRobot = robot_pos(3,i); %robot pose at each index
        xOb = obstacle_pos(1,i); yOb = obstacle_pos(2,i); %obstacle pose at each index
        x_r_1 = [x_r_1 xRobot];
        y_r_1 = [y_r_1 yRobot];
        x_o_1 = [x_o_1 xOb];
        y_o_1 = [y_o_1 yOb];
        plot(x_r_1,y_r_1,'-r','linewidth',line_width);hold on % plot travelled path by robot
        plot(x_o_1,y_o_1,'-g','linewidth',line_width);hold on % plot travelled path by obstacle
        
        [xob,yob] = create_circle(xOb,yOb,0.2);
        fill(xob,yob,'b');hold on %plot the obstacle position
        
        [vx,vy] = pmark(xRobot,yRobot,thetaRobot,0.2);
        fill(vx,vy,'r'); %plot the robot position
        hold off
        pause(0.2)
        grid on
        drawnow
        % for video generation
        F(i) = getframe(gcf); % to get the current frame
    end
    close(gcf)
    
  % create the video writer with 1 fps
  vidObj = VideoWriter('myVideo.avi');
  vidObj.FrameRate = 10;
  
  open(vidObj);
  for j = 1:length(F)
    frame = F(j);
    writeVideo(vidObj, frame);
  end
  close(vidObj);
end