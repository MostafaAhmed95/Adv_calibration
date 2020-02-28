%% robot parameters

% material
robot.E = 7e10;
robot.nu = 0.346;
robot.ro = 2699;
% parameters of the links
              % L    a    b  m
% robot.links = [0.7 0.15 0.11 5;       % link 1
%                0.4 0.15 0.11 5;       % link 2
%                0.8 0.15 0.11 5;       % link 3
%                0.5 0.15 0.11 5;       % link 4
%                0.3 0.13 0.09 5;       % link 5
%                0.2 0.13 0.09 5;       % link 6
%                0.2 0.13 0.09 5];      % link 7
robot.links = [0.675 0.35 0.30 5;       % link 1
               0.35 0.35 0.30 5;       % link 2
               1.15 0.25 0.20 5;       % link 3
               1.2 0.25 0.20 5;       % link 4
               0.001 0.25 0.20 5;       % link 5
               0.001 0.25 0.20 5;       % link 6
               0.24 0.25 0.20 5];      % link 7
% joint stiffness
% robot.Kq = 1e6 * [2 2 2 1 1 1];
robot.Kq = 1e6 *[2    3.2    3.5    1    1    1];
%[1.8519    3.4483    2.3810    0.3584    0.2874    0.4822]
% stiffness mode
robot.use_links = true;
robot.use_gravity = false;

% joints
robot.joint_no = 3;

%robot.theta_no = 6*(robot.joint_no)+robot.joint_no;
%since links are rigid
robot.theta_no = 3;
% tool
robot.tool = eye(4);
%robot.tool(1:3,4) = [0.001;0.001;0.001];

% mass recalculation
for i = 1:size(robot.links, 1)
    robot.links(i,4) = (pi/4)*(robot.links(i,2)^2-robot.links(i,3)^2)*robot.links(i,1)*robot.ro;
end

% limits of the joints
% robot.limits = [-3/4*pi,3/4*pi;
%                 -3/4*pi,3/4*pi;
%                 -3/4*pi,3/4*pi;
%                 -3/4*pi,3/4*pi;
%                 -3/4*pi,3/4*pi;
%                 -3/4*pi,3/4*pi];
  robot.limits = deg2rad([-179,179;
                5,140;
                -155,120;
                -350,350;
                -122,122;
                -350,350]);          
robot.orientation = false;

