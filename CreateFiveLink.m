function robot = CreateFiveLink
%CREATEFIVELINK Create a 5 link walking robot
%   

% TODO: Could also make this a fixed base robot (where the base is the toe
% stuck to the ground). Then The impact would really just be the a
% remapping.

robot.NB = 5;   % 5 bodies with 2 floating base (x,y)

% ---------- Joint Tree ---------- %
robot.parent = ...
   [
    0,      ... right calf  (parent: fixed base)
    1,      ... right thigh (parent: right calf)
    2,      ... torso       (parent: right thigh)
    3,      ... left thigh  (parent: torso)
    4,      ... left calf   (parent: left thigh)
    ];

robot.jtype{1} = 'r';       % torso
robot.jtype{2} = 'r';       % right hip
robot.jtype{3} = 'r';       % right knee
robot.jtype{4} = 'r';       % left thigh
robot.jtype{5} = 'r';       % left knee

% Floating base
% robot.parent = ...
%    [0,      ... floating x  (parent: base link)
%     1,      ... floating y  (parent: floating x)
%     2,      ... torso       (parent: floating y)
%     3,      ... right thigh (parent: torso)
%     4,      ... right calf  (parent: right thigh)
%     3,      ... left thigh  (parent: torso)
%     6,      ... left calf   (parent: left thigh)
%     ];
% 
% robot.jtype{1} = 'px';      % floating x
% robot.jtype{2} = 'py';      % floating y
% robot.jtype{3} = 'r';       % torso
% robot.jtype{4} = 'r';       % right hip
% robot.jtype{5} = 'r';       % right knee
% robot.jtype{6} = 'r';       % left thigh
% robot.jtype{7} = 'r';       % left knee

% ---------- Coordinates ---------- %
robot.torso_length = 0.625;
robot.thigh_length = 0.4;
robot.calf_length = 0.4;

thigh_in_torso_theta = 0.0;             % rad
thigh_in_torso_r = [0; 0.0];            % x, y
thigh_in_calf_r = [0; robot.thigh_length];          % x, y
calf_in_thigh_theta = 0;                            % rad
calf_in_thigh_r = [0; -robot.thigh_length];         % x, y
torso_in_thigh_r = [0; robot.thigh_length];

robot.Xtree{1} = plnr(0, [0; 0]);
robot.Xtree{2} = plnr(0, thigh_in_calf_r);
robot.Xtree{3} = plnr(0, torso_in_thigh_r);
robot.Xtree{4} = plnr(0, thigh_in_torso_r);
robot.Xtree{5} = plnr(0, calf_in_thigh_r);

% robot.Xtree{1} = plnr(0, [0; 0]);               % torso rel to floating y
% robot.Xtree{2} = plnr(thigh_in_torso_theta, thigh_in_calf_r);     % right thigh rel to right_calf
% robot.Xtree{3} = plnr(calf_in_thigh_theta, calf_in_thigh_r);      % right calf rel to fixed base
% robot.Xtree{4} = plnr(-thigh_in_torso_theta, thigh_in_torso_r);     % left thigh rel to torso
% robot.Xtree{5} = plnr(-calf_in_thigh_theta, calf_in_thigh_r);      % left calf rel to left thigh

% ---------- Inertias ---------- %
% TODO: Check
% Values from: https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=1013706
torso_m = 10; %20;
torso_com = [0; 0.2];
torso_I = 2.22;

thigh_m = 3; %6.8;
thigh_com = [0; -0.163];
thigh_I = 1.08;

calf_m = 1; %3.2;
calf_com = [0; -0.128];
calf_I = 0.93;

robot.I{3} = mcI(torso_m, torso_com, torso_I);
robot.I{2} = mcI(thigh_m, thigh_com, thigh_I);
robot.I{1} = mcI(calf_m, calf_com, calf_I);
robot.I{4} = mcI(thigh_m, thigh_com, thigh_I);
robot.I{5} = mcI(calf_m, calf_com, calf_I);

robot.gravity = [0; -9.81];

% ---------- Appearance ---------- %
robot.appearance.base = ...
  { 'tiles', [-2 2; 0 0; -2, 2], 0.5};

robot.appearance.body{3} = ...
    { 'box', [-0.04 0 -0.04; 0.04 robot.torso_length 0.04], ...
      'cyl', [0 0 -0.04; 0 0 0.04], 0.06, ...
      { 'colour', [0.1 0.3 0.6]}};
robot.appearance.body{2} = ...
    { 'box', [-0.04 0 -0.04; 0.04 robot.thigh_length 0.04], ...
      'cyl', [0 0 -0.04; 0 0 0.04], 0.04, ...
      { 'colour', [0.1 0.3 0.6]}};
robot.appearance.body{1} = ...
    { 'box', [-0.04 0 -0.04; 0.04 robot.calf_length 0.04], ...
      'cyl', [0 robot.calf_length -0.04; 0 robot.calf_length 0.04], 0.06, ...
      'cyl', [0 robot.calf_length -0.04; 0 robot.calf_length 0.04], 0.00, ...
      { 'colour', [0.1 0.3 0.6]}};
robot.appearance.body{4} = ...
    { 'box', [-0.04 0 -0.04; 0.04 -robot.thigh_length 0.04], ...
      'cyl', [0 0 -0.04; 0 0 0.04], 0.04, ...
      { 'colour', [0.1 0.3 0.6]}};
robot.appearance.body{5} = ...
    { 'box', [-0.04 0 -0.04; 0.04 -robot.calf_length 0.04], ...
      'cyl', [0 0 -0.04; 0 0 0.04], 0.06, ...
      'cyl', [0 -robot.calf_length -0.04; 0 -robot.calf_length 0.04], 0.00, ...
      { 'colour', [0.1 0.3 0.6]}};

robot.camera.up = [0 1 0];
robot.camera.direction = [0.0  0.7 2.1];
%robot.camera.body = 1;

% Robot links
robot.link{1} = [0; robot.calf_length];
robot.link{2} = [0; robot.thigh_length];
robot.link{3} = [0; robot.torso_length];
robot.link{4} = [0; -robot.calf_length];
robot.link{5} = [0; -robot.calf_length];

% ---------- Other constants ---------- %
robot.nj_act = 5; %4;
robot.act_coords = [1 2 3 4 5]; %[2 3 4 5];
robot.nq = 5;
robot.nv = 5;

robot.leg_1 = 1;
robot.leg_2 = 5;
robot.stance = robot.leg_1;
robot.swing = robot.leg_2;

robot.foot_r = [0;-robot.calf_length];

robot.torque_lims = (10)*[300; 300; 300; 300; 300]; %10*2*[300; 300; 300; 300; 300]; %[100; 100; 100; 100];
robot.joint_bounds_upper = [2*pi; pi/2; pi/2; 2*pi; 2*pi];
robot.joint_bounds_lower = [-2*pi; 0; -pi/2; -2*pi; -2*pi]; %[-2*pi; -2*pi; -2*pi; -2*pi; -2*pi];
robot.joint_vel_bounds = 10*[3.5; 3.5; 3.5; 3.5; 3.5];
% ---------- Global position ---------- %
robot.torso_pos = [0;0];
robot.torso_vel = [0;0];
end
