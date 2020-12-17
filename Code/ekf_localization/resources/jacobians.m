% 

clear
clc
close all

%%
% compute_jacobians.m script
syms x0 y0 z0 y p r real % robot 6D pose
% syms x0c y0c z0c yc pc rc real % sensor 6D pose on robot
syms xi yi zi Xi real % landmakr absolute 3D point
syms xi_ yi_ zi_ real % landmark relative 3D point

state_variables = [x0 y0 z0 y];

%%
% Homog. matrix for the robot pose:
R_x = [ cos(y)*cos(p) cos(y)*sin(p)*sin(r)-sin(y)*cos(r) cos(y)*sin(p)*cos(r)+sin(y)*sin(r) x0;
sin(y)*cos(p) sin(y)*sin(p)*sin(r)+cos(y)*cos(r) sin(y)*sin(p)*cos(r)-cos(y)*sin(r) y0;
-sin(p) cos(p)*sin(r) cos(p)*cos(r) z0;
0 0 0 1];


yc = -pi/2;
pc = 0;
rc= -pi;
% R_xc = [cos(yc)*cos(pc) cos(yc)*sin(pc)*sin(rc)-sin(yc)*cos(rc) cos(yc)*sin(pc)*cos(rc)+sin(yc)*sin(rc) 0;
% sin(yc)*cos(pc) sin(yc)*sin(pc)*sin(rc)+cos(yc)*cos(rc) sin(yc)*sin(pc)*cos(rc)-cos(yc)*sin(rc) 0;
% -sin(pc) cos(pc)*sin(rc) cos(pc)*cos(rc) 0;
% 0 0 0 1];

R_xc = [ 0, -1, 0, 0; -1, 0, 0, 0; 0, 0, -1, 0; 0,0,0,1];

%%
% Motion model
syms v_x v_y v_z omega_z delta_t real

T = R_x;
T(1,4) = 0;
T(2,4) = 0;
T(3,4) = 0;

global_velocity = T * [v_x; v_y; v_z; 1];
G = [x0 + delta_t * global_velocity(1); 
    y0 + delta_t * global_velocity(2); 
    z0 + delta_t * global_velocity(3); 
    y + delta_t * omega_z];

control_variables = [v_x; v_y; v_z; omega_z];

%%
% Observation model
% for poles
R_xxs_1 = simplify(inv(R_x));
Xi=[ xi; yi; zi; 1 ];

RES = R_xxs_1*Xi;
xi_ = simplify(RES(1));
yi_ = simplify(RES(2));
zi_ = simplify(RES(3));

syms H_p h_range h_yaw h_pitch J_hi_xv J_hi_yi
h_distance = sqrt(xi_^2+yi_^2);
h_azimuth = atan2(yi_, xi_);
h_elevation = atan2(zi_, sqrt( xi_^2 + yi_^2 ) );

H_p=[ h_distance ; h_azimuth ; h_elevation ];

%%
% for markers
syms xm ym zm rollm pitchm yawm real
syms H_m J_hm_xv rollm_ pitchm_ yawm_ real
syms inv_H_m real

% marker_variables = [xm; ym; zm; rollm; pitchm; yawm];
marker_variables = [xm; ym; zm];

R_m = [cos(yawm)*cos(pitchm) cos(yawm)*sin(pitchm)*sin(rollm)-sin(yawm)*cos(rollm) cos(yawm)*sin(pitchm)*cos(rollm)+sin(yawm)*sin(rollm) xm;
sin(yawm)*cos(pitchm) sin(yawm)*sin(pitchm)*sin(rollm)+cos(yawm)*cos(rollm) sin(yawm)*sin(pitchm)*cos(rollm)-cos(yawm)*sin(rollm) ym;
-sin(pitchm) cos(pitchm)*sin(rollm) cos(pitchm)*cos(rollm) zm;
0 0 0 1];

R_xxc = R_x * R_xc;
R_xxc_inv = simplify(inv(R_xxc));

% from global frame to local
H_m_local = R_xxc_inv * R_m;

xm_ = simplify(H_m_local(1, 4)); 
ym_ = simplify(H_m_local(2, 4)); 
zm_ = simplify(H_m_local(3, 4));
rollm_ = atan2(H_m_local(3,2), H_m_local(3,3));
pitchm_ = atan2( -H_m_local(3,1), sqrt(H_m_local(3,2)^2 + H_m_local(3,3)^2) );
yawm_ = atan2(H_m_local(2,1), H_m_local(1,1));

H_m = [ xm_ ; ym_ ; zm_; rollm_; pitchm_; yawm_];
% H_m = [ xm_ ; ym_ ; zm_];

% Inverse observation model: from local to global
inv_H_m = simplify(R_xxc * R_m);

ixm_ = simplify(inv_H_m(1,4));
iym_ = simplify(inv_H_m(2,4));
izm_ = simplify(inv_H_m(3,4));
irollm_ = atan2(inv_H_m(3,2), inv_H_m(3,3));
ipitchm_ = atan2( -inv_H_m(3,1), sqrt(inv_H_m(3,2)^2 + inv_H_m(3,3)^2) );
iyawm_ = atan2(inv_H_m(2,1), inv_H_m(1,1));

inv_H_m_ = [ixm_ ; iym_; izm_ ; irollm_; ipitchm_; iyawm_ ];
% inv_H_m_ = [ixm_ ; iym_; izm_ ];


%%

% Jacobians Motion model
disp('Computing jacobian of G wrt state variables');
J_g_xv=jacobian(G, state_variables);

disp('Computing jacobian of G wrt control variables....');
J_g_u=jacobian(G, control_variables);

% Jacobians Observation model
disp('Computing jacobian of H poles wrt state variables');
J_hp_xv=jacobian(H_p, state_variables);

disp('Computing jacobian of H marker wrt state variables');
J_hm_xv = jacobian(H_m, state_variables);

disp('Computing jacobian of H marker wrt to observation variables');
J_hm_obs = jacobian(H_m, marker_variables);

% Jacobians Inverse Observation model
disp('Computing jacobian of inverse observation model wrt to state variables');
J_inv_hm_xv = jacobian(inv_H_m_, state_variables);

disp('Computing jacobian of inverse observation model wrt to observation variables');
J_inv_hm_obs = jacobian(inv_H_m_, marker_variables);

%%

% acc = [x0/2; y0/2; z0/2; y/2];
% N = R_x * acc;
%  
% jacobian(N, state_variables)
% 
% 
% %% 
% 
% sigma = [1 2 3 7;
%          1 2 3 5;
%          1 2 3 4;
%          1 2 3 3];
% 
% H_test = [9 2 3 4;
%           8 2 3 4;
%           7 2 3 4];
% 
% Q = [1 2 7;
%      1 2 3;
%      1 2 0];
%  
%  S = H_test * sigma * transpose(H_test) + Q;
%  K = sigma * transpose(H_test) * S;
%  
%  innovation = [1;3;3];
%  
%  e2 = transpose(innovation) * inv(S) * innovation;
%  
%  new_sigma = (eye(7) - K * H_test) * sigma;
 
 
%%
% FUNCTIONS
function euler = rotationToEuler( R )
    roll =0;
    pitch = 0;
    yaw = 0;
    if (abs(R(3, 1)) >= 1)
        yaw = 0;
        if(R(3, 1) < 0)
            roll = atan2(R(1,2), R(1,3));
            pitch = pi/2;
        else
            roll = atan2(-R(1,2), -R(1,3));
            pitch = -pi/2;
        end
    else
        pitch = asin(R(3,1));
        roll = atan2(R(3,2)/cos(pitch), R(3,3)/cos(pitch));
        yaw = atan2(R(2,1)/cos(pitch), R(1,1)/cos(pitch));
    end
    euler = [roll; pitch; yaw];
end