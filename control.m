%% This is control function:
function [u, x_des] = control(t, state, trajectory, controller)

% Extract state
x1 = state(1); 
x2 = state(2); 
x3 = state(3); 
x = [x1, x2, x3];
dx1 = state(4); 
dx2 = state(5);
dx3 = state(6);
dx = [dx1, dx2, dx3];
%% Desired Trajectory Generation
% Extract Sine wave parameters from trajectory
A = trajectory.A ;
A0 = trajectory.A0;
nu = trajectory.nu;
%% Trajectory Movement

% for i = 1:3
%     x_des(:,i) = A(i)*sin(2*pi*nu(i)*t(:)) + A0(i);
%     dx_des(:,i) = A(i)*2*pi*nu(i)*cos(2*pi*nu(i)*t(:));
%     ddx_des(:,i) = -A(i)*(2*pi*nu(i))^2*sin(2*pi*nu(i)*t(:)); 
% end

%% Constand position (PTP Movement)
for i = 1:3
    x_des(:,i) = i;
    dx_des(:,i) = i-1;
    ddx_des(:,i) = i-2; 
end

%% Calculating control
%Find state errors
e = x_des - x;
de = dx_des - dx;
% Extract PD gains
k1 = controller.k1;
k2 = controller.k2;

L11 = 0.324; L12 = 0.312; L2 = 1.075; L31 = 0.225; c = 0.20;
p = 2700; %density (aluminium)

m1 = 0.7*(L11+L12*c*c)*p;
m2 = 0.7*(L2*c*c)*p;
m3 = 0.7*(L31*c*c)*p;

% m1 = (L11+L12*c*c)*p;
% m2 = (L2*c*c)*p;
% m3 = (L31*c*c)*p;

[Ixx1, Iyy1, Izz1]=Inertia_Tensor(L11, L12, c);
Ixy1 = 0;
Iyz1 = 0;
Ixz1 = 0;

[Ixx2, Iyy2, Izz2]=Inertia_Tensor(L2, c, c);
Ixy2 = 0;
Iyz2 = 0;
Ixz2= 0;
[Ixx3, Iyy3, Izz3]=Inertia_Tensor(L31, c, c);
Ixy3 = 0;
Iyz3 = 0;
Ixz3= 0;
alpha = [-pi/2 0 -pi/2];
R = SerialLink([
Revolute('d', L11, 'a', L12, 'alpha', alpha(1), 'm', m1, 'r', [-L12/2 -L11/2 0], 'I', [Ixx1 Iyy1 Izz1 Ixy1 Iyz1 Ixz1], 'B', 0, 'G', 0, 'Jm', 0, 'standard')
Revolute('d', 0, 'a', L2, 'alpha', alpha(2), 'm', m2, 'r', [-L2/2 0 0], 'I', [Ixx2 Iyy2 Izz2 Ixy2 Iyz2 Ixz2], 'B', 0, 'G', 0, 'Jm', 0, 'standard')
Revolute('d', 0, 'a', L31, 'alpha', alpha(3), 'm', m3, 'r', [-L31/2 0 0], 'I', [Ixx3 Iyy3 Izz3 Ixy3 Iyz3 Ixz3], 'B', 0, 'G', 0, 'Jm', 0, 'standard')
], ...
'name', 'three link (part of FANUC R-2000iC/165F)', ...
'comment', 'from Sevostianov Ilia');

D = R.inertia([x1 x2 x3]);

C = R.coriolis([x1 x2 x3], [dx1, dx2, dx3]);

G = R.gravload([x1 x2 x3]);

% Find nonlinear function
beta = C*dx'+G';
   
% Find ID control
%% PD Control
%u = k1*e';
% u = k1*e'+k2*de';
% u = k1*e'+k2*de'+ G';
  %% FeedBack Linearization
  % Calculate acceleration-like feedback and feedforward
% u_star = k1*e' + k2*de' + ddx_des' ;
% u = D*u_star + beta;
%% No control
%  u = 0;
 %% Robust Control
lambda = controller.lambda;
z = de' + lambda*e'; % Define a sliding surface
rho = 100;
if norm(z) == 0 % Find additional acceleration-like term 
    w = 0;
else 
    w = rho*z/norm(z);
end  
u_star = k1*e' + k2*de' + ddx_des' + w;
u = D*u_star + beta;
end