%% This is our equation to integrate (solve) obtained form dynamics in form:
%  D(q,dq) * ddq + beta(q,dq) = u
%  q, dq, ddq: n-dimensional vectors, of system 'positions' and their
%  derevitives
%  D(q,dq): n by n PD 'inertia' matrix
%  beta(q,dq): n-dimensional nonlinear vector function
%  u: n-dimensional control input

function dstate = dynamics(t, state, trajectory, controller)
% We choose following state: state = [x1, x2, dx1, dx2]
% with time derevitive given by: dstate = [dx1, ddx1, dx2, ddx2]

% Extract states
x1 = state(1); 
x2 = state(2); 
x3 = state(3); 
dx1 = state(4); 
dx2 = state(5);
dx3 = state(6);
dx = [dx1; dx2; dx3];

 L11 = 0.324; L12 = 0.312; L2 = 1.075; L31 = 0.225; c = 0.20;
   p = 2700; %density (aluminium)
   
   m1 = (L11+L12*c*c)*p;
   m2 = (L2*c*c)*p;
   m3 = (L31*c*c)*p;
   
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
   beta = C*dx+G';
   
 % Control input implimented as external function of state
[u, x_des] = control(t, state, trajectory, controller);  

% Equation for second order derevitive
ddx = D\(u - beta);

% Combine dx, ddx back to time derevitive of state
dstate = [dx; ddx]; 
end