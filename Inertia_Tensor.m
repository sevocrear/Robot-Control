function [Ixx, Iyy, Izz] = Inertia_Tensor(a, b, c)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
   p = 2700; %aluminium
   Ixx = p*a*b*c*(b^2+c^2)/12;
   Iyy = p*a*b*c*(c^2+c^2)/12;
   Izz = p*a*b*c*(b^2+a^2)/12;
end

