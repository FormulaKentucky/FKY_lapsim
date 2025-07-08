clc
clear
close all hidden

%% Import Car/Data
fprintf('Select a vehicle file.\n')
carname = uigetfile('*.csv');
%disp('Selected: ' + carname)

fprintf('Select a track file. \n')
trackname = uigetfile('*csv');
%disp('Selected: ' + trackname)

car = readtable(carname, 'VariableNamingRule', 'preserve');
track = readtable(trackname, 'VariableNamingRule', 'preserve');

%% Maximum Cornering Velocity Lookup Table
syms mu m g rho a cl v cd r cr
eqn = (mu*((m*g)+(0.5*rho*a*cl*v^2)))^2 == (((cr*((m*g) + (0.5*rho*a*cl*v^2))) + (0.5*rho*a*cd*v^2))^2) + ((m*v^2)/r)^2;
sol = solve(eqn, v);
vars = [mu, m, g, rho, a, cl, cd, cr, r];

mu = car{4,9};
m = car{3,11};
g = car{2,11};
rho = car{1,11};
a = car{3,10};
cl = car{1,10};
cd = car{11,8};

parameters = [mu, m, g, rho, a, cl, cd, cr];
vmax = zeros(numel(track(:,3)),1);

for i = 1:numel(track(:,3))
    parameters(9) = track{i, 3};
    vmax(i) = double(max(abs(vpa(subs(sol, vars, parameters))))); 
end


