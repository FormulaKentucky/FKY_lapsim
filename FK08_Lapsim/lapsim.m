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
eqn = (mu*((m*g)+(0.5*rho*a*cl*v^2)))^2 == (((cr*((m*g) + (0.5*rho*a*cl*v^2))) + (0.5*rho*a*cd*v^2))^2) + ((m*v^2)/r)^2
