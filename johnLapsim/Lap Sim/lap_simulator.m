%% FSAE Lap Simulator
%% Written by John Psyck (2025), with a lot of credit to: 
% Victor Cortes Abad (Laptime simulator: mass-point quasi-steady state)
% James Hakewill (Lap Time Simulation)
% MATLAB Documentation
% and Formula Kentucky, for allowing me to do projects like this.

clc
clear
close all hidden

%% Import Car and Track Data
fprintf('Please select a vehicle file for simulation.\n')
carname = uigetfile('*.csv');
% carname = 'FK07_CBR600RR.csv';
disp([carname ' selected.'])

fprintf('Please select a track file for simulation.\n')
trackname = uigetfile('*.csv');
% trackname = '2024_endurance_processed.csv';
disp([trackname ' selected.'])

tic % start a timer bc we cool like that baby

car = readtable(carname, 'VariableNamingRule', 'preserve');
track = readtable(trackname, 'VariableNamingRule', 'preserve');

%% Create Maximum Cornering Velocity Lookup Table
syms mu m g rho a cl v cd r cr % create symbolic system with these inputs to solve for cornering velocity
eqn = (mu*((m*g) + (0.5*rho*a*cl*v^2)))^2 == (((cr*((m*g) + (0.5*rho*a*cl*v^2))) + (0.5*rho*a*cd*v^2))^2) + ((m*v^2)/r)^2; % nasty equation, basically Ft^2 = Fx^2 + Fy^2
sol = solve(eqn, v); % magical matlab solver solves for v. neat! store solution as sol.
vars = [mu, m, g, rho, a, cl, cd, cr, r]; % setup variable array to clean up code

mu = car{4,9}; % import car data: tire mu
m = car{3,11}; % import car data: vehicle mass
g = car{2,11}; % import car physics data: accel due to gravity (m/s^2)
rho = car{1,11}; % import car physics data: air density (kg/m^3)
a = car{3,10}; % import car data: frontal area (m^2)
cl = car{1,10}; % import car data: coeff. of lift
cd = car{2,10}; % import car data: coeff. of drag
cr = car{11,8}; % import car data: coeff. of rolling resistance

params = [mu, m, g, rho, a, cl, cd, cr]; % define new array, which replaces each symbolic value with its actual numeric value, except radius!
vmax = zeros(numel(track(:,3)),1);

for i = 1:numel(track(:,3)) % evaluate all track sections maximum cornering speed
    params(9) = track{i, 3}; % read track corner radius data
    vmax(i) = double(max(abs(vpa(subs(sol, vars, params))))); % this is disgusting, but basically just finds a max cornering velocity and then converts to a useful number
end

%% Create Final Track/Car Array and Begin Simulation!
sec_length(:,1) = track{:,2}; % rip track length data into new matrix
cumdist = zeros(numel(sec_length(:,1)),1); % calculate cumulative distance and store (for displaying results)
for i = 1:numel(sec_length(:,1))
    cumdist(i) = sum(sec_length(1:i,1));
end

%% Compute Precursor Speeds (Accelerating)
vini = 18.5; % initial speed at start/stop line
ven = zeros(numel(sec_length(:,1)),1); % init entry speed array
vex = zeros(numel(sec_length(:,1)),1); % init exit speed array
vcompi = zeros(numel(sec_length(:,1)),1); % init speed comparison array
ven(1) = vini; % set initial entry speed to vini

for i = 1:numel(sec_length(:,1)) % precursor speed loop
    accel = interp1(car{:,1},car{:,3},ven(i)); % set accel for sector i to value from lookup table
    vex(i) = ven(i) + (accel * sec_length(i) / ven(i)); % calculate exit speed
    if i == numel(sec_length(:,1))
        break
    end
    if vex(i) > vmax(i) % compare calculated exit speed to maximum calculated corner speed
        ven(i+1) = vmax(i); % and clamp corner exit speed to max value if necessary
    else
        ven(i+1) = vex(i);
    end
end

%% Compute Definitive Speeds (Braking)
ven_def = zeros(numel(sec_length(:,1)),1); % init definitive entry speed array
vex_def = zeros(numel(sec_length(:,1)),1); % init definitive exit speed array
vex_des = zeros(numel(sec_length(:,1)),1); % init desired exit speed array
ven_max = zeros(numel(sec_length(:,1)),1); % init maximum entry speed array
ven_def(numel(sec_length(:)) + 1) = ven(numel(sec_length(:))); % calculate edge case entry speed at last sector

for i = 2:numel(sec_length(:)) % definitive speed loop
    j = numel(sec_length(:)) + 1 - i; % invert count from end - 1 to first sector
    decel = interp1(car{:,1},car{:,4},ven(j)); % set decel for sector j to value from lookup table
    if ven(j+1) ~= vex(j)
        vex_des(j) = ven(j+1);
        ven_max(j) = vex_des(j) - (decel * (sec_length(j))/vex_des(j));
        if ven_max(j) < ven(j)
            ven_def(j) = ven_max(j);
            vex_def(j) = ven_def(j+1);
        end
    else
        ven_def(j) = ven(j);
        vex_def(j) = vex(j);
    end
end

ven_def(numel(sec_length(:)) + 1) = []; % delete last row of array to return to total sector count

%% Generate Telemetry Data!
vel = zeros(numel(sec_length(:,1)),1); % init average velocity array
for i = 1:numel(sec_length(:)) % and calculate average velocity for each sector
    vel(i,1) = (ven(i)+vex(i))/2;
end

sec_time = zeros(numel(sec_length(:)),1); % init individual sector time array
total_time = zeros(numel(sec_length(:)),1); % init cumulative time array
for i = 1:numel(sec_length(:))
    sec_time(i) = sec_length(i) / vel(i); % calculate sector time using avg speed
    total_time(i) = sum(sec_time(1:i)); % and cumulative time too!
end

acc_act = zeros(numel(sec_length(:,1)),1); % init acceleration array
acc_pos = zeros(numel(sec_length(:,1)),1); % init possible acceleration array
for i = 1:numel(sec_length(:))
    acc_pos(i, 1) = interp1(car{:,1},car{:,3},vel(i)); % set accel for sector i to value from lookup table
end

tps = zeros(numel(sec_length(:,1)),1); % init throttle pos array
for i = 1:numel(sec_length(:)) % and calculate actual acceleration and throttle pos for each sector
    acc_act(i,1) = (vex_def(i) - ven_def(i)) / sec_time(i);
    tps(i,1) = 100 * (acc_act(i) / acc_pos(i)); % tps determined by fraction of available fwd acceleration
    if tps(i) < 0
        tps(i, 1) = 0; % set to 0 for negative accel (braking)
    elseif tps(i) > 100
            tps(i, 1) = 100;
    end
end

plot(cumdist, vel) % check speeds over track length for sanity
xlim([0 max(cumdist)])
ylim([0 max(car{:,1})])
title('Speed-Distance');
xlabel('Distance (m)');
ylabel('Speed (m/s)')

figure
plot(total_time, vel) % check speeds over lap time for sanity
xlim([0 max(total_time)])
ylim([0 max(car{:,1})])
title('Speed-Time');
xlabel('Time Elapsed (s)');
ylabel('Speed (m/s)')

figure
plot(total_time, tps) % plot tps during the lap
xlim([0 max(total_time)])
ylim([0 (1.1 * max(tps))])
title('TPS-Time');
xlabel('Time Elapsed (s)');
ylabel('Throttle Position (%)')

gentime = toc; % stop timer and then print basic info to console
fprintf('Simulation Complete! Time Taken: %.2fs.\nLap Time: %.1fs. Average Velocity: %.1fm/s. Total Distance: %.1fm.\n', gentime, sum(sec_time), mean(vel), sum(sec_length))