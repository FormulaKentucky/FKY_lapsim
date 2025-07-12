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

tic

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
cd = car{2,10};
cr = car{11,8};

parameters = [mu, m, g, rho, a, cl, cd, cr];
vmax = zeros(numel(track(:,3)),1);

for i = 1:numel(track(:,3))
    parameters(9) = track{i, 3};
    vmax(i) = double(max(abs(vpa(subs(sol, vars, parameters))))); 
end

%% Create Final Track/Car Array and Begin Simulation!
sec_length(:,1) = track{:,2};
cumdist = zeros(numel(sec_length(:,1)),1);
for i = 1:numel(sec_length(:,1))
    cumdist(i) = sum(sec_length(1:i,1));
end

%% Compute Precursor Speeds (Accelerating)
vini = 18.429;
ven = zeros(numel(sec_length(:,1)),1);
vex = zeros(numel(sec_length(:,1)),1);
%vcompi = zeros(numel(sec_length(:,1)),1);
ven(1) = vini;


for i = 1:numel(sec_length(:,1))
    accel = interp1(car{:,1},car{:,3},ven(i));
    vex(i) = ven(i) + (accel * sec_length(i) / ven(i));
    if i == numel(sec_length(:,1))
        break
    end
if vex(i) > vmax(i)
    ven(i+1) = vmax(i); % clamp to cornering limit
else
    ven(i+1) = vex(i); % carry forward if safe
end
end

%%Compute Definitive Speeds (Braking)
ven_def = zeros(numel(sec_length(:,1)),1); %initial definitive entry speed array
vex_def = zeros(numel(sec_length(:,1)),1); %initial definitive exit speed array
vex_des = zeros(numel(sec_length(:,1)),1); %ininital desired exit speed
ven_max = zeros(numel(sec_length(:,1)),1); %initial max entry speed
ven_def(numel(sec_length(:)) + 1) = ven(numel(sec_length(:))); %calc edge case entry speed at last sector

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

%%Telemetry data time
vel = zeros(numel(sec_length(:,1)), 1); %initial avg velocity array
for i = 1:numel(sec_length(:)) 
    vel(i,1) = (ven(i)+vex(i))/2; %calc avg velocity for each sector
end

sec_time = zeros(numel(sec_length(:,1)), 1); %initial individual sector time array
total_time = zeros(numel(sec_length(:,1)), 1); %initial total time array

for i = 1:numel(sec_length(:)) 
    sec_time(i) = sec_length(i)/vel(i); %sector time (s) = (sector length (m)) / (avg velocity (m/s))
    total_time(i) = sum(sec_time(1:i));
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

gentime = toc;

fprintf('Laptime complete! Time Taken: %.2fs.\nLap Time: %.1fs. Average Velocity: %.1fm/s. Total Distance: %.1fm.\n', gentime, sum(sec_time), mean(vel), sum(sec_length))
