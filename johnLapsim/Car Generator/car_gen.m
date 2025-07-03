%% Car Data Generator
%% Written by John Psyck (2025), with a lot of credit to: 
% Tractive Force Graphs, Skitter Yaeger
% MATLAB Documentation
% and Formula Kentucky, for allowing me to do projects like this.

clc
clear
close all hidden

%% Name... This... Car!
prompt_sus = {'Car Name (without extension):'};
dlgtitle = 'Car Name';
fieldsize = [1 40;];
definput = {'FK07'};
car_name = inputdlg(prompt_sus,dlgtitle,fieldsize,definput);

%% Input Engine Info!
engine_input = questdlg('Please select an engine:', ...
	'Engine Selection', ...
	'CBR600RR','KTM690','CR450F','CBR600RR');
switch engine_input
    case 'CBR600RR'
        disp([engine_input ' Selected.'])
        engine = 'cbr'; % name of engine, as shown in its file, without extensions (cbr_dat, comes in as cbr, for example)
    case 'KTM690'
        disp([engine_input ' Selected.'])
        engine = 'ktm';
    case 'CR450F'
        disp(' Selected.')
        engine = 'crf';
end

%% Input Suspension Info!
prompt_sus = {'Car Mass (kg):','Driver Mass (kg):', 'Tire Diameter (in):', 'Tire Friction Coefficient:',...
    'Swept Rotor Diameter (in):', 'Brake Caliper Area (in^2):', 'Brake Caliper Pistons:', 'Max. Braking Pressure (PSI):', 'Brake Pad Friction Coefficient:'};
dlgtitle = 'Suspension Inputs';
fieldsize = [1 40; 1 40; 1 40; 1 40; 1 40; 1 40; 1 40; 1 40; 1 40;];
definput = {'154','60','16','1.4','6.6','0.79','2','2000','0.5'};
answer_sus = inputdlg(prompt_sus,dlgtitle,fieldsize,definput);
car_mass = str2double(answer_sus{1});
driver_mass = str2double(answer_sus{2});
tire_d_eng = str2double(answer_sus{3});
tire_mu = str2double(answer_sus{4});
rotor_d_eng = str2double(answer_sus{5});
caliper_a_eng = str2double(answer_sus{6});
pistons = str2double(answer_sus{7});
brake_pressure_eng = str2double(answer_sus{8});
brake_mu = str2double(answer_sus{9});
tire_r=tire_d_eng * 0.0127; % converts tire radius from (in) to (m)
tire_d = 2 * tire_r; % calculates tire diameter (m)
tire_c = pi * tire_d; % calculates tire circumference (m)
rotor_r = rotor_d_eng * 0.0127; % converts rotor radius from (in) to (m)
caliper_a = caliper_a_eng / 1550; % converts caliper area from (in^2) to (m^2)
brake_pressure = brake_pressure_eng * 6895; % converts brake pressure from (psi) to (pa)

%% Input Aerodynamic Info!
prompt_aero = {'Lift Coefficient:','Drag Coefficient:', 'Frontal Area (drag and lift) (m^2)', 'Aero Mass (kg):'};
dlgtitle = 'Aero Inputs';
fieldsize = [1 40; 1 40; 1 40; 1 40];
definput = {'0.24','0.55', '0.75', '10'};
answer_aero = inputdlg(prompt_aero,dlgtitle,fieldsize,definput);
cl = str2double(answer_aero{1}); % aero package coefficient of lift
cd = str2double(answer_aero{2}); % aero package coefficient of drag
fr_area = str2double(answer_aero{3}); % aero package frontal area (m^2)
aero_mass = str2double(answer_aero{4}); % total mass of aero package (kg)

tic % start a timer, because why not

%% Importing Engine and Drivetrain Data!
file_suff = '_dat';
file_ext = '.csv';
engine_filename = append(engine, file_suff, file_ext); % file name for engine data
filename_finished = append(car_name{1}, '_', engine_input, file_ext); % define file name for finished car data file
engine_data = readmatrix(engine_filename); % take in csv of engine data sheet

rpm = engine_data(:,1)'; % import rpm data, must include points up to redline of engine
redline = max(rpm); % find redline from engine data
trq = engine_data(:,2)'; % import torque curve (Nm)
gear_ct = sum(~isnan(engine_data(:,3))); % count gears in engine
gear = engine_data(1:gear_ct,3)'; % import gearing data (not including primary or final drive ratios)
p_drive = engine_data(1,4); % import primary drive ratio (internal to engine, NOT sprocket ratio)
f_drive = engine_data(1,5); % import final drive ratio (sprocket ratio)
engine_mass = engine_data(1,6); % import engine mass (kg)
shift_t = engine_data(1,7); % import shift times, torque-torque
cr = engine_data(1,8); % import coefficient of rolling resistance, as measured

%% Pre-Calculations and Constants
mass = car_mass + driver_mass + engine_mass + aero_mass; % calculates total system mass (kg)
grav = 9.81; % accel due to gravity (m/s^2)
rho = 1.293; % air density at sea level (or chosen altitude) (kg/m^3)
pad_force_max = brake_mu * brake_pressure * caliper_a * pistons; % maximum braking force applied per piston (N)
brake_force_max = 4 * pad_force_max * (rotor_r/tire_r); % maximum braking force at wheel(s) (N). note that 4 is used here to describe all 4 wheels braking.

%% Generate Torque Curve!
rpm_range = 0:100:max(rpm);
trq_disp = interp1(rpm,trq,rpm_range,'spline');
plot(rpm,trq,'o',rpm_range,trq_disp,':.');
xlim([0 max(rpm)]);
title([engine_input, ' Torque Curve']);
xlabel('RPM');
ylabel('Torque (N-m)')
hold off

%% Generate Normal Force Table
vmax = (redline/(f_drive*p_drive*gear(gear_ct))) * tire_c / 60; % calculate maximum gear-limited velocity
vel = linspace(0, vmax)'; % generate pre-defined velocity table from 0 to maximum speed (m/s)
f_norm = zeros(1, numel(vel)); % array creation
for i =1:numel(vel)
    f_norm(i) = mass*grav + 0.5*rho*fr_area*cl*vel(i)^2; % calculate normal force on car based on mass and aero loading
end

%% Generate Drag/Rolling Resistance Force Table
f_drag = zeros(1, numel(vel)); % array creation
for i =1:numel(vel)
    f_drag(i) = f_norm(i)*cr + 0.5*rho*fr_area*cd*vel(i)^2; % calculate normal force on car based on mass and aero loading
end

%% Generate Forward Tractive Force Table
rpm_table = zeros(numel(vel), (gear_ct)); % generate array to store rpm values for each gear
trac_table = zeros(numel(vel), (gear_ct)); % generate array to store tractive force values for each gear
f_trac = zeros(numel(vel), 2); % generate array to store tractive force and gear values, based on speed!
for i = 1:gear_ct
    for j = 1:numel(vel)
        rpm_table(j, i) = (60*vel(j) / tire_c) * f_drive * p_drive * gear(i); % loop fills all 6 gear accel rows with their corresponding engine rpm
        trac_table(j, i) = interp1(rpm,trq,rpm_table(j,i),'spline')*(f_drive*gear(i))/tire_r; % calculate forward tractive force given an rpm
        if trac_table(j, i) > (f_norm(i) * tire_mu) % check if tractive drive force is greater than available traction. if so, clamp value.
            trac_table(j, i) = f_norm(i) * tire_mu;
        end
        if rpm_table(j, i) > redline % delete all values where rpm is invalid (over redline)
            rpm_table(j, i) = NaN;
            trac_table(j, i) = NaN;
        end
        if trac_table(j, i) == max(trac_table(j, 1:gear_ct)) % store best gear value in tractive force table
            f_trac(j, 2) = i;
        end
    end
end

for i = 1:numel(vel) % generate tractive force table
    f_trac(i,1) = max(trac_table(i,:));
end

%% Generate Tractive Force Graph
figure
gear_legend = cell(gear_ct+1,1);
for i = 1:gear_ct
    plot(vel, trac_table(:,i))
    gear_legend{i}=append('Gear: ', num2str(i));
    hold on
end
gear_legend{gear_ct+1} = 'Available Traction';
plot(vel, f_norm.*tire_mu)
gear_legend{gear_ct+2} = 'Drag Force';
plot(vel, f_drag)
legend(gear_legend)
title([car_name{1}, ' w/ ', engine_input, ' Tractive Forces against Velocity']);
xlim([0 max(vel)]);
xlabel('Speed (m/s)');
ylabel('Tractive Force (N)')
hold off

%% Generate Rearward Tractive Force Table
f_brake = zeros(numel(vel), 1);
for i = 1:numel(vel)
    if brake_force_max > f_norm(i) * tire_mu % check to see if braking is traction limited
        f_brake(i) = (f_norm(i) * tire_mu) + f_drag(i); % set brake force to traction limit if needed
    else
        f_brake(i) = brake_force_max + f_drag(i); % set brake force to maximum if not traction limited
    end
end

%% Generate Accel/Decel Lookup table
ax = zeros(numel(vel), 5);
ax(:, 1) = vel; % vel
ax(:,2) = f_norm;
for i = 1:numel(vel)
    ax(i,3) = (f_trac(i,1) - f_drag(i))/mass; % forward acceleration table
    ax(i,4) = (f_brake(i) + f_drag(i))/mass; % rearward acceleration table
end
ax(:,5) = f_trac(:,2); % optimum gear based on vel

%% Graph Max and Min Accel Values vs. Velocity
figure
plot(ax(:,1), ax(:,3))
hold on
plot(ax(:,1), ax(:,4))
title([car_name{1}, ' w/ ', engine_input, ' Acceleration Capabilities']);
legend('Acceleration (Tractive)', 'Deceleration (Braking)')
xlim([0 max(vel)]);
xlabel('Speed (m/s)');
ylabel('Acceleration (m/s^2)')

%% Generate Final Car File for Simulation!
car_file = ax; % carry over accel lookup table
car_file(1:numel(rpm),6) = rpm'; % carry over engine info
car_file(1:numel(trq),7) = trq';
car_file(1:gear_ct,8) = gear(:)';
car_file(1+gear_ct,8) = p_drive;
car_file(2+gear_ct,8) = f_drive;
car_file(3+gear_ct,8) = engine_mass;
car_file(4+gear_ct,8) = shift_t;
car_file(5+gear_ct,8) = cr;
for i = 1:numel(answer_sus) % carry over susp info
    car_file(i,9) = str2double(answer_sus{i});
end
for i = 1:numel(answer_aero) % carry over aero info
    car_file(i,10) = str2double(answer_aero{i});
end
car_file(1, 11) = rho; % carry over density and gravity for ease of use in lapsim
car_file(2, 11) = grav;
car_file(3,11) = mass;
car_file_table=table(car_file(:,1), car_file(:,2), car_file(:,3), car_file(:,4), ...
    car_file(:,5), car_file(:,6), car_file(:,7), car_file(:,8), car_file(:,9), car_file(:,10), car_file(:,11),...
    'VariableNames', ...
    ["Velocity (m/s)","Normal Force (N)","Ax (m/s^2)","Abx (m/s^2)", "Gear", "RPM", "Torque (N-m)", ...
    "Drivetrain Info", "Susp Info", "Aero Info",  "Physics Info"]);

writetable(car_file_table, filename_finished)

gentime = toc;
fprintf('Car File Generated Succesfully! Time Taken: %.2fs\nFile Name: %s\n', gentime, filename_finished)