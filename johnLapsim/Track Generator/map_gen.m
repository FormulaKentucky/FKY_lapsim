%% Track Map Generator
%% Written by John Psyck (2024), with a lot of credit to: 
% Are Mjaavatten (sector radius and length calculations)
% Victor Cortes Abad (Laptime simulator: mass-point quasi-steady state)
% MATLAB Documentation
% and Formula Kentucky, for allowing me to do projects like this.

clc
clear
close all hidden

%% Import Track Data - now as a prompt so anyone can use it! Wow!
prompt = {'Track File Name (without extension):','Desired Sector Length (m):', 'Track Altitude (m):'};
dlgtitle = 'Inputs';
fieldsize = [1 40; 1 40; 1 40];
definput = {'2024_endurance','6', ''};
answer = inputdlg(prompt,dlgtitle,fieldsize,definput);

tic

trackname = answer{1};
sector_length = str2double(answer{2});
altitude = str2double(answer{3});

%% Importing Data!
file_ext = '.csv';
filename = append(trackname, file_ext); % file name for track data. uses gps lat/long/alt, origin row num, rotation angle, and mirrored logical as inputs
filename_finished = append(trackname, '_processed', file_ext); % target name for finished track map file. year_name_finished.csv is standard.
track_raw = readmatrix(filename);
altitude_check = max(isnan(track_raw(:,3)));
if altitude_check ~= 0
    track_raw(:,3) = altitude;
end
origin_num = track_raw(1,4); % choose a data point to be the origin, normally done visually with track map
origin = [track_raw(origin_num,1), track_raw(origin_num,2), track_raw(origin_num, 3)]; % set origin of coordinate system
[track_raw(:,1), track_raw(:,2)] = latlon2local(track_raw(:,1), track_raw(:,2), track_raw(:,3), origin); % convert from latlong to meter coordinate system
track_array_length = numel(track_raw(:,1)); % measure number of data points for track

%% Rotating and Aligning Track! (also check if track is closed or not)
theta = track_raw(1,5); % rotation angle, in degrees ccw
R = [cosd(theta) -sind(theta); sind(theta) cosd(theta)]; % rotation matrix go brrrt
track = track_raw(:, 1:2) * R; % matrix multiplication to rotate track
track_mirrored = track_raw(1,6); % check if track is mirrored
if track_mirrored == 1
    track(:, 1) = -track(:,1);
end
track_closed = track_raw(1,7);

%% Nice Graph of Original Track Data (if wanted)
scatter(track(:,1),track(:,2), 2, "red", "filled")
hold on

%% First Track Truncation Time!
track(1,3) = 0; % initialize first distance to 0
for i = 2:track_array_length % loop to fill out distance and cumulative distance arrays
    track(i,3) = sqrt( (track(i,1)-track(i-1,1))^2 + (track(i,2)-track(i-1,2))^2 ); % calculate distance between each point
    track(i,4) = sum(track(1:i,3)); % and sum to find cumulative distance
end
track(:,3) = track(:,4); % move column left
track(:,4) = []; % delete extra column of individual distances
track_length = ceil(max(track(:,3))); % find track distance from csv (m)

dist = 0; % initialize distance traveled at 0 meters
n = 1; % initialize starting point for truncated track array

% this loop just checks if the distance traveled is greater than the sector length
% and if so, updates the new, truncated track with a point
for j = 1:track_array_length 
    if track(j,3) >= dist
        track_trunc(n, 1:2) = track(j, 1:2); % set truncated track coords to same as the original data
        n = n + 1; % increase counter
        dist = dist + (sector_length/2); % integrate distance
    end
    track_trunc(n, :) = track_trunc(1,:); % set last point to the same coordinates as the start to close loop
    if dist >= track_length && mod(numel(track_trunc(:,1)), 2) ~= 0 % check for last data point, and check for even-ness
        track_trunc(n-1,:) = []; % delete second to last row to make sure that the truncated track is divisible by two
    end
end

%% Nice Graph Overlaying the New Track! (if wanted)
% scatter(track_trunc(:,1),track_trunc(:,2), 2, "red", "filled")

%% Now, further truncate points to create sectors of three points each!
for k = 1:(numel(track_trunc(:,1))/2)
    track_sectors(k, 1) = k;
    track_sectors(k,2:3) = track_trunc(1 + 2*(k-1),1:2);
end
% track_sectors(k+1, 1) = k + 1;
% track_sectors(k+1, 2:3) = track_sectors(1, 2:3);

%% Sector Graph (looks good, now in UK blue!)
scatter(track_sectors(:,2),track_sectors(:,3), 8, track_sectors(:,1), "filled")
annotation("arrow", [0.6 0.7], [0.2 0.2], "Color", [0.1490    0.5490    0.8660])
colormap(flipud(abyss))
colorbar("eastoutside")
axis equal padded
grid on
title('2024 Endurance Track Map')
xlabel('Meters (x)')
ylabel('Meters (y)')
% legend('Sectors (colored by #)')
legend('GPS Data', 'Sectors (colored by #)')
% legend('GPS Data', 'Truncated Track', 'Sectors (colored by #)')

%% Finally, use Are Mjaavatten's code to find sector radius and arc lengths!
sec_in = track_sectors(:, 2:3);

function [R,M,k] = circumcenter(A,B,C)
% Center and radius of the circumscribed circle for the triangle ABC
%  A,B,C  3D coordinate vectors for the triangle corners
%  R      Radius
%  M      3D coordinate vector for the center
%  k      Vector of length 1/R in the direction from A towards M
%         (Curvature vector)
  D = cross(B-A,C-A);
  b = norm(A-C);
  c = norm(A-B);
  if nargout == 1
    a = norm(B-C);     % slightly faster if only R is required
    R = a*b*c/2/norm(D);
    if norm(D) == 0
      R = Inf;
    end
    return
  end
  E = cross(D,B-A);
  F = cross(D,C-A); 
  G = (b^2*E-c^2*F)/norm(D)^2/2;
  M = A + G;
  R = norm(G);  % Radius of curvature
  if R == 0
    k = G;
  elseif norm(D) == 0
    R = Inf;
    k = D;
  else
    k = G'/R^2;   % Curvature vector
  end
end

function [L,R,k] = curvature(X)
% Radius of curvature and curvature vector for 2D or 3D curve
%  [L,R,k] = curvature(X)
%   X:   2 or 3 column array of x, y (and possibly z) coordiates
%   L:   Cumulative arc length
%   R:   Radius of curvature
%   k:   Curvature vector
% The scalar curvature value is 1./R
% Version 2.6: Calculates end point values for closed curve
  N = size(X,1);
  dims = size(X,2);
  if dims == 2
    X = [X,zeros(N,1)];  % Use 3D expressions for 2D as well
  end
  L = zeros(N,1);
  R = NaN(N,1);
  k = NaN(N,3);
  for i = 2:N-1
    [R(i),~,k(i,:)] = circumcenter(X(i,:)',X(i-1,:)',X(i+1,:)');
    L(i) = L(i-1)+norm(X(i,:)-X(i-1,:));
  end
  if norm(X(1,:)-X(end,:)) < 1e-10 % Closed curve. 
    [R(1),~,k(1,:)] = circumcenter(X(end-1,:)',X(1,:)',X(2,:)');
    R(end) = R(1);
    k(end,:) = k(1,:);
    L(end) = L(end-1) + norm(X(end,:)-X(end-1,:));
  end
  i = N;
  L(i) = L(i-1)+norm(X(i,:)-X(i-1,:));
  if dims == 2
    k = k(:,1:2);
  end
end

[length_cumulative, radius, curv_vec] = curvature(sec_in); % save length (cumulative), radius, and curve vectors
length = length_cumulative; % duplicate length to make sector length array

for l = 2:k
    length(l) = length_cumulative(l) - length_cumulative(l-1); % change cumulative length array to discretized lengths per sector
end


%% Write results to a file that can be shared and used in the future!
sectors = [track_sectors(1:l-1,1) length(2:l) radius(2:l)]; % finally, define track sectors and save to file for use!
sectors(k-1, 3) = sectors(k-2,3);
sectors(1,4) = max(length_cumulative);
sectors(1,5) = theta;
sectors(1,6) = track_mirrored;
sectors(1,7) = track_closed;
sectors = array2table(sectors, 'VariableNames', {'Number', 'Sector Length (m)', 'Sector Radius (m)', 'Total Length (m)', 'Rotation (deg)', 'Mirrored', 'Closed'});

writetable(sectors, filename_finished, WriteVariableNames=true);

gentime = toc;

fprintf('Track File Generated Succesfully! Time Taken: %.2fs\nFile Name: %s\n', gentime, filename_finished)

% hold off
% figure
% plot(length_cumulative, radius)
% title('Curve Radius vs. Distance')
% xlim([0 max(length_cumulative)])
% ylim([0 1000])