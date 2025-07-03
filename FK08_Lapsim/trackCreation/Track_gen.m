%%Track generator for Formula Kentucky Aero Subsystem
%% Written by William Trussell and Eli Cossell. Credit to:
% Are Mjaavatten (sector radius and length calculations)
% Victor Cortes Abad (Laptime simulator: mass-point quasi-steady state)
% MATLAB Documentation
% And John Psyck (thanks for all the great work you've done)

clear
clc
close all hidden
%% Use prompt to import track data
prompt = {'Track File Name (without extension):', ...
          'Desired Sector Length (m):', ...
          'Track Altitude (m):'}; 
% title of prompt
dlgtitle = 'Inputs';

% size of prompt text box
fieldsize = [1 40; 1 40; 1 40];

% default values
definput = {'2024_endurance', '6', ''};

%displays prompt
answer = inputdlg(prompt, dlgtitle, fieldsize, definput);

% Cancel check
if isempty(answer)
    error('User cancelled input dialog.');
end

% Extract and validate inputs
trackname = strtrim(answer{1});
if isempty(trackname)
    error('Track name is required.');
end

sector_length = str2double(answer{2});
if isnan(sector_length) || sector_length <= 0
    error('Sector length must be a positive number.');
end

altitude = str2double(answer{3});
if isnan(altitude)
    altitude = 0; % Optional: default altitude fallback
end

tic
%% Time to import data

file_ext = '.csv'; % defines file extension
filename = append(trackname, file_ext); % adds file extension to file name
filename_final = append(trackname, '_processed', file_ext); % creates a final file name
% filename_processed.csv

track_raw = readmatrix(filename); % read in raw file

% Checks to see if file is compatible
if size(track_raw,1) < 7
    error('File must have 7 columns: lat, long, alt, orgin, rotation angle, mirrored, closed')
end

origin_row_index = track_raw(1,4); % Check for origin
origin = track_raw(origin_row_index,1:3); % set origin in lat/long
[x, y] = latlon2local(track_raw(:,1), track_raw(:,2), track_raw(:,3), origin); % convert to x,y coords

track_alt = track_raw(:,3); % create altitude columns
track_array_length = size(track_raw, 1); % calc size of columns

track_xy = [x, y, track_alt, track_raw(:,4), track_raw(:,5), track_raw(:,6), track_raw(:,7)]; % track file in x,y coords
%% Rotate and align track (also check to see if closed) we rotate track to align it properly and make calculations easier
theta = track_xy(1,5); % rotation angle
R = [cosd(theta) -sind(theta); sind(theta) cosd(theta)]; % rotation matrix
rotated_xy = track_xy(:, 1:2) * R; % matrix multiplication to rotate track, new track file created
track_mirrored = track_xy(1,6); % check if track is mirrored
if track_mirrored == 1
    rotated_xy(:, 1) = -rotated_xy(:,1); % flips negative of first column 
end
track_closed = track_xy(1,7); % checks if track is closed
%% Graphs OG track (Lots of points, kinda messy)
scatter(rotated_xy(:,1),rotated_xy(:,2), 2, "red", "filled")
hold on
%% First Track Truncation Time! (Reducing number of points

rotated_xy(1,3) = 0; % set first value as 0
for i = 2:track_array_length 
    rotated_xy(i,3) = norm(rotated_xy(i,1:2) - rotated_xy(i-1,1:2)); % calculates distance from point to point
end

rotated_xy(:,4) = cumsum(rotated_xy(:,3)); %cumulative distance of track
distance_traveled = rotated_xy(:,4);      % save distance traveled
rotated_xy(:,3) = [];           % remove segment distances (don't need, just need distance traveled)

track_length = ceil(max(distance_traveled)); % find length of track
%% Truncation loop

dist = 0; % set distance as zero
n = 1;
track_trunc = []; % create new track file

for j = 1:track_array_length
    if distance_traveled(j) >= dist
        track_trunc(n, 1:2) = rotated_xy(j, 1:2); % if the distance traveled is greater than
        dist = dist + (sector_length / 2);        % dist, a sector is created and distance is integrated 
        n = n + 1;
    end
end

% Close the loop
track_trunc(n, :) = track_trunc(1, :);

% Ensure even number of points
if mod(size(track_trunc, 1), 2) ~= 0
    track_trunc(end-1, :) = [];
end
%% Plot result
scatter(track_trunc(:,1), track_trunc(:,2), 2, 'red','filled');
%% further truncate

for k = 1:(numel(track_trunc(:,1))/2)
    track_sectors(k, 1) = k;
    track_sectors(k,2:3) = track_trunc(1 + 2*(k-1),1:2);
end
%% Sector Graph (looks good, now in UK blue!)
h1 = scatter(rotated_xy(:,1), rotated_xy(:,2), 2, 'red', 'filled');  % GPS data
hold on
h2 = scatter(track_sectors(:,2), track_sectors(:,3), 8, [0 0.4470 0.7410], 'filled');  % sectors

% Aesthetics
annotation("arrow", [0.6 0.7], [0.2 0.2], "Color", [0.1490 0.5490 0.8660])
axis equal padded
grid on
title('2024 Endurance Track Map')
xlabel('Meters (x)')
ylabel('Meters (y)')

% Correct legend
legend([h1, h2], 'GPS Data', 'Sectors (colored by #)')
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

writetable(sectors, filename_final, WriteVariableNames=true);

gentime = toc;

fprintf('Track File Generated Succesfully! Time Taken: %.2fs\nFile Name: %s\n', gentime, filename_final)

