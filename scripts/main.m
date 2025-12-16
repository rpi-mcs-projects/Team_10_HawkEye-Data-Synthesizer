%function [radar_heatmap, visible_cart_v] = new
addpath('functions');

car = load(sprintf('../CAD/CAD_model_1.mat')); % open cad model

pc = pointCloud(car.cart_v); % create point cloud from CAD
ptcld = pc.Location; % 3D coordinates of Point Cloud
lim = [min(ptcld);max(ptcld)]; % find the limits in all three dimensions
[bbox_x, bbox_y, bbox_z] = meshgrid(lim(:,1),lim(:,2),lim(:,3)); % 8 vertices of the bounding box of the point cloud
bbox = [bbox_x(:), bbox_y(:), bbox_z(:)];

%% ROTATE Point Cloud
%angle = 60;
angle = randi(360);
ar = angle/180 * pi;

rotate2d =  @(x, M) (x(:, 1:2) * M);

R = [cos(ar), -sin(ar); sin(ar), cos(ar)]; % create rotation matrix
ptcld(:,1:2) = rotate2d(ptcld, R); % rotate the point cloud
bbox(:,1:2) = rotate2d(bbox, R); % rotate the bounding box

%% Process point cloud and bounding box
% Translate the point cloud down 12.5 meters to match radar height from
% paper and back 6 meters away from radar
tranx = randi([-3000, 3000]);
%tranx = 2000;
trany = randi([0, 12000]);
%trany = 8000;
ptcld = ptcld + [tranx, trany, -1250];
bbox = bbox + [tranx, trany, -1250];
% Convert to meters
ptcld = ptcld/1000;
bbox = bbox/1000;

%% Plot Transformed point cloud
figure;
x=ptcld(:,1);
y=ptcld(:,2);
z=ptcld(:,3);
scatter3(x, y, z, 10, 'filled', 'k'); hold on;
scatter3(bbox(:,1), bbox(:,2), bbox(:,3), 'r');
% grid on;
title('Car Model');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)'); axis equal;
xlim([-3 3]);
ylim([0 12]);

%% Create occluded point cloud
occluded_pts = occlude_points(ptcld);

%% Plot occluded point cloud
figure;
x=occluded_pts(:,1);
y=occluded_pts(:,2);
z=occluded_pts(:,3);
scatter3(x, y, z, 10, 'filled', 'k'); hold on;
scatter3(bbox(:,1), bbox(:,2), bbox(:,3), 'r');
% grid on;
title('Car Model');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)'); axis equal;
xlim([-3 3]);
ylim([0 12]);

%% Simulate Specularity
spec_cloud = shininess(occluded_pts, bbox);

%% Plot Reflective point cloud
figure;
x=spec_cloud(:,1);
y=spec_cloud(:,2);
z=spec_cloud(:,3);
scatter3(x, y, z, 10, 'filled', 'k'); hold on;
scatter3(bbox(:,1), bbox(:,2), bbox(:,3), 'r');
% grid on;
title('Car Model');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)'); axis equal;
xlim([-3 3]);
ylim([0 12]);

%% CODE AFTER THIS POINT TAKEN FROM HAWKEYE GITHUB REPOSITORY, WE COULD NOT FIGURE OUT HOW TO RECREATE THIS IN TIME
%% simulate radar
signal_array = simulate_radar_signal(spec_cloud);

%% Radar signal processing, generating 3D radar heatmaps
radar_heatmap = radar_dsp(signal_array);
sz = size(radar_heatmap);
hmap_cart = zeros(sz(1),3);
[hmap_cart(:,1),hmap_cart(:,2),hmap_cart(:,3)] = sph2cart(radar_heatmap(:,1),radar_heatmap(:,2),radar_heatmap(:,3));

% Visulize the radar heatmap top view
radar_heatmap_top = squeeze(max(radar_heatmap,[],3));
figure
imagesc(radar_heatmap_top);
set(gca,'XDir','reverse')
set(gca,'YDir','normal')
colormap jet; caxis([0 1e11]);
xlabel('Range'); ylabel('Azimuth');
set(gca,'FontSize',30) % Creates an axes and sets its FontSize to 18

% Visulize the radar heatmap front view
radar_heatmap_front = squeeze(max(radar_heatmap,[],1));
figure;
imagesc(radar_heatmap_front.');
set(gca,'XDir','reverse')
colormap jet; caxis([0 1e11]);
xlabel('Azimuth'); ylabel('Elevation');
set(gca,'FontSize',30) % Creates an axes and sets its FontSize to 18



