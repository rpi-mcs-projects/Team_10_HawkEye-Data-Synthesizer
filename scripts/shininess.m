function blobs = shininess(cloud,bbox)

    % local function to find the distance from a point pt to a line with end points points v1 and v2
    function d = point_to_line(pt, v1, v2)
          a = [v1 - v2,0];
          b = [pt - v2,0];
          d = norm(cross(a,b)) / norm(a);
    end
    
    %% Calculate orientation angle of each point's surface
    N_pt_car = length(cloud);
    pt_specular = zeros(N_pt_car,1);

    bbox_edge = zeros(4,1); % edges of the bounding box represented as complex numbers
    bbox_coords = zeros(4,2,2); % coordinates of bbox corners
    
    bbox_coords(1,:,:) = squeeze(bbox([1,2],1:2));
    bbox_coords(2,:,:) = squeeze(bbox([1,3],1:2));
    bbox_coords(3,:,:) = squeeze(bbox([2,4],1:2));
    bbox_coords(4,:,:) = squeeze(bbox([3,4],1:2));
    for ke = 1:4
        bbox_edge(ke,:) = bbox_coords(ke,2,1)-bbox_coords(ke,1,1) + 1j*(bbox_coords(ke,2,2)-bbox_coords(ke,1,2));
    end

    % find the specularity for each point
    for kp = 1:N_pt_car

        if min(sqrt((cloud(kp,1)-bbox(1:4,1)).^2+(cloud(kp,2)-bbox(1:4,2)).^2))<0.5
            % if the point is very close to a corner of the bounding box
            pt_specular(kp) = 0;
        else
            % find the distance between the point and all 4 edges to
            % find the nearest edge
            pt_edge_dist = zeros(4,1);
            for ke = 1:4
                pt_edge_dist(ke) = point_to_line([cloud(kp,1),cloud(kp,2)], squeeze(bbox_coords(ke,1,:)).', squeeze(bbox_coords(ke,2,:)).');
            end
            [~,edge_nearest] = min(pt_edge_dist);
            % find the angle between the radar beam and the surface tangent
            % uses orientation of nearest bounding box edge as an
            % approximation of the car surface orientation
            pt_specular(kp) = abs(angle(bbox_edge(edge_nearest)) - angle(cloud(kp,1)+1j*cloud(kp,2)));
            pt_specular(kp) = abs(mod(pt_specular(kp),pi)/pi*180-90); % normal angle closer to 0 is better for reflectivity
            % find the elevation angle theta
            pt_theta = angle(sqrt(cloud(kp,1)^2+cloud(kp,2)^2)+1j*abs(cloud(kp,3)));
            pt_theta = abs(mod(pt_theta,pi)/pi*180);
            % the larger relative normal orientation is the specularity of the point
            pt_specular(kp) = max(pt_specular(kp),pt_theta);
        end
    end

    %% Blob center
    % Find the centers of the blobs
    blb_ctr = find(pt_specular==0); % all corners are selected
    blb_ctr_cart = [];
    if ~isempty(blb_ctr)
        blb_ctr_idx = datasample(blb_ctr,min(10,length(blb_ctr)));
        blb_ctr_cart = [blb_ctr_cart;cloud(blb_ctr_idx,:)];
    end

    blb_ctr = find((pt_specular>0)&(pt_specular<15));
    if ~isempty(blb_ctr)
        blb_ctr_idx = datasample(blb_ctr,min(20,length(blb_ctr)));
        blb_ctr_cart = [blb_ctr_cart;cloud(blb_ctr_idx,:)];
    end

    blb_ctr = find((pt_specular>15)&(pt_specular<25));
    if ~isempty(blb_ctr)
        blb_ctr_idx = datasample(blb_ctr,min(5,length(blb_ctr)));
        blb_ctr_cart = [blb_ctr_cart;cloud(blb_ctr_idx,:)];
    end

    if ~isempty(blb_ctr_cart) % check if we have any blobs

        blb_size = 0.3; % blob size around the center
        blobs = [];
        for kb = 1:size(blb_ctr_cart,1)
            dis_pt2blb = (cloud(:,1) - blb_ctr_cart(kb,1)).^2 + (cloud(:,2) - blb_ctr_cart(kb,2)).^2 + (cloud(:,3) - blb_ctr_cart(kb,3)).^2;
            ptInBlb = find(dis_pt2blb < blb_size^3);
            blobs = [blobs;[cloud(ptInBlb,1),cloud(ptInBlb,2),cloud(ptInBlb,3)]];
        end
        blobs = unique(blobs,'row'); % remove points that overlap multiple blobs
    end
end