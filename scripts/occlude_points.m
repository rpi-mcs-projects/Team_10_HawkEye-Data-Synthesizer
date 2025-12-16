function [occld] = occlude_points(cld)
% Remove occluded body of the car
    
    % convert cartesian coordinates of the point cloud to spherical coordinates
    dim = size(cld);
    N = dim(1); % number of points in point cloud
    sph_v = zeros(N,3); 
    [sph_v(:,1),sph_v(:,2),sph_v(:,3)] = cart2sph(cld(:,1),cld(:,2),cld(:,3));
    limits = [min(sph_v);max(sph_v)]; % limits in all three dimensions in the spherical coordinates            
    
    % define spherical coordinate resolution (number of angles we're
    % checking between min and max view angles)
    % Arbitrary resolutions
    phi_res = 0.2/180*pi; %angle from x axis
    theta_res = 0.5/180*pi; %angle from z axis
    rho_res = 0.02; %radius
    
    % sph_m only covers viewing angles where points exist
    sph_m_phi = limits(1,1):phi_res:limits(2,1)+phi_res;
    sph_m_theta = limits(1,2):theta_res:limits(2,2)+theta_res;
    sph_m_rho = limits(1,3):rho_res:limits(2,3)+rho_res;
    sph_m_size = [length(sph_m_phi),length(sph_m_theta),length(sph_m_rho)];
    % sph_m will contain the "value" for each spherical point
    sph_m = zeros(sph_m_size);

    % Normalizes spherical coordinates to start at (0,0,0)
    phi_m_idx = round((sph_v(:,1) - limits(1,1))/phi_res)+1;
    theta_m_idx = round((sph_v(:,2) - limits(1,2))/theta_res)+1;
    rho_m_idx = round((sph_v(:,3) - limits(1,3))/rho_res)+1;
    
    % initially sets each spherical point to be active
    for k_pt = 1:N
        sph_m(phi_m_idx(k_pt),theta_m_idx(k_pt),rho_m_idx(k_pt)) = 1;
    end

    %% Find closest point to origin at every angle; These are the points visible to the radar
    % sets all points to initially be 0
    visible_sph_m = zeros(size(sph_m));
    for kphi = 1:sph_m_size(1)
        for ktheta = 1:sph_m_size(2)
            krho = find(sph_m(kphi,ktheta,:)>0,1); % find first point at angle phi,theta with rho>0
            visible_sph_m(kphi,ktheta,krho) = sph_m(kphi,ktheta,krho); % visible points become 1
        end
    end

    visible_sph_m_idx = find(visible_sph_m); % finds the indices of visible points in visible_sph_m
    sph_v_idx = [];
    [sph_v_idx(:,1),sph_v_idx(:,2),sph_v_idx(:,3)] = ind2sub(sph_m_size,visible_sph_m_idx);
    visible_sph_v = [sph_m_phi(sph_v_idx(:,1));sph_m_theta(sph_v_idx(:,2));sph_m_rho(sph_v_idx(:,3))].';
    
    % convert spherical coords back to cartesian coords
    occld = zeros(size(visible_sph_v));
    [occld(:,1),occld(:,2),occld(:,3)] = sph2cart(visible_sph_v(:,1),visible_sph_v(:,2),visible_sph_v(:,3));

end