function [X_estimated, P_estimated] = EKF_update(Init, z, motion, obj, time, sampling_time, X_estimated, P_estimated, x_predicted, P_predicted, meas)
% EKF with NVC model
% customized for MS-MTT-EKF
% -------
% Inputs
% -------
% z_gnss: GNSS measurements -> [x y]: size(z_gnss) = (:,2); 
% z_obj: Anchor target measurements -> [x y]: size(z_obj) = (:,2);
% gnss_covariance:  
% Q: driving noise covariance
% time: exact time instant
% sampling_time: sampling time
% X_estimated: Estimated vehicle state -> [x y vx vy]': size(X_estimated)
%                                           = (4:ScenarioLength)
% P_estimated: Estimated vehicle covariance -> 
% -------
% Outputs
% X_estimated
% P_estimated

    z_gnss = z.val(1).val;
    z_target = z.val(2).val;
%     z_angle = z.val(3).val;
    
    
    H_gnss = [eye(2) zeros(2)];    % Generate Measurement matrix for GNSS
%     [H_target,rho_bar] = buildJacobianMatrixH(x_predicted , obj); % Jacobian Matrix (Range) for targets 
    [H_range,rho_range] = buildJacobianMatrixH(x_predicted , obj); % Jacobian Matrix (Range) for targets 
    [H_angle,delta_rho_angle] = buildJacobianMatrixH_angle(x_predicted , obj, meas); % Jacobian Matrix (Range) for targets 

    rho = [z_gnss; z_target; delta_rho_angle]; delta_rho_angle
    H = [H_gnss; H_range; H_angle]; % Generate Fused Measurement Matrix

    R = blkdiag(z.val(1).cov, z.val(2).cov, z.val(3).cov); % Generate Fused measurement noise covariance
    %update
    G = P_predicted * H' * inv(H*P_predicted*H' + R); % Calculate Kalman Gain 

    X_estimated(:,time) = x_predicted + G*(rho - [x_predicted(1:2); rho_range; zeros(size(delta_rho_angle))]); % State Update
    P_estimated(:,:,time) = P_predicted - G * H * P_predicted; % Covariance Update
end

function [H, rho_bar] = buildJacobianMatrixH(x, u) % Generate Jacobian Matrix
% x: predicted vehicle state
% u: target locations
    number_of_measurements = size(u,1);
    rho_bar = zeros(number_of_measurements,1);
    H = zeros(size(u,1), 4);
    if number_of_measurements
        for m = 1: number_of_measurements
            distance = sqrt( sum ( (u(m,1:2) - x(1:2)').^2, 2));
            H(m,1:2) = (x(1:2)' - u(m,1:2))./distance; 
            rho_bar(m) = distance;
        end
    end
end

function [H, delta_rho_bar] = buildJacobianMatrixH_angle(x, u, meas) % Generate Jacobian Matrix
% x: predicted vehicle state
% u: target locations
% meas: measured target locations
    number_of_measurements = size(u,1);
    delta_rho_bar = zeros(number_of_measurements,1);
    H = zeros(size(u,1), 4);
    if number_of_measurements
        for m = 1: number_of_measurements
            distance = sqrt( sum ( (u(m,1:2) - x(1:2)').^2, 2));
            H(m,1:2) = flip((u(m,1:2)-x(1:2)')./distance^2 .* [1 -1]);

            delta_rho_bar(m) = angleDiff(x(1:2)', u(m,1:2), meas(m,1:2));
        end
    end
end

function tet = angleDiff(x, u, m)
% x: vehicle location
% u: target location
% m: measured target location
    a = u - x; 
    b = m - x;
    % Calculate the dot product of the two vectors
    dot_product = dot(a, b);
    
    % Calculate the magnitudes of the vectors
    magnitude_a = norm(a);
    magnitude_b = norm(b);
    
    % Calculate the cosine of the angle
    cos_theta = dot_product / (magnitude_a * magnitude_b);
    
    % Calculate the angle in radians
    tet = acos(cos_theta);
    
    % Convert the angle to degrees
%     tet = rad2deg(theta_rad);
    
    % Calculate the cross product (2D vectors give a scalar result)
    cross_product_z = a(1) * b(2) - a(2) * b(1);
    
    % Determine the sign of the angle
    if cross_product_z < 0
        tet = -tet;
    end
end