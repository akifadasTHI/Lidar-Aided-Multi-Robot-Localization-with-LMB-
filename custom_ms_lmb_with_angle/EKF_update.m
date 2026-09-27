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
    z_angle = z.val(3).val;
    
    H_gnss = [eye(2) zeros(2)];    % Generate Measurement matrix GNSS
    [H_range,rho_range] = buildJacobianMatrixH(z_gnss , obj); % Jacobian Matrix (Range) for targets 
    [H_angle,rho_angle] = buildJacobianMatrixH_angle(z_gnss , obj); % Jacobian Matrix (Angle) for targets 

    rho = [z_gnss; z_target]; %rho_angle 
    H = [H_gnss; H_range; H_angle]; % Generate Fused Measurement Matrix

    R = blkdiag(z.val(1).cov, z.val(2).cov, z.val(3).cov); % Generate Fused measurement noise covariance
    %update
    G = P_predicted * H' * inv(H*P_predicted*H' + R); % Calculate Kalman Gain 

    y = z_angle' - rho_angle; % delta angle -> normalize in the next row
%     yyy = wrapToPi(y);
    X_estimated(:,time) = x_predicted + G*([rho - [x_predicted(1:2); rho_range]; atan2(sin(y), cos(y))]); % State Update
    P_estimated(:,:,time) = P_predicted - G * H * P_predicted; % Covariance Update
end

function [H, rho_bar] = buildJacobianMatrixH(x, u) % Generate Jacobian Matrix
% x: predicted vehicle state
% u: target locations
    number_of_measurements = size(u,1);
    rho_bar = zeros(number_of_measurements,1);
    H = zeros(size(u,1), 4);
    if number_of_measurements
        p = u - x(1:2)'; % target loc according to car
        for m = 1: number_of_measurements
            dx = p(m,1); dy = p(m,2);
            distance = sqrt( sum ( (u(m,1:2) - x(1:2)').^2, 2));
            H(m,1:2) = [-dx -dy]./distance; 
            rho_bar(m) = distance;
        end
    end
end

function [H, rho_bar] = buildJacobianMatrixH_angle(x, u) % Generate Jacobian Matrix
% x: predicted vehicle state
% u: target locations
    number_of_measurements = size(u,1);
    rho_bar = zeros(number_of_measurements,1);
    H = zeros(size(u,1), 4);
    if number_of_measurements
        p = u - x(1:2)'; % target loc according to car
        for m = 1: number_of_measurements
            dx = p(m,1); dy = p(m,2);
            H(m,1:2) = [dy / (dx^2 + dy^2), -dx / (dx^2 + dy^2)];

            rho_bar(m) = atan2(dy, dx);
        end
    end
end
