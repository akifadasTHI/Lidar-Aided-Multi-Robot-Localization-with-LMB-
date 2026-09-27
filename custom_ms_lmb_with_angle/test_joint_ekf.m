clc, close, clear 

vehicle = 1;
% Initialize
MC =  400;
for mc = 1:MC
    obj_loc = randi(10,7,2); % 
%     obj_loc = [];
    sampling_time = 0.1;
    x_true = [1:sampling_time:10; 1:sampling_time:10];
    
    x_estimated = zeros(4, size(x_true,2));
    P_estimated = zeros(4,4,size(x_true,2));
    motion.sigma = 0.01; % To generate motion noise covariance Q 
    
    cov_gnss = [motion.sigma^2 * eye(2), zeros(2); zeros(2,4)];
    z_obj = zeros(size(obj_loc,1),4);
    
    % Generate Measurement noise covariance
    sigma_gnss = 5;     R_gnss = sigma_gnss^2 * eye(2);
    sigma_target = 0.1;   R_target = sigma_target^2 * eye(size(obj_loc,1));
    
    Init.cov = diag([10000^2, 10000^2, 100^2, 100^2]);
   
    for t = 1: size(x_true, 2)
        z_gnss = x_true(:,t) + sigma_gnss * randn(2,1); % Generate GNSS measurements
        Init.x = [z_gnss; 0; 0]; % Initialize at measured first location
%         Init.x = [x_true(:,t); 0; 0]; 

        if ~isempty(obj_loc) 
            z_target = sqrt ( sum ((obj_loc + sigma_target * randn(size(obj_loc)) - x_true(:,t)').^2, 2));
        else 
            z_target = [];
        end

        % Fill measurement structure
        z.val(1).cov = R_gnss;
        z.val(1).val = z_gnss;
    
        z.val(2).cov = R_target;
        z.val(2).val = z_target;
    
        obj.loc = obj_loc;
    
        [x_estimated, P_estimated] = EKF(Init, z, motion, obj, t, sampling_time, x_estimated, P_estimated);
        clc
        if isnan(x_estimated(1,t))
            error("Diverged")
        end
    end
    rmse(mc) = rmse_cal(x_estimated, x_true);
end
disp(mean(rmse))

figure
plot(x_true(1,:), x_true(2,:),'.'), hold on
plot(x_estimated(1,:), x_estimated(2,:),'*')

if ~isempty(obj_loc)
    plot(obj_loc(:,1), obj_loc(:,2), 'v');
end

function rmse = rmse_cal(x_est, x_true)
    rmse = sqrt(mean(sum( (x_true(1:2 ,:) - x_est(1:2,: )).^2, 1)));
end