clc, close, clear 
set(0,'DefaultTextFontSize',10);
set(0,'DefaultAxesFontSize',10);
set(0,'DefaultLineLineWidth',2);clc
set(0,'DefaultTextInterpreter','latex')

% Joint Localization and Tracking 
% Objects with leveraged certainity are exploited as stationary anchors

MC =  1; count  = 0;
simulation_time = 600;      % Total duration of scenario
time_EKF_solo   = 200;           % EKF-alone duration

rmse_mc = nan(4,MC);
for mc = 1:MC
    clearvars -except MC mc count simulation_time time_EKF_solo rmse_mc
    diverged = 0;
    tic;
    % Initialize Filter parameters and Import data
    initialize;
    for t = 1:simulation_time 
        if t <= time_EKF_solo 
        % RUN KF alone for vehicle 1 and 2         
              
            % Run EKF for Veh1
            z.val(1).cov = R_gnss_1;
            z.val(1).val = z_gnss{1}(:,t);
            [X_pred, P_pred] = EKF_prediction(Init1, motion, t, sampling_time, ...
                                            x_estimated{1}, P_estimated{1});
            [x_estimated{1}, P_estimated{1}] = EKF_update(Init1, z, motion, obj.loc_mu, t,...
                                                    sampling_time, x_estimated{1},...
                                                    P_estimated{1}, X_pred, P_pred, meas_obj{1});
            
            % Run EKF for Veh2
            z.val(1).cov = R_gnss_2;
            z.val(1).val = z_gnss{2}(:,t);
            [X_pred, P_pred] = EKF_prediction(Init2,motion, t,...
                                            sampling_time, x_estimated{2},...
                                            P_estimated{2});
            [x_estimated{2}, P_estimated{2}] = EKF_update(Init2, z, motion, obj.loc_mu, t,....
                                                   sampling_time, x_estimated{2},...
                                                   P_estimated{2}, X_pred, P_pred, meas_obj{1});
%             if t >= 200
%                 plot_traj;
%             end
        else % Run LMBM using state from previous loop
 
            % Predict Veh 1
            [X_pred1, P_pred1] = EKF_prediction(Init1, motion, t,...
                                            sampling_time, x_estimated{1},...
                                            P_estimated{1});
            % Predict Veh2
            [X_pred2, P_pred2] = EKF_prediction(Init2,motion, t,...
                                            sampling_time, x_estimated{2},...
                                            P_estimated{2});
            z_v_single{1} = X_pred1(1:2);   z_v_single{2} = X_pred2(1:2); 
            % -------------------------------------- RSU
            % Project targets from Loc to Glob
            [z_t] = loc2glob_target(z_t, z_v_single, t);
            
            % --------------------------------------
            % Generate measurements cell
            % to run prewritten MS-LMB filter
            term1 = mat2cell(z_t.glob{1}{t}', 2, [ones(1, size(z_t.glob{1}{t}, 1), 1)])';
            term2 = mat2cell(z_t.glob{2}{t}', 2, [ones(1, size(z_t.glob{2}{t}, 1), 1)])';
            measurements = {term1; term2};
        
            % --------------------------------------
            % Run MS-LMBM Filter
            % Input: Model parameters and Measurements at time k
            % Output: Estimated States cell containing all LMBM estimations
            if t == time_EKF_solo+1 % Initialize LMBM filter
                lmbm_initialize;
            end
            
            [stateEstimates, objects, hypotheses] = runMultisensorLmbmFilter(model, measurements, t-time_EKF_solo, stateEstimates, objects, hypotheses);
            
            % Extract target locations
            obj.loc_mu = cell2mat(stateEstimates.mu{t-time_EKF_solo})';
            obj.loc_label = stateEstimates.labels{t-time_EKF_solo};
            
            meas_ind{1} = tar2meas(obj.loc_mu, measurements{1}, 1.5); % Target - Measurement Association
            meas_ind{2} = tar2meas(obj.loc_mu, measurements{2}, 1.5); % Target - Measurement Association
 
            if ~isempty(obj.loc_mu)     % Generate Object Range and Angle Measurements
                % Calculate range measurement (Range works fine)
                [sorted_ind, I] = sort(meas_ind{1}); 
                z_target_range{1} = sqrt ( sum ((reshape(cell2mat(measurements{1}(I(1:sum(~isnan(sorted_ind))))), 2, [])' - z_gnss{1}(:,t)').^2, 2));
                
                % Calculate angle measurement
                meas_obj{1} = reshape(cell2mat(measurements{1}(I(1:sum(~isnan(sorted_ind))))), 2, [])';
                dx = meas_obj{1}(:,1) - X_pred1(1); % measurement loc according to car X
                dy = meas_obj{1}(:,2) - X_pred1(2); % measurement loc according to car Y
                z_target_angle{1} = atan2(dy, dx); % Calculate Angle in rad [-pi, pi]     

                [sorted_ind, I] = sort(meas_ind{2});
                z_target_range{2} = sqrt(sum((reshape(cell2mat(measurements{2}(I(1:sum(~isnan(sorted_ind))))), 2, [])' - z_gnss{2}(:,t)').^2, 2));
                meas_obj{2} = reshape(cell2mat(measurements{2}(I(1:sum(~isnan(sorted_ind))))), 2, [])';
                dx = meas_obj{2}(:,1) - X_pred2(1); % measurement loc according to car X
                dy = meas_obj{2}(:,2) - X_pred2(2); % measurement loc according to car Y
                z_target_angle{2} = atan2(dy, dx); % Calculate Angle in rad [-pi, pi]            
            else 
                z_target_range{1} = []; z_target_range{2} = [];
            end

            sigma_target_1 = 5;   sigma_target_ang_1 = 5; % Basically, do not update the loc of first vehicle
            R_target_1 = sigma_target_1^2 * eye(size(z_target_range{1},1));
            R_target_ang_1 = sigma_target_ang_1^2 * eye(size(z_target_range{1},1));
                
            sigma_target_2 = .1;  sigma_target_ang_2 = 0.08; 
            R_target_2 = sigma_target_2^2 * eye(size(z_target_range{2},1));
            R_target_ang_2 = sigma_target_ang_2^2 * eye(size(z_target_range{2},1));

            % Update Vehicle 1
            z.val(1).cov = R_gnss_1;
            z.val(1).val = z_gnss{1}(:,t);
            z.val(2).cov = R_target_1;
            z.val(2).val = z_target_range{1};
            z.val(3).cov = R_target_ang_1;
            z.val(3).val = z_target_angle{1}';

            [x_estimated{1}, P_estimated{1}] = EKF_update(Init1, z, motion, obj.loc_mu(1:size(z_target_range{1},1),:),...
                                        t, sampling_time, x_estimated{1},...
                                        P_estimated{1}, X_pred1, P_pred1, meas_obj{1});
            % Update Vehicle 2 
            z.val(1).cov = R_gnss_2;
            z.val(1).val = z_gnss{2}(:,t);
            z.val(2).cov = R_target_2;
            z.val(2).val = z_target_range{2};
            z.val(3).cov = R_target_ang_2;
            z.val(3).val = z_target_angle{2}';

            [x_estimated{2}, P_estimated{2}] = EKF_update(Init2, z, motion, obj.loc_mu(1:size(z_target_range{2},1),:),...
                                        t, sampling_time, x_estimated{2},...
                                        P_estimated{2}, X_pred2, P_pred2, meas_obj{2});
            if t >= 20 
                plot_jlt;
                clf
            end
%             plot_jlt;
%             clf
%             disp("Total number of objects: " + num2str(size(obj.loc_mu,1)));
        end
        rmse{1}(t,1) = rmse_cal(x_estimated{1}, X_truth.veh{1}, t);
        rmse{1}(t,2) = rmse_cal(z_gnss{1}, X_truth.veh{1},  t);
        rmse{2}(t,1) = rmse_cal(x_estimated{2}, X_truth.veh{2}, t);
        rmse{2}(t,2) = rmse_cal(z_gnss{2}, X_truth.veh{2},  t);
        if rmse{1}(t,1) > 3 || rmse{2}(t,1) > 3 || rmse{1}(t,2) > 3 || rmse{2}(t,2) > 3 
    %         error("JLT diverged at time " + num2str(t))
            disp("Diverged")
            diverged = 1;
            break
        end
    end
    toc;
    if ~diverged
        count = count + 1;
        rmse_mc(1,count) = mean(rmse{1}(time_EKF_solo:simulation_time,1));
        rmse_mc(2,count) = mean(rmse{1}(time_EKF_solo:simulation_time,2));
        rmse_mc(3,count) = mean(rmse{2}(time_EKF_solo:simulation_time,1));
        rmse_mc(4,count) = mean(rmse{2}(time_EKF_solo:simulation_time,2));
        
        figure
        cdfplot(rmse{2}(203:314,1))
        hold on 
        cdfplot(rmse{2}(203:314,2))
        xlabel("Position Error [m]")
        legend("CSP", "GNSS-Bias")
%         clf

        disp("MC: " + num2str(mc) + "/" + num2str(MC))
    end
%     disp("Processing rate: ", num2str(Tend - tStart)/simulation_time);
    % Calculate Positioning Errors
end

% disp("Positioning Error of Veh1:    Bias-Compansated: " + mean(rmse_mc{1}(time_EKF_solo:simulation_time,2)) + "m         JLT: " + mean(rmse_mc{1}(time_EKF_solo:simulation_time,1)) + "m")
% disp("Positioning Error of Veh2:    Bias-Compansated: " + mean(rmse_mc{2}(time_EKF_solo:simulation_time,2)) + "m         JLT: " + mean(rmse_mc{2}(time_EKF_solo:simulation_time,1)) + "m")

disp("Positioning Error of Veh1:    Bias-Compansated: " + num2str(nanmean(rmse_mc(2,:))) + "m         JLT: " + nanmean(rmse_mc(1,:)) + "m")
disp("Positioning Error of Veh2:    Bias-Compansated: " + nanmean(rmse_mc(4,:)) + "m         JLT: " + nanmean(rmse_mc(3,:)) + "m")

function rmse = rmse_cal(x_est, x_true, t)
    rmse = sqrt(mean(sum( (x_true(1:2, t) - x_est(1:2, t)).^2, 1)));
end

