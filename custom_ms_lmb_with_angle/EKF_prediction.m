function [x_predicted, P_predicted] = EKF_prediction(Init, motion, time, sampling_time, X_estimated, P_estimated)

    F   = [eye(2) sampling_time * eye(2); zeros(2) eye(2)]; % Motion matrix
    L   = [sampling_time^2/2* eye(2); sampling_time*eye(2)]; % Motion driving matrix
    
    if time == 1    % Initialize
        longState = [Init.x; zeros(7-length(Init.x),1)];
        largeCov = zeros(7); largeCov(1:4,1:4) = Init.cov;
        imm = trackingIMM('TransitionProbabilities', 0.99);
        % Initialize the state and state covariance in terms of the first model
        initialize(imm, longState, largeCov);
%     else
%         x_predicted = F * X_estimated(:,time-1);   % State prediction
%         P_predicted = F * P_estimated(:,:,time-1) * F'+ motion.sigma^2 * L*L'; % Covariance prediction
    end
    [x_predicted, P_predicted] = predict(imm, 0.1);
end