function [meas_ind] = tar2meas(tar, measur, threshold)
% Find which targets generate which measurements
%   [meas_ind] = tar2meas(tar, meas, threshold)
% tar: global target locations
% meas: measurement set for single vehicle
% meas_ind: indices that match measurement to targets

meas = reshape(cell2mat(measur),2, [])';

% Generate distance matrix
distance_mat = [];
% Calculate target location rmse
for m = 1:size(meas,1)
    for t = 1:size(tar,1)
        distance_mat(t,m) = sqrt(sum((tar(t,:) - meas(m,:)).^2,2));
    end
end
order = nan(1,size(meas,1));
distance_vect = [];
for ind = 1:length(tar)
    [minn, I] = min(distance_mat');
    [minnn, II] = min(minn);
    if distance_mat(II, I(II)) < threshold
        order(I(II)) = II;
        distance_vect(end+1) = (distance_mat(II, I(II)));
    end
    distance_mat(II,:) = 999*ones(1,size(order,2));
end

meas_ind = order;
end