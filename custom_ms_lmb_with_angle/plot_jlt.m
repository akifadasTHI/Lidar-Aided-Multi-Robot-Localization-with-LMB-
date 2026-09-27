x_min = [-5 50]; y_min = [-15 20];
color = ['b','r'];

% subplot(211)
% Plot vehicles positions
plot(X_truth.veh{1}(1,:), X_truth.veh{1}(2,:), '--', 'Color', [0.6 0.6 0]), hold on
plot(z_gnss{1}(1,t), z_gnss{1}(2,t), 'v')
plot(z_gnss{2}(1,t), z_gnss{2}(2,t), 'v')
plot(X_truth.veh{2}(1,:), X_truth.veh{2}(2,:), '--', 'Color', [0.6 0.6 0])
plot(z_gnss{2}(1,t), z_gnss{2}(2,t), 'v')
plot(X_truth.veh{2}(1,t), X_truth.veh{2}(2,t), 'v', 'Color', [0 0 0])


plot(x_estimated{1}(1,t), x_estimated{1}(2,t),'v','Color','b')
plotCovariance (P_estimated{1}(1:2,1:2,t) , x_estimated{1}(1,t) , x_estimated{1}(2,t) , 3 , 'Update')
hold on
plot(x_estimated{2}(1,t), x_estimated{2}(2,t),'v','Color', 'r')
plotCovariance (P_estimated{2}(1:2,1:2,t) , x_estimated{2}(1,t) , x_estimated{2}(2,t) , 3 , 'Update')

for s = 1: length(measurements)
    for o =  1: length(measurements{s})
        plot(measurements{s}{o}(1), measurements{s}{o}(2), 'x', 'Color', color(s)) , hold on
        if ~isnan(meas_ind{s}(o))
%             text(measurements{s}{o}(1), measurements{s}{o}(2) + 0.3, num2str(meas_ind{s}(find(o == meas_ind{s}))));
            plot([measurements{s}{o}(1) obj.loc_mu(meas_ind{s}(o),1)],...
                [measurements{s}{o}(2) obj.loc_mu(meas_ind{s}(o),2)]);

        end
    end
end

for gt = 1:5
    plot( X_truth.target(1:4:end), X_truth.target(2:4:end), 'o', 'Color', [0 0 0])
end

% Plot target locations if any
if length(obj.loc_mu)
    for o = 1: length(obj.loc_mu)
        plot(obj.loc_mu(o,1), obj.loc_mu(o,2), 'o', 'Color', 'g')
%         text(obj.loc_mu(o,1), obj.loc_mu(o,2) + 0.8, num2str(obj.loc_label(2,o)))
    end
end

distance_mat = [];
% Calculate target location rmse
for gt = 1:5
    for ob = 1:length(obj.loc_mu)
        distance_mat(ob,gt) = sqrt(sum((X_truth.target(4*(gt-1)+1:4*(gt-1)+2) - obj.loc_mu(ob,1:2)).^2,2));
    end
end
order = nan(1,5);
distance_vect = [];
for ind = 1:5
    [minn, I] = min(distance_mat);
    [minnn, II] = min(minn);
    if distance_mat(I(II), II) < 0.8
        order(ind) = I(II);
        distance_vect(end+1) = (distance_mat(I(II), II));
    end
    distance_mat(:,II) = [];
end

rmse_target = sqrt(mean(distance_vect.^2));


title("Time: " + num2str(t) + " RMSE target: " + num2str(rmse_target) + "Matched Targets: " + num2str(length(distance_vect)))
xlim(x_min); ylim(y_min);
xlabel("m"), ylabel("m")

pos_err = sqrt(mean(sum( (X_truth.veh{2}(1:2 ,t) - x_estimated{2}(1:2,t)).^2, 1)));
current_axis = gca;
current_axis.Position(3) = 0.6;
annotation('textbox', [0.75, 0.2, 0.2, 0.6], 'String', "Pos Error of Veh2: " + num2str(pos_err) + "m ")

pause(0.01);

function rmse = rmse_cal(x_est, x_true, t)
    rmse = sqrt(mean(sum( (x_true(1:2 ,1:t) - x_est(1:2,1:t)).^2, 1)));
end