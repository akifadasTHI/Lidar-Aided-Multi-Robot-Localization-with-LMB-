x_min = [5 40]; y_min = [-5 25];
color = ['b','r'];

% subplot(211)
% Plot vehicles positions
plot(X_truth.veh{1}(1,:), X_truth.veh{1}(2,:), '--', 'Color', [0.6 0.6 0]), hold on
plot(X_truth.veh{2}(1,:), X_truth.veh{2}(2,:), '--', 'Color', [0.6 0.6 0])


plot(x_estimated{1}(1,t), x_estimated{1}(2,t),'v','Color','b')
plotCovariance (P_estimated{1}(1:2,1:2,t) , x_estimated{1}(1,t) , x_estimated{1}(2,t) , 3 , 'Update')
hold on
plot(x_estimated{2}(1,t), x_estimated{2}(2,t),'v','Color', 'r')
plotCovariance (P_estimated{2}(1:2,1:2,t) , x_estimated{2}(1,t) , x_estimated{2}(2,t) , 3 , 'Update')

for gt = 1:5
    plot( X_truth.target(1:4:end), X_truth.target(2:4:end), 'o', 'Color', [0 0 0])
end

title("Time: " + num2str(t))
xlim(x_min); ylim(y_min);
xlabel("m"), ylabel("m")
pause(.1);

clf