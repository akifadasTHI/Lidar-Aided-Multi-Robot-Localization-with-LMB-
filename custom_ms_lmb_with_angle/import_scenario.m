function [X_truth, z_v, z_t, Nsteps, Nv, P_vehicle] = import_scenario()
    load("data\pmbm.mat") 
    load("data\glob_obj_loc.mat")
    
    for i = 1: size(vehicleStateGT,1)/4
        X_truth.veh{i} = vehicleStateGT((i-1)*4+1:(i)*4,:); 
        z_v{i} = reshape(faultyGps(i,:,:), 2,[],1);
        P_vehicle{i} = faultyGps_cov(i,:,:);
        z_t_glob{i} = glob_obj_loc{i};
        for t = 1: 3880
            z_t.loc{i}{t} = glob_obj_loc{i}{t} - z_v{i}(:,t)';
            z_t.glob{i}{t} = nan(size(z_t.loc{i}{t}));
        end
        X_truth.target = Fea_vect_true;
    end    
    Nsteps = size(z_v{1},3);
    Nv = size(vehicleStateGT,1)/4;
    clear i o faultyGps_cov faultyGps Fea_vect_true Fea_vect_boxes Fea_vect vehicleStateGT
end