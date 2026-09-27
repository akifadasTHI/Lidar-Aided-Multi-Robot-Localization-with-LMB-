function [z_t] = loc2glob_target(z_t, z_v, t)
% Project Targets from Loc to Glob only Translation
%   [z_t] = loc2glob_target(z_t, z_v, t)
% z_t: 
% z_v: 
    Nv = length(z_v);
    for v = 1: Nv
        loc_target_position = cell2mat(z_t.loc{v}(t));
        glob_target_position = loc_target_position + z_v{v}';
        z_t.glob{v}{t} = glob_target_position;
    end
end