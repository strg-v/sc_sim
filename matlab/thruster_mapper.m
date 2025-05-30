function [durations] = thruster_mapper(wrench, time)

T = 0.1;

force = 0.5;
thruster_forces = 0.5 * ones(24, 1);

thruster_position_xp = [0.175, 0.0, 0.0];
thruster_position_xm = [-0.175, 0.0, 0.0];
thruster_position_yp = [0.0, 0.175, 0.0];
thruster_position_ym = [0.0, -0.175, 0.0];
thruster_position_zp = [0.0, 0.0, 0.175];
thruster_position_zm = [0.0, 0.0, -0.175];

thruster_positions = [
    % XP
    thruster_position_xp;
    thruster_position_xp;
    thruster_position_xp;
    thruster_position_xp;
     % XM
    thruster_position_xm;
    thruster_position_xm;
    thruster_position_xm;
    thruster_position_xm;
    % YP
    thruster_position_yp;
    thruster_position_yp;
    thruster_position_yp;
    thruster_position_yp;
    % YM
    thruster_position_ym;
    thruster_position_ym;
    thruster_position_ym;
    thruster_position_ym;
    % ZP
    thruster_position_zp;
    thruster_position_zp;
    thruster_position_zp;
    thruster_position_zp;
    % ZM
    thruster_position_zm;
    thruster_position_zm;
    thruster_position_zm;
    thruster_position_zm;
];

thruster_direction_xp = [1, 0, 0];
thruster_direction_xm = [-1, 0, 0];
thruster_direction_yp = [0, 1, 0];
thruster_direction_ym = [0, -1, 0];
thruster_direction_zp = [0, 0, 1];
thruster_direction_zm = [0, 0, -1];

thruster_directions = [
    % XP
    thruster_direction_yp;
    thruster_direction_ym;
    thruster_direction_zp;
    thruster_direction_zm;
    % XM
    thruster_direction_yp;
    thruster_direction_ym;
    thruster_direction_zp;
    thruster_direction_zm;
    % YP
    thruster_direction_xp;
    thruster_direction_xm;
    thruster_direction_zp;
    thruster_direction_zm;
    % YM
    thruster_direction_xp;
    thruster_direction_xm;
    thruster_direction_zp;
    thruster_direction_zm;
    % ZP
    thruster_direction_xp;
    thruster_direction_xm;
    thruster_direction_yp;
    thruster_direction_ym;
    % ZM
    thruster_direction_xp;
    thruster_direction_xm;
    thruster_direction_yp;
    thruster_direction_ym;
];

thruster_matrix_force = (thruster_forces .* thruster_directions)';
thruster_matrix_torque = cross(thruster_matrix_force', thruster_positions)';

thruster_matrix = [
    thruster_matrix_force;
    thruster_matrix_torque
    ];

impulse = wrench .* T;

n_thrusters = length(thruster_matrix');

cvx_begin quiet
    variable t(n_thrusters, 1)
    minimize (norm(t, 2))
    subject to
        thruster_matrix * t == impulse
        t >= 0
cvx_end

durations = t

