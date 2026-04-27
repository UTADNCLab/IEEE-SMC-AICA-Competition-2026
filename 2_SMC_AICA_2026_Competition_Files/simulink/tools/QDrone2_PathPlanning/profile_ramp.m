% YOU CAN USE AS IS OR MODIFY

function t = profile_ramp(wp, linear_vel, angular_vel)
% PROFILE_RAMP
% Compute cumulative waypoint times based on linear and angular speed limits.
%
% Inputs:
%   wp          - Nx4 waypoint matrix: [x y z yaw]
%   linear_vel  - max linear velocity
%   angular_vel - max angular velocity
%
% Output:
%   t           - Nx1 cumulative time vector

    t = zeros(size(wp, 1), 1);

    for k = 2:size(wp, 1)
        % Time needed for translation
        dt_lin = norm(wp(k, 1:3) - wp(k-1, 1:3), 2) / linear_vel;

        % Time needed for yaw change
        dt_ang = abs(wp(k, 4) - wp(k-1, 4)) / angular_vel;

        % Segment time must satisfy both
        dt = max(dt_lin, dt_ang);

        % Cumulative time
        t(k) = t(k - 1) + dt;
    end
end