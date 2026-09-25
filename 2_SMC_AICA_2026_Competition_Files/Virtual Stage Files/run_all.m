clc; clear;

load tools/QDrone2_PathPlanning/qdrone2_plans.mat
load tools/QCar2_PathPlanning/qcar2_pathposes.mat

[init_car_pos, init_drone_pos] = read_initial_positions("spawn_locations.txt");

system(['start "" quarc_run -q -Q -t tcpip://localhost:17000 *.rt-win64']);

system("python setup_env.py")

open_system('QDrone2_Navigator'); 
set_param('QDrone2_Navigator','SimulationCommand','start')

open_system('QCar2_Navigator'); 
set_param('QCar2_Navigator','SimulationCommand','start');

system('start "" python game.py');
 
system(['start "Virtual QCar Model" quarc_run -D -r -t tcpip://localhost:17000 virtual_DriveStack.rt-win64 -uri tcpip://localhost:17001']);

system(['start "Virtual QDrone Model" quarc_run -D -r -t tcpip://localhost:17000 virtual_FlightStack.rt-win64 -uri tcpip://localhost:17002']);






function [init_car_pos, init_drone_pos] = read_initial_positions(filename)

    fid = fopen(filename, 'r');
    if fid == -1
        error('Cannot open file');
    end

    values_all = [];

    while ~feof(fid)
        line = strtrim(fgetl(fid));

        % Skip empty lines or comments
        if isempty(line) || startsWith(line, '#')
            continue;
        end

        % Split by comma
        parts = strsplit(line, ',');

        % Convert to numbers
        nums = str2double(parts);

        % Append to array
        values_all = [values_all, nums]; %#ok<AGROW>
    end

    init_car_pos = values_all(1:4)';
    init_drone_pos = values_all(5:end)';
    fclose(fid);
end