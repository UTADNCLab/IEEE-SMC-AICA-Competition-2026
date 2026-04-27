clear; clc;
load qcar2_paths.mat

N = size(qcar2_paths, 1);

qcar2_pathposes = cell(N);

for i = 1:N
    for j = 1:N
        path = qcar2_paths{i, j};

        if size(path, 1) > 1
            dx = diff(path(:,1));
            dy = diff(path(:,2));
    
            yawRad = atan2(dy, dx);
            yawRad(end+1) = yawRad(end);   % match waypoint length
            yawRad = unwrap(yawRad);       % avoid jumps near ±pi
        
            % Store [x y yawDeg]
            qcar2_pathposes{i, j} = [path, yawRad];
        else
            qcar2_pathposes{i, j} = [path, 0];
        end
    end
end

% -------------------------------------------------------------------------
% Find maximum number of rows
% -------------------------------------------------------------------------
maxRows = 0;

for i = 1:N
    for j = 1:N
        thisPath = qcar2_paths{i,j};
        if ~isempty(thisPath)
            maxRows = max(maxRows, size(thisPath, 1));
        end
    end
end

% -------------------------------------------------------------------------
% Pad all paths to have the same number of rows by repeating last row
% -------------------------------------------------------------------------
for i = 1:N
    for j = 1:N
        thisPath = qcar2_pathposes{i,j};

        if isempty(thisPath)
            % Optional: keep empty, or fill with NaNs
            qcar2_pathposes{i,j} = nan(maxRows, 3);
            continue;
        end

        numRows = size(thisPath, 1);

        if numRows < maxRows
            lastRow = thisPath(end, :);
            padRows = repmat(lastRow, maxRows - numRows, 1);
            qcar2_pathposes{i,j} = [thisPath; padRows];
        end
    end
end

clearvars -except qcar2_pathposes
save qcar2_pathposes