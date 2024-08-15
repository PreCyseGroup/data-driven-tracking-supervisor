function [Z] = innerZonotopeApprox(A, b)
    % A, b define the polyhedron Ax <= b
    % This function returns a simple 2D zonotope approximation
    
    % Initial settings
    dim = size(A, 2); % Dimension of the space
    numDirections = 4; % Number of directions for initial exploration
    theta = linspace(0, 2 * pi, numDirections + 1); % Angular directions
    theta(end) = []; % Remove redundant angle
    directions = [cos(theta); sin(theta)]; % Direction vectors

    % Find an initial point inside the polyhedron
    % Solving a linear program to find a feasible point
    f = zeros(dim, 1); % Objective function (irrelevant here)
    x0 = linprog(f, A, b); % Feasible point within the polyhedron
    
    % Initialize zonotope (center at the feasible point, no generators initially)
    center = x0;
    generators = [];
    
    % Iterate over all directions to find initial generators
    for i = 1:numDirections
        dir = directions(:, i);
        
        % Maximize the length in the given direction subject to staying inside the polyhedron
        % This is done by moving from the center towards the polyhedron's boundary
        [alpha_max] = linprog(-dir, A, b - A * center); % Maximize in negative direction for minimization
        
        if ~isempty(alpha_max) && alpha_max > 0
            % Add new generator if the expansion is possible
            generators = [generators, alpha_max * dir];
        end
    end
    
    % Constructing the zonotope from the center and generators
    % Note: For visualization or further computation, convert generators to vertices or use a dedicated zonotope representation.
    Z.center = center;
    Z.generators = generators;

    % Z now represents the zonotope approximation
    % Additional steps could be added to refine or optimize the zonotope
end
