function [table, coefficients] = Calculate_Divided_Differences(x, y)
    n = length(y);
    table = zeros(n, n+1); % initialize matrix for storing values
    table(:, 1) = x(:); % Set Column 1 as the x values
    table(:, 2) = y(:); % Set Column 2 as the y values
    
    for j = 3:n+1 % Start at column 3 for divided differences
        for i = j-1:n
            % Divided difference formula
            % The denominator uses x(i) and x(i-(j-2))
            table(i,j) = (table(i,j-1) - table(i-1,j-1)) / (x(i) - x(i-(j-2)));
        end
    end

    coefficients = zeros(n,1); % Extract Divided Difference Coefficients
    for i = 1:n
        coefficients(i) = table(i, i+1);
    end
end