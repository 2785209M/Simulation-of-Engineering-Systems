function [xf, thetaUvals] = Interpolation(x, y, stepsize)
    n = length(x);
    xf = 0:stepsize:x(n);

    fprintf('Manual Interpolation Values: \n')
    for i=1:length(x)
        % For comparison to check the functions work
        fprintf('f(%d) := %f\n', x(i), Manual_Interpolation(x(i)));
    end
    
    % Perform Calculations
    [table, coefficients] = Calculate_Divided_Differences(x,y);
    thetaUvals = Polynomial_Evaluation(x, stepsize, coefficients);
    
    % Print Results
    Plot_Results(x,y,xf,thetaUvals);
    
    % Display Table
    colNames = cell(1, n+1);
    colNames{1} = 'x';
    colNames{2} = 'f_x';
    for i = 2:n
        colNames{i+1} = sprintf('%d', i-1);
    end
    
    T = array2table(table, 'VariableNames',colNames);
    T = round(T, 5, 'significant');
    fprintf('\nDivided Differences Table: \n');
    disp(T);
end