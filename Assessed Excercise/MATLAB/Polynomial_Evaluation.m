function thetaUvals = Polynomial_Evaluation(x, stepsize, coefficients)
    n = length(x);
    xf = 0:stepsize:x(n);
    thetaUvals = zeros(size(xf));

    for k = 1:length(xf)
        thetaU = coefficients(1); % Initial value (Starts as the first coefficient)
        prodTerm = 1; % Initialize product term (Acculmulation of the factors)

        for i = 2:n
            prodTerm = prodTerm * (xf(k) - x(i-1)); % Increase the product term by one degree
            thetaU = thetaU + coefficients(i) * prodTerm; % Multiply the coefficient by the product term
        end
        thetaUvals(k) = thetaU; % Add values to output matrix
    end
end


