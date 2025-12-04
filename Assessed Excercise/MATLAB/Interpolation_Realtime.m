function thetaU = Interpolation_Realtime(x, xvalues, coefficients)
    thetaU = coefficients(1); % Initial value (Starts as the first coefficient)
    prodTerm = 1; % Initialize product term (Acculmulation of the factors)

    for i = 2:length(coefficients)
        prodTerm = prodTerm * (x - xvalues(i-1)); % Increase the product term by one degree
        thetaU = thetaU + coefficients(i) * prodTerm; % Multiply the coefficient by the product term
    end
end

