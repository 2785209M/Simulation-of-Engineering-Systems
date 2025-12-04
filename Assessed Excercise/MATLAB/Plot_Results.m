function Plot_Results(x,y,xf,thetaUvals)
    % Plot Results
    figure;
    plot(x, y, 'ro', 'MarkerSize', 10, 'LineWidth', 2, ...
         'DisplayName', 'Data Points'); hold on;
    plot(xf, thetaUvals, 'b-', 'LineWidth', 1.5, ...
         'DisplayName', 'Newton Interpolation');
    xlabel('x'); ylabel('\theta_U');
    title('Newton Divided Difference Interpolation');
    legend('show', 'Location', 'northwest');
    grid on;
end