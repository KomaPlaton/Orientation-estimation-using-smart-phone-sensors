% Function to plot normal distribution
function plot_normal_distribution(mean_val, cov_val)
    x = linspace(mean_val - 3*sqrt(cov_val), mean_val + 3*sqrt(cov_val), 100);
    y = normpdf(x, mean_val, sqrt(cov_val));
    plot(x, y, 'LineWidth', 2);
end