%% Particle Filter Simulation for Indoor Tracking

% Load fingerprint database
fingerprintDatabase = [
    60.0, 0.0, -54.884615;
    120.0, 0.0, -63.521739;
    180.0, 0.0, -63.181818;
    240.0, 0.0, -70.597015;
    300.0, 0.0, -68.24;
    360.0, 0.0, -71.701493;
    420.0, 0.0, -79.272727;
    480.0, 0.0, -67.47619;
    540.0, 0.0, -86.388889;
    592.0, 0.0, -84.951613;
    0.0, 60.0, -43.148148;
    60.0, 60.0, -61.895833;
    120.0, 60.0, -61.327273;
    180.0, 60.0, -71.516667;
    240.0, 60.0, -65.492063;
    300.0, 60.0, -74.25641;
    360.0, 60.0, -72.936508;
    420.0, 60.0, -72.790323;
    480.0, 60.0, -81.253731;
    540.0, 60.0, -86.1;
    592.0, 60.0, -74.276923;
    0.0, 120.0, -46.888889;
    60.0, 120.0, -65.916667;
    120.0, 120.0, -79.382353;
    180.0, 120.0, -69.409836;
    240.0, 120.0, -80.852459;
    300.0, 120.0, -74.114754;
    360.0, 120.0, -75.738095;
    420.0, 120.0, -83.590909;
    480.0, 120.0, -74.276923;
    540.0, 120.0, -72.557143;
    592.0, 120.0, -73.955882;
    0.0, 180.0, -60.671875;
    60.0, 180.0, -77.909091;
    120.0, 180.0, -73.469697;
    180.0, 180.0, -67.5;
    240.0, 180.0, -71.202899;
    300.0, 180.0, -75.539683;
    360.0, 180.0, -72.961039;
    420.0, 180.0, -74.930556;
    480.0, 180.0, -76.205479;
    540.0, 180.0, -78.695652;
    592.0, 180.0, -72.597701;
    0.0, 240.0, -71.38806;
    60.0, 240.0, -71.736842;
    120.0, 240.0, -71.178082;
    180.0, 240.0, -65.166667;
    240.0, 240.0, -68.535211;
    300.0, 240.0, -69.842105;
    360.0, 240.0, -67.305556;
    420.0, 240.0, -81.272727;
    480.0, 240.0, -74.82716;
    540.0, 240.0, -75.301205;
    592.0, 240.0, -76.673913;
    0.0, 300.0, -78.78125;
    60.0, 300.0, -71.439394;
    120.0, 300.0, -68.820896;
    180.0, 300.0, -73.105263;
    240.0, 300.0, -72.426471;
    300.0, 300.0, -70.929577;
    360.0, 300.0, -70.396825;
    420.0, 300.0, -75.833333;
    480.0, 300.0, -71.354839;
    540.0, 300.0, -70.279412;
    592.0, 300.0, -79.741935;
    0.0, 360.0, -79.571429;
    60.0, 360.0, -73.353846;
    120.0, 360.0, -73.516129;
    180.0, 360.0, -77.462687;
    240.0, 360.0, -86.337662;
    300.0, 360.0, -71.763158;
    360.0, 360.0, -85.268657;
    420.0, 360.0, -72.942857;
    480.0, 360.0, -75.514286;
    540.0, 360.0, -76.131579;
    592.0, 360.0, -69.708333;
    0.0, 420.0, -79.774648;
    60.0, 420.0, -80.309859;
    120.0, 420.0, -74.550725;
    180.0, 420.0, -74.056338;
    240.0, 420.0, -73.957746;
    300.0, 420.0, -81.4;
    360.0, 420.0, -71.263158;
    420.0, 420.0, -70.662921;
    480.0, 420.0, -87.4;
    540.0, 420.0, -72.014706;
    592.0, 420.0, -77.785714;
    0.0, 480.0, -82.202899;
    60.0, 480.0, -76.661538;
    120.0, 480.0, -73.5625;
    180.0, 480.0, -72.164179;
    240.0, 480.0, -81.405797;
    300.0, 480.0, -76.529412;
    360.0, 480.0, -78.515152;
    420.0, 480.0, -85.060606;
    480.0, 480.0, -80.117647;
    540.0, 480.0, -76.25;
    592.0, 480.0, -77.314286
];

% Define simulation parameters
numParticles = 500; % Number of particles
numSteps = 250; % Number of simulation steps
noiseStdDev = 1; % Standard deviation of noise added to RSSI
motionNoise = 10; % Noise added to particle motion

% Initialize particles
particles = [rand(numParticles, 1) * 600, rand(numParticles, 1) * 600]; % Random positions within a 600x600 area
weights = ones(numParticles, 1) / numParticles; % Equal initial weights

% Simulate RSSI at a target position (simulate a moving device)
truePosition = [300, 300];

for step = 1:numSteps
    % Simulate RSSI measurement with noise
    trueRSSI = interpolateRSSI(fingerprintDatabase, truePosition);
    noisyRSSI = trueRSSI + noiseStdDev * randn;

    % Update weights based on RSSI likelihood
    for i = 1:numParticles
        predictedRSSI = interpolateRSSI(fingerprintDatabase, particles(i, :));
        weights(i) = exp(-((predictedRSSI - noisyRSSI)^2) / (2 * noiseStdDev^2));
    end
    weights = weights / sum(weights); % Normalize weights

    % Resample particles
    indices = randsample(1:numParticles, numParticles, true, weights);
    particles = particles(indices, :);

    % Add motion noise to particles (simulate random movement)
    particles = particles + motionNoise * randn(numParticles, 2);

    % Estimate position as weighted average of particles
    estimatedPosition = sum(particles .* weights, 1);

    % Plot particles and true position
    scatter(particles(:, 1), particles(:, 2), 10, 'b', 'filled'); hold on;
    scatter(truePosition(1), truePosition(2), 100, 'r', 'filled');
    scatter(estimatedPosition(1), estimatedPosition(2), 100, 'g', 'filled');
    xlim([0, 600]); ylim([0, 600]);
    title(['Step ' num2str(step)]);
    legend('Particles', 'True Position', 'Estimated Position');
    pause(0.1);
    hold off;

    % Update true position (simulate movement)
    truePosition = truePosition + [10 * randn, 10 * randn]; % Random walk
end

%% Function to interpolate RSSI from fingerprint database
function rssi = interpolateRSSI(database, position)
    distances = sqrt((database(:, 1) - position(1)).^2 + (database(:, 2) - position(2)).^2);
    [~, idx] = min(distances);
    rssi = database(idx, 3);
end
