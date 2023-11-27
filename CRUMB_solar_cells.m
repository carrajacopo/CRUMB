clc
clear
close all

%Power demands (W)
sleep = 0.1*3.3/1000;   % ESP32-WROOM-32U power consumption: 
                        % 0.05 mA when in deep sleep mode (only if optimal design, 0.1 more realistic) --> to be muliplied by 3.3V; 
                        % 10-20 mW when in sleep mode; 
                        % 50-100 mW when in active mode
active = 50/1000 + 10/1000 + 15*3.3/1000;     % added sensor consumption and receiving LoRa
transmit = 50/1000 + 120*3.3/1000; % LoRa Module power consumption:
                                   % SX1276 20-120 mA tx and 10.8-12 mA rx;
                                   % HopeRF 35-120 mA tx and 16 mA rx

dod = 0.5; %depth of discharge

%LUNAR DAY (29 1/2 days) in SECONDS
l_day = 29.5 * 24 * 3600;
l_day = round(l_day);
orbperiod = l_day;

Teclipse = round(l_day/2);
eclipsestart = round(l_day/4);
eclipseend = round(l_day*3/4);


payloadstart = round(l_day/6);
payloadend = round(l_day*5/6);



%Set a vector for power usage every second and add power requirements
vector = zeros(1,orbperiod);

%Add power usage of satellite bus
vector = vector + sleep;


%% spikes of transmissions
interval = 6*60*60;
j = 0;
for i = 1:length(vector(1:eclipsestart/2)) % midday till DUSK (3/4ths of the day)

    %Add power usage of transmit for 600 seconds (10min)
    if (1 <= j)  && (j <= 10*60)
        vector(1,i) = vector(1,i) + transmit;
    end
        % Decrease the interval count
    interval = interval - 1;
    % Check if the interval count reaches 0
    if interval == 0
        % Reset the count and generate a new random interval
        interval = randi([3*60*60, 9*60*60]);
        j = 0;
    end
    j = j+1;
end
% from DUSK till DAWN ->
interval = 24*60*60;
j = 0;
for i = eclipsestart/2:eclipseend+eclipsestart/2 % DUSK (3/4ths of the day) - night - till DAWN (1/4th of the day)

    %Add power usage of transmit for 60 seconds (1min)
    if (1 <= j)  && (j <= 60)
        vector(1,i) = vector(1,i) + transmit;
    end
    % Decrease the interval count
    interval = interval - 1;
    % Check if the interval count reaches 0
    if interval == 0
        % Reset the count and generate a new random interval
        interval = randi([12*60*60, 36*60*60]);
        j = 0;
    end
    j = j+1;
end

interval = 6*60*60;
j = 0;
for i = ((orbperiod-eclipseend)/2 + eclipseend):orbperiod % DAWN till midday

    %Add power usage of transmit for 600 seconds (10min)
    if (1 <= j)  && (j <= 10*60)
        vector(1,i) = vector(1,i) + transmit;
    end
    % Decrease the interval count
    interval = interval - 1;
    % Check if the interval count reaches 0
    if interval == 0
        % Reset the count and generate a new random interval
        interval = randi([3*60*60, 9*60*60]);
        j = 0;
    end
    j = j+1;
end

%% spikes of active mode, to check for receiveing and to check sensors
interval = 1*60*60;
j = 0;
for i = 1:length(vector(1:eclipsestart/2))

    %Add power usage of rx for 30*60 seconds (30min)
    if (1 <= j)  && (j <= 30*60)
        vector(1,i) = vector(1,i) + active;
    end
    % Decrease the interval count
    interval = interval - 1;
    % Check if the interval count reaches 0
    if interval == 0
        % Reset the count and generate a new random interval
        interval = randi([1*30*60, 1*90*60]);
        j = 0;
    end
    j = j+1;
end

interval = 24*60*60;
j=0;
for i = eclipsestart/2:eclipseend+eclipsestart/2

    %Add power usage of rx for 60 seconds (1m)
    if (1 <= j)  && (j <= 60)
        vector(1,i) = vector(1,i) + active;
    end
    % Decrease the interval count
    interval = interval - 1;
    % Check if the interval count reaches 0
    if interval == 0
        % Reset the count and generate a new random interval
        interval = randi([12*60*60, 36*60*60]);
        j = 0;
    end
    j = j+1;
end

interval = 1*60*60;
j = 0;
for i = ((orbperiod-eclipseend)/2 + eclipseend):orbperiod

    %Add power usage of rx for 30*60 seconds (30min)
    if (1 <= j)  && (j <= 30*60)
        vector(1,i) = vector(1,i) + active;
    end
    % Decrease the interval count
    interval = interval - 1;
    % Check if the interval count reaches 0
    if interval == 0
        % Reset the count and generate a new random interval
        interval = randi([1*30*60, 1*90*60]);
        j = 0;
    end
    % %Reset the count every 1 * 60 * 60 seconds (1hrs)
    % if j == 1 * 60 * 60
    %     j = 0;
    % end
    j = j+1;
end


%% thermal
for i = 70:length(vector)

    %Add power usage of thermal for 70 seconds
    if (1 <= j)  && (j <= 60*2+10)
        vector(1,i) = vector(1,i);
    end
    %Reset the count every 30 minutes
    if j == 60*30
        j = 0;
    end
    j = j+1;
end


%% PLOT THE POWER CONSUMPTION
%Plot the vector
subplot(1,1,1)
hold on
rectangle(Position=[eclipsestart,0,eclipseend-eclipsestart,0.6], FaceColor=[0.7 0.8 0.9], EdgeColor=[0.9 0.9 0.9])
hold on;  grid on;  grid minor
set(gca, "Layer", "top")

plot(vector,'LineWidth',0.8,'color', [0.4 0.65 0.95]) % power consumption in W
grid minor
ylim([0, inf]);
xlim([0, inf]);
xlabel('Time (s)','FontSize',13);
ylabel('Power (Watt)', 'FontSize',13);
set(gca,'FontSize',13)
title('Power Consumption & Battery Charge vs. Time (active mode 50 mW)') 


% %Plot the dotted lines for the eclipse
hold on
% xline(eclipsestart,'-.','LineWidth',1,'Color',[0.8500 0.3250 0.0980])
% xline(eclipseend,'-.','LineWidth',1,'Color',[0.8500 0.3250 0.0980])


%% SOLAR POWER GENERATION

%Calculate the integral --> area under the curve

timedelta = 1;

dark_power_consumption = vector(eclipsestart:eclipseend);
light_power_consumption = [vector(1:eclipsestart), vector(eclipseend:end)];

dark_energy_consumption = sum(dark_power_consumption * timedelta);
light_energy_consumption = sum(light_power_consumption * timedelta);

"Total Energy Consumption over one Orbit in Joules (Ws):"
total_energy_consumption = dark_energy_consumption + light_energy_consumption

"Average Power needed over orbit:"
average_power = total_energy_consumption / orbperiod

"Average Power Generation needed in sunlight:"
average_solar_generation = average_power * orbperiod/(orbperiod-(eclipseend - eclipsestart))


average_solar_generation = (26.3*3)/1000; % true solar panels for comparison, in W. 3 solar cells
                                          % each cell has 26.3 mW at
                                          % maximum power point

incident_solar_generation = [ones(1, eclipsestart), zeros(1, eclipseend - eclipsestart), ones(1, orbperiod - eclipseend)] * average_solar_generation;


for i = 1:length(vector(1:eclipsestart))
    angle = (i*pi/2) / (eclipsestart) ; %angle incidence, following a quarter of a sine curve
    incident_solar_generation(i) = average_solar_generation * sin(angle + pi/2);
end
for i = 1:length(vector(1:eclipsestart))
    angle = (i*pi/2) / (eclipsestart) ; %angle incidence, following a quarter of a sine curve
    incident_solar_generation(i + eclipseend) = average_solar_generation * sin(angle);
end


plot(incident_solar_generation,'LineWidth',1.5, 'Color',[0.8500 0.3250 0.0980]) % power generation in Joule


%% BATTERY STATE

battery_full_state = 2*0.2405*3600; %two batteries of 0.2405 Wh each

battery_state = zeros(1, orbperiod);

battery_state(1) = battery_full_state; %starting at 100% charge
for i = (2:orbperiod)
    battery_state(i) = battery_state(i-1) + incident_solar_generation(i) - vector(i);
    if(battery_state(i) > battery_full_state)
        battery_state(i) = battery_full_state;
    end
    if(battery_state(i)<0)
        battery_state(i) = 0;
    end
end
yyaxis right
% ylim([0, battery_full_state * 1.5]);
ylabel('Energy (Joules)','FontSize',13);
plot(battery_state,'LineWidth',1.5,'color',[0.4660 0.6740 0.1880])

battery_state(1) = battery_full_state * 0.7; %starting at 70% charge
for i = (2:orbperiod)
    battery_state(i) = battery_state(i-1) + incident_solar_generation(i) - vector(i);
    if(battery_state(i) > battery_full_state)
        battery_state(i) = battery_full_state;
    end
    if(battery_state(i)<0)
        battery_state(i) = 0;
    end
end
plot(battery_state,'--', 'LineWidth',1.5,'color', [0.9290 0.6940 0.1250])

battery_state(1) = battery_full_state * 0.5; %starting at 50%
for i = (2:orbperiod)
    battery_state(i) = battery_state(i-1) + incident_solar_generation(i) - vector(i);
    if(battery_state(i) > battery_full_state * 0.5)
        battery_state(i) = battery_full_state * 0.5;
    end
    if(battery_state(i)<0)
        battery_state(i) = 0;
    end
end
plot(battery_state,'--', 'LineWidth',1.5,'color', [0.6350 0.0780 0.1840])

yline(battery_full_state*(1-dod), '-.', 'LineWidth',0.8,'color', [0.4940 0.1840 0.5560]); %batt limit 30% DoD
legend('Power Usage', 'Solar Generation', ...
    'Battery Charge (starting at 100% charge)', 'Battery Charge (starting at 70% charge)', ...
    'Battery Charge 50% depleted','50% DoD','Location','Southwest')

set(gcf, 'WindowState', 'maximized');
set(gca,'color',[0.95 0.95 0.6]);