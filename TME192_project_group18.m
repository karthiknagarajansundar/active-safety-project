clear;clc;close all
%% Task 1

%Given
velMov= 90*(5/18);  % kmph to mps
velStat = 0;    % kmph mps
R_time = 1.5;   % s
a = -5;   % mps^2

%a      v^2 - u^2 = 2as
range_a = abs(velMov^2/2/a);

%b      s = ut + 1/2 at^2
syms TTC_b
TTC_b = solve(velMov*TTC_b + 1/2*a*TTC_b^2 -range_a==0 , TTC_b , 'Real' , true);
TTC_b = double(TTC_b(1));

%c
range_c = range_a + (velMov * R_time);
TTC_c = TTC_b + R_time;

% d
clear;clc; close all

%Given
velMov= 90*(5/18);  % kmph to mps
R_time = 1.5;   % s
a = -5;   % mps^2
range_d = 90; % m
figure(); hold on;
for time=0:0.1:6.5
    if time <= R_time
        range_d_temp = range_d - velMov*time;
        temp = range_d_temp;
    else
        t = time-R_time;
        range_d_temp = temp - (velMov*t + (0.5*a*(t^2)));
    end
    if range_d_temp == 0
        plot(time,range_d_temp,'r*');
        text(time+0.2,range_d_temp,'Crash')
        break
    end
    plot(time,range_d_temp,'b.');  
    plot([0 4.5], [0 0], 'linewidth',1)
%     yline(5)
end
xlabel("Time(s)");
ylabel("Distance(m)");





%% TASK 2
clear;clc;close all; opengl software
load('RadarData.mat')
RadarDataCorrected = ([]);

%%
% removing fields with all NaNs
RadarDataCorrected = RadarData;    %copy of original dataset
indices=[];
for field = 1:length(RadarDataCorrected)
    if (all(isnan(RadarData(field).RadarRange)))
        
        %if all are nans then append its corresponding index
        indices(end+1)=field;
    end
end

%remove data corresponding to that index
RadarDataCorrected([indices])=[];  

% remove delay, calculate acceleration and percent of nans and removing them
indices = [];
for field = 1:length(RadarDataCorrected)
    % syncing to avoid 200ms delay from radar data
    RadarDataCorrected(field).RadarTime = RadarDataCorrected(field).RadarTime - (200/1000);
    
    % acceleration = dv/dt
    RadarDataCorrected(field).VehicleAcceleration = diff(RadarDataCorrected(field).VehicleSpeed)./diff(RadarDataCorrected(field).VehicleTime);
    
    % if number of nans >95% of data or number of data poins < 50 we are
    % removing it
    nansperct = sum(isnan(RadarDataCorrected(field).RadarRange)) / length(RadarDataCorrected(field).RadarRange);
    if or(nansperct >0.95, (numel(RadarDataCorrected(field).RadarRange)<50))
        indices(end+1) = field;
    end
end

%remove data corresponding to that index
RadarDataCorrected([indices])=[];  

%fill all missing data between radar data and vehicle dynamics
for field = 1:length(RadarDataCorrected)
    
    %adding datas using linear model
    RadarDataCorrected(field).RadarRange = fillmissing(RadarDataCorrected(field).RadarRange,'linear');
    
    RadarDataCorrected(field).RadarRangeRate = fillmissing(RadarDataCorrected(field).RadarRangeRate,'linear');
    
    RadarDataCorrected(field).RadarAcce = fillmissing(RadarDataCorrected(field).RadarAccel,'linear');
end

%Discarding field 19 since it does not contani required information
%that has been tested while running the code.
RadarDataCorrected(19)=[]; 

% saving RadarDataCorrected
save('RadarDataCorrected.mat', 'RadarDataCorrected')

% identifying breaking maneuver
%initializing new variables
start_bm = [];  %start breaking maneuver
end_bm = [];    %end breaking maneuver
min_acc_list = []; min_acc_i_list = [];

for field = 1:length(RadarDataCorrected)
    % min acc and its coresponding index
    [min_acc, min_acc_i] = min(RadarDataCorrected(field).VehicleAcceleration);
    
    %saving minimum acceleration and corresponding index to lists
    min_acc_list(end+1)=min_acc;
    min_acc_i_list(end+1)=min_acc_i;
    if(1)
        % range of braking maneuver
        temp_index_left = min_acc_i;
        while (RadarDataCorrected(field).VehicleAcceleration(temp_index_left)<-0.5)
            temp_index_left = temp_index_left-1;
        end
        start_bm(end+1)=temp_index_left;

        temp_index_right = min_acc_i;
        while (RadarDataCorrected(field).VehicleAcceleration(temp_index_right)<-0.5)
            temp_index_right = temp_index_right+1;
        end
        end_bm(end+1)=temp_index_right;
    end
end

%% Plots
for field=1:length(RadarDataCorrected)
        figure(field); hold on;
        plot(RadarDataCorrected(field).VehicleTime,RadarDataCorrected(field).VehicleSpeed,'LineWidth',1);
        plot(RadarDataCorrected(field).VehicleTime(1:end-1),smooth(RadarDataCorrected(field).VehicleAcceleration),'LineWidth',1);
        plot(RadarDataCorrected(field).RadarTime,RadarDataCorrected(field).RadarRange,'LineWidth',1);
        plot(RadarDataCorrected(field).VehicleTime,RadarDataCorrected(field).VehicleYawRate,'LineWidth',1);
        xline(RadarDataCorrected(field).VehicleTime(start_bm(field)),'--','LineWidth',1);
        xline(RadarDataCorrected(field).VehicleTime(end_bm(field)),'--','LineWidth',1);
        legend('VehicleSpeed','VehicleAcceleration', 'RadarRange','VehicleYawRate','startBM','endBM');
        
        % creating struct
        table_sm.participant(field,1) = RadarDataCorrected(field).TestPerson;
        table_sm.mean_acceleration(field,1) = mean(RadarDataCorrected(field).VehicleAcceleration(start_bm(field):end_bm(field)));
        table_sm.min_acceleration(field,1)= min_acc_list(field);
        table_sm.speed_at_brake_onset(field,1) = RadarDataCorrected(field).VehicleSpeed(start_bm(field));
        table_sm.acc_brake_onset(field,1) = RadarDataCorrected(field).VehicleAcceleration(start_bm(field));
        table_sm.range_at_brake_onset(field,1) = RadarDataCorrected(field).RadarRange(start_bm(field));
        table_sm.ttc_at_brake_onset(field,1) = table_sm.range_at_brake_onset(field,1)/table_sm.speed_at_brake_onset(field,1);
end

% converting struct to Table
TableTask2_group18 = struct2table(table_sm); 

% saving table
save('TableTask2_group18.mat', 'TableTask2_group18')






%% TASK 3

clear;clc;close all
opengl software

% load table
load("TableTask2.mat");

% scatter plot figure speed vs TTC
figure(1); hold on;
for i = unique(T.TestPerson)'
   tempTestPerson = find(T.TestPerson == i);
   for j = tempTestPerson
       scatter(T.speed_at_BO(j) , T.TTC_at_BO(j), 'filled')
   end
end
legend('1','2','4','5','6','7','8','9');
xlabel("Speed-BO");
ylabel("TTC-BO");

% histogram of all TTC 
figure(2);
histogram(T.TTC_at_BO);

% distribution is skewed when the data points cluster more 
% toward one side of the scale than the other, creating a curve that is 
% not symmetrical. THUS it is SKEWED dist.

% 5th and 95th percentiles
p5 = prctile(T.TTC_at_BO,5)
p95 = prctile(T.TTC_at_BO,95)

% mean & min acc
figure(3); hold on;
histogram(T.mean_acc)
histogram(T.min_acc)
legend('MeanAcc', 'MinAcc')

% mean & min acc
figure(4);
histfit(T.mean_acc)
legend('MeanAcc' , 'MeanAccDist')
figure(5);
histfit(T.min_acc)
legend('MinAcc' , 'MinAccDist')





%% Task 4
clear; clc; close all

% Task 4a,b
velMov= 90*(5/18);  % kmph to mps
velStat = 0;    % kmph mps
R_time = 1.5;   % s
a = -5;   % mps^2

% FCW conservative
thresh_c = 4
s_c = velMov*R_time;
s_c = s_c+ velMov*(thresh_c-R_time) + 0.5*a*((thresh_c-R_time)^2)

% FCW aggressive
thresh_a = 2.5
s_a = velMov*R_time;
s_a = s_a+ velMov*(thresh_a-R_time) + 0.5*a*((thresh_a-R_time)^2)

% Task 4d
max_decel = -9.8 % max decelaration
range_AEB = abs(velMov^2/2/max_decel)

t = abs(velMov/max_decel)
clear TTC_AEB
syms TTC_AEB
TTC_AEB = solve(velMov*TTC_AEB + 1/2*max_decel*TTC_AEB^2 -range_AEB==0 , TTC_AEB , 'Real' , true);
TTC_AEB = double(TTC_AEB(1))

% Task 4f
TTC_AEB_1 =3
s_aeb = velMov*TTC_AEB_1 + 0.5*(max_decel)*(TTC_AEB_1^2)
% tried with different values and ended up with -9.6 so that distance is
% 31.8 similar to range_AEB(previous task)
s_aeb_acc = velMov*TTC_AEB_1 + 0.5*(-9.6)*(TTC_AEB_1^2)
% Task 4i
%Given
velMov= 90*(5/18);  % kmph to mps
R_time = 1.5;   % s
a = -5;   % mps^2
range_d = 90; % m
figure(); hold on;
for time=0:0.1:6.5
    if time <= R_time
        range_d_temp = range_d - velMov*time;
        temp = range_d_temp;
    else
        t = time-R_time;
        range_d_temp = temp - (velMov*t + (0.5*a*(t^2)));
    end
    if range_d_temp == 0
        plot(time,range_d_temp,'r*');
        text(time+0.2,range_d_temp,'Crash');
        break
    end
    plot(time,range_d_temp,'b.');  
end
plot([0 4.5], [0 0], 'linewidth',1);
yline(s_a);
text(3.5,s_a+3,'FCW Agg');
yline(s_c);
text(3.5,s_c+3,'FCW Cons');
yline(range_AEB);
text(3.5,range_AEB+3,'AEB dec');
yline(s_aeb);
text(3.5,s_aeb-3,'AEB TTC');
xlabel("Time(s)");
ylabel("Distance(m)");
