function Data = load_vp_si()

load aa3_lsr2; % LASER, TLsr
load aa3_dr;   % speed, steering, time
load aa3_gpsx; % La_m, Lo_m, timeGps

Data.Laser.ranges = double(LASER)/100; % [m]
Data.Laser.time = double(TLsr)/1000;   % [s]
Data.Control.ve = speed;               % [m/s]
Data.Control.alpha = steering;         % [rad]
Data.Control.time = time/1000;         % [s]
Data.Gps.x = Lo_m;                     % [m]
Data.Gps.y = La_m;                     % [m]
Data.Gps.time = timeGps/1000;          % [s]

