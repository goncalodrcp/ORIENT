imu_sub = rossubscriber('/imu')

i = 1;

while 1
    
    imudata(i) = receive(imu_sub,2);
    
    i = i + 1;
    
end