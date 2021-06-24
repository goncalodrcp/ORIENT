
dir = 'D:\IST\ORIENT_repos\Tests\Mariana\C++examples\SensorRecordings\';

quaternion = fopen(strcat(dir, 'quaternion.data'));
raw = fopen(strcat(dir, 'raw.data'));

line_q = fgetl(quaternion);
line_r = fgetl(raw);

i = 1;
while ischar(line_q)
    split = textscan(line_q, '%d64, %f, %f, %f, %f');
    t(i, 1) = split{1};
    q(i, 1) = split{2};
    q(i, 2) = split{3};
    q(i, 3) = split{4};
    q(i, 4) = split{5};
    line_q = fgetl(quaternion);
    i = i + 1;
end

% i = 1;
% while ischar(line_r)
%     split = textscan(line_r, '%d64, %f, %f, %f, %f, %f, %f');
%     t(i, 1) = split{1};
%     acc(i, 1) = split{2};
%     acc(i, 2) = split{3};
%     acc(i, 3) = split{4};
%     gyro(i, 1) = split{5};
%     gyro(i, 2) = split{6};
%     gyro(i, 3) = split{7};
%     line_r = fgetl(raw);
%     i = i + 1;
% end

