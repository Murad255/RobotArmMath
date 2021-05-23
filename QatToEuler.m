function [a,b,c] = QatToEler(qw,qx,qy,qz)
%QATERNIONTOELER Summary of this function goes here
%   Detailed explanation goes here
a = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));
b = asin(max(min(2 * (qw * qy - qx * qz), 1), -1));
c = atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy));
end

