%计算本体系上的力和力矩
function out = motorFM(in)
constant

omega = in(13:16);

T = zeros(4,1);
M = zeros(4,1);

for i=1:4
T(i) = sign(omega(i))*UAV_kf*(omega(i)^2); %推力
M(i) = -sign(omega(i))*UAV_km*(omega(i)^2); %力矩 
end

out(1:4)= T;
out(5:8)= M;

end