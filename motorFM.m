%���㱾��ϵ�ϵ���������
function out = motorFM(in)
constant

omega = in(13:16);

T = zeros(4,1);
M = zeros(4,1);

for i=1:4
T(i) = sign(omega(i))*UAV_kf*(omega(i)); %����
M(i) = -sign(omega(i))*UAV_km*(omega(i)); %���� 
end

out(1:4)= T;
out(5:8)= M;

end