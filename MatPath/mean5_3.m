function [ y ] = mean5_3( x, m )
%MEAN5_3 Summary of this function goes here
%   Detailed explanation goes here
% x为被处理的数据
% m 为循环次数
n=length(x);
a=x;
for k=1: m
    b(1) = (69*a(1) +4*(a(2) +a(4)) -6*a(3) -a(5)) /70;
    b(2) = (2* (a(1) +a(5)) +27*a(2) +12*a(3) -8*a(4)) /35;
    
    for j=3:n-2
    b (j) = (-3*(a(j-2) +a(j+2)) +12*(a(j-1) +a(j+1)) +17*a(j)) /35;
    end
    
    b(n-1) = (2*(a(n) +a(n-4)) +27*a(n-1) +12*a(n-2) -8*a(n-3)) /35;
    b(n) = (69*a(n) +4* (a(n-1) +a(n-3)) -6*a(n-2) -a(n-4)) /70;
    a=b;
end
y = a;

end

