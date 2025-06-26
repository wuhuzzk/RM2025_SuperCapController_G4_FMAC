%{
	@File : C2nd.m
	@Description :此脚本用于把Z域2p2z传递函数转换为Fmac的配置参数
	@author: TJL
%}
function y=C2nd(C)
[num,den]= tfdata(C,'v');
y=[num,den];
disp(y)
cnt=0;
while max(abs(y))>1
y=y/2;
cnt=cnt+1;
end
y=y*32768;
y=round(y);
len_hy=int16((length(y)+1)/2);
X= ['#define fmac_B0 (',num2str(y(1)),')',newline,'#define fmac_B1 (',num2str(y(2)),')',newline,'#define fmac_B2 (',num2str(y(3)),')',newline,'#define fmac_A1 (',num2str(-1*y(len_hy+1)),')',newline,'#define fmac_A2 (',num2str(-1*y(len_hy+2)),')',newline,'#define post_shift (',num2str(cnt),')'];
disp(X);