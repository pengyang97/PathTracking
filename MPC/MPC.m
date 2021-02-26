clc
clear all

Kp = 1.0 ; 
dt = 0.1  ;% [s]
Length = 2.9 ;% [m] wheel base of vehicle
Nx=3; % 状态量的个数
Nu =2; % 控制量的个数
Np =60; % 预测步长
Nc=30; % 控制步长
Row=10; % 松弛因子
Q=100*eye(Nx*Np,Nx*Np); % 状态量加权矩阵Q，调节每个预测点上，三个状态量各自在目标函数中比重    
R=1*eye(Nc*Nu); % 控制量加权矩阵，用于调节控制量偏差在目标函数中的比重
    
max_steer =60 * pi/180; % in rad 车辆最大转角（弧度）
target_v =30.0 / 3.6; % 目标车速（m/s）
% 设定参考轨迹（生成曲线）
cx = 0:0.1:200; % sampling interception from 0 to 200, with step 0.1
for i = 1:length(cx) % here we create a original reference line, which the vehicle should always follow when there is no obstacles;
    cy(i) = -sin(cx(i)/10)*cx(i)/8;
end
% for i = 501: length(cx)
%     cy(i) = -sin(cx(i)/10)*cx(i)/8; % cy(500);
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% here we provide another reference line for testing, now you dont need to
% use it
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p = [cx', cy'];
 % 计算一阶导数
 for i = 1:length(cx)-1
 pd(i) = (p(i+1,2)-p(i,2))/(p(i+1,1)-p(i,1));
 end
 pd(end+1) = pd(end);
  % 计算二阶导数（这里是用中心差商近似代替微分）
  % f'(x0)=(f(x0+h)-f(x0-h))/2h
  % f(x0+h)=f(x0)+hf'(x0)+(h^2/2!)f"(x0)+(h^3/3!)f"'(ξ)
 for i =2: length(cx)-1
     pdd(i) = (p(i+1,2)-2*p(i,2) + p(i-1,2))/(0.5*(-p(i-1,1)+p(i+1,1)))^2;
 end
      pdd(1) = pdd(2);
     pdd(length(cx)) = pdd(length(cx)-1);
  % 计算曲率 
  % y"/(1+y'^2)^1.5
  for i  = 1:length(cx)-1
     k(i) = (pdd(i))/(1+pd(i)^2)^(1.5);
  end
  
  cx= cx'; % 参考轨迹的X坐标
  cy =cy'; % 参考轨迹的Y坐标
  cyaw = atan(pd'); % 参考轨迹每一点处航向角
  ck = k'; % 参考轨迹每一点处的曲率
%%%%%%%%%%%%%%%%%%%%   above things are preprocessing   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

i = 1;
T = 80; % 总的仿真时长
lastIndex = length(cx); % 参考轨迹最后一个轨迹点的序号
x = 0.1; y = -0.1; yaw = 0.1; v = 0.1; % 车辆初始时刻的状态
U = [0.01;0.01];
vd1_p = 0;
vd2_p = 0;
vd_p = [vd1_p; vd2_p];
ind =0; % 记录当前车辆运动到参考轨迹的序号
figure % 绘制参考轨迹
     plot(cx,cy,'r-')
     hold on
     
% 车辆行驶在参考轨迹期间     
while ind < length(cx)
   
  [delta,v,ind,e,U, vd_p ] = mpc_control(x,y,yaw,cx,cy,cyaw,ck,dt,Length,Q,R,U,target_v,vd_p) ;
   
    if abs(e)> 3% we do not allow the vehicle deviation larger than 4
        fprintf('diviation too big!\n')
        break;
    end
    delta
     [x,y,yaw,v] = update(x,y,yaw,v, delta, dt,Length, max_steer); %update the vehicle state for next iteration
     posx(i) = x;
     posy(i)  =y;
     i = i+1;

     plot(posx(i-1),posy(i-1),'bo')
     pause(0.01);
     
      hold on
end

%% supplimented functions are as follows:
% 仿真中用到的函数定义
% function"Update" updates vehicle states更新车辆的状态（车辆的三个位置坐标以及车速）
function [x, y, yaw, v] = update(x, y, yaw, v, delta,dt,Length,max_steer)% update x, y, yaw, and velocity
 delta = max(min(max_steer, delta), -max_steer);
    x = x + v * cos(yaw) * dt + randn(1)* 0;
    y = y + v * sin(yaw) * dt +  randn(1)* 0;
    yaw = yaw + v / Length * tan(delta) * dt ;
   v = v ;
end

function [a] = PIDcontrol(target_v, current_v, Kp) %we originally control v separatelyPID比例速度控制器
a = Kp * (target_v - current_v);
end

function [angle] = pipi(angle) % the unit of angle is in rad, but in this case, you dont need to use it ;
% 将弧度限制在[-pi,pi]范围中
if (angle > pi)
    angle =  angle - 2*pi;
elseif (angle < -pi)
    angle = angle + 2*pi;
else
    angle = angle;
end
end

% mpc控制主函数
function    [delta,v,ind,e,U, vd_p ] = ...
    mpc_control(x,y,yaw,cx,cy,cyaw,ck,dt,Length,Q,R,U,target_v,vd_p) 

u = [x,y,yaw]; % 传入车辆最初的状态量
[ind, e] = calc_target_index(x,y,cx,cy,cyaw); % find current vehicle location, it is represented by reference index
vd1 = target_v ; % 目标车速
k = ck(ind); % 车辆在参考轨迹点上的曲率
vd2 = atan(Length * k); % 目标前轮转角

r = [cx(ind), cy(ind), cyaw(ind)];
    Nx=3; % 状态量的个数
    Nu =2; % 控制量的个数
    Np =60; % 预测步长
    Nc=30; % 控制步长
    Row=10; % 松弛因子
    t_d =u(3); % 车辆的航向角状态
    
    kesi=zeros(Nx+Nu,1); % 记录当前车辆的状态量以及控制量偏差
    kesi(1)=u(1)-r(1);%u(1)==X(1)
    kesi(2)=u(2)-r(2);%u(2)==X(2)
    kesi(3)=t_d-r(3); %u(3)==X(3)
    kesi(4)=U(1);
    kesi(5)=U(2);
%     fprintf('Update start, u(1)=%4.2f\n',U(1))
%     fprintf('Update start, u(2)=%4.2f\n',U(2))
    T = dt;
    % Mobile Robot Parameters
    L = Length;
    % Mobile Robot variable
    
% 矩阵初始化   
    delta_u=zeros(Nx,Nu);
    
    a=[1    0   -vd1*sin(t_d)*T;
       0    1   vd1*cos(t_d)*T;
       0    0   1;];
    b=[cos(t_d)*T   0;
       sin(t_d)*T   0;
       tan(vd2)*T/L      vd1*T/(L * (cos(vd2)^2))];
    A_cell=cell(2,2);
    B_cell=cell(2,1);
    A_cell{1,1}=a;
    A_cell{1,2}=b;
    A_cell{2,1}=zeros(Nu,Nx);
    A_cell{2,2}=eye(Nu);
    B_cell{1,1}=b;
    B_cell{2,1}=eye(Nu);
    A=cell2mat(A_cell);
    B=cell2mat(B_cell);
    C=[1 0 0 0 0;0 1 0 0 0;0 0 1 0 0;];
    PHI_cell=cell(Np,1);
    THETA_cell=cell(Np,Nc);
    for j=1:1:Np
        PHI_cell{j,1}=C*A^j;
        for k=1:1:Nc
            if k<=j
                THETA_cell{j,k}=C*A^(j-k)*B;
            else 
                THETA_cell{j,k}=zeros(Nx,Nu);
            end
        end
    end
    PHI=cell2mat(PHI_cell);%size(PHI)=[Nx*Np Nx+Nu]
    THETA=cell2mat(THETA_cell);%size(THETA)=[Nx*Np Nu*(Nc+1)]
    H_cell=cell(2,2);
    H_cell{1,1}=THETA'*Q*THETA+R;
    H_cell{1,2}=zeros(Nu*Nc,1);
    H_cell{2,1}=zeros(1,Nu*Nc);
    H_cell{2,2}=Row;
    H=cell2mat(H_cell);
    H = 1/2*(H+ H');
     error=PHI*kesi;
    f_cell=cell(1,2);
    f_cell{1,1}=2*error'*Q*THETA;
    f_cell{1,2}=0;
%     f=(cell2mat(f_cell))';
    f=cell2mat(f_cell);
    
 %% 以下为约束生成区域
 % 不等式约束
    A_t=zeros(Nc,Nc); % 见falcone论文P181
    for p=1:1:Nc
        for q=1:1:Nc
            if q<=p 
                A_t(p,q)=1;
            else 
                A_t(p,q)=0;
            end
        end 
    end 
    A_I=kron(A_t,eye(Nu)); % 对应于falcone论文的约束处理的矩阵A，求克罗内积
    Ut=kron(ones(Nc,1),U); % 
    umin=[-0.2;-0.54;]; % 维数与控制变量个数相同
    umax=[0.2;0.332];
    delta_umin=[-0.05;-0.0082;];
    delta_umax=[0.05;0.0082];
%     delta_umin = [ -2.2 ; -0.64];
%     delta_umax = [  0.2 ;  0.64];
    Umin=kron(ones(Nc,1),umin);
    Umax=kron(ones(Nc,1),umax);
    A_cons_cell={A_I zeros(Nu*Nc,1);-A_I zeros(Nu*Nc,1)};
    b_cons_cell={Umax-Ut;-Umin+Ut};
    A_cons=cell2mat(A_cons_cell); % （求解方程）状态量不等式约束增益矩阵，转换为绝对值得取值范围
    b_cons=cell2mat(b_cons_cell); % （求解方程）状态量不等式约束的取值
   % 状态量约束
    M=10; 
    delta_Umin=kron(ones(Nc,1),delta_umin);
    delta_Umax=kron(ones(Nc,1),delta_umax);
    lb=[delta_Umin;0]; %（求解方程）状态量下界，包含控制时域内控制增量和松弛因子
    ub=[delta_Umax;M]; % （求解方程）状态量上界，包含控制时域内控制增量和松弛因子

    %% 开始求解过程
%     options = optimset('Algorithm','interior-point-convex');
    options = optimoptions('quadprog','Display','iter','MaxIterations',100,'TolFun',1e-16);
    %options = optimset('Algorithm','interior-point-convex'); 
    [X,fval,exitflag]=quadprog(H,f,A_cons,b_cons,[],[],lb,ub,[],options);
    %% 计算输出
%     for i = 1:length(X)
%     if X == [] 
%         delta_u(1)=randn(1)*0.01;
%         delta_u(2)=randn(1)*0.01;
%     elseif  X(i) == 0
%         delta_u(1)=randn(1)*0.01;
%         delta_u(2)=randn(1)*0.01;
%     else
%         delta_u(1)=X(1);
%         delta_u(2)=X(2);
%     end
%     end
        delta_u(1)=X(1); % 标准二次型求解出的控制量增量
        delta_u(2)=X(2);
        
    U(1)=kesi(4)+delta_u(1) ; % 用于存储上一时刻的控制量
    U(2)=kesi(5)+delta_u(2); %kesi here is previous step U
    u_real(1)=U(1)+vd1; %  vd1 and vd2 here is U_ref 得出当前时刻控制周期内车辆实际的控制量
    u_real(2)=U(2)+vd2;
   delta=  u_real(2);
    v = u_real(1);
%     U(1)=kesi(4)+delta_u(1) + vd1 - vd_p(1);
%     U(2)=kesi(5)+delta_u(2) + vd2 - vd_p(2);

    kesi(4) = U(1);
    kesi(5) = U(2);
    vd_p  = [vd1;vd2 ]; 
%     delta =  U(2)
%     v = 3

end

function [ind, error] = calc_target_index(x,y, cx,cy,cyaw)% find my location, and lateral error
% 找到当前时刻车辆的位置（找到距离参考轨迹路点最近的点）
N =  length(cx);
Distance = zeros(N,1);
for i = 1:N
Distance(i) =  sqrt((cx(i)-x)^2 + (cy(i)-y)^2);
end
[value, location]= min(Distance); % 找到当前车辆的位置距离参考轨迹上路点最近的那个点以及他所在的索引号
ind = location

dx1 =  cx(ind) - x;
dy1 =  cy(ind) - y ;
angle = pipi(cyaw(ind)-atan(dy1/dx1)); % 参考轨迹路点与车辆当前航向角之差
heading = cyaw(ind)*180/pi % 当前参考路点的航向角（单位角度）
    if y<cy(ind) % 车辆位置位于参考轨迹右边时，误差为负，反之为正
        error = -value;
    else
        error = value;
    end
% error = value;
end

