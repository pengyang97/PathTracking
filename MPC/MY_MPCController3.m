function [sys,x0,str,ts] = MY_MPCController3(t,x,u,flag)
%   该函数是写的第3个S函数控制器(MATLAB版本：R2011a)
%   限定于车辆运动学模型，控制量为速度和前轮偏角，使用的QP为新版本的QP解法
%   [sys,x0,str,ts] = MY_MPCController3(t,x,u,flag)
%
% is an S-function implementing the MPC controller intended for use
% with Simulink. The argument md, which is the only user supplied
% argument, contains the data structures needed by the controller. The
% input to the S-function block is a vector signal consisting of the
% measured outputs and the reference values for the controlled
% outputs. The output of the S-function block is a vector signal
% consisting of the control variables and the estimated state vector,
% potentially including estimated disturbance states.

switch flag,
 case 0
  [sys,x0,str,ts] = mdlInitializeSizes; % Initialization flag = 0初始化
  
 case 2
  sys = mdlUpdates(t,x,u); % Update discrete states flag =2 更新离散状态
  
 case 3
  sys = mdlOutputs(t,x,u); % Calculate outputs flag =3 计算输出
 
% flag = 1 mdlDerivatives计算导数；
% flag = 4 mdlGetTimeOfNextVarHit计算下一步的采样步长
% flag = 9 mdlTerminate结束仿真时的任务
 case {1,4,9} % Unused flags 
  sys = [];
  
 otherwise
  error(['unhandled flag = ',num2str(flag)]); % Error handling
end
% End of dsfunc.

%==============================================================
% Initialization
%==============================================================

function [sys,x0,str,ts] = mdlInitializeSizes

% Call simsizes for a sizes structure, fill it in, and convert it 
% to a sizes array.

sizes = simsizes; %该函数返回一个未初始化的sizes结构，包含了输入、输出、状态的数量，以及其他块特性
sizes.NumContStates  = 0; %连续状态的数量，这里为0
sizes.NumDiscStates  = 3; %离散状态的数量，这里为3，指的是车辆的X，Y坐标以及车辆的航向角ψ
sizes.NumOutputs     = 2; %输出的数量，这里为2，指的是控制器输出的两个控制量，车速v和前轮转角δ
sizes.NumInputs      = 3; %输入的数量，这里为3******后面如果提示这里有错误，就改为5（网上有的出现这个错误）
sizes.DirFeedthrough = 1; %直接馈通标志 Matrix D is non-empty.
sizes.NumSampleTimes = 1; %采样时间的数量
sys = simsizes(sizes); % 将sizes结构中的信息传递给sys，sys是一个保持Simulink所用信息的量
x0 =[0;0;0]; % 车辆初始开始时的状态值   
global U; % 声明全局变量U
U=[0;0];
% Initialize the discrete states.
str = [];             % Set str to an empty matrix.必须设置为空矩阵
ts  = [0.1 0];       % sample time: [period, offset]设置采样的时间以及偏移量，即仿真从0时刻开始，每隔0.1s运行一次
%End of mdlInitializeSizes
		      
%==============================================================
% Update the discrete states
%==============================================================
function sys = mdlUpdates(t,x,u)
  
sys = x; % 这里稍微有点疑问，为什么直接将状态赋值，不应该是状态更新的方程吗？
%End of mdlUpdate.

%==============================================================
% Calculate outputs
%==============================================================
function sys = mdlOutputs(t,x,u)
    global a b u_piao;
    global U;
    global kesi;
    tic % 启动秒表计时器，函数会记录执行 tic 命令时的内部时间，toc会显示已用的时间
    Nx=3; % 状态量的个数
    Nu =2; % 控制量的个数
    Np =60; % 预测步长
    Nc=30; % 控制步长
    Row=10; % 松弛因子
    fprintf('Update start, t=%6.3f\n',t)
    t_d =u(3)*pi/180; % CarSim输出的为角度，角度转换为弧度u(3)*3.1415926/180
    

%    %直线路径
%     r(1)=5*t;
%     r(2)=5;
%     r(3)=0;
%     vd1=5;
%     vd2=0;
    %半径为25m的圆形轨迹,速度为5m/s,方程都是以参数方程的形式给出的
    r(1)=25*sin(0.2*t); % 参考轨迹每个时刻对应的X坐标
    r(2)=25+10-25*cos(0.2*t); % 参考轨迹每个时刻对应的Y坐标
    r(3)=0.2*t; % 参考轨迹每个时刻对应的航向角
    vd1=5; % 车辆的目标车速
    vd2=0.104; % 车辆的目标前轮转角
%     %半径为25m的圆形轨迹,速度为3m/s
%     r(1)=25*sin(0.12*t);
%     r(2)=25+10-25*cos(0.12*t);
%     r(3)=0.12*t;
%     vd1=3;
%     vd2=0.104;
	%半径为25m的圆形轨迹,速度为10m/s
%      r(1)=25*sin(0.4*t);
%      r(2)=25+10-25*cos(0.4*t);
%      r(3)=0.4*t;
%      vd1=10;
%      vd2=0.104;
%     %半径为25m的圆形轨迹,速度为4m/s
%      r(1)=25*sin(0.16*t);
%      r(2)=25+10-25*cos(0.16*t);
%      r(3)=0.16*t;
%      vd1=4;
%      vd2=0.104;
    kesi=zeros(Nx+Nu,1);
    kesi(1)=u(1)-r(1);%u(1)==X(1)
    kesi(2)=u(2)-r(2);%u(2)==X(2)
    kesi(3)=t_d-r(3); %u(3)==X(3)
    kesi(4)=U(1); % 方便将在仿真过程中每个时刻得出的控制量增量进行依次累计叠加
    kesi(5)=U(2);
    fprintf('Update start, u(1)=%4.2f\n',U(1))
    fprintf('Update start, u(2)=%4.2f\n',U(2))

    T=0.1;
    T_all=40;%临时设定，总的仿真时间，主要功能是防止计算期望轨迹越界
    % Mobile Robot Parameters
    L = 2.6;
    % Mobile Robot variable
    
    
%矩阵初始化   
    u_piao=zeros(Nx,Nu);
    Q=100*eye(Nx*Np,Nx*Np); % 最终的状态加权矩阵，用于调节每个预测点上，三个状态量各自在目标函数中比重    
    R=5*eye(Nu*Nc); % 最终的状态（控制量）加权矩阵，用于调节“控制量偏差”和“状态量偏差”在目标函数中的比重
    a=[1    0   -vd1*sin(t_d)*T;
       0    1   vd1*cos(t_d)*T;
       0    0   1;];
    b=[cos(t_d)*T   0;
       sin(t_d)*T   0;
       tan(vd2)*T/L      vd1*T/(cos(vd2)^2);];
   % 下面这些矩阵是转换后得到新的车辆线性模型离散状态空间表达式的矩阵
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
    % 下面矩阵是在新的车辆线性模型离散状态空间表达式经过向前预测Np个时域得到的预测模型表达式的系数矩阵ψ和θ
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
    % 控制步长为Nc，且满足Np>=Nc，忽略Nc之后的控制作用，通项中只取前Nc+1项
    THETA=cell2mat(THETA_cell);%size(THETA)=[Nx*Np Nu*(Nc+1)]
    H_cell=cell(2,2);
    H_cell{1,1}=THETA'*Q*THETA+R;
    H_cell{1,2}=zeros(Nu*Nc,1);
    H_cell{2,1}=zeros(1,Nu*Nc);
    H_cell{2,2}=Row;
    H=cell2mat(H_cell);

     error=PHI*kesi; % 预测模型的误差
    f_cell=cell(1,2);
    f_cell{1,1}=2*error'*Q*THETA;
    f_cell{1,2}=0;
%     f=(cell2mat(f_cell))';
    f=cell2mat(f_cell);
    
 %% 以下为约束生成区域
 %不等式约束
    A_t=zeros(Nc,Nc);%见falcone论文 P181
    for p=1:1:Nc
        for q=1:1:Nc
            if q<=p 
                A_t(p,q)=1;
            else 
                A_t(p,q)=0;
            end
        end 
    end 
    A_I=kron(A_t,eye(Nu));%对应于falcone论文约束处理的矩阵A,求克罗内克积
    Ut=kron(ones(Nc,1),U);%此处感觉论文里的克罗内科积有问题,暂时交换顺序
    umin=[-0.2;-0.54;];%维数与控制变量的个数相同
    umax=[0.2;0.332];
    delta_umin=[-0.05;-0.0082;];%delta_umin=[0.05;-0.0082;];原代码速度变化下界没有加负号
    delta_umax=[0.05;0.0082];
    Umin=kron(ones(Nc,1),umin);
    Umax=kron(ones(Nc,1),umax);
    A_cons_cell={A_I zeros(Nu*Nc,1);-A_I zeros(Nu*Nc,1)};
    b_cons_cell={Umax-Ut;-Umin+Ut};
    A_cons=cell2mat(A_cons_cell);%（求解方程）状态量不等式约束增益矩阵，转换为绝对值的取值范围
    b_cons=cell2mat(b_cons_cell);%（求解方程）状态量不等式约束的取值
   % 状态量约束
    M=10; 
    delta_Umin=kron(ones(Nc,1),delta_umin);
    delta_Umax=kron(ones(Nc,1),delta_umax);
    lb=[delta_Umin;0];%（求解方程）状态量下界，包含控制时域内控制增量和松弛因子
    ub=[delta_Umax;M];%（求解方程）状态量上界，包含控制时域内控制增量和松弛因子
    
    %% 开始求解过程
    %options = optimset('Algorithm','active-set');新版quadprog不能用有效集法，这里使用内点法
    options = optimset('Algorithm','interior-point-convex'); 
    [X,fval,exitflag]=quadprog(H,f,A_cons,b_cons,[],[],lb,ub,[],options);
    %% 计算输出
    u_piao(1)=X(1);
    u_piao(2)=X(2);
    U(1)=kesi(4)+u_piao(1);%用于存储上一个时刻的控制量
    U(2)=kesi(5)+u_piao(2);
    u_real(1)=U(1)+vd1;
    u_real(2)=U(2)+vd2;
    sys= u_real;
    toc
% End of mdlOutputs.