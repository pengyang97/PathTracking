function [sys,x0,str,ts] = MY_MPCController3(t,x,u,flag)
%   �ú�����д�ĵ�3��S����������(MATLAB�汾��R2011a)
%   �޶��ڳ����˶�ѧģ�ͣ�������Ϊ�ٶȺ�ǰ��ƫ�ǣ�ʹ�õ�QPΪ�°汾��QP�ⷨ
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
  [sys,x0,str,ts] = mdlInitializeSizes; % Initialization flag = 0��ʼ��
  
 case 2
  sys = mdlUpdates(t,x,u); % Update discrete states flag =2 ������ɢ״̬
  
 case 3
  sys = mdlOutputs(t,x,u); % Calculate outputs flag =3 �������
 
% flag = 1 mdlDerivatives���㵼����
% flag = 4 mdlGetTimeOfNextVarHit������һ���Ĳ�������
% flag = 9 mdlTerminate��������ʱ������
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

sizes = simsizes; %�ú�������һ��δ��ʼ����sizes�ṹ�����������롢�����״̬���������Լ�����������
sizes.NumContStates  = 0; %����״̬������������Ϊ0
sizes.NumDiscStates  = 3; %��ɢ״̬������������Ϊ3��ָ���ǳ�����X��Y�����Լ������ĺ���Ǧ�
sizes.NumOutputs     = 2; %���������������Ϊ2��ָ���ǿ��������������������������v��ǰ��ת�Ǧ�
sizes.NumInputs      = 3; %���������������Ϊ3******���������ʾ�����д��󣬾͸�Ϊ5�������еĳ����������
sizes.DirFeedthrough = 1; %ֱ����ͨ��־ Matrix D is non-empty.
sizes.NumSampleTimes = 1; %����ʱ�������
sys = simsizes(sizes); % ��sizes�ṹ�е���Ϣ���ݸ�sys��sys��һ������Simulink������Ϣ����
x0 =[0;0;0]; % ������ʼ��ʼʱ��״ֵ̬   
global U; % ����ȫ�ֱ���U
U=[0;0];
% Initialize the discrete states.
str = [];             % Set str to an empty matrix.��������Ϊ�վ���
ts  = [0.1 0];       % sample time: [period, offset]���ò�����ʱ���Լ�ƫ�������������0ʱ�̿�ʼ��ÿ��0.1s����һ��
%End of mdlInitializeSizes
		      
%==============================================================
% Update the discrete states
%==============================================================
function sys = mdlUpdates(t,x,u)
  
sys = x; % ������΢�е����ʣ�Ϊʲôֱ�ӽ�״̬��ֵ����Ӧ����״̬���µķ�����
%End of mdlUpdate.

%==============================================================
% Calculate outputs
%==============================================================
function sys = mdlOutputs(t,x,u)
    global a b u_piao;
    global U;
    global kesi;
    tic % ���������ʱ�����������¼ִ�� tic ����ʱ���ڲ�ʱ�䣬toc����ʾ���õ�ʱ��
    Nx=3; % ״̬���ĸ���
    Nu =2; % �������ĸ���
    Np =60; % Ԥ�ⲽ��
    Nc=30; % ���Ʋ���
    Row=10; % �ɳ�����
    fprintf('Update start, t=%6.3f\n',t)
    t_d =u(3)*pi/180; % CarSim�����Ϊ�Ƕȣ��Ƕ�ת��Ϊ����u(3)*3.1415926/180
    

%    %ֱ��·��
%     r(1)=5*t;
%     r(2)=5;
%     r(3)=0;
%     vd1=5;
%     vd2=0;
    %�뾶Ϊ25m��Բ�ι켣,�ٶ�Ϊ5m/s,���̶����Բ������̵���ʽ������
    r(1)=25*sin(0.2*t); % �ο��켣ÿ��ʱ�̶�Ӧ��X����
    r(2)=25+10-25*cos(0.2*t); % �ο��켣ÿ��ʱ�̶�Ӧ��Y����
    r(3)=0.2*t; % �ο��켣ÿ��ʱ�̶�Ӧ�ĺ����
    vd1=5; % ������Ŀ�공��
    vd2=0.104; % ������Ŀ��ǰ��ת��
%     %�뾶Ϊ25m��Բ�ι켣,�ٶ�Ϊ3m/s
%     r(1)=25*sin(0.12*t);
%     r(2)=25+10-25*cos(0.12*t);
%     r(3)=0.12*t;
%     vd1=3;
%     vd2=0.104;
	%�뾶Ϊ25m��Բ�ι켣,�ٶ�Ϊ10m/s
%      r(1)=25*sin(0.4*t);
%      r(2)=25+10-25*cos(0.4*t);
%      r(3)=0.4*t;
%      vd1=10;
%      vd2=0.104;
%     %�뾶Ϊ25m��Բ�ι켣,�ٶ�Ϊ4m/s
%      r(1)=25*sin(0.16*t);
%      r(2)=25+10-25*cos(0.16*t);
%      r(3)=0.16*t;
%      vd1=4;
%      vd2=0.104;
    kesi=zeros(Nx+Nu,1);
    kesi(1)=u(1)-r(1);%u(1)==X(1)
    kesi(2)=u(2)-r(2);%u(2)==X(2)
    kesi(3)=t_d-r(3); %u(3)==X(3)
    kesi(4)=U(1); % ���㽫�ڷ��������ÿ��ʱ�̵ó��Ŀ������������������ۼƵ���
    kesi(5)=U(2);
    fprintf('Update start, u(1)=%4.2f\n',U(1))
    fprintf('Update start, u(2)=%4.2f\n',U(2))

    T=0.1;
    T_all=40;%��ʱ�趨���ܵķ���ʱ�䣬��Ҫ�����Ƿ�ֹ���������켣Խ��
    % Mobile Robot Parameters
    L = 2.6;
    % Mobile Robot variable
    
    
%�����ʼ��   
    u_piao=zeros(Nx,Nu);
    Q=100*eye(Nx*Np,Nx*Np); % ���յ�״̬��Ȩ�������ڵ���ÿ��Ԥ����ϣ�����״̬��������Ŀ�꺯���б���    
    R=5*eye(Nu*Nc); % ���յ�״̬������������Ȩ�������ڵ��ڡ�������ƫ��͡�״̬��ƫ���Ŀ�꺯���еı���
    a=[1    0   -vd1*sin(t_d)*T;
       0    1   vd1*cos(t_d)*T;
       0    0   1;];
    b=[cos(t_d)*T   0;
       sin(t_d)*T   0;
       tan(vd2)*T/L      vd1*T/(cos(vd2)^2);];
   % ������Щ������ת����õ��µĳ�������ģ����ɢ״̬�ռ����ʽ�ľ���
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
    % ������������µĳ�������ģ����ɢ״̬�ռ����ʽ������ǰԤ��Np��ʱ��õ���Ԥ��ģ�ͱ���ʽ��ϵ������׺ͦ�
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
    % ���Ʋ���ΪNc��������Np>=Nc������Nc֮��Ŀ������ã�ͨ����ֻȡǰNc+1��
    THETA=cell2mat(THETA_cell);%size(THETA)=[Nx*Np Nu*(Nc+1)]
    H_cell=cell(2,2);
    H_cell{1,1}=THETA'*Q*THETA+R;
    H_cell{1,2}=zeros(Nu*Nc,1);
    H_cell{2,1}=zeros(1,Nu*Nc);
    H_cell{2,2}=Row;
    H=cell2mat(H_cell);

     error=PHI*kesi; % Ԥ��ģ�͵����
    f_cell=cell(1,2);
    f_cell{1,1}=2*error'*Q*THETA;
    f_cell{1,2}=0;
%     f=(cell2mat(f_cell))';
    f=cell2mat(f_cell);
    
 %% ����ΪԼ����������
 %����ʽԼ��
    A_t=zeros(Nc,Nc);%��falcone���� P181
    for p=1:1:Nc
        for q=1:1:Nc
            if q<=p 
                A_t(p,q)=1;
            else 
                A_t(p,q)=0;
            end
        end 
    end 
    A_I=kron(A_t,eye(Nu));%��Ӧ��falcone����Լ�������ľ���A,������ڿ˻�
    Ut=kron(ones(Nc,1),U);%�˴��о�������Ŀ����ڿƻ�������,��ʱ����˳��
    umin=[-0.2;-0.54;];%ά������Ʊ����ĸ�����ͬ
    umax=[0.2;0.332];
    delta_umin=[-0.05;-0.0082;];%delta_umin=[0.05;-0.0082;];ԭ�����ٶȱ仯�½�û�мӸ���
    delta_umax=[0.05;0.0082];
    Umin=kron(ones(Nc,1),umin);
    Umax=kron(ones(Nc,1),umax);
    A_cons_cell={A_I zeros(Nu*Nc,1);-A_I zeros(Nu*Nc,1)};
    b_cons_cell={Umax-Ut;-Umin+Ut};
    A_cons=cell2mat(A_cons_cell);%����ⷽ�̣�״̬������ʽԼ���������ת��Ϊ����ֵ��ȡֵ��Χ
    b_cons=cell2mat(b_cons_cell);%����ⷽ�̣�״̬������ʽԼ����ȡֵ
   % ״̬��Լ��
    M=10; 
    delta_Umin=kron(ones(Nc,1),delta_umin);
    delta_Umax=kron(ones(Nc,1),delta_umax);
    lb=[delta_Umin;0];%����ⷽ�̣�״̬���½磬��������ʱ���ڿ����������ɳ�����
    ub=[delta_Umax;M];%����ⷽ�̣�״̬���Ͻ磬��������ʱ���ڿ����������ɳ�����
    
    %% ��ʼ������
    %options = optimset('Algorithm','active-set');�°�quadprog��������Ч����������ʹ���ڵ㷨
    options = optimset('Algorithm','interior-point-convex'); 
    [X,fval,exitflag]=quadprog(H,f,A_cons,b_cons,[],[],lb,ub,[],options);
    %% �������
    u_piao(1)=X(1);
    u_piao(2)=X(2);
    U(1)=kesi(4)+u_piao(1);%���ڴ洢��һ��ʱ�̵Ŀ�����
    U(2)=kesi(5)+u_piao(2);
    u_real(1)=U(1)+vd1;
    u_real(2)=U(2)+vd2;
    sys= u_real;
    toc
% End of mdlOutputs.