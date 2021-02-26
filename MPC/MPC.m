clc
clear all

Kp = 1.0 ; 
dt = 0.1  ;% [s]
Length = 2.9 ;% [m] wheel base of vehicle
Nx=3; % ״̬���ĸ���
Nu =2; % �������ĸ���
Np =60; % Ԥ�ⲽ��
Nc=30; % ���Ʋ���
Row=10; % �ɳ�����
Q=100*eye(Nx*Np,Nx*Np); % ״̬����Ȩ����Q������ÿ��Ԥ����ϣ�����״̬��������Ŀ�꺯���б���    
R=1*eye(Nc*Nu); % ��������Ȩ�������ڵ��ڿ�����ƫ����Ŀ�꺯���еı���
    
max_steer =60 * pi/180; % in rad �������ת�ǣ����ȣ�
target_v =30.0 / 3.6; % Ŀ�공�٣�m/s��
% �趨�ο��켣���������ߣ�
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
 % ����һ�׵���
 for i = 1:length(cx)-1
 pd(i) = (p(i+1,2)-p(i,2))/(p(i+1,1)-p(i,1));
 end
 pd(end+1) = pd(end);
  % ������׵����������������Ĳ��̽��ƴ���΢�֣�
  % f'(x0)=(f(x0+h)-f(x0-h))/2h
  % f(x0+h)=f(x0)+hf'(x0)+(h^2/2!)f"(x0)+(h^3/3!)f"'(��)
 for i =2: length(cx)-1
     pdd(i) = (p(i+1,2)-2*p(i,2) + p(i-1,2))/(0.5*(-p(i-1,1)+p(i+1,1)))^2;
 end
      pdd(1) = pdd(2);
     pdd(length(cx)) = pdd(length(cx)-1);
  % �������� 
  % y"/(1+y'^2)^1.5
  for i  = 1:length(cx)-1
     k(i) = (pdd(i))/(1+pd(i)^2)^(1.5);
  end
  
  cx= cx'; % �ο��켣��X����
  cy =cy'; % �ο��켣��Y����
  cyaw = atan(pd'); % �ο��켣ÿһ�㴦�����
  ck = k'; % �ο��켣ÿһ�㴦������
%%%%%%%%%%%%%%%%%%%%   above things are preprocessing   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

i = 1;
T = 80; % �ܵķ���ʱ��
lastIndex = length(cx); % �ο��켣���һ���켣������
x = 0.1; y = -0.1; yaw = 0.1; v = 0.1; % ������ʼʱ�̵�״̬
U = [0.01;0.01];
vd1_p = 0;
vd2_p = 0;
vd_p = [vd1_p; vd2_p];
ind =0; % ��¼��ǰ�����˶����ο��켣�����
figure % ���Ʋο��켣
     plot(cx,cy,'r-')
     hold on
     
% ������ʻ�ڲο��켣�ڼ�     
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
% �������õ��ĺ�������
% function"Update" updates vehicle states���³�����״̬������������λ�������Լ����٣�
function [x, y, yaw, v] = update(x, y, yaw, v, delta,dt,Length,max_steer)% update x, y, yaw, and velocity
 delta = max(min(max_steer, delta), -max_steer);
    x = x + v * cos(yaw) * dt + randn(1)* 0;
    y = y + v * sin(yaw) * dt +  randn(1)* 0;
    yaw = yaw + v / Length * tan(delta) * dt ;
   v = v ;
end

function [a] = PIDcontrol(target_v, current_v, Kp) %we originally control v separatelyPID�����ٶȿ�����
a = Kp * (target_v - current_v);
end

function [angle] = pipi(angle) % the unit of angle is in rad, but in this case, you dont need to use it ;
% ������������[-pi,pi]��Χ��
if (angle > pi)
    angle =  angle - 2*pi;
elseif (angle < -pi)
    angle = angle + 2*pi;
else
    angle = angle;
end
end

% mpc����������
function    [delta,v,ind,e,U, vd_p ] = ...
    mpc_control(x,y,yaw,cx,cy,cyaw,ck,dt,Length,Q,R,U,target_v,vd_p) 

u = [x,y,yaw]; % ���복�������״̬��
[ind, e] = calc_target_index(x,y,cx,cy,cyaw); % find current vehicle location, it is represented by reference index
vd1 = target_v ; % Ŀ�공��
k = ck(ind); % �����ڲο��켣���ϵ�����
vd2 = atan(Length * k); % Ŀ��ǰ��ת��

r = [cx(ind), cy(ind), cyaw(ind)];
    Nx=3; % ״̬���ĸ���
    Nu =2; % �������ĸ���
    Np =60; % Ԥ�ⲽ��
    Nc=30; % ���Ʋ���
    Row=10; % �ɳ�����
    t_d =u(3); % �����ĺ����״̬
    
    kesi=zeros(Nx+Nu,1); % ��¼��ǰ������״̬���Լ�������ƫ��
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
    
% �����ʼ��   
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
    
 %% ����ΪԼ����������
 % ����ʽԼ��
    A_t=zeros(Nc,Nc); % ��falcone����P181
    for p=1:1:Nc
        for q=1:1:Nc
            if q<=p 
                A_t(p,q)=1;
            else 
                A_t(p,q)=0;
            end
        end 
    end 
    A_I=kron(A_t,eye(Nu)); % ��Ӧ��falcone���ĵ�Լ������ľ���A��������ڻ�
    Ut=kron(ones(Nc,1),U); % 
    umin=[-0.2;-0.54;]; % ά������Ʊ���������ͬ
    umax=[0.2;0.332];
    delta_umin=[-0.05;-0.0082;];
    delta_umax=[0.05;0.0082];
%     delta_umin = [ -2.2 ; -0.64];
%     delta_umax = [  0.2 ;  0.64];
    Umin=kron(ones(Nc,1),umin);
    Umax=kron(ones(Nc,1),umax);
    A_cons_cell={A_I zeros(Nu*Nc,1);-A_I zeros(Nu*Nc,1)};
    b_cons_cell={Umax-Ut;-Umin+Ut};
    A_cons=cell2mat(A_cons_cell); % ����ⷽ�̣�״̬������ʽԼ���������ת��Ϊ����ֵ��ȡֵ��Χ
    b_cons=cell2mat(b_cons_cell); % ����ⷽ�̣�״̬������ʽԼ����ȡֵ
   % ״̬��Լ��
    M=10; 
    delta_Umin=kron(ones(Nc,1),delta_umin);
    delta_Umax=kron(ones(Nc,1),delta_umax);
    lb=[delta_Umin;0]; %����ⷽ�̣�״̬���½磬��������ʱ���ڿ����������ɳ�����
    ub=[delta_Umax;M]; % ����ⷽ�̣�״̬���Ͻ磬��������ʱ���ڿ����������ɳ�����

    %% ��ʼ������
%     options = optimset('Algorithm','interior-point-convex');
    options = optimoptions('quadprog','Display','iter','MaxIterations',100,'TolFun',1e-16);
    %options = optimset('Algorithm','interior-point-convex'); 
    [X,fval,exitflag]=quadprog(H,f,A_cons,b_cons,[],[],lb,ub,[],options);
    %% �������
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
        delta_u(1)=X(1); % ��׼�����������Ŀ���������
        delta_u(2)=X(2);
        
    U(1)=kesi(4)+delta_u(1) ; % ���ڴ洢��һʱ�̵Ŀ�����
    U(2)=kesi(5)+delta_u(2); %kesi here is previous step U
    u_real(1)=U(1)+vd1; %  vd1 and vd2 here is U_ref �ó���ǰʱ�̿��������ڳ���ʵ�ʵĿ�����
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
% �ҵ���ǰʱ�̳�����λ�ã��ҵ�����ο��켣·������ĵ㣩
N =  length(cx);
Distance = zeros(N,1);
for i = 1:N
Distance(i) =  sqrt((cx(i)-x)^2 + (cy(i)-y)^2);
end
[value, location]= min(Distance); % �ҵ���ǰ������λ�þ���ο��켣��·��������Ǹ����Լ������ڵ�������
ind = location

dx1 =  cx(ind) - x;
dy1 =  cy(ind) - y ;
angle = pipi(cyaw(ind)-atan(dy1/dx1)); % �ο��켣·���복����ǰ�����֮��
heading = cyaw(ind)*180/pi % ��ǰ�ο�·��ĺ���ǣ���λ�Ƕȣ�
    if y<cy(ind) % ����λ��λ�ڲο��켣�ұ�ʱ�����Ϊ������֮Ϊ��
        error = -value;
    else
        error = value;
    end
% error = value;
end

