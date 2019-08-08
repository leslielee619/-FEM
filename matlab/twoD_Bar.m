%����һ���ĸ���ܵ�matlab����
function twoD_Bar()
%�������������������װ��
E=2.95e11;
A=0.0001;
x1=0;
y1=0;
x2=0.4;
y2=0;
x3=0.4;
y3=0.3;
x4=0;
y4=0.3;
alpha1=0;
alpha2=90;
alpha3=atan(0.75)*180/pi;
k1=Bar2D2Node_Stiffness(E,A,x1,y1,x2,y2,alpha1);
k2=Bar2D2Node_Stiffness(E,A,x2,y2,x3,y3,alpha2);
k3=Bar2D2Node_Stiffness(E,A,x1,y1,x3,y3,alpha3);
k4=Bar2D2Node_Stiffness(E,A,x3,y3,x4,y4,alpha1);
KK=zeros(8);
KK=Bar2D2Node_Assembly(KK,k1,1,2);
KK=Bar2D2Node_Assembly(KK,k2,2,3);
KK=Bar2D2Node_Assembly(KK,k3,1,3);
K=Bar2D2Node_Assembly(KK,k4,3,4);
U=K([3,5,6],[3,5,6])\[2e4;0;-2.5e4];
U=[0;0;U(1);0;U(2);U(3);0;0];
P=K*U;
stress=P/A;
strain=stress/E;
disp('��������������ĸ���ܵ�������')
disp('�Եõ����������նȾ���K,�ڵ�λ��U���ڵ���P���ڵ�Ӧ��stress���ڵ�Ӧ��strain')
K
U
P
stress
strain
end


function k=Bar2D2Node_Stiffness(E,A,x1,y1,x2,y2,alpha)
%�ú������㵥Ԫ�ĸնȾ���   p48,3-57
%���뵯��ģ��E��������A
%��4���ڵ㣨1��2��3��4����4����Ԫ��һ��k�����������ڵ㣨��1��2������ά�ģ�������նȾ���Ӧ����4���ڵ㣬�����ǰ�ά�ġ�
%�����һ���ڵ����꣨x1,y1�����ڶ����ڵ����꣨x2,y2�����Ƕ�alpha����λ�Ƕȣ�
%�����Ԫ�նȾ���k(4X4)
%-------------------------------------------------
L=sqrt((x2-x1)^2+(y2-y1)^2);
x=alpha*pi/180;
C=cos(x);
S=sin(x);
k=E*A/L*[C*C C*S -C*C -C*S; C*S S*S -C*S -S*S;
-C*C -C*S C*C C*S; -C*S -S*S C*S S*S];
end


function z = Bar2D2Node_Assembly(KK,k,i,j)
%�ú������е�Ԫ�նȾ������װ
%���뵥Ԫ�նȾ���k����Ԫ�Ľڵ���i��j
%�������նȾ���KK
%--------------------------------------------------------
DOF(1)=2*i-1;
DOF(2)=2*i;
DOF(3)=2*j-1;
DOF(4)=2*j;
for n1=1:4
    for n2=1:4
        KK(DOF(n1),DOF(n2))= KK(DOF(n1),DOF(n2))+k(n1,n2);
    end
end
z=KK;
end




