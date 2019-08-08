function oneD_Bar(E,A,L,P)
%PΪ������֪�Ľڵ���
% oneD_Bar(2e5,0.6,0.1,[100;0])  �н�
%�ܽڵ���P=R+F=[Rx1+F1 Rx2+F2]'=[P1 P2]'
%�ܽڵ�λ��q=[u1 u2]'
%BC(u)
%���P1��P2����֪����P=[P1 P2]'����ֻ��֪P1����P=[P1]'��u=k\P
i=1;
j=2;
KK=zeros(2);
k=Bar1D2Node_stiffness(E,A,L);
KK=Bar1D2Node_Assembly(KK,k,i,j);
u=k\P;
stress=Bar1D2Node_Stress(k,u,A);
forces=Bar1D2Node_Force(k,u);
strain=Bar1D2Node_Strain(E,stress);
disp('�ڵ�λ��u���ڵ�Ӧ��stress���ڵ�Ӧ��strain��')
u
stress
strain
disp('��������ܸնȷ��̣��ܽڵ������ܽڵ�λ�ƣ�')
KK
forces
u




function k=Bar1D2Node_stiffness(E,A,L)
%�ú�������һά���⣬�˵�Ԫ�ĸնȾ��� p45,3-39
%���뵯��ģ��E����������A�ͳ���L
%�����Ԫ�նȾ���k(2*2)
%-----------------------------------
k=(E*A/L)*[1 -1;-1 1];

function z=Bar1D2Node_Assembly(KK,k,i,j)
%�ú����Ե�Ԫ�նȾ��������װ  
%���뵥Ԫ�նȾ���k����Ԫ�ڵ���i��j
%�������նȾ���KK,1D����ĵ�Ԫ�նȾ����������նȾ���
%--------------------------------------
DOF(1)=i;
DOF(2)=j;
for n1=1:2
    for n2=1:2
        KK(DOF(n1),DOF(n2))=KK(DOF(n1),DOF(n2))+k(n1,n2);
    end
end
z=KK;

function stress=Bar1D2Node_Stress(k,u,A)
%���㵥Ԫ��Ӧ��
%���뵥Ԫ�նȾ���k����Ԫ��λ������u
%������A���㵥ԪӦ��ʸ��
%�����ԪӦ��stress
%---------------------------------------
stress=k*u/A;

function forces=Bar1D2Node_Force(k,u)
%���㵥Ԫ�ڵ���ʸ��
%���뵥Ԫ�նȾ���k�͵�Ԫλ������u
%�����Ԫ�ڵ���ʸ��forces
%---------------------------------------
forces=k*u;

function strain=Bar1D2Node_Strain(E,stress)
%���㵥Ԫ�ڵ�Ӧ��
%���뵯��ģ��E�͵�Ԫ�ڵ�Ӧ��
%�����ԪӦ��strain
%-------------------------------------------
strain=stress/E;

