function oneD_Bar(E,A,L,P)
%P为输入已知的节点力
% oneD_Bar(2e5,0.6,0.1,[100;0])  有解
%总节点力P=R+F=[Rx1+F1 Rx2+F2]'=[P1 P2]'
%总节点位移q=[u1 u2]'
%BC(u)
%如果P1，P2均已知，则P=[P1 P2]'，或只已知P1，则P=[P1]'。u=k\P
i=1;
j=2;
KK=zeros(2);
k=Bar1D2Node_stiffness(E,A,L);
KK=Bar1D2Node_Assembly(KK,k,i,j);
u=k\P;
stress=Bar1D2Node_Stress(k,u,A);
forces=Bar1D2Node_Force(k,u);
strain=Bar1D2Node_Strain(E,stress);
disp('节点位移u，节点应力stress，节点应变strain：')
u
stress
strain
disp('依次输出总刚度方程，总节点力，总节点位移：')
KK
forces
u




function k=Bar1D2Node_stiffness(E,A,L)
%该函数计算一维问题，杆单元的刚度矩阵 p45,3-39
%输入弹性模量E，横截面面积A和长度L
%输出单元刚度矩阵k(2*2)
%-----------------------------------
k=(E*A/L)*[1 -1;-1 1];

function z=Bar1D2Node_Assembly(KK,k,i,j)
%该函数对单元刚度矩阵进行组装  
%输入单元刚度矩阵k，单元节点编号i、j
%输出整体刚度矩阵KK,1D问题的单元刚度矩阵就是整体刚度矩阵
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
%计算单元的应力
%输入单元刚度矩阵k，单元的位移列阵u
%横截面积A计算单元应力矢量
%输出单元应力stress
%---------------------------------------
stress=k*u/A;

function forces=Bar1D2Node_Force(k,u)
%计算单元节点力矢量
%输入单元刚度矩阵k和单元位移列阵u
%输出单元节点力矢量forces
%---------------------------------------
forces=k*u;

function strain=Bar1D2Node_Strain(E,stress)
%计算单元节点应变
%输入弹性模量E和单元节点应力
%输出单元应变strain
%-------------------------------------------
strain=stress/E;

