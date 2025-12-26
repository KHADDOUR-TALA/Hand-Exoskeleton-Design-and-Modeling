clc
clear
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%Deriving DKM for MCP with  misaligning displacemens
syms H V dirac1 dirac2 dirac3 alpha theta L5 L6 q5 
DH_table_arm=[V 0 0 pi/2;
    H 0 0 -pi/2;
    0 0 dirac1 0;
    0 pi/2 dirac2 pi/2;
    0 pi/2 dirac3 0;
    0 -pi/2 0 alpha-pi/2;
    0 -pi/2 0 theta;
    L6+q5 0 0 -pi/2;
    L5 -pi/2 0 -pi/2;
    
];
k=1;
T01=DHmatrixT( DH_table_arm(k,1),DH_table_arm(k,2),DH_table_arm(k,3),DH_table_arm(k,4) );k=k+1;
T12=DHmatrixT( DH_table_arm(k,1),DH_table_arm(k,2),DH_table_arm(k,3),DH_table_arm(k,4) );k=k+1;
T23=DHmatrixT( DH_table_arm(k,1),DH_table_arm(k,2),DH_table_arm(k,3),DH_table_arm(k,4) );k=k+1;
T34=DHmatrixT( DH_table_arm(k,1),DH_table_arm(k,2),DH_table_arm(k,3),DH_table_arm(k,4) );k=k+1;
T45=DHmatrixT( DH_table_arm(k,1),DH_table_arm(k,2),DH_table_arm(k,3),DH_table_arm(k,4) );k=k+1;
T56=DHmatrixT( DH_table_arm(k,1),DH_table_arm(k,2),DH_table_arm(k,3),DH_table_arm(k,4) );k=k+1;
T67=DHmatrixT( DH_table_arm(k,1),DH_table_arm(k,2),DH_table_arm(k,3),DH_table_arm(k,4) );k=k+1;
T78=DHmatrixT( DH_table_arm(k,1),DH_table_arm(k,2),DH_table_arm(k,3),DH_table_arm(k,4) );k=k+1;
T89=DHmatrixT( DH_table_arm(k,1),DH_table_arm(k,2),DH_table_arm(k,3),DH_table_arm(k,4) );k=k+1;
T02=simplify(T01*T12);
T03=simplify(T01*T12*T23);
T04=simplify(T01*T12*T23*T34);
T05=simplify(T01*T12*T23*T34*T45);
T06=simplify(T01*T12*T23*T34*T45*T56);
T09=simplify(T01*T12*T23*T34*T45*T56*T67*T78*T89);



