clc
clear
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%Deriving DKM for Hand fingers(excluding the thumb)
syms q1 q2 q3 q4 L1 L2 L3 L4
DH_table_arm=[0 0 q1 0 ;
     L1 pi/2 0 q2;
     L2 -pi/2 0 q3;
     L3 0 0 q4;
     L4 pi/2 0 pi/2;
];
k=1;
T01=DHmatrixT( DH_table_arm(k,1),DH_table_arm(k,2),DH_table_arm(k,3),DH_table_arm(k,4) );k=k+1;
T12=DHmatrixT( DH_table_arm(k,1),DH_table_arm(k,2),DH_table_arm(k,3),DH_table_arm(k,4) );k=k+1;
T23=DHmatrixT( DH_table_arm(k,1),DH_table_arm(k,2),DH_table_arm(k,3),DH_table_arm(k,4) );k=k+1;
T34=DHmatrixT( DH_table_arm(k,1),DH_table_arm(k,2),DH_table_arm(k,3),DH_table_arm(k,4) );k=k+1;
T45=DHmatrixT( DH_table_arm(k,1),DH_table_arm(k,2),DH_table_arm(k,3),DH_table_arm(k,4) );k=k+1;
T02=simplify(T01*T12);
T03=simplify(T01*T12*T23);
T04=simplify(T01*T12*T23*T34);
T05=simplify(T01*T12*T23*T34*T45);


