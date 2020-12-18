function relabel_dq = relabel_dq_gen(s)
R = [1 0 0 0 0 0 0 ;
      0 1 0 0 0 0 0 ; 
      0 0 1 0 0 0 0 ;
      0 0 0 0 0 1 0 ;
      0 0 0 0 0 0 1 ;
      0 0 0 1 0 0 0 ;
      0 0 0 0 1 0 0 ];
relabel_dq = R*dqPlus_gen(s);

end