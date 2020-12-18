function CoM = robot_CoM_gen(s, C)

totalMass = 2*C.m_femur+2*C.m_tibia+C.m_torso+C.m_hip;

pmF1 = C.m_femur * pComFemur1_gen(s);
pmT1 = C.m_tibia * pComTibia1_gen(s);
pmF2 = C.m_femur * pComFemur2_gen(s);
pmT2 = C.m_tibia * pComTibia2_gen(s);
pmTor = C.m_torso * pComTorso_gen(s);
pmHip = C.m_hip* [s(1,:);s(2,:)];

CoM = (pmF1+ pmT1+pmF2+pmT2+pmTor+pmHip)/totalMass;

end