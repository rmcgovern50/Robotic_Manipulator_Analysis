function q_home = UR5_homeconfiguration()
    ur5 = loadrobot('universalUR5','Gravity', [0,0,-9.81]);
    %ur3 = loadrobot('universalUR3','Gravity', [0,0,-9.81]);
    %ur10 = loadrobot('universalUR10','Gravity', [0,0,-9.81]);
    ur5.DataFormat = "row";
    %define the startg and end of a path
    q_home = homeConfiguration(ur5);
end

