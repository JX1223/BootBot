function dqPlus = dqPlus_gen(s)
postImpact = postImpact_gen(s);
NDof = NDoF_gen(s);
dqPlus = simplify(postImpact(1:NDof));

end