function Fimpact = Fimpact_gen(s)
postImpact = postImpact_gen(s);
NDof = NDoF_gen(s);
Fimpact = simplify(postImpact(NDof+1:NDof+2));

end