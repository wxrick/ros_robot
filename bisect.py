from bisect import bisect
from random import random

P = [0.10,0.25,0.60,0.05]

cdf = [P[0]]
for i in xrange(1, len(P)):
    cdf.append(cdf[-1] + P[i])

random_ind = bisect(cdf,random())