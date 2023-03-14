import math

f = 66.666e6
f0 = 1e9

ftw = math.floor(pow(2,48) * (f/f0))

print(ftw)
