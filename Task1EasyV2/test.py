import math
from math import atan
a = [-1,-1]
b = [-1,-1]

def ThirdPointFinder(r,a1,b1,c,x0,y0):
    d = r*r - c*c/(a1*a1+b1*b1);
    mult = math.sqrt (d / (a1*a1+b1*b1));
    a[0] = x0 + b1 * mult;
    b[0] = x0 - b1 * mult;
    a[1] = y0 - a1 * mult;
    b[1] = y0 + a1 * mult;
t = list(map(float, input().split()))
beta_min = t[0]
beta_max = t[1]
gamma_min = t[2]
gamma_max = t[3]
l1 = t[4]
l2 = t[5]
x0 = t[6]
y0 = t[7]
ThirdPointFinder(l1,-2*x0,-2*y0,x0*x0+y0*y0+l1*l1-l2*l2,x0,y0)
print(a)
print(b)

M_PI = math.pi

beta = atan(abs(a[1])/abs(a[0]))
if(a[1] < 0 and a[0] > 0):
    beta = 2 * M_PI - beta
elif(a[1] < 0 and a[0] < 0):
    beta = M_PI + beta
elif(a[1] > 0 and a[0] < 0):
    beta = M_PI - beta
if(not(beta_min < beta and beta_max > beta)):
    beta = atan(abs(b[1])/abs(b[0]))
    if(b[1] < 0 and b[0] > 0):
        beta = 2 * M_PI - beta
    elif(b[1] < 0 and b[0] < 0):
        beta = M_PI + beta
    elif(b[1] > 0 and b[0] < 0):
        beta = M_PI - beta


print(beta)


#1.084576009762349 2.421516740747265 4.813285450222808 5.923564496001442 79101.44982083338 82730.00577200124 -27289.74933608115 -20911.53006847488
