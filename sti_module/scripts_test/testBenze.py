import numpy as np

numPts=30                                   # number of points in Bezier Curve
controlPts=[[9.989,4.778],[9.989,5.778],[8.489,5.778]]            # control points
t=np.array([i*1/numPts for i in range(0,numPts+1)])
print(t)
for i in t:
    print(i)

B_x=(1-t)*((1-t)*controlPts[0][0]+t*controlPts[1][0])+t*((1-t)*controlPts[1][0]+t*controlPts[2][0])
B_y=(1-t)*((1-t)*controlPts[0][1]+t*controlPts[1][1])+t*((1-t)*controlPts[1][1]+t*controlPts[2][1])

print(B_x)
print(B_y)