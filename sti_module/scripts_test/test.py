# import numpy as np

# b = np.array([0, 1, 0, 1.5385])
# print(b)
# b = b[:, np.newaxis]

# print("----------------------------------------------")
# print(b)
# A = np.array([[1.728, 1.44, 1.2, 1], [15.625, 6.25, 2.5, 1], [4.32, 2.4, 1, 0], [18.75, 5, 1, 0]])

# print("----------------------------------------------")
# print(np.dot(np.linalg.inv(A), b))

# rx = np.arange(2.5, 1.2, -0.1)
# print(rx)

# from scipy.interpolate import CubicSpline
# import numpy as np
# import matplotlib.pyplot as plt

# plt.style.use('seaborn-poster')


# x = [0, 1, 2]
# y = [1, 3, 2]

# # x = [0, 1]
# # y = [1, 3]

# # use bc_type = 'natural' adds the constraints as we described above
# f = CubicSpline(x, y, bc_type='natural')
# x_new = np.linspace(0, 2, 100)
# y_new = f(x_new)

# plt.figure(figsize = (10,8))
# plt.plot(x_new, y_new, 'b')
# plt.plot(x, y, 'ro')
# plt.title('Cubic Spline Interpolation')
# plt.xlabel('x')
# plt.ylabel('y')
# plt.show()


# import numpy as np
# import matplotlib.pyplot as plt
# from scipy import interpolate
# x = np.arange(0, 10)
# y = np.exp(-x/3.0)
# print(x)
# print(y)
# f = interpolate.interp1d(x, y)

# xnew = np.arange(0, 9, 0.1)
# ynew = f(xnew)   # use interpolation function returned by `interp1d`
# plt.plot(x, y, 'o', xnew, ynew, '-')
# plt.show()

# for i in range(5):
#     print(i)
# totalNum = 100
# start = 3
# finish = (start+300)%totalNum
# print(finish)

# for i in range(30,30,1):
#     print(i)

a = 0.44899999999999984 
b = -1.1102230246251565e-16 
c = 0.42924399999999924

if  b == 0.0:
    X_g1 = X_g2 = -c/a
    Y_g1 = -sqrt(_L*_L - (X_g1 - Xhc)*(X_g1 - Xhc)) + Yhc
    Y_g2 = sqrt(_L*_L - (X_g2 - Xhc)*(X_g2 - Xhc)) + Yhc
    
else:
    la = (1.0 + (a/b)*(a/b))
    lb = -2.0*(Xhc - (a/b)*((c/b) + Yhc))
    lc = Xhc*Xhc + ((c/b) + Yhc)*((c/b) + Yhc) - _L*_L
    denlta = lb*lb - 4.0*la*lc

    X_g1 = (-lb + sqrt(denlta))/(2.0*la)
    X_g2 = (-lb - sqrt(denlta))/(2.0*la)

    Y_g1 = (-c - a*X_g1)/b
    Y_g2 = (-c - a*X_g2)/b