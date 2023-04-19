import matplotlib.pyplot as plt
import matplotlib.pyplot as plt

import numpy as np
import math

def calc_entropy(x):
    return -(x * math.log2(x) + (1 - x) * math.log2(1 - x))

for x in np.arange(0.01, 1, 0.01):
    y = calc_entropy(x)
    #print("for the value of x = {}".format(x))
    #print("We got the value of y = {}\n".format(y))
    plt.scatter(x,y)

# add axis labels and a title
plt.xlabel('x')
plt.ylabel('entropy')
plt.title('Plot of f(x)')
plt.show()
