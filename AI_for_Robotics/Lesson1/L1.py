'''
Author: Jingsheng Lyu
Date: 2021-02-03 15:44:01
LastEditors: Jingsheng Lyu
LastEditTime: 2021-02-03 18:18:46
FilePath: /robotics/AI_for_Robotics/Lesson1/L1.py
Github: https://github.com/jingshenglyu
Web: https://jingshenglyu.github.io/
E-Mail: jingshenglyu@gmail.com
'''
# Modify the empty list uniform distribution,
# as expressed in a list of 5 probabilities

p = [0.2, 0.2, 0.2, 0.2, 0.2]
#  = [0, 1, 0, 0, 0]
# p = []
# n = 5

# for i in range(n):
#     p.append(1./n)

# # measurement parameters
# pHit = 0.6
# pMiss = 0.2

# # After measurement
# p[0] *= pMiss
# p[1] *= pHit
# p[2] *= pHit
# p[3] *= pMiss
# p[4] *= pMiss

# # sum of probabilities
# print(p)
# print(sum(p))

# Outcome:
# [0.04000000000000001, 0.12, 0.12, 0.04000000000000001, 0.04000000000000001]
# 0.3600000000000001

world = ['green', 'red', 'red', 'green', 'green']
measurement = ['red', 'red']
motions = [1,1]

pHit = 0.6
pMiss = 0.2
pExact = 0.8
pOvershoot = 0.1
pUndershoot = 0.1


def sense(p, Z):
    q = []
    # Solution from Udacity
    # for i in range(len(p)):
    #     hit = (Z == world[i])
    #     q.append(p[i] * (hit * pHit + (1-hit) * pMiss))

    # Solution from Jingsheng Lyu
    for i in range(len(p)):
        if Z == world[i]:
            q.append(p[i] * pHit)
        elif Z != world[i]:
            q.append(p[i] * pMiss)

    s = sum(q)
    for i in range(len(p)):
        q[i] = q[i] / s

    return q


# after_p = sense(p, Z)
# sum_after_p = sum(after_p)
# print(sum_after_p)
# print(after_p)

# for i in range(len(after_p)):
#     new_p = []
#     new_p.append(after_p[i] / sum_after_p)

# for k in range(len(measurement)):
#     p = sense(p,measurement[k])

def move(p, U):
    q = []
    for i in range(len(p)):
        s = pExact * p[(i-U) % len(p)]
        s = s + pOvershoot * p[(i-U-1) % len(p)]
        s = s + pUndershoot * p[(i-U+1) % len(p)]
        q.append(s)

    return q

# move twice 
# p = move(p, 1)
# p = move(p, 1)

# move thousand time
# for i in range(1000):
#     p = move(p, 1)

for k in range(len(measurement)):
    p = sense(p, measurement[k])
    p = move(p, motions[k])

print(p)


# Summary
# Belief = Probability
# Sense = Product (Followed by Normalization)
# Move = Convolution (Addition)