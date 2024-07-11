import numpy as np


#  @description: 求变换矩阵的逆矩阵
#  @param: R 旋转矩阵
#  @param: T 平移矩阵
#  @return: M_inv 逆矩阵


# 定义 R 和 T
R = np.array([[0.994838, 0.0187061, -0.0997379],
              [-0.0209378, 0.999552, -0.0213763],
              [0.0992934, 0.0233543, 0.994784]])  

T = np.array([[-0.0379673], [-0.120289], [0.41831]])        # 3x1 列向量


# 计算 R 的转置矩阵 (即 R 的逆矩阵)
R_inv = R.T

# 计算逆矩阵
M_inv = np.block([
    [R_inv, -R_inv @ T],
    [np.zeros((1, R.shape[1])), np.array([[1]])]
])

print("原矩阵 M:")
print(np.block([
    [R, T],
    [np.zeros((1, R.shape[1])), np.array([[1]])]
]))

print("\n逆矩阵 M_inv:")
print(M_inv)
