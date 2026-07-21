# L1：简单弹簧阻尼系统卡尔曼滤波示例

## 1. 什么是卡尔曼滤波

卡尔曼滤波（Kalman Filter）是一种**状态估计算法**。它会结合：

- 系统模型给出的预测（不一定需要精确物理模型，也可以是简单运动假设、在线辨识模型或神经网络学习到的模型）；
- 带有噪声的传感器测量；

得到比单独使用测量值更稳定的状态估计。

在第一课中我们首先介绍使用卡尔曼滤波的最简单情况，也就是估计弹簧阻尼系统的**位置和速度**。虽然传感器只测量位置，但滤波器仍可根据系统模型间接估计速度。

---

## 2. 系统状态与模型

代码将状态定义为：

$$
x =
\begin{bmatrix}
\text{位置} \\
\text{速度}
\end{bmatrix}
$$

对应代码：

```python
# 状态 x = [位置, 速度]
```

弹簧阻尼系统满足：

$$
m\ddot{x} + c\dot{x} + kx = 0
$$

代码使用时间步长 `dt` 对该系统进行离散化，并得到状态转移矩阵：

$$
A =
\begin{bmatrix}
1 & dt \\
-\dfrac{k}{m}dt & 1-\dfrac{c}{m}dt
\end{bmatrix}
$$

因此，相邻时刻的状态关系为：

$$
x_i = A x_{i-1}
$$

对应代码：

```python
true[i] = A @ true[i - 1]
```

测量矩阵为：

$$
H =
\begin{bmatrix}
1 & 0
\end{bmatrix}
$$

它表示传感器只测量状态中的位置，不直接测量速度。

---

## 3. 噪声与初始值

`Q` 是模型噪声协方差矩阵：

$$
Q =
\begin{bmatrix}
10^{-6} & 0 \\
0 & 10^{-5}
\end{bmatrix}
$$

`R` 是测量噪声协方差：

$$
R = 0.05^2
$$

真实初始状态和滤波器初始估计分别为：

$$
true[0] =
\begin{bmatrix}
1 \\
0
\end{bmatrix},
\qquad
estimate[0] =
\begin{bmatrix}
0.8 \\
0
\end{bmatrix}
$$

初始估计协方差为：

$$
P = I
$$

其中，`P` 表示当前状态估计的不确定性。

---

## 4. 卡尔曼滤波流程

每个时间步包含两个阶段：**预测**和**更新**。

### 4.1 预测状态

使用系统模型预测当前状态：

$$
x = A\,estimate[i-1]
$$

对应代码：

```python
x = A @ estimate[i - 1]
```

### 4.2 预测协方差

更新预测状态的不确定性：

$$
P = APA^T + Q
$$

对应代码：

```python
P = A @ P @ A.T + Q
```

### 4.3 计算卡尔曼增益

$$
K = PH^T\left(HPH^T + R\right)^{-1}
$$

对应代码：

```python
K = P @ H.T @ np.linalg.inv(H @ P @ H.T + R)
```

`K` 决定滤波器在多大程度上相信测量值：

- `R` 越大，测量噪声越强，滤波器越相信模型预测；
- `P` 越大，预测越不确定，滤波器越相信测量值。

### 4.4 使用测量更新状态

测量残差为：

$$
measurement[i] - Hx
$$

状态更新公式为：

$$
x = x + K\left(measurement[i] - Hx\right)
$$

对应代码：

```python
x = x + (K @ (measurement[i] - H @ x)).ravel()
```

### 4.5 更新协方差

$$
P = \left(I-KH\right)P
$$

对应代码：

```python
P = (np.eye(2) - K @ H) @ P
```

最后保存当前估计：

```python
estimate[i] = x
```

---

## 5. 整体流程

```text
上一时刻估计 estimate[i-1]
             │
             ▼
      状态预测 x = A estimate[i-1]
             │
             ▼
       协方差预测 P = A P Aᵀ + Q
             │
             ▼
      读取带噪声位置 measurement[i]
             │
             ▼
   计算卡尔曼增益 K = P Hᵀ(HPHᵀ+R)⁻¹
             │
             ▼
       使用测量修正状态 x
             │
             ▼
       更新协方差 P = (I-KH)P
             │
             ▼
         保存 estimate[i]
```

最终图像比较了：

- `true[:, 0]`：真实位置；
- `measurement`：带噪声的位置测量；
- `estimate[:, 0]`：卡尔曼滤波估计的位置。

卡尔曼滤波结果通常比原始测量更平滑，同时能够较好地跟随真实运动。
