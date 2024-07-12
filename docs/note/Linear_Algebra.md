# 线性代数

## 概述

​	随着学习的深入,线性代数在控制论,优化,机器视觉,机器学习等有着非常重要的作用,几乎每一个工程都需要运用到线性代数的知识,将从MIT的线性代数公开课开始,来记录自己对于线性代数的理解.

## 1.方程组的几何解释

$$
2x - y &= 0 \\
-x + 2y &= 3
$$

$$
\downarrow
$$

$$
\begin{bmatrix}
  2 & -1  \\
  -1 & 2 \\
\end{bmatrix}
\begin{bmatrix}
  x \\
  y \\
\end{bmatrix}
=
\begin{bmatrix}
  0 \\
  3 \\
\end{bmatrix}
$$
可以写为:
$$
A\vec{x} = \vec{b}
$$
可以看做**列向量的线性组合**:
$$
x\begin{bmatrix}
  2 \\
  -1\\
\end{bmatrix}+
y\begin{bmatrix}
  -1 \\
  2 \\
\end{bmatrix}
=
\begin{bmatrix}
  0 \\
  3 \\
\end{bmatrix}
$$

$$
\downarrow
$$

$$
x\vec{a} + y\vec{b} = \vec{c}
$$

对于三个未知量的方程:
$$
2x-y = 0\\
-x+2y-z = -1\\
-3y+4z = 4
$$

$$
\downarrow
$$

$$
A=\begin{bmatrix}
2  &-1 &0 \\
-1 & 2 &-1\\ 
0  &-3 &4
\end{bmatrix}
\quad
\vec{b}=\begin{bmatrix}0\\-1\\4\end{bmatrix}
$$

**行向量求解**:三个平面交汇与一点.

**列向量求解**:三个向量线性组合与$\vec{b}$汇合.

**列向量的线性组合能够布满整个空间,即对所有$\vec{b}$有解.**

### 计算方法

1. 行向量对于$\vec{b}$进行点乘
2. 列向量的线性组合进行相加

$$
x\begin{bmatrix}
  2 \\
  -1\\
  0
\end{bmatrix}+
y\begin{bmatrix}
  -1 \\
  2 \\
  -3
\end{bmatrix}
+z\begin{bmatrix}
0\\
-1\\
4
\end{bmatrix}
=
\begin{bmatrix}
  0 \\
  -1 \\
  4
\end{bmatrix}
$$

## 2.矩阵消元

对于矩阵进行消元:
$$
\begin{bmatrix}1&3&0\\3&8&1\\0&4&1\end{bmatrix}
\rightarrow
\begin{bmatrix}1&2&1\\0&2&-2\\0&4&1\end{bmatrix}
\rightarrow
\begin{bmatrix}1&2&1\\0&2&-2\\0&0&5\end{bmatrix}
$$
对于$\vec{b}$进行变化:
$$
\begin{bmatrix}2\\12\\2\end{bmatrix}
\rightarrow
\begin{bmatrix}2\\6\\2\end{bmatrix}
\rightarrow
\begin{bmatrix}2\\6\\-10\end{bmatrix}
$$
对于方程组即:
$$
x+2y+z = 2\\
2y-2z = 6\\
5z=-10
$$
**若主元为0,则无法进行消元,可以使用行交换.**

### 线性组合

对于列向量:
$$
\begin{bmatrix}a&b&c\\d&e&f\\g&h&m\end{bmatrix}
\begin{bmatrix}a_1\\a_2\\a_3\end{bmatrix}
=
a_1\begin{bmatrix}a\\d\\g\end{bmatrix} + a_2\begin{bmatrix}b\\e\\h\end{bmatrix} + a_3\begin{bmatrix}c\\f\\g\end{bmatrix}
$$
对于横向量:
$$
\begin{bmatrix}a_1&a_2&a_3\end{bmatrix}\begin{bmatrix}a&b&c\\d&e&f\\g&h&m\end{bmatrix}
=
\begin{bmatrix}a_1(a+d+g)&a_2(b+e+h)&a_3(c+f+m)\end{bmatrix}
$$
**问题:**

如何将矩阵$\begin{bmatrix}1&2&1\\0&2&-2\\0&4&1\end{bmatrix}$转化为$\begin{bmatrix}1&2&1\\0&2&-2\\0&0&5\end{bmatrix}$?	

观察可得左边矩阵$$(row_2 * -2) + row_3$$可以转化为右边矩阵

通过**线性组合**可得变换矩阵
$$
\begin{bmatrix}1&0&0\\0&1&0\\0&-2&1\end{bmatrix}\begin{bmatrix}1&2&1\\0&2&-2\\0&4&1\end{bmatrix}
=
\begin{bmatrix}1&2&1\\0&2&-2\\0&0&5\end{bmatrix}
$$

### 置换矩阵$P$

**行变换**:
$$
\begin{bmatrix}0&1\\1&0\end{bmatrix}\begin{bmatrix}a&b\\c&d\end{bmatrix}
=
\begin{bmatrix}c&d\\a&b\end{bmatrix}
$$
**列变换**:
$$
\begin{bmatrix}a&b\\c&d\end{bmatrix}\begin{bmatrix}0&1\\1&0\end{bmatrix}
=
\begin{bmatrix}b&d\\a&c\end{bmatrix}
$$

### 逆矩阵$A^{-1}$

逆矩阵的作用:**恢复变换矩阵$A$所带来的变化.**
$$
\begin{bmatrix}1&0&0\\3&1&0\\0&0&1\end{bmatrix}
\begin{bmatrix}1&0&0\\-3&1&0\\0&0&1\end{bmatrix}
=
\begin{bmatrix}1&0&0\\0&1&0\\0&0&1\end{bmatrix}
$$

$$
A^{-1}A=I
$$

$I$为单位矩阵.



## 3.矩阵乘法

设存在三个矩阵具有以下关系:
$$
AB=C
$$

- $$**考虑点个元素**
  $$
  A_{mn}B_{np}=C_{mp}
  $$
  设$(i,j)$为第$i$行,第$j$列元素,则:
  $$
  C_{i,j}=\sum_{k=1}^{n}A_{ik}\cdot B_{kj}
  $$

- 

- **考虑列**
  $$
  A\cdot B_{col_1}+A\cdot B_{col_2} +\cdots = C
  $$

$C$:$A$矩阵的线性组合.

$B$:怎样的组合实现了$A \rightarrow C$.

- **考虑行**

$A$的某一行乘以$B$的所有行等于$C$的某一行.

$C$:为矩阵$B$的线性组合.

- **考虑列和行**

$$
\begin{bmatrix}2&7\\3&8\\4&9\end{bmatrix}\begin{bmatrix}1&6\\0&0\end{bmatrix}
=
\begin{bmatrix}2\\3\\4\end{bmatrix}\begin{bmatrix}1&6\end{bmatrix} +
\begin{bmatrix}7\\8\\4\end{bmatrix}\begin{bmatrix}0&0\end{bmatrix}
$$

### 方块矩阵

$$
\begin{bmatrix}A_1&A_2\\A_3&A_4\end{bmatrix}
\begin{bmatrix}B_1&B_2\\B_3&B_4\end{bmatrix}
=
\begin{bmatrix}A_1B_1+A_2B_3&\cdots\\\cdots&\cdots\end{bmatrix}
$$

### 逆矩阵

**左逆等于右逆:**
$$
A^{-1}A=I=AA^{-1}\\
$$
$\begin{bmatrix}1&3\\2&6\end{bmatrix}$为不可逆矩阵,由于$col_1$和$col_2$向量处于同一直线.

**任何线性组合都在这条直线上**,无法通过逆矩阵形成单位阵.

**假设**存在逆矩阵,可以找到一个$\vec{x}$使得$A\vec{x}=0$,则:
$$
A^{-1}A\vec{x} = 0
$$
可知$\vec{x}$一定为0,但是当$x=\begin{bmatrix}-3\\1\end{bmatrix}$时,$A\vec{x}=0$存在,所以该矩阵不存在逆矩阵.



### 解逆矩阵

$$
\begin{bmatrix}1&3\\2&7\end{bmatrix}
\begin{bmatrix}a&c\\b&d\end{bmatrix}
=I
$$

$$
\downarrow
$$

$$
a\begin{bmatrix}1\\2\end{bmatrix}+
b\begin{bmatrix}3\\7\end{bmatrix}
=
\begin{bmatrix}1\\0\end{bmatrix}
$$

$$
c\begin{bmatrix}1\\2\end{bmatrix}+
d\begin{bmatrix}3\\7\end{bmatrix}
=
\begin{bmatrix}0\\1\end{bmatrix}
$$

类似与解方程组,**有没有一种方法可以同时处理多个方程?**

**高斯-若尔思想:**

假设存在一个矩阵$E$,使得
$$
EA=I
$$
所以:
$$
E=A^{-1}
$$
则可以实现:
$$
E\begin{bmatrix}A&I\end{bmatrix} = \begin{bmatrix}I&A^{-1}\end{bmatrix}
$$

## 4.A=LU分解

$$
ABB^{-1}A^{-1} = I\\
B^{-1}A^{-1}AB = I
$$

$$
\downarrow
$$

$$
AA^{-1} = I\\
(A^{-1})^TA^{T} = I
$$

**转置交换顺序:**类似与先穿袜子后穿鞋,那么反过来先拖鞋后穿袜子.



### 奇异矩阵

1. 矩阵为方阵
2. 行列式为0
3. 不具有满秩

### A=LU分解

$A$为原始矩阵,$L$(lower triangle)为下三角矩阵,$U$为上三角矩阵.

**目的:**为了让原始矩阵$A$**转化会三角矩阵来减少时间复杂度**.
$$
E_{21}A =U\\
\downarrow
\\
\begin{bmatrix}1&0\\-4&1\end{bmatrix}\begin{bmatrix}2&1\\8&7\end{bmatrix}
=
\begin{bmatrix}2&1\\0&3\end{bmatrix}
$$
两边同时左乘$E_{21}^{-1}$:
$$
E_{21}^{-1}E_{21}A =E_{21}^{-1}U\\
\downarrow\\
\begin{bmatrix}1&0\\4&1\end{bmatrix}\begin{bmatrix}1&0\\-4&1\end{bmatrix}\begin{bmatrix}2&1\\8&7\end{bmatrix}
=
\begin{bmatrix}1&0\\4&1\end{bmatrix}\begin{bmatrix}2&1\\0&3\end{bmatrix}
$$
可得:
$$
A =E_{21}^{-1}U\\
\downarrow\\
\begin{bmatrix}2&1\\8&7\end{bmatrix}
=
\begin{bmatrix}1&0\\4&1\end{bmatrix}\begin{bmatrix}2&1\\0&3\end{bmatrix}
$$
令$E_{21}^{-1}=L$:
$$
A=LU
$$
$L和U$都为三角矩阵.

同理拓展到3*3矩阵也可以得到$A=LU$.

> First point:  Every inverse matrix $E^{-1}$ is lower triangular.  Its off-diagonal entry is $l_{ij}$, to undo the subtraction produced by $-l_{ij}$·  The main diagonals of E and E-1 contain l's. 
> Our example above had $l_{21}$ = 3 and $E=\begin{bmatrix}1&0\\-3&1\end{bmatrix}$ and $L=E^{-1}=\begin{bmatrix}1&0\\3&1\end{bmatrix}$. 
> Second  point:  Equation  (2)  shows  a  lower  triangular matrix  (the  product  of  the  $E_{ij}$) multiplying  A.  It  also  shows  all  the  $E_{ij}$  multiplying  U to  bring  back  A.  This lower
> triangular product of inverses is L. 

举个例子,方程组:
$$
Ax=b
$$
如果通过消元法,计算复杂度是$\frac{1}{3}n^3 + n^2$.需要对于每一行A进行消元,同时也要对b进行变换.

如果使用**A=LU分解**,那么:
$$
LUx=b\\
\downarrow\\
\begin{align*}
a_1x+a_2y+a_3z+\cdots=d_1\\	
b_1y+b_2z+\cdots=d_2\\
c_1z+\cdots=d_3\\
\end{align*}
$$
**这样解方程组速度会更快**.

### 置换矩阵$P$

**性质:**$P^{-1}=P^{T}$
$$
\begin{bmatrix}1&0&0\\0&1&0\\0&0&1\end{bmatrix}
\begin{bmatrix}1&0&0\\0&0&1\\0&1&0\end{bmatrix}
\begin{bmatrix}0&1&0\\1&0&0\\0&0&1\end{bmatrix}
\begin{bmatrix}0&1&0\\0&0&1\\0&1&0\end{bmatrix}
\begin{bmatrix}0&0&1\\0&1&0\\1&0&0\end{bmatrix}
\begin{bmatrix}0&0&1\\1&0&0\\0&0&1\end{bmatrix}
$$
数量为$n!$

## 5.转置,逆,向量空间,置换

### 转置矩阵

$$
A_{ij}^T = A_{ji}
$$


$$
\begin{bmatrix}1&3\\2&3\\4&7\end{bmatrix}^T
=
\begin{bmatrix}1&2&4\\3&3&7\end{bmatrix}
$$

### 置换矩阵

**置换的作用:**将主元为0的行进行置换
$$
PA=LU
$$

$$
P^{-1} = P^T\\
P^TP=I
$$

### 对称矩阵

形如矩阵$A$为对称矩阵:
$$
A=\begin{bmatrix}3&1&7\\1&2&9\\7&9&4\end{bmatrix}
$$

$$
A^T = A
$$

若存在矩阵$R$,则$RR^T$总是对称矩阵,例如:
$$
\begin{bmatrix}1&3\\2&3\\4&1\end{bmatrix}
\begin{bmatrix}1&2&4\\3&3&1\end{bmatrix}
=
\begin{bmatrix}10&11&7\\11&13&11\\7&11&9\end{bmatrix}
$$

$$
(RR^T)^T=(R^T)^TR^T=RR^T
$$

### 向量空间

$X-Y$平面,由所有二维向量组成的空间(一定包括$(0,0)$)
$$
R^3:三维实向量组成的空间
$$

$$
R^n:n维实向量组成的空间
$$

**向量空间对于数乘和加法封闭**:所有线性组合乘法或者是加法,仍处于空间内.

**列空间$C(A)$:**所有列向量的线性组合.例如
$$
A=\begin{bmatrix}1&3\\2&3\\4&1\end{bmatrix}
$$
$A$矩阵的列向量组成的空间是在$R^3$下的子空间$R^2$:一个平面

### 子空间

$R^3$的子空间:

1. 自己本身
2. 平面
3. 线
4. 原点
