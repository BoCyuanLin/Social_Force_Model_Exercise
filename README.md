# Note for Social Force Model for Pedestrain Dynamics
- <img src="https://latex.codecogs.com/gif.latex?O_t=\text { Onset event at time bin } t " /> 
- <img src="https://latex.codecogs.com/gif.latex?s=\text { sensor reading }  " /> 
- <img src="https://latex.codecogs.com/gif.latex?P(s | O_t )=\text { Probability of a sensor reading value when sleep onset is observed at a time bin } t " />
## Formular of Social Force Model
### 1. Pedestrains with Pedestrains
(Note that $\alpha, \beta$ means the different pedestrains.)
Someone wants to reach a certain destination $\vec{r}^{0}_\alpha$ as comfortable as possible, therefore usually taking the shortest possible way having the shape of a polygon with edges $\vec{r}^{0}_\alpha := \vec{r}^{1}_\alpha, ..., \vec{r}^{n}_\alpha.$ The desired direction:  
$$
\vec{e}_{\alpha} (t) = \frac{\vec{r}^{k}_\alpha - \vec{r}_\alpha (t)}{||\vec{r}^{k}_\alpha - \vec{r}_\alpha (t)||}
$$
$\vec{r}_\alpha (t) :=$ actual position of pedestrain $\alpha$ at time $t$.
$\vec{r}^{k}_\alpha (t) :=$ steer for the nearest point at time $t$.

If the pedestrains is not deiturbed, he or she will walk to the desired direction with a desired speed $v^{0}_{\alpha}$. Then, the actual velocity $\vec{v}^{0}_{\alpha}(t) = v^{0}_{\alpha} \vec{e}_{\alpha}$. The acceleration term:
$$
\vec{F}^{0}_{\alpha} (\vec{v}_{\alpha}, v^{0}_{\alpha} \vec{e}_{\alpha}) = \frac{v^{0}_{\alpha} \vec{e}_{\alpha} - \vec{v}_{\alpha}}{\tau_{\alpha}}
$$

If the motion of the pedestrain is influenced by other pedestrains, he or she will keeps a certain distance with the privat sphere. The vectorial quantities repulsive effects of other $\beta$ (monotonic decrease):
$$
\vec{f}_{\alpha\beta}(\vec{r}_{\alpha\beta}) = -\triangledown_{\vec{r}_{\alpha\beta}} V_{\alpha\beta}[b(\vec{r}_{\alpha\beta})], \ where
\\
V_{\alpha \beta}(b(\vec{r}_{\alpha\beta})) = V^{0}_{\alpha \beta} e^{-\frac{b}{\sigma}}
$$
$b(\vec{r}_{\alpha\beta})$ denotes the semi-minor axis of the ellipse given by:
$$
b(\vec{r}_{\alpha\beta}) = \frac{\sqrt{(||\vec{r}_{\alpha\beta}||+||\vec{r}_{\alpha\beta} - v_{\beta} \Delta t\vec{e}_{\beta}||)^{2}-(v_{\beta} \Delta t)^{2}}}{2}, \ where \ \vec{r}_{\alpha\beta} = \vec{r}_{\alpha} - \vec{r}_{\beta} 
$$
$v_{\beta} \Delta t :=$ order of the step width of pedestrain $\beta$.
### 2. Pedestrains with Space Objects
A pedestrian will keeps a certain distance from borders of buildings, walls, obstacles to avoid the danger of getting hurt. So, a border evokes a repulsive effect (monotonic decrease):
$$
\vec{F}_{\alpha B}(\vec{r}_{\alpha B}) = -\triangledown_{\vec{r}_{\alpha B}} U_{\alpha B}(||\vec{r}_{\alpha B}||), \ where \ \vec{r}_{\alpha B} = \vec{r}_{\alpha}-\vec{r}_{\alpha B}^{(min)}
\\
U_{\alpha B}(||\vec{r}_{\alpha B}||) = U^{0}_{\alpha B} e^{-\frac{||\vec{r}_{\alpha B}||}{R}}
$$
$B :=$ the border
$\vec{r}_{\alpha B}^{(min)} :=$ the point of B that is nearest to pedestrain $\alpha$. 

### 3. Pedestrains with Attractive Objects
Pedestrians are sometimes attracted by other persons or objects. The attractive effects (monotonic decrease):
$$
\vec{f}_{\alpha i}(||\vec{r}_{\alpha i}||, t) = -\triangledown_{\vec{r}_{\alpha i}} W_{\alpha i}(||\vec{r}_{\alpha i}||, t), \ where \ \vec{r}_{\alpha i} = \vec{r}_{\alpha}-\vec{r}_{i}
$$

### 4. Field of Sight Processing
The formula for repulsive and attractive effects only hold for $\beta, B, i$ in the sight of $\alpha$. Therefore,
$$
s (\vec{e}, \vec{f}):=
\begin{cases}
1, \ if \ \vec{e} \cdot \vec{f} \geq ||\vec{f}||cos\phi \\
c, \ otherwise.
\end{cases}
$$
Now, the repulsive and attractive effects are given by
$$
\vec{F}_{\alpha\beta}(\vec{e}_{\alpha},\vec{r}_{\alpha\beta}) = s (\vec{e}_{\alpha}, -\vec{f}_{\alpha\beta})\vec{f}_{\alpha\beta}(\vec{r}_{\alpha\beta}) 
\\
\vec{F}_{\alpha i}(\vec{e}_{\alpha},\vec{r}_{\alpha i}, t) = s (\vec{e}_{\alpha}, \vec{f}_{\alpha i})\vec{f}_{\alpha i}(\vec{r}_{\alpha i}, t) 
$$

### 5. Social Force Model
$$
\vec{F}_{\alpha}(t) = \vec{F}^{0}_{\alpha}(\vec{v}_{\alpha}, v^{0}_{\alpha} \vec{e}_{\alpha}) + \sum_{\beta} \vec{F}_{\alpha\beta}(\vec{e}_{\alpha},\vec{r}_{\alpha\beta})+ \sum_{B}\vec{F}_{\alpha B}(\vec{r}_{\alpha B}) + \sum_{i}\vec{F}_{\alpha i}(\vec{e}_{\alpha},\vec{r}_{\alpha i}, t).
$$
Then, the social force model is given by:
$$
\frac{d \vec{w}_{\alpha}}{dt} = \vec{F}_{\alpha}(t) + fluctuations(\text{random value} ).
$$
### 6. Maximum Acceptable Speed
Since the actual speed is limited by a pedestrian’s maximal acceptable speed $v^{(max)}_{\alpha}$. Hence, the realized motion is given by:
$$
\frac{d \vec{r}_{\alpha}}{dt} = \vec{v}_{\alpha}(t) = \vec{w}_{\alpha}(t) g(\frac{v^{(max)}_{\alpha}}{||\vec{w}_{\alpha}||}), \ where \\
g(\frac{v^{(max)}_{\alpha}}{||\vec{w}_{\alpha}||}) := \begin{cases}
1, \ if \ \frac{v^{(max)}_{\alpha}}{||\vec{w}_{\alpha}||} \geq 1 \\
\frac{v^{(max)}_{\alpha}}{||\vec{w}_{\alpha}||}, \ otherwise.
\end{cases}
\Leftrightarrow g(\frac{v^{(max)}_{\alpha}}{||\vec{w}_{\alpha}||}) := min\{1, \frac{v^{(max)}_{\alpha}}{||\vec{w}_{\alpha}||} \}
$$
## Implementation of SFM
### Enviornment
C++ (Programming Language)
Visual Studio 2019 (IDE)
### Given Parameters
The followings are the given parameters in the paper:
| Parameters            | Values              |
| --------------------- | ------------------- |
| $V^{0}_{\alpha\beta}$ | 2.1                 |
| $\sigma$              | 0.3                 |
| $\phi$                | 100                 |
| $c$                   | 0.5                 |
| $U^{0}_{\alpha B}$    | 10                  |
| R                     | 0.2                 |
| $v^{(max)}_{\alpha}$  | $1.3v^{0}_{\alpha}$ |
| $\Delta t$            | 2                   |
| $\tau_{\alpha}$       | 0.5                 |

### Pedestrains with Pedestrains


### Pedestrains with Obstacles


## Reference
[1] Helbing, D. & Mulnár, P. Social force model for pedestrian dynamics. Phys. Rev. E 51, 4282– 4286 (1995).
[2] https://github.com/svenkreiss/socialforce (Python SFM using Numpy)
