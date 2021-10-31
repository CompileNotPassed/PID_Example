# PID_Controller

# 0x01 Intro

PID算法通过检测偏差信号，通过对**P(Proportion), I(Integral), D(Differential)**通过线性组合构成控制量对受控对象进行控制。

# 0x02 Theories

考虑某时刻$t$，此时系统的调节目标为$target(t)$，输出量$out(t)$.

此时有偏差$err(t)=target(t)-out(t)$.

## 位置式PID

### 时域表达式

$$
u(t)=K_Perr(t)+K_I\int_0^t{err(t)dt}+K_D\frac{derr(t)}{t}
$$

对于计算机上PID控制的实现，由于积分和微分的特殊性，故采用离散化的形式实现。

### 离散表达式

设系统采样周期为$T$，则对于第$k$个采样周期，有偏差值$err(k)=target(k)-out(k)$

于是对于第$k$次采样，有
$$
u(k)=K_Perr(k)+K_I\sum err(k)+K_D(err(k)-err(k-1))
$$

### Feature

位置式PID由于有积分项$\sum err(k)$，累加后导致最后的输出值`output`与之前所有的状态都有关系，存在累加效应。

如果控制对象的状态值出现问题，引起$err(k)$的大幅度变化，与积分项继续累加之后，后续可能难以修正。

而且，当误差值$err(k)$开始反向变化时，积分项需要一定的时间消减出饱和状态，对于这种情况，一般应采用积分分离来消除影响。

## 积分分离

在系统启动，结束，或更改设定值时，由于$\sum err(t)$的存在，会造成积分的积累。这时可采用积分分离方法，在系统启动，结束，或更改设定值时，消去积分项，使PID控制器转变为PD控制器；当系统采样值接近目标值时，采用积分项，转变为PID控制器，提高控制精度。

对于离散表达式，有
$$
u(k)=K_Perr(k)+\beta K_I\sum err(k)+K_D(err(k)-err(k-1))
$$
设定$\epsilon$>0作为系统阈值，

$(5)$中$\beta$作为积分项的开关函数，取值如下
$$
\beta = \begin{cases}  
1 &  |err(t)|\leq\epsilon\\
0 &  |err(t)|>\epsilon
\end{cases}
$$

## 增量式PID

对于第$k-1$次采样，易得
$$
u(k-1)=K_Perr(k-1)+K_I\sum err(k-1)+K_D(err(k-1)-err(k-2))
$$
对于第$k$次采样周期的增量$\Delta u(k)$，有$\Delta u(k)=u(k)-u(k-1)$.

由$(2)-(3)$可得，
$$
\Delta u(k)=K_P(err(k)-err(k-1))+K_I err(k)+K_D(err(k)-2err(k-1)+err(k-2))
$$

## Feature

对于增量式PID，只有$err(k)$，$err(k-1)$，$err(k-2)$三项起作用，不会像位置式PID一样造成积分累加效应。系统只基于邻近的采样时刻进行修正。系统在发生问题时，增量式PID对系统的影响相对较小。

# 0x03 Implementation

## 伪代码

```
previous_error := 0
integral := 0

loop:
	error := setpoint - measured_value
	integral := integral + error * dt
	derivative := (error - previous_error)/dt
	output := Kp * error +Ki * integral + Kd * derivative
	previous_error:= error
	goto loop
```

## C

```c
#include<stdio.h>
typedef struct PID_Arguments{
    double kp,kd,ki;
}PID_Arguments;

typedef struct PID_State{
    double measuredValue,target,timeElapsed,previousError,integral,output;
}PID_State;

typedef int status;

PID_State PID_Controller(PID_Arguments arguments,PID_State state){
    double error = state.target - state.measuredValue;
    //Update integral=\Sum err(k)
    state.integral += error;
    double increment = arguments.kp*error + arguments.ki*state.integral + arguments.kd*(error-state.previousError)/state.timeElapsed;
    //Update output
    state.output = state.measuredValue+increment;
    //Update previousError
    state.previousError = error;
    //Return modified state(PID_state)
    return state;
}

status PID_Init(PID_State *state){
    //Only for Test use 
    state->timeElapsed=1.0;
    state->measuredValue=50.0;
    state->target=100.0;
    //Test End

    state->previousError=0.0;
    state->integral=0.0;
    state->output=0.0;
    return 0;
}

int main(){
    //Use file output.out to output data
    FILE *outFile;
    outFile = freopen("output.out","w",stdout);


    PID_Arguments arguments;
    PID_State state;
    //Input Arguments
    scanf("%lf %lf %lf",&arguments.kp,&arguments.ki,&arguments.kd);
    printf("Kp=%lf Ki=%lf Kd=%lf\n",arguments.kp,arguments.ki,arguments.kd);
    //Init state
    if(PID_Init(&state)){
        printf("Error\n");
        return 0;
    }

    //Iterate times
    int i=100;

    for (;i>=0;i--)
    {
        //Use output as measuredValue in next iteration
        state.measuredValue = state.output;
        //printf("output=%lf\n",state.output);
        printf("%lf\n",state.output);
        //printf("measuredValue=%lf,",state.measuredValue);
        state=PID_Controller(arguments,state);
    }

    //Close output
    fclose(outFile);
    return 0;
}
```



# 0x04 PID参数的整定

![](https://i.loli.net/2021/10/22/rfsFK389BQa6R5M.png)

## 调参大法

参数整定找最佳，从小到大顺序查，
先是比例后积分，最后再把微分加，
曲线振荡很频繁，比例度盘要放大，
曲线漂浮绕大湾，比例度盘往小扳，
曲线偏离回复慢，积分时间往下降，
曲线波动周期长，积分时间再加长，
曲线振荡频率快，先把微分降下来，
动差大来波动慢，微分时间应加长，
理想曲线两个波，前高后低四比一，
一看二调多分析，调节质量不会低。

