# ROS小车STM32控制部分

使用RTThread实时操作系统＋STM32VET6 + STM32CubeMX进行开发

## 初始化LED灯





## ADC电压采集





## 编码器的读取





## PWM电机控制

[PWM原理 PWM频率与占空比详解-CSDN博客](https://blog.csdn.net/as480133937/article/details/103439546?ops_request_misc=%7B%22request%5Fid%22%3A%22170113534116800188519361%22%2C%22scm%22%3A%2220140713.130102334..%22%7D&request_id=170113534116800188519361&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~top_positive~default-1-103439546-null-null.142^v96^pc_search_result_base9&utm_term=pwm&spm=1018.2226.3001.4187)

问题1:

为什么只能用

```
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
```

不能用

```
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_ALL);
```

## PID电机控制



## 移动底盘的控制

