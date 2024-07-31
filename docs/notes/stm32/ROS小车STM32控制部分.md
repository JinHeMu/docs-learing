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

## 增量式PID电机控制



## 移动底盘的控制





## 陀螺仪数据的读取和转向环的控制



# RT-Thread 设备

操作系统的设备最大的优点是将底层驱动函数通过设备模型,只将其中的接口引出,只需要操作其中的方法即可,并且移植方便.(面向对象的思想)




## IO设备模型

1. 创建和注册I/O设备
2. 查找设备
3. 打开设备
4. 访问设备,操作接口和方法

```c
/* common device interface */
rt_err_t  demo_init(rt_device_t dev){rt_kprintf("demo init...\n");return 0;}
rt_err_t  demo_open(rt_device_t dev, rt_uint16_t oflag){rt_kprintf("demo open...\n");return 0;}
rt_err_t  demo_close(rt_device_t dev){rt_kprintf("demo close...\n");return 0;}


int rt_demo_init(void)
{
    rt_device_t demo_dev;
    demo_dev = rt_device_create(RT_Device_Class_Char, 32);
    if (demo_dev == RT_NULL) {
        LOG_E("rt_device_create faild...\n");
        return -1;
    }

    demo_dev->init = demo_init;
    demo_dev->open = demo_open;
    demo_dev->close = demo_close;

    rt_device_register(demo_dev, "demo", RT_DEVICE_FLAG_RDONLY);

    return 0;
}
```

```c
rt_device_t dev;

int main(int argc, char **argv) {
    dev = rt_device_find("demo");
    if (dev == RT_NULL) {
        LOG_E("rt_device_find faild...\n");
        return -1;
    }


    rt_device_init(dev);
    rt_device_open(dev, RT_DEVICE_OFLAG_RDWR);
    rt_device_close(dev);

    return 0;
}
```

## UART设备

1. 打开UART设备
2. 配置板级初始化

```c
#define BSP_USING_UART1
#define BSP_UART1_TX_PIN       "PA9"
#define BSP_UART1_RX_PIN       "PA10"

#define BSP_USING_UART2
#define BSP_UART2_TX_PIN       "PA2"
#define BSP_UART2_RX_PIN       "PA3"
```

3. 寻找设备
4. 打开设备(打开设备的模式)
5. 配置设备参数值,打开相应的功能(接收中断等)
6. 利用中断轮询DMA等方法进行数据的发送或接受

```c
rt_thread_t uart2_th;
rt_device_t uart2_dev;
rt_err_t ret;
struct serial_configure u2_configs = RT_SERIAL_CONFIG_DEFAULT;
rt_sem_t uart2_sem;


rt_err_t rx_callback(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(uart2_sem);
    return RT_EOK;
}


void serial_thread_entry(void *parameter)
{
    int16_t buffer;
    while(1)
    {
     while(rt_device_read(uart2_dev, 0, &buffer, 1) != 1)
     {
         rt_sem_take(uart2_sem, RT_WAITING_FOREVER);
     }
     rt_kprintf("%c", buffer);
    }
}

int main(int argc, char **argv) {

    uart2_dev = rt_device_find("uart2");
    if(uart2_dev == RT_NULL)
    {
        LOG_E("rt_device_find[uart2] faild...\n");
        return -2;
    }

    ret = rt_device_open(uart2_dev, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    if(ret < 0)
    {
        LOG_E("rt_device_open[uart2] faild...\n");
        return ret;
    }

    rt_device_control(uart2_dev, RT_DEVICE_CTRL_CONFIG, &u2_configs);

    rt_device_set_rx_indicate(uart2_dev, rx_callback);

    uart2_sem = rt_sem_create("rx_sem", 0, RT_IPC_FLAG_FIFO);

    uart2_th = rt_thread_create("uart2_th", serial_thread_entry, NULL, 1024, 20, 10);
    rt_thread_startup(uart2_th);

    rt_device_write(uart2_dev, 0, "uart2 config\n", rt_strlen("uart2 config\n"));
    rt_kprintf("uart1 config\n");


    return RT_EOK;
}
```



## MPU6050的移植(使用RTThread设备模型)

1. 添加软件包
2. 打开传感器功能和软件模拟iic
3. 设置板级配置,和初始化iic设备

```c
/*#define BSP_USING_I2C1*/
#define BSP_USING_I2C1
#ifdef BSP_USING_I2C1
#define BSP_I2C1_SCL_PIN GET_PIN(B, 10)
#define BSP_I2C1_SDA_PIN GET_PIN(B, 11)
#endif
```
4. 初始化设备,加入命令行进行测试

```c
/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-10-23     flybreak     the first version
 */

#include <rtthread.h>
#include "mpu6xxx.h"

/* Default configuration, please change according to the actual situation, support i2c and spi device name */
#define MPU6XXX_DEVICE_NAME  "i2c2"

/* Test function */
static int mpu6xxx_test()
{
    struct mpu6xxx_device *dev;
    struct mpu6xxx_3axes accel, gyro;
    int i;

    /* Initialize mpu6xxx, The parameter is RT_NULL, means auto probing for i2c*/
    dev = mpu6xxx_init(MPU6XXX_DEVICE_NAME, RT_NULL);

    if (dev == RT_NULL)
    {
        rt_kprintf("mpu6xxx init failed\n");
        return -1;
    }
    rt_kprintf("mpu6xxx init succeed\n");

    for (i = 0; i < 5; i++)
    {
        mpu6xxx_get_accel(dev, &accel);
        mpu6xxx_get_gyro(dev, &gyro);

        rt_kprintf("accel.x = %3d, accel.y = %3d, accel.z = %3d ", accel.x, accel.y, accel.z);
        rt_kprintf("gyro.x = %3d gyro.y = %3d, gyro.z = %3d\n", gyro.x, gyro.y, gyro.z);

        rt_thread_mdelay(100);
    }

    mpu6xxx_deinit(dev);

    return 0;
}
MSH_CMD_EXPORT(mpu6xxx_test, mpu6xxx sensor test function);

```
