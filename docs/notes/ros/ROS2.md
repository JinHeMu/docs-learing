

# 文件目录

1. 工作空间	ros2_workplace
2. 包储存空间    ros2_workplace/src
3. 编译在工作空间下进行colcon build

# 1.节点

ROS2中每一个节点也是只负责一个单独的模块化的功能（比如一个节点负责控制车轮转动，一个节点负责从激光雷达获取数据、一个节点负责处理激光雷达的数据、一个节点负责定位等等）

**如何通讯**

话题,服务,动作,参数

# 2.工作空间


工作空间是包含若干个功能包的目录，一开始大家把工作空间理解成一个文件夹就行了。这个文件夹包含下有`src`

**功能包**

功能包可以理解为存放节点的地方，ROS2中功能包根据编译方式的不同分为三种类型。

- `ament_python`，适用于python程序
- `cmake`，适用于C++
- `ament_cmake`，适用于C++程序,是cmake的增强版

# 3. Colcon


Colcon 是一个用于构建和管理多包项目的命令行工具，广泛应用于机器人操作系统（ROS）2和其他使用多包构建系统的项目。它提供了一个统一的界面来构建、测试和安装多个软件包，简化了复杂项目的管理和开发流程。

# 4. RCLCPP编写节点

1. 创建工作空间和功能包

   ```bash
   mkdir -p chapt2_ws/src
   cd chapt2_ws/src
   ros2 pkg create example_cpp --build-type ament_cmake --dependencies rclcpp
   ```

   - pkg create 是创建包的意思
   - --build-type 用来指定该包的编译类型，一共有三个可选项`ament_python`、`ament_cmake`、`cmake`
   - --dependencies 指的是这个功能包的依赖，ros2的C++客户端接口`rclcpp`

   ```bash
   .
   └── src
       └── example_cpp
           ├── CMakeLists.txt
           ├── include
           │   └── example_cpp
           ├── package.xml
           └── src
   
   5 directories, 2 files
   ```

2. 创建节点

   在`example_cpp/src`下创建一个`node_01.cpp`文件

3. 编写代码

```cpp
#include "rclcpp/rclcpp.hpp"
int main(int argc, char **argv)
{
    /* 初始化rclcpp  */
    rclcpp::init(argc, argv);
    /*产生一个node_01的节点*/
    auto node = std::make_shared<rclcpp::Node>("node_01");
    // 打印一句自我介绍
    RCLCPP_INFO(node->get_logger(), "node_01节点已经启动.");
    /* 运行节点，并检测退出信号 Ctrl+C*/
    rclcpp::spin(node);
    /* 停止运行 */
    rclcpp::shutdown();
    return 0;
}
```

4. 修改CMakeLists

```cmake
add_executable(node_01 src/node_01.cpp)
ament_target_dependencies(node_01 rclcpp)
install(TARGETS
  node_01
  DESTINATION lib/${PROJECT_NAME}
)
```

# 5.话题

话题是ROS2中最常用的通信方式之一，话题通信采取的是订阅发布模型

节点1->发布topic_name->订阅->节点2

**消息接口**

可以让cpp和py进行通讯

**发布者**

1. 创建节点
2. 编写发布者，想要创建发布者，只需要调用`node`的成员函数`create_publisher`并传入对应的参数即可。（看API）
3. 导入消息接口 

```cmake
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)# 寻找std_msgs!!!

add_executable(topic_publisher_01 src/topic_publisher_01.cpp)
ament_target_dependencies(topic_publisher_01 rclcpp std_msgs)# 添加依赖!!!
```

```xml
<buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>  
<test_depend>ament_lint_auto</test_depend> <test_depend>ament_lint_common</test_depend>
```

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"//包含头文件
class TopicPublisher01 : public rclcpp::Node
{
public:
    // 构造函数,有一个参数为节点名称,初始化类
    TopicPublisher01(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());
    }

private:
    // 声明节点
};
```

4. 创建发布者

```cpp
class TopicPublisher01 : public rclcpp::Node
{
public:
    TopicPublisher01(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());
        // 创建发布者
        command_publisher = this->create_publisher<std_msgs::msg::String>("command", 10);
    }
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_publisher;//定义发布者

};
```

5. 使用定时器来发布代码

```cpp
class TopicPublisher01 : public rclcpp::Node
{
public:
    TopicPublisher01(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());
        // 创建发布者
        command_publisher = this->create_publisher<std_msgs::msg::String>("command", 10);
        // 创建定时器 500ms为周期
        /*
        
        std::bind(&TopicPublisher01::timer_callback, this) 将 TopicPublisher01 类的成员函数 timer_callback 与当前对象的 this 指针绑定在一起。
        返回的结果是一个可调用对象，这个对象在被调用时会执行 this 对象的 timer_callback 成员函数。
        
        */
        timer = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&TopicPublisher01::timer_callback, this));
      
    }
private:

    void timer_callback()
    {
        // 创建消息
        std_msgs::msg::String message;
        message.data = "forward";
        // 日志打印
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        command_publisher->publish(message);
    }
    
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_publisher;

};
```

**订阅者**

1. 创建节点,修改CMakeLists.txt,修改依赖,导入消息包
2. 创建订阅者,设置回调函数

```cpp
// 创建一个订阅者订阅话题
        command_subscribe_ = this->create_subscription<std_msgs::msg::String>("command", 10, std::bind(&TopicSubscribe01::command_callback, this, std::placeholders::_1));


     // 声明一个订阅者
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscribe_;
     // 收到话题数据的回调函数
    void command_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        double speed = 0.0f;
        if(msg->data == "forward")
        {
            speed = 0.2f;
        }
        RCLCPP_INFO(this->get_logger(), "收到[%s]指令，发送速度 %f", msg->data.c_str(),speed);
    }
```

# 6.服务

服务是双向的,而话题主要是单向的.

1. 创建功能包和节点,导入消息接口,修改CMakeLists.txt 和依赖
2. 创建服务端
3. 创建客户端



# 7.信息接口

1. 创建功能包	
   `ros2 pkg create example_ros2_interfaces --build-type ament_cmake --dependencies rosidl_default_generators geometry_msgs`

2. 接着创建文件夹和文件，注意话题接口放到`msg`文件夹下，以`.msg`结尾。服务接口放到`srv`文件夹下，以`srv`结尾。

```
.
├── CMakeLists.txt
├── msg
│   ├── RobotPose.msg
│   └── RobotStatus.msg
├── package.xml
└── srv
    └── MoveRobot.srv

2 directories, 5 files
```

3. 修改CMakeLists.txt

`rosidl_generate_interfaces(${PROJECT_NAME}  "msg/RobotPose.msg"  "msg/RobotStatus.msg"  "srv/MoveRobot.srv"  DEPENDENCIES geometry_msgs )`

4. 修改package.xml

`<member_of_group>rosidl_interface_packages</member_of_group>` 

5. 保存并编译
6. 在节点中导入头文件
   `#include "example_ros2_interfaces/srv/move_robot.hpp" #include "example_ros2_interfaces/msg/robot_status.hpp"`