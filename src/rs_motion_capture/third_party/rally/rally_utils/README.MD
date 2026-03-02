# rally_utils

`rally_utils`是一个通用基础库，包含了`C++`开发者日常开发所需要的各种基础工具，如日志，配置表，时间等高频通用操作.

## 1. 依赖

在开始使用之前，确保您的环境是**ubuntu2004**，并且预装**ros**.

## 2. 使用方法

`rally_utils`可作为子模块嵌入到您的工程中。您也可以参考工程[rally_example](http://gitlab.robosense.cn/xuantie/common/rally/rally_example)

- step1. 使用cmake指令,载入`rally_utils`.

```cmake
add_subdirectory(xxxx/rally_utils)
```

- step2. 在你的库或者可执行文件中链接`rally_utils`.

```cmake

target_link_libraries(xxx
        rally_utils
        )

```

完成以上操作，您就可以使用`rally_utils`了.

## 3. 示例

- [打印日志](http://gitlab.robosense.cn/xuantie/common/rally/rally_example/tree/master/example/rally_utils/log)
- [管理配置表](http://gitlab.robosense.cn/xuantie/common/rally/rally_example/tree/master/example/rally_utils/config)
- [获取时间](http://gitlab.robosense.cn/xuantie/common/rally/rally_example/tree/master/example/rally_utils/time)

