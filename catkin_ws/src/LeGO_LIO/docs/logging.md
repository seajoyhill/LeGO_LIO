# LeGO-LIO 轻量日志

`include/logger.hpp` 是一个 C++14 header-only 日志器，不依赖 ROS、glog 或 spdlog。
它支持：

- `trace/debug/info/warn/error/fatal/off` 运行时优先级；
- 控制台和文件双输出；
- 流式与 `printf` 风格接口；
- 毫秒时间戳、线程 ID、源码位置；
- 多线程安全写入；
- ROS 参数、环境变量和 C++ API 配置。

## 使用

`utility.h` 已包含日志头，因此现有节点可直接写：

```cpp
LOG_DEBUG << "corner count=" << cornerCount;
LOG_INFO  << "mapping initialized";
LOG_WARN  << "few correspondences";
LOG_ERROR << "failed to load calibration";

LOGF_INFO("pose: x=%.3f y=%.3f", x, y);
```

被优先级过滤的流式日志不会计算右侧表达式。
`FATAL` 仅表示最高日志级别，不会自动调用 `abort()`。

## ROS 参数

三个节点启动时都会读取私有参数 `~log/*`。默认配置位于 `config/params.yaml`：

```yaml
log:
  level: info
  file: /tmp/lego_lio_{node}.log
  console: true
  append: true
  flush_level: warn
```

`{node}` 会替换成 ROS 节点名，避免多个进程同时写同一个文件。将 `file` 设为空字符串可关闭文件输出；文件的父目录需要预先存在。

优先级是最低输出级别。例如 `level: warn` 只输出 `warn/error/fatal`；`level: debug` 输出 `debug` 及以上级别。

## 环境变量

不通过 ROS 启动时可使用：

```bash
export LEGO_LIO_LOG_LEVEL=debug
export LEGO_LIO_LOG_FILE=/tmp/lego_lio.log
export LEGO_LIO_LOG_CONSOLE=true
export LEGO_LIO_LOG_APPEND=true
export LEGO_LIO_LOG_FLUSH_LEVEL=warn
```

## C++ API

```cpp
using lego_lio::log::Level;
auto& logger = lego_lio::log::Logger::instance();
logger.setLevel(Level::Debug);
logger.setConsoleEnabled(true);
logger.setFile("/tmp/lego_lio.log", true);  // true: append
logger.setFlushLevel(Level::Warn);
```

## 独立文件日志器

可以主动创建 `Logger`。每个实例拥有完全独立的等级、控制台开关和文件 sink，适合将标定、轨迹、调试数据等写到单独文件：

```cpp
using lego_lio::log::Level;

lego_lio::log::Logger calibrationLog(Level::Debug, false);  // 不输出到终端
if (!calibrationLog.setFile("/tmp/calibration.log", false)) {  // false: 覆盖写入
    throw std::runtime_error("cannot open calibration log");
}
calibrationLog.setFlushLevel(Level::Info);

LOG_DEBUG_TO(calibrationLog) << "raw corner count=" << cornerCount;
LOGF_INFO_TO(calibrationLog, "extrinsic: x=%.3f y=%.3f z=%.3f", x, y, z);
```

通用形式是 `LOG_TO(logger, level)` 和 `LOGF_TO(logger, level, ...)`。独立 logger 不读取 `LEGO_LIO_LOG_*` 环境变量，也不受 `~log/*` 参数影响；因此它不会改变全局 logger 或其他独立 logger 的输出目标。
