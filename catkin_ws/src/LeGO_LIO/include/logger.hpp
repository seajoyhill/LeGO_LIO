#ifndef LEGO_LIO_LOGGER_HPP_
#define LEGO_LIO_LOGGER_HPP_

// A small C++14, header-only logger for LeGO-LIO.
// No ROS or third-party logging dependency is required.

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace lego_lio {
namespace log {

enum class Level : int {
    Trace = 0,
    Debug,
    Info,
    Warn,
    Error,
    Fatal,
    Off
};

inline const char* levelName(Level level)
{
    switch (level) {
    case Level::Trace: return "TRACE";
    case Level::Debug: return "DEBUG";
    case Level::Info:  return "INFO ";
    case Level::Warn:  return "WARN ";
    case Level::Error: return "ERROR";
    case Level::Fatal: return "FATAL";
    case Level::Off:   return "OFF  ";
    }
    return "UNKWN";
}

inline std::string lowercase(std::string text)
{
    for (char& value : text) {
        if (value >= 'A' && value <= 'Z')
            value = static_cast<char>(value - 'A' + 'a');
    }
    return text;
}

inline bool parseLevel(const std::string& text, Level& level)
{
    const std::string value = lowercase(text);
    if (value == "trace") level = Level::Trace;
    else if (value == "debug") level = Level::Debug;
    else if (value == "info") level = Level::Info;
    else if (value == "warn" || value == "warning") level = Level::Warn;
    else if (value == "error") level = Level::Error;
    else if (value == "fatal") level = Level::Fatal;
    else if (value == "off" || value == "none") level = Level::Off;
    else return false;
    return true;
}

inline bool parseBool(const char* text, bool fallback)
{
    if (!text)
        return fallback;
    const std::string value = lowercase(text);
    if (value == "1" || value == "true" || value == "yes" || value == "on")
        return true;
    if (value == "0" || value == "false" || value == "no" || value == "off")
        return false;
    return fallback;
}

class Logger;

class LogLine {
public:
    LogLine(Logger& logger, Level level, const char* file, int line,
            const char* function)
        : logger_(&logger), level_(level), file_(file), line_(line), function_(function)
    {
    }

    ~LogLine();

    std::ostream& stream() { return stream_; }

private:
    Logger* logger_;
    Level level_;
    const char* file_;
    int line_;
    const char* function_;
    std::ostringstream stream_;
};

class Logger {
public:
    // Constructs an independent logger. It has no output file until setFile()
    // is called, so it is suitable for a per-module/per-dataset log.
    explicit Logger(Level initialLevel = Level::Info, bool consoleEnabled = true,
                    Level flushLevel = Level::Warn)
        : level_(static_cast<int>(initialLevel)),
          consoleEnabled_(consoleEnabled),
          flushLevel_(static_cast<int>(flushLevel))
    {
    }

    ~Logger()
    {
        flush();
    }

    static Logger& instance()
    {
        static Logger logger;
        static const bool environmentLoaded = (logger.configureFromEnvironment(), true);
        (void)environmentLoaded;
        return logger;
    }

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    void setLevel(Level level)
    {
        level_.store(static_cast<int>(level), std::memory_order_relaxed);
    }

    bool setLevel(const std::string& level)
    {
        Level parsed;
        if (!parseLevel(level, parsed))
            return false;
        setLevel(parsed);
        return true;
    }

    Level level() const
    {
        return static_cast<Level>(level_.load(std::memory_order_relaxed));
    }

    bool shouldLog(Level level) const
    {
        return level != Level::Off
            && static_cast<int>(level) >= level_.load(std::memory_order_relaxed);
    }

    void setConsoleEnabled(bool enabled)
    {
        consoleEnabled_.store(enabled, std::memory_order_relaxed);
    }

    bool consoleEnabled() const
    {
        return consoleEnabled_.load(std::memory_order_relaxed);
    }

    void setFlushLevel(Level level)
    {
        flushLevel_.store(static_cast<int>(level), std::memory_order_relaxed);
    }

    bool setFlushLevel(const std::string& level)
    {
        Level parsed;
        if (!parseLevel(level, parsed))
            return false;
        setFlushLevel(parsed);
        return true;
    }

    // An empty path disables the file sink. The parent directory must exist.
    bool setFile(const std::string& path, bool append = true)
    {
        std::lock_guard<std::mutex> lock(sinkMutex_);
        file_.close();
        file_.clear();
        filePath_.clear();
        if (path.empty())
            return true;

        const std::ios_base::openmode mode = std::ios::out
            | (append ? std::ios::app : std::ios::trunc);
        file_.open(path.c_str(), mode);
        if (!file_.is_open())
            return false;
        filePath_ = path;
        return true;
    }

    std::string filePath() const
    {
        std::lock_guard<std::mutex> lock(sinkMutex_);
        return filePath_;
    }

    void flush()
    {
        std::lock_guard<std::mutex> lock(sinkMutex_);
        std::clog.flush();
        std::cerr.flush();
        if (file_.is_open())
            file_.flush();
    }

    void write(Level level, const char* file, int line, const char* function,
               const std::string& message) noexcept
    {
        if (!shouldLog(level))
            return;
        try {
            const std::string output = formatLine(level, file, line, function, message);
            const bool flushNow = static_cast<int>(level)
                >= flushLevel_.load(std::memory_order_relaxed);

            std::lock_guard<std::mutex> lock(sinkMutex_);
            if (consoleEnabled_.load(std::memory_order_relaxed)) {
                std::ostream& stream = level >= Level::Warn ? std::cerr : std::clog;
                stream << output << '\n';
                if (flushNow)
                    stream.flush();
            }
            if (file_.is_open()) {
                file_ << output << '\n';
                if (flushNow)
                    file_.flush();
            }
        } catch (...) {
            // Logging must never terminate a SLAM processing thread.
        }
    }

    template <typename... Args>
    void writef(Level level, const char* file, int line, const char* function,
                const char* format, Args&&... args) noexcept
    {
        if (!shouldLog(level))
            return;
        try {
            const int size = std::snprintf(
                nullptr, 0, format, std::forward<Args>(args)...);
            if (size < 0) {
                write(level, file, line, function, "<invalid log format>");
                return;
            }
            std::vector<char> buffer(static_cast<std::size_t>(size) + 1U);
            std::snprintf(buffer.data(), buffer.size(), format,
                          std::forward<Args>(args)...);
            write(level, file, line, function, std::string(buffer.data()));
        } catch (...) {
        }
    }

private:
    void configureFromEnvironment()
    {
        // Environment configuration is applied only to the process-wide logger.
        // Independently constructed Logger objects remain fully isolated.
        if (const char* value = std::getenv("LEGO_LIO_LOG_LEVEL"))
            setLevel(value);
        if (const char* value = std::getenv("LEGO_LIO_LOG_CONSOLE"))
            setConsoleEnabled(parseBool(value, true));
        if (const char* value = std::getenv("LEGO_LIO_LOG_FLUSH_LEVEL"))
            setFlushLevel(value);
        if (const char* value = std::getenv("LEGO_LIO_LOG_FILE")) {
            const bool append = parseBool(std::getenv("LEGO_LIO_LOG_APPEND"), true);
            setFile(value, append);
        }
    }

    static const char* baseName(const char* path)
    {
        if (!path)
            return "?";
        const char* base = path;
        for (const char* cursor = path; *cursor != '\0'; ++cursor) {
            if (*cursor == '/' || *cursor == '\\')
                base = cursor + 1;
        }
        return base;
    }

    static std::tm localTime(std::time_t time)
    {
        std::tm result{};
#if defined(_WIN32)
        localtime_s(&result, &time);
#else
        localtime_r(&time, &result);
#endif
        return result;
    }

    static std::string formatLine(Level level, const char* file, int line,
                                  const char* function, const std::string& message)
    {
        const auto now = std::chrono::system_clock::now();
        const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;
        const std::time_t time = std::chrono::system_clock::to_time_t(now);
        const std::tm local = localTime(time);

        std::ostringstream stream;
        stream << std::put_time(&local, "%Y-%m-%d %H:%M:%S")
               << '.' << std::setfill('0') << std::setw(3) << milliseconds.count()
               << " [" << levelName(level) << "]"
               << " [tid " << std::this_thread::get_id() << "] "
               << baseName(file) << ':' << line;
        if (function && *function)
            stream << " " << function;
        stream << " | " << message;
        return stream.str();
    }

    std::atomic<int> level_;
    std::atomic<bool> consoleEnabled_;
    std::atomic<int> flushLevel_;
    mutable std::mutex sinkMutex_;
    std::ofstream file_;
    std::string filePath_;
};

inline LogLine::~LogLine()
{
    logger_->write(level_, file_, line_, function_, stream_.str());
}

}  // namespace log
}  // namespace lego_lio

#define LOG_TO(logger, level)                                                    \
    for (bool log_once__ = (logger).shouldLog(level); log_once__; log_once__ = false) \
        ::lego_lio::log::LogLine((logger), level, __FILE__, __LINE__, __func__).stream()

#define LOG_AT(level) LOG_TO(::lego_lio::log::Logger::instance(), level)

#define LOG_TRACE LOG_AT(::lego_lio::log::Level::Trace)
#define LOG_DEBUG LOG_AT(::lego_lio::log::Level::Debug)
#define LOG_INFO  LOG_AT(::lego_lio::log::Level::Info)
#define LOG_WARN  LOG_AT(::lego_lio::log::Level::Warn)
#define LOG_ERROR LOG_AT(::lego_lio::log::Level::Error)
#define LOG_FATAL LOG_AT(::lego_lio::log::Level::Fatal)

#define LOG_TRACE_TO(logger) LOG_TO((logger), ::lego_lio::log::Level::Trace)
#define LOG_DEBUG_TO(logger) LOG_TO((logger), ::lego_lio::log::Level::Debug)
#define LOG_INFO_TO(logger)  LOG_TO((logger), ::lego_lio::log::Level::Info)
#define LOG_WARN_TO(logger)  LOG_TO((logger), ::lego_lio::log::Level::Warn)
#define LOG_ERROR_TO(logger) LOG_TO((logger), ::lego_lio::log::Level::Error)
#define LOG_FATAL_TO(logger) LOG_TO((logger), ::lego_lio::log::Level::Fatal)

#define LOGF_TO(logger, level, ...)                                               \
    do {                                                                           \
        ::lego_lio::log::Logger& log_logger__ = (logger);                         \
        if (log_logger__.shouldLog(level))                                         \
            log_logger__.writef(level, __FILE__, __LINE__, __func__, __VA_ARGS__); \
    } while (false)

#define LOGF_AT(level, ...) LOGF_TO(::lego_lio::log::Logger::instance(), level, __VA_ARGS__)

#define LOGF_TRACE(...) LOGF_AT(::lego_lio::log::Level::Trace, __VA_ARGS__)
#define LOGF_DEBUG(...) LOGF_AT(::lego_lio::log::Level::Debug, __VA_ARGS__)
#define LOGF_INFO(...)  LOGF_AT(::lego_lio::log::Level::Info, __VA_ARGS__)
#define LOGF_WARN(...)  LOGF_AT(::lego_lio::log::Level::Warn, __VA_ARGS__)
#define LOGF_ERROR(...) LOGF_AT(::lego_lio::log::Level::Error, __VA_ARGS__)
#define LOGF_FATAL(...) LOGF_AT(::lego_lio::log::Level::Fatal, __VA_ARGS__)

#define LOGF_TRACE_TO(logger, ...) LOGF_TO((logger), ::lego_lio::log::Level::Trace, __VA_ARGS__)
#define LOGF_DEBUG_TO(logger, ...) LOGF_TO((logger), ::lego_lio::log::Level::Debug, __VA_ARGS__)
#define LOGF_INFO_TO(logger, ...)  LOGF_TO((logger), ::lego_lio::log::Level::Info, __VA_ARGS__)
#define LOGF_WARN_TO(logger, ...)  LOGF_TO((logger), ::lego_lio::log::Level::Warn, __VA_ARGS__)
#define LOGF_ERROR_TO(logger, ...) LOGF_TO((logger), ::lego_lio::log::Level::Error, __VA_ARGS__)
#define LOGF_FATAL_TO(logger, ...) LOGF_TO((logger), ::lego_lio::log::Level::Fatal, __VA_ARGS__)

#endif  // LEGO_LIO_LOGGER_HPP_
