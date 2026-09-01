#include "logger.hpp"

#include <cassert>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <string>
#include <thread>
#include <vector>

int main()
{
    const std::string path = "/tmp/lego_lio_logger_test.log";
    const std::string dedicatedPath = "/tmp/lego_lio_logger_dedicated_test.log";
    std::remove(path.c_str());
    std::remove(dedicatedPath.c_str());

    lego_lio::log::Logger& logger = lego_lio::log::Logger::instance();
    logger.setConsoleEnabled(false);
    logger.setLevel(lego_lio::log::Level::Info);
    logger.setFlushLevel(lego_lio::log::Level::Trace);
    assert(logger.setFile(path, false));

    int filteredSideEffect = 0;
    LOG_DEBUG << "filtered " << ++filteredSideEffect;
    LOG_INFO << "stream message " << 42;
    LOGF_WARN("formatted message %.1f", 3.5);

    // A separately constructed Logger owns an independent file sink.
    lego_lio::log::Logger datasetLogger(lego_lio::log::Level::Debug, false);
    datasetLogger.setFlushLevel(lego_lio::log::Level::Trace);
    assert(datasetLogger.setFile(dedicatedPath, false));
    LOG_DEBUG_TO(datasetLogger) << "dedicated stream message";
    LOGF_INFO_TO(datasetLogger, "dedicated formatted message=%d", 7);
    datasetLogger.flush();

    const int threadCount = 4;
    const int messagesPerThread = 25;
    std::vector<std::thread> threads;
    for (int thread = 0; thread < threadCount; ++thread) {
        threads.emplace_back([thread]() {
            for (int message = 0; message < messagesPerThread; ++message)
                LOGF_INFO("worker=%d message=%d", thread, message);
        });
    }
    for (std::thread& thread : threads)
        thread.join();
    logger.flush();

    assert(filteredSideEffect == 0);
    std::ifstream input(path.c_str());
    const std::string content((std::istreambuf_iterator<char>(input)),
                              std::istreambuf_iterator<char>());
    assert(content.find("filtered") == std::string::npos);
    assert(content.find("stream message 42") != std::string::npos);
    assert(content.find("formatted message 3.5") != std::string::npos);
    assert(content.find("dedicated") == std::string::npos);

    std::ifstream dedicatedInput(dedicatedPath.c_str());
    const std::string dedicatedContent((std::istreambuf_iterator<char>(dedicatedInput)),
                                       std::istreambuf_iterator<char>());
    assert(dedicatedContent.find("dedicated stream message") != std::string::npos);
    assert(dedicatedContent.find("dedicated formatted message=7") != std::string::npos);
    assert(dedicatedContent.find("stream message 42") == std::string::npos);

    std::size_t workerLines = 0;
    std::size_t position = 0;
    while ((position = content.find("worker=", position)) != std::string::npos) {
        ++workerLines;
        position += 7;
    }
    assert(workerLines == static_cast<std::size_t>(threadCount * messagesPerThread));

    logger.setFile("");
    datasetLogger.setFile("");
    std::remove(path.c_str());
    std::remove(dedicatedPath.c_str());
    return 0;
}
