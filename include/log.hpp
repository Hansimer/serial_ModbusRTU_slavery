#pragma once

#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <mutex>
#include <ctime>
#include <cstdarg>
#include <chrono>
#include <string>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>
#include <unistd.h>
#include <ament_index_cpp/get_package_prefix.hpp>


#define  MAX_LOG_FILES 5
#define LOG_NAME "log.log"
#define LOG_MAX_SIZE ( 10240) // 10 KB
#define LOG_File_SIZE (20* 10240) // 20 KB

#ifndef PROJECT_NAME
    #define PROJECT_NAME "serial_ModbusRTU_slavery"
#endif

using namespace std;


class myLogger 
{

    private:
        std::ofstream log_file;
        std::mutex mtx;
        static myLogger* instance; //单例模式
    
        std::string package_name = PROJECT_NAME ;
        std::string pkg_prefix = ament_index_cpp::get_package_prefix(package_name);
        std::string log_path = pkg_prefix + "/log/";
        std::string log_file_full_path = log_path + LOG_NAME;

        bool file_exists(const std::string& path) 
        {
            struct stat buffer{};
            return (stat(path.c_str(), &buffer) == 0);
        }

            // ==============================
            // 获取文件大小
            // ==============================
            size_t get_file_size(const std::string& path) {
                struct stat st{};
                if (stat(path.c_str(), &st) == 0) {
                    return st.st_size;
                }
                return 0;
            }

        /// @brief 自动循环管理log文件，保持最新的5个日志文件
        void rotate_logs() 
        {
            log_file.close(); // 先关闭当前日志文件
            for (int i = MAX_LOG_FILES - 1; i > 0; --i) {
                std::string old_fn = log_path + LOG_NAME + "." + std::to_string(i);
                std::string new_fn = log_path + LOG_NAME + "." + std::to_string(i + 1);
                if (file_exists(old_fn)) {
                    std::rename(old_fn.c_str(), new_fn.c_str());
                }
            }

            std::string current = log_path + LOG_NAME;
            if (file_exists(current)) {
                std::rename(current.c_str(), (log_path + LOG_NAME + ".1").c_str());
            }
            log_file.open(log_file_full_path, std::ios::out | std::ios::trunc); // 创建新日志文件

        }
            // ==============================
            // 检查是否超过大小，需要滚动
            // ==============================
            void check_rotate() {

                if (get_file_size(log_file_full_path) >= LOG_File_SIZE) {
                    
                    rotate_logs();
                }
            }

        /// @brief 获取当前时间字符串，格式为 "YYYY-MM-DD HH:MM:SS.mmm"
        /// @return 
        std::string get_time() 
        {
        // 获取系统时间
            auto now = std::chrono::system_clock::now();
            auto time_t = std::chrono::system_clock::to_time_t(now);

            // 转本地时间
            std::tm tm = *std::localtime(&time_t);

            // 计算毫秒
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

            // 拼接字符串
            std::stringstream ss;
            ss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
            ss << "." << std::setw(3) << std::setfill('0') << ms.count();

            return ss.str();
        }

        /// @brief 递归创建目录（类似 mkdir -p）
        void create_dir_recursive(const std::string& path) 
        {
            std::string p = path;
            // 去掉末尾的 '/'
            while (!p.empty() && p.back() == '/') {
                p.pop_back();
            }
            if (p.empty()) {
                return;
            }
            // 逐级创建
            size_t pos = 0;
            while ((pos = p.find('/', pos + 1)) != std::string::npos) {
                std::string sub = p.substr(0, pos);
                if (!sub.empty()) {
                    ::mkdir(sub.c_str(), 0755);
                }
            }
            ::mkdir(p.c_str(), 0755);
        }

        myLogger() 
        {
            // 确保日志目录存在（新安装的包 install 前缀下可能还没有 log/ 目录）
            create_dir_recursive(log_path);
            check_rotate();
            log_file.open(log_file_full_path, std::ios::out | std::ios::app);
            if (!log_file.is_open()) {
                throw std::runtime_error("Failed to open log file: " + log_file_full_path);
            }
        }   

    public:
            myLogger(const myLogger&) = delete;
            myLogger& operator=(const myLogger&) = delete;
            static myLogger* get_instance() 
            {
                static std::mutex instance_mutex;
                std::lock_guard<std::mutex> lock(instance_mutex);
                if (instance == nullptr) {
                    instance = new myLogger();
                }
                return instance;
            }   

            void info(const char* file, int line,const char* fmt, ...  ) {
                std::lock_guard<std::mutex> lock(mtx);
                if (!log_file.is_open()) return;
                // 先检查是否需要切割
                    check_rotate();
                char buf[LOG_MAX_SIZE];
                char full_fmt[LOG_MAX_SIZE];
                // 1. 拼接文件+行号前缀
                snprintf(full_fmt, sizeof(full_fmt), "[%s:%d] %s", file, line, fmt);
                va_list ap;
                va_start(ap, fmt);
                 vsnprintf(buf, sizeof(buf), full_fmt, ap);
                
                va_end(ap);

                
                if (log_file.is_open())
                    log_file << "[" << get_time() << "] INFO  " << buf << std::endl;
            }

            void warn(const char* file, int line,const char* fmt, ...) {
                std::lock_guard<std::mutex> lock(mtx);
                if (!log_file.is_open()) return;
                // 先检查是否需要切割
                    check_rotate();
                char buf[LOG_MAX_SIZE];
                char full_fmt[LOG_MAX_SIZE];
                // 1. 拼接文件+行号前缀
                snprintf(full_fmt, sizeof(full_fmt), "[%s:%d] %s", file, line, fmt);
                va_list ap;
                va_start(ap, fmt);
                 vsnprintf(buf, sizeof(buf), full_fmt, ap);
                va_end(ap);

              
                if (log_file.is_open())
                    log_file << "[" << get_time() << "] WARN  " << buf << std::endl;
            }

            void error(const char* file, int line,const char* fmt, ...) {
                std::lock_guard<std::mutex> lock(mtx);
                if (!log_file.is_open()) return;
                // 先检查是否需要切割
                    check_rotate();
                char buf[LOG_MAX_SIZE];
                char full_fmt[LOG_MAX_SIZE];
                // 1. 拼接文件+行号前缀
                snprintf(full_fmt, sizeof(full_fmt), "[%s:%d] %s", file, line, fmt);
                va_list ap;
                va_start(ap, fmt);
                 vsnprintf(buf, sizeof(buf), full_fmt, ap);
                va_end(ap);

              
                if (log_file.is_open())
                    log_file << "[" << get_time() << "] ERROR " << buf << std::endl;
                
                std::exit(EXIT_FAILURE) ; //直接退出  
            }
        


};

inline myLogger* myLogger::instance = nullptr;

// 全局日志宏（任意类、任意地方直接调用）
#define LOG_INFO(...)    myLogger::get_instance()->info( __FILE__, __LINE__, __VA_ARGS__)
#define LOG_WARN(...)    myLogger::get_instance()->warn(__FILE__, __LINE__, __VA_ARGS__)
#define LOG_ERROR(...)   myLogger::get_instance()->error(__FILE__, __LINE__, __VA_ARGS__)