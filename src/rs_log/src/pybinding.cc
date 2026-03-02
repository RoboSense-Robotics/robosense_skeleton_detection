#include <mutex>

#include <pybind11/pybind11.h>

#ifndef MODULE_NAME
#define MODULE_NAME "RSLOG_PY"
#endif
#include "rs_log/init.h"

namespace py = pybind11;

class RSLogAdapter {
public:
  virtual ~RSLogAdapter() = default;

  static void init(std::string const &module_name, std::string const &log_dir,
                   std::string const &log_level) {
    std::call_once(instance_flag_, [&module_name, &log_dir, &log_level]() {
      module_name_ = module_name;
      if (log_level == "DEBUG") {
        is_debug_level_ = true;
      }
      robosense::log::Init(module_name.c_str(), log_dir.c_str(),
                           log_level.c_str());
    });
  }

  inline static bool is_debug_level() { return is_debug_level_; }

#define RSLOG_ADAPTER_DEFINE_FUNC(LEVEL, UPPER_LEVEL)                          \
  static void LEVEL(std::string const &msg) {                                  \
    if (!Py_IsInitialized()) {                                                 \
      AERROR << "Python interpreter not initialized!";                         \
      return;                                                                  \
    }                                                                          \
    std::string filename{}, function{};                                        \
    int lineno{};                                                              \
    get_file_info(filename, lineno, function);                                 \
    google::LogMessage(filename.c_str(), lineno, google::UPPER_LEVEL).stream() \
        << LEFT_BRACKET << module_name_ << RIGHT_BRACKET << LEFT_BRACKET       \
        << function << RIGHT_BRACKET << msg;                                   \
  }

  RSLOG_ADAPTER_DEFINE_FUNC(debug, INFO)
  RSLOG_ADAPTER_DEFINE_FUNC(info, INFO)
  RSLOG_ADAPTER_DEFINE_FUNC(warn, WARNING)
  RSLOG_ADAPTER_DEFINE_FUNC(error, ERROR)
  RSLOG_ADAPTER_DEFINE_FUNC(fatal, FATAL)
#undef RSLOG_ADAPTER_DEFINE_FUNC

protected:
  RSLogAdapter() = default;

private:
  static void get_file_info(std::string &filename, int &lineno,
                            std::string &function) {
    try {
      py::gil_scoped_acquire acquire;
      py::module inspect = py::module::import("inspect");
      py::object currentframe_func = inspect.attr("currentframe");

      py::object frame = currentframe_func();
      if (!frame.is_none()) {
        py::object fileinfo = frame.attr("f_code");
        filename = fileinfo.attr("co_filename").cast<std::string>();
        lineno = frame.attr("f_lineno").cast<int>();
        function = fileinfo.attr("co_name").cast<std::string>();
      }
    } catch (const py::error_already_set &e) {
      AERROR << "Python error: " << e.what();
    } catch (const std::exception &e) {
      AERROR << "C++ error: " << e.what();
    }
  }

private:
  inline static std::once_flag instance_flag_;
  inline static std::string module_name_ = "UNKNOWN";
  inline static bool is_debug_level_ = false;
};

PYBIND11_MODULE(rs_log, m) {
  m.doc() = "python wrapper for library rs_log";

  py::class_<RSLogAdapter>(m, "logger")
      .def_static("init", &RSLogAdapter::init, py::arg("module_name"),
                  py::arg("log_dir") = "/mnt/ssd/RDCS_LOG_ROOT",
                  py::arg("log_level") = "INFO")
      .def_static(
          "debug",
          [](std::string const &msg) {
            if (RSLogAdapter::is_debug_level()) {
              RSLogAdapter::debug(msg);
            }
          },
          py::arg("msg"))
      .def_static("info", &RSLogAdapter::info, py::arg("msg"))
      .def_static("warn", &RSLogAdapter::warn, py::arg("msg"))
      .def_static("warning", &RSLogAdapter::warn, py::arg("msg"))
      .def_static("error", &RSLogAdapter::error, py::arg("msg"))
      .def_static("fatal", &RSLogAdapter::fatal, py::arg("msg"));
}
