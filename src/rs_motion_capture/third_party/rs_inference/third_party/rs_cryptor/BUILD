load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "librs_cryptor",
    srcs = glob(["src/*.cpp"]),
    hdrs = glob(["include/**/*.h"]),
    includes = ["include"],
    strip_include_prefix = "include",
)

apollo_package()

cpplint()
