# Copyright 2023 Figure AI, Inc

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "tf2",
    srcs = glob(["src/**/*.cpp"]),
    hdrs = glob([
        "include/tf2/**/*.h",
        "include/tf2/**/*.inl",
    ]),
    deps = [
        "@fmt"
    ],
    copts = ["--std=c++20"],
    strip_include_prefix = "include",
)
