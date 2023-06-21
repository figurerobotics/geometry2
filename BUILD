# Copyright 2023 Figure AI, Inc

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "tf2",
    srcs = glob(["tf2/src/**/*.cpp"]),
    hdrs = glob([
        "tf2/include/tf2/**/*.h",
        "tf2/include/tf2/**/*.inl",
    ]),
    copts = ["--std=c++20"],
    strip_include_prefix = "tf2/include",
)
