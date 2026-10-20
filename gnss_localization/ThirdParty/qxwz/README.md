# 千寻 SDK 1.29.5 (RK3588)

本目录来源于千寻 SDK `qx-1.29-rk3588`：

- `include/qxwz_sdk.h`、`include/qxwz_types.h`
- `lib/aarch64/libqxwz-pssdk-1.29.5-20260916--unix--production-release.a`
- `lib/aarch64/libqxwz-pssdk-1.29.5-20260916--unix--production-release.so`

`sdk_client_node` 默认链接其中的 aarch64 静态库。上游 1.29 关闭了默认服务器域名，
节点通过 `qxwz_sdk_config_t.serv_conf` 显式传入 OpenAPI/OSS 域名和端口；坐标设置使用
`qxwz_sdk_set_nosr_coord_epoch_config`，不再使用旧版 `qxwz_sdk_set_coord_sys`。

当前随包的 release 库只覆盖 aarch64。x86 开发机默认只编译 `sdk_client.cpp` 做 API 检查，
不生成可执行文件；如提供匹配架构的 1.29.x 库，可用
`-DQXWZ_SDK_LIB=<path>` 配置。
