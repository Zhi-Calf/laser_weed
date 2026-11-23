# cpp-httplib (vendored)

| 项 | 值 |
|---|---|
| 上游 | https://github.com/yhirose/cpp-httplib |
| 版本 | v0.14.3 (与 Ubuntu 24.04 apt 的 libcpp-httplib-dev 同版本) |
| 文件 | `httplib.h` (单头文件, 未修改) |
| 许可 | MIT, 见 `LICENSE` |
| sha256 | `2b857880236a38a64383e72e19d5af5a1b9a8f856884435ad2f4940cbcc4229a` |

随包携带而不用 apt: Ubuntu 22.04 (RK3588) 的 apt 版本是 0.10.x, 两边 API 有差异;
单头文件随包走, 车上与开发机编译的是同一份代码, 也少装一个系统包。

升级: 换掉 `httplib.h` 与 `LICENSE`, 更新上表的版本与 sha256。
