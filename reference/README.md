# 共用依赖与固件参考

这里保留2026-10-30源码交付包中明确选定的依赖，不是第二条开发主线。目录有CATKIN_IGNORE；编译我方工程从根src开始。

- Slicer_Workspace：选定的传感器、滤波、通信和最新记录固件；部分包经过裁剪。
- field_verified_3849abee：有既有现场试验对应关系的固件参考。
- external_installed_headers：已安装SDK头文件身份参考，不等于完整SDK源码或库。
- provenance：原来源资料、部署与候选差异、本次目录映射与哈希。

队友完整学习仓库：[potato-ugv-online-navigation-radio](https://github.com/20DCYy06/potato-ugv-online-navigation-radio)。本次比较版本为643a704；本目录的快照来自另一个已核验交付包，不默认与GitHub当前main一致。

离线软件检查显式指定UGV_SLICER_ROOT到这里的Slicer_Workspace；运行实车另需真正编译过的依赖overlay。详见[源码与复现](../docs/05_sources_and_reproduction.md)。
