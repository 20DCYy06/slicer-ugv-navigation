# 本地仓库整理记录

输入是已核验的2026-10-30源码包与前轮学习文档。候选源码提交27488d3a75f3a827de432c9056e65aa34e428dac；队友仓库比较版本643a704c5fa042b5e9f08d3c37ddad3a38e8b56a。

## 路径变化

| 原源码包 | 本仓库 |
|---|---|
| slicer_ugv_ros1/src | src |
| slicer_ugv_ros1/tools | tools |
| slicer_ugv_ros1/docs | docs/history，排除AI流程计划 |
| slicer_ugv_ros1/README.md | docs/history/PROJECT_README_27488d3.md |
| Slicer_Workspace | reference/Slicer_Workspace |
| reference/field_verified_3849abee | reference/field_verified_3849abee |
| external_installed_headers | reference/external_installed_headers |
| provenance | reference/provenance |
| validation | evidence/archive_20261030 |

未再次复制jetson_deployed_ugv整棵树，其与候选的比较和patch仍保留。未复制旧AGENTS/自动执行计划、原包入口README等为新运行指令。原包完整FILE_MANIFEST/SHA256SUMS在本目录加original_bundle前缀，仅用于追溯原包，不当成本仓库的路径清单。

## 修改范围

生产src、tools、Dockerfile和控制参数原样保留。新增总览/依赖/来源说明，重写学习文档和历史文档的本地链接；缺失归档链接明确标记。根Git忽略和行尾规则适应新布局。Livox参考副本.gitignore追加!package.xml，避免Git漏掉包声明；该修改只影响发布文件选择。

[repository_files.json](repository_files.json)记录每个发布文件的哈希、复制来源及是否改变。该文件自身不纳入自身哈希。原来源材料中的旧绝对路径和历史hash不冒充当前运行参数。

[packaging_validation.json](../../evidence/packaging_validation.json)记录本次源码身份、包发现、链接、隐藏文件和Git选择检查。本次没有重跑原431项、没有实车验收、没有发布远端。
