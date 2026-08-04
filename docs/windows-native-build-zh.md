# ODM Windows 原生编译记录

> 记录时间：2026-07-05。本文档记录在中文 Windows 11 系统上原生编译 OpenDroneMap (ODM 3.6.0) 及生成安装包的完整过程，包括遇到的所有问题、根因分析和解决办法。

## 1. 环境

### 官方要求（参照 `.github/workflows/publish-windows.yml`）

| 依赖 | CI 使用的版本 |
|---|---|
| 操作系统 | Windows Server 2022（英文区域设置） |
| Python | 3.12.9 |
| CMake | 3.24.x |
| CUDA Toolkit | 12.8.1 |
| MSVC | VS 2022 |

### 本机实际环境

| 依赖 | 本机版本 | 备注 |
|---|---|---|
| 操作系统 | Windows 11 Pro（中文区域，代码页 936/GBK） | 多个问题的根源 |
| Python | 3.12.7（Anaconda，`D:\Anaconda`） | 满足 configure.py 的 3.12.x 检查 |
| CMake | 系统装的 4.0.4（不可用）→ 便携版 3.31.6 | 见问题 1 |
| CUDA | 12.9（`CUDA_PATH` 系统未设置） | 见问题 7 |
| MSVC | VS 2022 Community 17.14（`D:\VS2022`，MSVC 14.44） | |
| Git | 2.51（`D:\Program Files (x86)\Git`） | 其 `usr\bin` 提供 patch.exe |
| GPU | RTX 4070 Ti SUPER（sm_89） | |

## 2. 构建流程

ODM Windows 原生构建入口是 `python configure.py build`，内部依次执行：

1. 创建 `venv/` 虚拟环境，安装 setuptools 和 numpy
2. 从 GitHub 下载官方预编译 vcpkg 依赖包（约 1GB，含 GDAL/Boost/CGAL 等，见 `vcpkg.json`）
3. CMake SuperBuild（`SuperBuild/CMakeLists.txt`）编译约 20 个 C++ 组件：OpenCV、GFlags、Ceres、OpenSfM、OpenMVS、PDAL、Entwine、Untwine、MvsTexturing、PoissonRecon、LAStools、Draco、PopSift（CUDA）等
4. 用 vcpkg 的 GDAL 从源码编译 GDAL/Fiona/Rasterio 三个 Python 绑定
5. `pip install -r requirements.txt`

本机使用的构建脚本（`E:\tools\odm-build.cmd`）：

```bat
@echo off
call "D:\VS2022\VC\Auxiliary\Build\vcvars64.bat"
set "PATH=E:\tools\cmake-3.31.6-windows-x86_64\bin;%PATH%;D:\Program Files (x86)\Git\usr\bin"
set "CUDA_PATH=C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.9"
cd /d E:\ODM
python configure.py build
```

要点：先进 vcvars64 环境；便携版 CMake 放 PATH 最前（压过系统的 4.0.4）；Git 的 `usr\bin` 放 PATH 最后（提供 patch.exe 又不遮蔽 MSVC 工具）；显式设置 `CUDA_PATH`。

## 3. 编译期问题与解决

### 问题 1：CMake 4.x 拒绝旧的 cmake_minimum_required

**现象**：SuperBuild 顶层 `cmake_minimum_required(VERSION 3.1)`，CMake 4.0 已移除对 <3.5 的兼容，配置直接失败；多个子项目（Ceres 2.0、MVE 等）同样受影响。

**解决**：不动系统 CMake，下载便携版 CMake 3.31.6 到 `E:\tools\cmake-3.31.6-windows-x86_64`，仅在构建脚本 PATH 中生效。（官方 CI 锁定 3.24.x 就是这个原因。）

### 问题 2：LAStools 编译失败（C2015/C2001/C2143 语法错误连环报）

**现象**：`lasmerge.cpp(131)` 等报"常量中的字符太多/常量中有换行符"。

**根因**：LAStools 源码是 Windows-1252 编码，字符字面量里有单字节 0x96（en-dash）。英文系统（代码页 1252）下 MSVC 把它当一个字符能编译过（官方 CI 恰好如此）；中文系统（代码页 936/GBK）下 0x96 是双字节字符前导字节，把后面的引号"吃"掉导致解析错乱。

**解决**：`SuperBuild/CMakeLists.txt` 给 lastools 单独加编译选项：

```cmake
set(LASTOOLS_CXX_FLAGS "-DCMAKE_CXX_FLAGS=/DWIN32 /D_WINDOWS /W3 /GR /EHsc /source-charset:.1252 /execution-charset:.1252")
```

### 问题 3：构建中断后重新配置，报 "could not find TARGET opencv"

**现象**：首轮构建失败后重跑 CMake 配置，OpenSfM 的 `DEPENDS ceres opencv gflags` 找不到 opencv 目标。

**根因**：`SETUP_EXTERNAL_PROJECT` 宏先 `find_package`，找到已安装的库就跳过创建外部项目目标。OpenCV 首轮已编译安装，重配置时被 find_package 命中 → 目标不再创建 → 依赖它的项目悬空。SuperBuild 隐含"配置只跑一次"的假设。

**解决**：续跑时强制创建目标（有 stamp 文件不会真重编）：

```
cmake .. -DCMAKE_BUILD_TYPE=Release -DODM_BUILD_OpenCV=ON -DODM_BUILD_GFlags=ON -DODM_BUILD_Ceres=ON -DCMAKE_TOOLCHAIN_FILE=...\vcpkg\scripts\buildsystems\vcpkg.cmake
```

另注意：`configure.py` 里只要 `SuperBuild/build` 和 `SuperBuild/install` 存在就会跳过 SuperBuild 编译，所以中断后需手动跑 `cmake --build . --config Release -j2` 完成 C++ 部分，再跑 `configure.py build` 走 Python 部分。

### 问题 4：Ceres 补丁步骤退出码 9009（命令找不到）

**现象**：`ceres-patch.rule` 自定义生成退出码 9009。

**根因**：`External-Ceres.cmake` 的 `PATCH_COMMAND patch -p1 < ceres.patch` 用的是 Unix `patch` 工具。GitHub CI 的 Windows runner 自带 Git 的 `usr\bin` 在 PATH 里，本机默认没有。

**解决**：把 `D:\Program Files (x86)\Git\usr\bin`（内含 patch.exe）追加到构建 PATH **末尾**（避免其中的 Unix 同名工具遮蔽 MSVC）。

### 问题 5：odm_orthophoto 配置失败，找到错误的 OpenCV

**现象**：`find_package(OpenCV)` 命中 `D:\3rdParty\opencv-4.8.0\build`（本机自装的旧 OpenCV），报"没有兼容的二进制"。

**根因**：odm_orthophoto 没有传 `OpenCV_DIR`；OpenMVS 在 Windows 上传的是 Linux 风格路径 `lib/cmake/opencv4`（不存在）。CMake 的 config 搜索会从 PATH 环境变量反推安装前缀，本机 PATH 里恰好有旧 OpenCV 的目录。

**解决**：像 OpenSfM 一样按平台显式传 `OpenCV_DIR`（Windows 上 SuperBuild OpenCV 的 config 在 `install/x64/vc17/lib`）。

### 问题 6：OpenMVS 的 CUDA 编译失败 `nvcc fatal: Unknown arch name 'sass_90'`

**根因**：`OpenMVS_MAX_CUDA_COMPATIBILITY=ON` 时，OpenMVS 用 `nvcc --list-gpu-arch` 和 `--list-gpu-code` 的输出逐项配对生成 `-gencode`。CUDA 12.9 引入 family-specific code 后两个列表长度/顺序不再对应，配对出 `arch=compute_100,code=sass_90` 这类非法参数。官方 CI 用 12.8.1 未触发。

**解决**：关掉 max-compat，显式指定架构列表（经 `LIST_SEPARATOR |` 传入外部项目）：

```cmake
-DOpenMVS_MAX_CUDA_COMPATIBILITY=OFF
-DCMAKE_CUDA_ARCHITECTURES=61-real|75-real|86-real|89-real|90-real|120
```

覆盖 GTX 10 系到 RTX 50 系，末位 120 带 PTX 向前兼容。

### 问题 7：`import cv2` 失败——OpenCV 静默跳过了 Python 绑定

**现象**：SuperBuild 全部编完、requirements 装完后，venv 里没有 cv2。OpenCV 配置摘要显示 `Unavailable: ... python3`。

**根因**（三层连环）：
1. venv 基于 Anaconda Python 创建，OpenCV 的检测逻辑无法从 venv 的 python.exe 反查到基础解释器的 `Include\` 和 `libs\python312.lib`，`PYTHON3LIBS_FOUND` 为空 → cv2 模块被**静默**跳过（构建照样成功）；
2. 修复传参后仍不生效——OpenCV `find_python()` 开头 `if(NOT ${found})`，上次失败留下的 `PYTHON3INTERP_FOUND=TRUE` INTERNAL 缓存把整个检测短路了，必须删掉 `build/opencv/CMakeCache.txt` 强制全新检测；
3. 再次重配后编译报 `Invalid character escape '\A'`——传入的 `D:\Anaconda\...` 反斜杠路径在 `target_link_libraries` 的字符串解析中被当转义符，需 `file(TO_CMAKE_PATH)` 转正斜杠。

**解决**：`External-OpenCV.cmake` Windows 分支用 `execute_process` 向 venv 解释器动态查询 include 目录和 `pythonXY.lib` 路径，转正斜杠后显式传 `-DPYTHON3_INCLUDE_DIR` / `-DPYTHON3_LIBRARY`。

**强制重编单个外部项目的方法**：删 `SuperBuild\build\<项目>\stamp\Release\` 下的 `*-configure`、`*-build`、`*-install`、`*-done` 戳记；如需重跑其内部 CMake 检测，还要删该项目构建目录的 `CMakeCache.txt`。

## 4. 安装包生成与可移植性问题

### 生成方法

```
python configure.py dist
```

自动下载 VC++ 运行库、便携版 Python 3.12.9（embed 版）、Inno Setup，产出 `dist\ODM_Setup_3.6.0.exe`（约 234MB，未签名）。安装包自带 Python，装机时 `winpostinstall.bat` 把 venv 的 `pyvenv.cfg` 指向内嵌解释器，理论上不依赖目标机任何环境。

### 问题 8：装到"干净"目录后 `import ctypes` 失败（DLL load failed）

**现象**：静默安装测试，`run.py` 起步就挂在 `vmem → ctypes`。sys.path 里赫然出现 `D:\Anaconda\Lib` 和 `D:\Anaconda\DLLs`，`_ctypes.pyd` 从 Anaconda 加载（其依赖的 ffi DLL 不在搜索路径）。

**根因**：官方内嵌 Python 用的是删除了 `._pth` 的 embed 版（"less-pth"，为了让 pyvenv.cfg 生效）。副作用：没有 `._pth` 时 python.exe 会读取注册表 `HKLM/HKCU\Software\Python\PythonCore\3.12\PythonPath` 并合并进 sys.path。本机 Anaconda 注册了这个键。**任何装有 Python 3.12 的最终用户机器都会踩**（官方安装包同样潜伏此问题）。

**解决**：`winpostinstall.bat` 安装时在内嵌 python.exe 旁写 `python312._pth`：

```
python312.zip
.
..\..
..\Lib\site-packages
import site
```

`._pth` 存在时解释器完全忽略注册表和 PYTHON* 环境变量，sys.path 被锁定为列出的相对路径（stdlib zip、扩展模块目录、ODM 根目录、venv site-packages），任何机器行为一致。相比给 `run.bat` 加 `python -E`，此法覆盖所有 python 调用（OpenSfM 的 `opensfm.bat` 等子进程也是裸调 `python`）。

### 问题 9：装到"干净"目录后 `import cv2` 失败（缺 avif.dll）

**现象**：`opencv_imgcodecs4120.dll` 及依赖它的 highgui/videoio 加载失败。`dumpbin /dependents` 显示 imgcodecs 依赖 `avif.dll`——不在打包产物里，只在 `D:\Anaconda\Library\bin` 有。

**根因**：OpenCV 配置时 `find_package` 经 PATH 反推发现了 Anaconda 的 libavif 并链了进去（vcpkg 依赖集里本无 libavif）。开发机上 Anaconda 的 DLL 兜底所以一切正常，装到没有 Anaconda 的机器立刻断链。典型的"构建机环境污染"问题。

**解决**：`External-OpenCV.cmake` 显式 `-DWITH_AVIF=OFF`，与官方依赖集一致；重编 OpenCV。

### 验证方法（建议每次打包后执行）

1. 静默安装到临时目录：`ODM_Setup_x.exe /VERYSILENT /NORESTART /SUPPRESSMSGBOXES /DIR=<临时目录>`
2. 在安装目录环境下验证导入：`ctypes、cv2、osgeo.gdal、fiona、rasterio`
3. `python run.py --help` 冒烟测试
4. **全量依赖扫描**（脚本 `E:\tools\scan_deps.ps1`）：对安装树全部 DLL/pyd/exe 用 `dumpbin /dependents` 提取导入表，逐项核对"存在于安装树内 或 是 System32 系统库"，任何落空项即为可移植性缺陷。本次最终版扫描 653 个 PE 文件，零未解析依赖。
5. `unins000.exe /VERYSILENT` 卸载清理

## 5. 修复提交清单（分支 `windows-build-fixes`）

| 提交 | 内容 | 上游价值 |
|---|---|---|
| `Fix LAStools build failure on non-Western Windows locales` | 问题 2 | 高（所有非西文系统构建者） |
| `Point odm_orthophoto and OpenMVS at the SuperBuild OpenCV on Windows` | 问题 5 | 高 |
| `Replace OpenMVS MAX_CUDA_COMPATIBILITY with an explicit arch list` | 问题 6 | 高（CUDA ≥12.9 均触发） |
| `Pass explicit python include dir and library to OpenCV on Windows` | 问题 7 | 中（venv 基于非 python.org 解释器时） |
| `Disable AVIF when building OpenCV` | 问题 9 | 高（构建机污染防御） |

问题 8 的修复（`winpostinstall.bat` 写 `._pth`）不在本分支，它由分支
`fix-embedded-python-registry-pythonpath` 的提交
`Prevent registry PythonPath from polluting sys.path on Windows installs`
承载，并已单独提交上游 PR。**注意：本分支的工作区不含该修复**，若要从本分支
重新打安装包，需先合并或 cherry-pick 那个提交，否则产出的安装包在装有其他
Python 3.12 的机器上会因注册表 PythonPath 污染而无法启动。

问题 1/3/4 属于构建环境准备，不涉及仓库改动，已记录在上文和构建脚本中。
