# PlatformIO 编译选项完整总结

根据 PlatformIO 官方文档，以下是 `platformio.ini` 中所有与**编译构建相关**的配置选项汇总：

---

## 🔧 Build Options 列表

| 选项 | 类型 | 说明 |
|------|------|------|
| `build_type` | String (单值) | 构建类型：`release` / `debug` / `test`，默认 `release` |
| `build_flags` | String (多值) | 添加编译器/链接器标志，作用于所有源码 |
| `build_src_flags` | String (多值) | 仅作用于 `src/` 目录下项目源码的编译标志 |
| `build_unflags` | String (多值) | 移除平台默认设置的编译标志 |
| `build_src_filter` | String (模板) | 过滤包含/排除参与构建的源文件 |
| `targets` | String (多值) | 指定 `pio run` 默认执行的目标任务 |

---

## 📋 各选项详解

### 1️⃣ `build_flags`（核心选项）
用于传递编译器/预处理器/汇编器/链接器的所有标志：

```ini
[env:myenv]
build_flags =
  ; 宏定义
  -D VERSION=1.0
  -D DEBUG=1
  -D BUILD_TIME=$UNIX_TIME
  
  ; 头文件路径
  -I/opt/include
  -I"${platformio.packages_dir}/framework-foo/include"
  
  ; 警告控制
  -Wall
  -Werror
  
  ; 优化级别
  -O2
  
  ; 链接库
  -L/opt/lib
  -lfoo
  
  ; 链接器选项（需 -Wl, 前缀）
  -Wl,--gc-sections
  
  ; 汇编器选项（需 -Wa, 前缀）
  -Wa,-ahlms=listing.txt
```

🔹 **作用域映射表**（标志自动归类到对应变量）：

| 标志格式 | 影响变量 | 用途 |
|---------|---------|------|
| `-D name[=def]` | CPPDEFINES | 预处理器宏定义 |
| `-U name` | CPPDEFINES | 取消宏定义 |
| `-Idir` | CPPPATH | 头文件搜索路径 |
| `-Wall`, `-Werror` | CCFLAGS | C/C++ 编译警告 |
| `-include file` | CCFLAGS | 强制包含头文件 |
| `-Wa,option` | ASFLAGS/CCFLAGS | 汇编器选项 |
| `-Wl,option` | LINKFLAGS | 链接器选项 |
| `-llibrary` | LIBS | 链接库名 |
| `-Ldir` | LIBPATH | 库文件搜索路径 |

🔹 **内置变量**（可在 `build_flags` 中引用）：
- `$PIOENV` - 当前环境名称
- `$PIOPLATFORM` - 平台名称
- `$PIOFRAMEWORK` - 框架列表
- `$PROJECT_DIR` - 项目根目录
- `$BUILD_DIR` - 构建输出目录
- `$UNIX_TIME` - Unix 时间戳
- `$PYTHONEXE` - Python 解释器路径

🔹 **动态标志**：使用 `!` 前缀执行外部命令生成标志
```ini
build_flags = !python scripts/get_git_rev.py
; 或 Unix:
build_flags = !echo '-D GIT_HASH="'$(git rev-parse --short HEAD)'"'
```

---

### 2️⃣ `build_src_flags`
与 `build_flags` 语法相同，但**仅作用于项目源码**（`src/` 目录），不影响第三方库的编译：
```ini
[env:myenv]
build_flags = -D GLOBAL_DEF=1          ; 作用于所有代码
build_src_flags = -D SRC_ONLY_DEF=1    ; 仅作用于 src/ 下的代码
```

---

### 3️⃣ `build_unflags`
用于**移除平台默认添加的编译标志**，常用于替换优化级别或标准：
```ini
[env:custom]
; 移除平台默认的 -Os 优化和 C++11 标准，改用 -O2 和 C++17
build_unflags = -Os -std=gnu++11
build_flags = -O2 -std=gnu++17
```

---

### 4️⃣ `build_src_filter`
使用 `+<pattern>`（包含）和 `-<pattern>`（排除）模板过滤源文件，支持 GLOB 模式：
```ini
[env:myenv]
; 默认值: +<*> -<.git/> -<.svn/>

; 示例：只编译 .c/.cpp，排除汇编文件
build_src_filter =
  +<**/*.c>
  +<**/*.cpp>
  -<**/*.S>
  -<**/*.asm>

; 示例：排除特定目录
build_src_filter =
  +<*>
  -<tests/>
  -<extras/>
  -<*.bak>
```

---

### 5️⃣ `build_type`
设置构建配置类型，影响调试信息和优化：
```ini
[env:release]
build_type = release    ; 默认，开启优化，无调试信息

[env:debug]  
build_type = debug      ; 无优化，含调试符号 (-g)

[env:test]
build_type = test       ; 用于单元测试的配置
```
> 📌 调试专属标志请使用 `debug_build_flags` 选项单独配置

---

### 6️⃣ `targets`
指定 `pio run` 默认执行的任务目标：
```ini
[env:deploy]
; 构建后自动上传并打开串口监视器
targets = upload, monitor

; 其他常用 targets:
; clean - 清理构建文件
; envdump - 输出环境变量
; size - 显示固件大小
; program - 烧录（区别于 upload）
```

---

## 💡 实用技巧

1. **多行写法**：每个标志单独一行，以两个空格缩进
2. **路径含空格**：用引号包裹 `-I"my path/include"`
3. **字符串化宏**：`'-D MYSTR="value with \"quotes\""'`
4. **继承配置**：使用 `${section.option}` 语法复用配置
5. **查看变量**：`pio run --target envdump` 输出所有构建变量

---

## 🔗 相关文档
- 完整配置参考：[platformio.ini 文档](https://docs.platformio.org/en/latest/projectconf/index.html)
- GCC 编译选项：[GCC Command Options](https://gcc.gnu.org/onlinedocs/gcc/)
- 高级脚本：[Advanced Scripting](https://docs.platformio.org/en/latest/scripting/index.html)

> ⚠️ 注意：`build_flags` 中的标志最终由底层工具链（GCC/Clang/ARMCC 等）解析，请确保标志与目标平台兼容。
