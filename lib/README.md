本目录用于存放工程内私有/本地 Arduino 库。

支持的结构（推荐 Arduino Library Layout）：
- lib/
  - MyLib/
    - library.properties   # 可选，但更标准
    - src/
      - MyLib.h
      - MyLib.cpp

放入后，VS Code 已配置 includePath 包含 lib/**，可直接在草图中：

#include <MyLib.h>

若使用官方/第三方公共库，建议通过 Arduino Library Manager 或 Arduino CLI 安装到 ~/Arduino/libraries 下。
