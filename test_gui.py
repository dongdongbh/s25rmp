import pybullet as p

# 使用 DIRECT 模式，不启动 GUI 窗口
p.connect(p.DIRECT)

# 加载 EGL 渲染插件
plugin_path = "cpython-38-x86_64-linux-gnu.so"
egl_plugin = p.loadPlugin(plugin_path)
if egl_plugin < 0:
    print("Failed to load EGL plugin, rendering disabled")
else:
    print("EGL plugin loaded successfully!")

# 继续你的模拟代码
