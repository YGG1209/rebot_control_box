# 安装
克隆代码库
```
git clone https://github.com/YGG1209/rebot_control_box.git
```
切换到feature/rebot_rtde分支，并更新代码

## 安装打包工具
**！注意首先需要注册好PyPI账号**、

```
cd /home/rebot/rebot_rtde

python3 -m venv --prompt rebot_rtde .venv
source .venv/bin/activate

python -m pip install --upgrade pip build twine
```

## 生成发布文件
```
python -m build
```
成功后，dist 目录通常会生成：
```
dist/
├── rebot_rtde-0.1.0-py3-none-any.whl
└── rebot_rtde-0.1.0.tar.gz
```

## 上传正式的PyPI
确认要公开发布后执行：
```
python -m twine upload --username __token__ \
  dist/rebot_rtde-0.1.0-py3-none-any.whl \
  dist/rebot_rtde-0.1.0.tar.gz
```
提示输入密码或 API Token 时，粘贴完整的token


# rebot_rtde 版本更新与发布

以下以 `0.1.0` 更新到 `0.1.1` 为例。

## 1. 修改版本号

完成代码更新后，修改 `pyproject.toml`：

```toml
version = "0.1.1"
```

同时修改 `src/rebot_rtde_control/__init__.py`：

```python
__version__ = "0.1.1"
```

两个文件中的版本号必须保持一致。

## 2. 重新打包

进入项目目录，激活发布环境并构建：

```bash
cd /home/rebot/rebot_rtde
source .venv-publish/bin/activate
python -m build
```

生成的新版本文件为：

```text
dist/rebot_rtde-0.1.1-py3-none-any.whl
dist/rebot_rtde-0.1.1.tar.gz
```

## 3. 上传新版本

```bash
python -m twine upload --username __token__ \
  dist/rebot_rtde-0.1.1-py3-none-any.whl \
  dist/rebot_rtde-0.1.1.tar.gz
```

`__token__` 原样保留。出现输入提示时，粘贴 PyPI API Token 并回车。
终端不显示输入字符是正常现象。

以后发布时，将上述版本号替换为本次的新版本号。

## 4. 用户升级

上传成功后，用户执行：

```bash
python -m pip install --upgrade rebot_rtde
```

查看已安装的版本：

```bash
python -m pip show rebot_rtde
```

## 注意事项

- 每次更新已发布的代码，都要使用新的版本号。
- PyPI 不允许覆盖已经上传的同名发行文件。
- 不需要删除 PyPI 上的旧版本。
- 上传时只指定本次的新版本文件。
- 已安装的用户不会自动更新，需要主动执行升级命令。
