# TITA 上电与 SSH 连接指南（USB / Wi-Fi）

TITA能够使用 USB 和 WIFI 连接


## 2. 准备工作

- 机器人电池已安装、锁紧且有电。机器人平整。人员安全。

**官方说明：随包装附带的刷机线不可作为调试线使用。请，不要进入刷机模式。**

## 3. 上电并连接调试线

1. 通过本体电源按钮（位于电池上）正常上电。
2. 等待内部计算机提示音播报启动完成。

## 4. 连接机载上位机

### 4.1 连接方式1: USB 连接

使用支持数据传输的 USB 3.0 Type-C 线，连接 DBG 接口。在外接电脑终端设置机器人 IP：

```bash
tita_ip=192.168.42.1
```

### 4.1 连接方式2: 连接到共同的 WIFI 网络（推荐方式）

需要先通过USB连接机器人后，使用上位机中提供的 TUI `wifi-app` 连接到和外接电脑同处一个wifi，并且获取局域网下的ip地址。

1. 通过 USB 连接后，在TITA机载电脑中执行
```bash
sudo wifi-app -on # 根据 TUI 的提示操作
ip addr # 获取 ip
```

得到在局域网下的 ip 为 `tita_ip`

### 4.2 连接方式3: Wi-Fi 热点连接

推荐使用连接方式2, 连接更稳定。如要使用TITA开热点，参见 [AP模式的manual](https://tita-development-manual-uc.readthedocs.io/zh-cn/latest/pages/TITA-wifi_app.html#id1)

## 5. SSH 登录

[连接](#4-选择连接方式) TITA 并设置 `tita_ip` 后。

在外接电脑终端执行：

```bash
ssh "robot@$tita_ip"
```

官方默认账号及密码：

```text
用户名：robot
密码：apollo
```

## 7. 参考资料

- [TITA 官方开发手册](https://tita-development-manual-uc.readthedocs.io/zh-cn/latest/index.html)
- [Ubuntu 系统教程：Connect TITA Robot System](https://tita-development-manual-uc.readthedocs.io/zh-cn/latest/pages/ubuntu-flash.html)
- [官方 Quick Start](https://tita-development-manual-uc.readthedocs.io/zh-cn/latest/pages/quickstart.html)
- [官方 Wi-Fi 便携式应用](https://tita-development-manual-uc.readthedocs.io/zh-cn/latest/pages/TITA-wifi_app.html)
- [本仓库 README](../README.md)
