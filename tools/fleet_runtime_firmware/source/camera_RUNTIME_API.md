# 相机运行时固件 2026.11.16-runtime1

本版本在相机端限制 MJPEG 图传帧率，默认 5 FPS，运行时范围 1～30。
传感器继续采集最新画面，帧率表示网络输出上限，不表示曝光频率或保证达到该值。
传感器、网络和客户端处理能力可能使实际输出更低。

## 网页与命令

- `http://相机IP/settings`：编辑当前参数 JSON，点击“应用并保存”。
- `http://相机IP/`：原 Wi-Fi 配网页面，保存后重启。
- `GET /api/status`：固件版本、设备名、IP、参数、Wi-Fi 省电状态和 SNTP 状态。
- `GET /api/settings`：只返回当前参数。
- `POST /api/settings`：接受下面字段的部分更新，校验、应用传感器、写 NVS 后返回成功。
- `POST /api/wifi`：`{"ssid":"实验网络","password":"实际密码"}`，保存后约 1.5 秒重启切换网络。
- `/handshake` 和 `/api/handshake` 保留旧字段，新增 `firmware_version`。
- 视频地址仍为 `http://相机IP:81/`。

示例（把相机 IP 替换为实际值；不要用底盘 IP）：

```bash
curl --noproxy '*' http://相机IP/api/status
curl --noproxy '*' -H 'Content-Type: application/json' \
  -d '{"fps":5,"jpeg_quality":20,"resolution":"320x240"}' \
  http://相机IP/api/settings
curl --noproxy '*' -H 'Content-Type: application/json' \
  -d '{"auto_exposure":false,"exposure":300,"auto_gain":false,"gain":5}' \
  http://相机IP/api/settings
curl --noproxy '*' -H 'Content-Type: application/json' \
  -d '{"auto_white_balance":false,"white_balance_mode":3}' \
  http://相机IP/api/settings
```

| 字段 | 类型及范围 | 默认及说明 |
|---|---|---|
| `fps` | 整数 1～30 | 5，图传输出上限 |
| `jpeg_quality` | 整数 4～63 | 12，数值越小质量/码率通常越高 |
| `resolution` | 字符串 | 默认 `640x480`；支持 `160x120`, `320x240`, `640x480`, `800x600`, `1024x768`, `1280x1024`, `1600x1200`，还受实际传感器最大尺寸限制 |
| `auto_exposure` | JSON 布尔 | `true` 自动曝光 |
| `exposure` | 整数 0～1200 | 300，仅手动曝光时写硬件；是传感器寄存器单位，**不是微秒** |
| `exposure_level` | 整数 -2～2 | 0，自动曝光亮度补偿 |
| `auto_white_balance` | JSON 布尔 | `true` |
| `white_balance_mode` | 整数 0～4 | 0 自动；1 晴天、2 阴天、3 办公室、4 家居 |
| `auto_gain` | JSON 布尔 | `true` 自动增益 |
| `gain` | 整数 0～30 | 0，仅手动增益时写硬件；为驱动增益等级，不是 dB |

自动白平衡必须使用 `auto_white_balance=true, white_balance_mode=0`；
手动白平衡必须使用 `false` 与 1～4。切换时将这两个字段一起提交。
白平衡提供自动/预置模式，未宣称支持未经标定的 Kelvin 色温或独立 RGB 增益。
自动曝光/增益开启时，`exposure`/`gain` 是存储的手动设定值，不是实时自动测量值。
`GET /api/status` 的 `settings` 是上述设定值；`sensor_consistent` 为 false 时图传停止，
应重新设置参数或重启，不能把不完整的传感器配置当作成功。

参数拒绝未知字段、重复键、错误类型、非整数和越界值。
更改参数时持有与图传共享的互斥锁，视频发送/归还帧后才更改传感器，
再排空旧帧。启动时按 UXGA 初始化，并显式分配两个各 1 MiB 的 JPEG 缓冲，
避免沿用原 VGA 自动计算的小缓冲。极端高细节/低压缩率帧仍可能超过缓冲，
驱动会拒绝超限帧；这不保证所有画面组合都能达到目标帧率。
繁忙时返回 HTTP 503，稍后重试；传感器或 NVS 失败返回 500 并尝试回滚，
`settings_persisted` 和 `sensor_consistent` 告知回滚状态。HTTP 200 的 `saved=true`
表示已经应用并完成 NVS 提交。高分辨率/低压缩值可能降低实际帧率并增大带宽。

MJPEG 响应头 `X-Camera-Stream-FPS` 记录本次连接开始时的配置，
修改后每帧使用新值；需要重新连接才能看到更新的响应头。
`X-Camera-Firmware` 是该固件版本。

## NVS 与 Wi-Fi

相机参数存为 `cam_runtime/settings_v1` 单记录，包含 schema 版本；缺省或无效记录
使用默认值并在状态中标注未保存，首次成功设置后持久化。
启动遇 NVS 空间不足/格式错误不会自动整片擦除，以保留各车凭据与设置。

Wi-Fi 初次升级继续读取原 `wifi_cfg/ssid` 与 `wifi_cfg/pass`。
首次修改 Wi-Fi 后改用 `wifi_cfg/sta_v1` 单记录，避免 SSID/密码分别写入时不匹配。
原 CGI 配网地址仍调用同一保存接口；密码不会出现在状态 API 或请求日志中。
32 字节 SSID 与 64 位十六进制 PSK 完整保留；普通 WPA 密码 8～63 字节，
空密码表示开放网络。更改 Wi-Fi 属于运行配置，不需刷机，但会重启并暂时中断连接。
若回退到旧固件，旧固件仍读取旧 SSID/密码键，可能需要重新配网。

STA 启动、重新获 IP、AP 启动路径都显式设 `WIFI_PS_NONE`。
`GET /api/status` 的 `wifi.power_save` 来自驱动实际查询，正常为 false。
这关闭站点省电，不保证路由器队列、拥塞或应用处理没有延迟。

## 时间同步与版本

STA 获得 IP 后启动 SNTP，固定主机 `10.1.1.249`、每 60 秒同步，禁用 DHCP
提供的 NTP 服务器覆盖。`time_sync.synced` 仅在最近 180 秒有成功同步且 Wi-Fi
仍连接时为 true；未同步时 `last_sync_unix_us` 为 null。时钟同步不表示已测量
端到端视觉时延，也不承诺微秒级同步精度。主电脑需要正常运行 NTP 服务。

`GET /api/status` 例：

```json
{
  "firmware_version":"2026.11.16-runtime1",
  "firmware":{"version":"2026.11.16-runtime1","project":"gc0308_http_stream",
    "idf_version":"v5.5.3","build_date":"...","build_time":"...","elf_sha256":"..."},
  "name":"Maturo_CAM_XXXXXXXXXXXX",
  "time_sync":{"server":"10.1.1.249","synced":false,"last_sync_unix_us":null,
    "sync_interval_seconds":60,"last_sync_age_seconds":null},
  "wifi":{"ssid":"已保存的SSID","mode":"STA","power_save":false}
}
```

主机 session 记录应保存 `name`、`firmware` 与 `time_sync`，不能把缺失版本
默认成新固件版本。相机不主动向主机推日志，由主机在 session 开始/结束读取。

## 构建与验证范围

ESP-IDF 5.5.3，ESP32-S3，4 MB Flash，8 MB Octal PSRAM；保留项目名和分区表。
只替换活动应用槽即可升级；不要整片擦除或写入其他车的 NVS。

```bash
bash tests/run_stream_tests.sh
IDF_PATH=/你的/esp-idf bash tests/run_settings_tests.sh
idf.py build
```

离线测试直接编译实际发送函数，验证限帧、失败时帧归还和互斥锁、运行时改帧率；
参数测试覆盖范围/JSON 类型/重复键/模式冲突、传感器/NVS 失败回滚。
本次已构建并离线验证，**运行时新版本尚未在实物刷写验证**，不能继承上一版
固定 5 FPS 的硬件验证结论。批量刷写前需验证单台改参数、重启保留、视频、
Wi-Fi 切换和 SNTP 状态。
