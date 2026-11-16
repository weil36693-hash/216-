小车底层运动与动捕读取接口（ROS 2 Humble / Python）

作用
  按配置中的车辆名，把差速速度发布到对应 ROS_DOMAIN_ID 的 /cmd_vel。
  支持单车、任选多车、全部 25 车，以及不同车辆的不同速度。
  Fleet 只负责运动命令；无需动捕、雷达、里程计或其它传感器在线。
  Mocap 提供独立的只读位姿接口，可与 Fleet 同时使用。
  网页另提供基于动捕的 CSV 时间轨迹跟踪；Fleet 底层接口仍可单独使用。
  避障、在线规划和其它实验策略由调用程序负责。

准备
  cd ~/leap-ros
  source /opt/ros/humble/setup.bash
  colcon build --packages-select leap_fleet
  source install/setup.bash

  Fleet 速度接口本身不启动 Agent、不配置或重启车辆。
  底盘通信可由网页“连接管理”启动，也可沿用已有 ./start_fleet.sh。
  config/fleet.json 中 name/domain_id 决定路由，可选 cmd_vel_topic 默认 /cmd_vel。
  原有 IP、pose_topic 等字段无需改动。
  对同一辆车使用一个运动命令来源；旧导航、旧轨迹程序同样会写 /cmd_vel。

Python 程序（最常用方式）
  from leap_fleet import Fleet, Velocity

  with Fleet(config="/home/ros-car/leap-ros/config/fleet.json",
             cars=["car01", "car02"]) as fleet:
      if not fleet.wait_for_subscribers(timeout=5.0):
          raise RuntimeError("尚未发现所选车辆的速度订阅者")

      # 单次发送；v 单位 m/s，w 单位 rad/s。
      # v > 0 前进，v < 0 后退，w > 0 绕车体 +Z 逆时针转向。
      # 对应 Twist.linear.x、Twist.angular.z；其余轴始终为 0。
      fleet["car01"].set_velocity(v=0.05, w=0.0)
      fleet["car01"].stop()

      # 同一轮分别给多辆车不同命令；先验证全部输入再发布。
      fleet.set_velocities({
          "car01": (0.05, 0.0),
          "car02": Velocity(0.0, 0.1),
      })
      fleet.stop()

      # 定频发送 2 秒并在结束或中断时向本次参与车辆发送 0。
      fleet.run_for({"car01": (0.05, 0.0), "car02": (0.05, 0.0)},
                    duration=2.0, rate=20.0)

  with 退出时默认以20Hz持续发送零速度1秒，再释放本次创建的 ROS Context。
  run_for 已完成持续停车且没有后续命令时，close 不再重复这一停车窗口。
  无 with 时可用 fleet.open() / fleet.close()，close 可重复调用。
  只选择一辆车即单车；省略 cars 则选择配置中全部车辆。
  car.set_velocity 与 fleet.set_velocity(car_id, v, w) 等价。
  fleet.stop(["car01", "car02"]) 仅向指定车辆发送一轮 0。
  fleet.stop_repeated(["car01", "car02"], duration=1.0, rate=20.0) 持续发送零速度。
  单车发送异常不会跳过同批其它车辆；持续停车会完成后续轮次后汇总异常。

接入自己的控制算法
  import time
  from leap_fleet import Fleet

  with Fleet(cars=["car01", "car02"]) as fleet:
      if not fleet.wait_for_subscribers(5.0):
          raise RuntimeError("车辆未连接")
      for step in range(100):
          # 把这一行替换成算法输出；可另用既有脚本/ROS订阅读取动捕和传感器。
          commands = {"car01": (0.05, 0.0), "car02": (0.03, 0.1)}
          fleet.set_velocities(commands)
          time.sleep(0.05)

  set_velocity/set_velocities 每次只发布一轮，不自动重发、保持或限制速度。
  持续运动时由算法循环发送。stop 也只是发布零速度，不会取消调用者的循环。
  run_for 是固定速度的阻塞便捷方法，不是后台任务或轨迹执行器。
  多车批量发布快速依次进行，不承诺硬实时同步或通信级原子执行。
  wait_for_subscribers 只确认 DDS 订阅匹配，不代表底盘已经执行命令。
  close 的最终零速度发布不等于车辆已物理停车的确认。

命令行
  本工作区可用 ./drive_fleet.sh，脚本会加载环境并使用根目录 config/fleet.json。

  # 仅打印配置，不初始化 ROS、不发送命令。
  ./drive_fleet.sh list

  # 单车前进 2 秒，以默认 20 Hz 发布，结束发送 0。
  ./drive_fleet.sh velocity --cars car01 --v 0.05 --w 0 --duration 2

  # 任选多车；不传 --cars 则选择全部配置车辆。
  ./drive_fleet.sh velocity --cars car01 car02 --v 0.05 --duration 2

  # 停止指定车辆 / 全部车辆。
  ./drive_fleet.sh stop --cars car01 car02
  ./drive_fleet.sh stop

  不同车辆不同速度：准备 commands.json，例如
  {
    "car01": {"v": 0.05, "w": 0.0},
    "car02": {"v": 0.0, "w": 0.1}
  }
  ./drive_fleet.sh batch commands.json --duration 2 --rate 20

  batch 仅打开 JSON 中的车辆；显式 --cars 可限制允许的车辆集合。
  --wait 5 为所有选中车辆共同的发现等待上限；--wait 0 跳过等待。
  也可直接 ros2 run leap_fleet drive velocity --cars car01 --v 0.05 --duration 2
  --config 可显式指定其它 JSON。Python 和直接 ros2 run 的默认配置优先级：
  LEAP_FLEET_CONFIG 环境变量、源码工作区 config/fleet.json、已安装的配置副本。
  安装副本来自构建时的 config/fleet.json；修改根配置后可显式传路径或重新构建。

验证
  PYTHONPATH=src/leap_fleet python3 -m unittest discover -s src/leap_fleet/test -v
  colcon test --packages-select leap_fleet

  scripts/verification/fleet_velocity_domains.py 使用随机话题和本机模拟订阅者，
  验证 25 个独立域的命令路由，不使用真实 /cmd_vel，不连接小车。
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ROS_LOCALHOST_ONLY=1 python3 scripts/verification/fleet_velocity_domains.py

  模拟验证不能替代真实 25 辆车在无线网络下的连接与运动验证。

本机网页操作台
  首次新增网页后重新构建：
  source /opt/ros/humble/setup.bash
  colcon build --packages-select leap_fleet

  启动网页服务后，可以在页面内检测并连接小车：
  ./start_fleet_console.sh
  浏览器打开 http://127.0.0.1:8765/
  页面只监听本机。打开页面只读取状态；点击“检测并连接所选”才启动底盘服务。
  已经由终端连接的车辆直接复用，保持原启动终端运行。

  先体验完整流程（完全模拟，不创建ROS节点或连接小车）：
  ./start_fleet_console.sh --demo
  可用 --port 8766 更换网页端口，--config 文件路径更换车队配置。
  演示界面有醒目标识；切换实机模式需结束演示服务，再去掉 --demo 重新启动。

  操作员流程：
  0. 在“连接管理”选择本次要连接的车辆（离线车辆也可选）。
     点击“检测并连接所选”：逐车检查网络/通信设置，再启动各车底盘服务。
     默认连接目标为全部配置车辆；可清空后只选 car04 等本次使用的车。
     连接目标与下面的运动选车互相独立，连接成功不会自动选入运动任务。
     车辆离线、配置不匹配、等待数据、已连接分别显示，一辆失败不影响其它车。
     “仅检测”只检查状态和配置，不启动底盘服务或修改车辆。
     同次检测读取固件版本和车端超时停车状态：旧固件未提供字段时显示“未报告”。
     修复固件可显示“已启用 · 500 ms”；这是带成功检测时间的快照，不是实时轮询。
     重新检测失败时保留上次快照并标记失效，不能据旧值判断当前状态；序号不是ACK。
     默认连接不改车配置；参数不匹配时，页面展示当前值与目标值。
     核对后可点击该车的“应用配置并连接（会重启）”，保存通信设置并重启该车。
     配置预览与实际情况变化时会刷新预览，需核对新的差异后再操作。
  1. 看顶部服务状态与车辆列表。服务在线和车辆连接是两回事。
     “连接正常”=最近2秒内收到有效 /odom，且发现速度命令订阅接口。
     “接口待确认”=只发现部分接口/数据；“数据过期”=曾收到但超过2秒；
     “未连接”=尚未收到底盘数据且未发现命令接口；“连接异常”可查看具体错误。
     这不是ping检测，配置中有车辆不会自动显示在线。
  2. 搜索编号/IP，逐个选择或选择全部在线车辆；选中数量和实际编号保持可见。
     首次运行时若全部离线，先在“连接管理”检测，并核对车辆供电/网络。
  3. 设置统一v/w，或切到“逐车设置”分别填写；填持续时间0.1～60秒。
     前进、后退、左转、右转只是快捷填值，不会立即控制车辆。
     此时核对任务摘要中的车辆、速度、时长。
  4. 点击“启动本次任务”。网页按20Hz发送，显示倒计时和里程计实测v/w。
     执行期间锁定当前配置；同一控制台只执行一个任务。
     开始前检查连接，运动过程不加入避障或其它算法策略。
  5. 正常到时进入持续停车；提前结束点“停止本次”。“全车发送零速度”
     立即覆盖本页任务，并独立对全部配置车辆尝试停车，不等一辆车完成才处理下一辆。
     页面显示“正在停止”，随后逐车显示里程计近零反馈、未确认原因和发布计数。
     任务结束后“再次停止本次车辆”仍然可用；反馈未确认的车不能直接开始下一次运动。
  6. 点击控制栏标题中的“查看反馈”，或在右侧控制栏内滚动查看逐车结果。
     桌面控制栏可独立滚动，反馈不会被底部停止按钮遮住；移动端按页面自然排列。
     在“操作与结果”查看任务/失败信息并导出当前记录，然后重新选择/设置下一次。
     里程计读数过期时以状态和数据年龄为准，不能把旧的0速度当作当前停车证明。

  页面刷新后重新读取后端当前任务，不会重发运动命令。关闭浏览器不会取消
  后端已经开始的有限时长任务；结束时仍发布零速度。Ctrl+C退出服务会结束
  本页正在执行的任务，尝试停车并释放已创建的控制通道和本页连接。
  从未创建过控制通道的车辆不会因退出而收到命令。
  事件记录在内存中保留最近100条；需要保存实验记录时使用页面的日志导出。
  独立的Python算法程序不会被网页停止按钮永久禁止；若它继续发速度，
  会覆盖网页发布的零速度。网页操作和算法控制应按实验安排使用同一个控制源。
  网页按配置的cmd_vel_topic发布；可选odom_topic默认/odom，方便接入其它底盘。

  连接管理与进程生命周期：
  - “已连接/外部管理”表示已有终端启动的底盘，网页不会接管或关闭它。
  - “本页管理”表示由网页后端启动；重复连接不会启动另一套 Agent。
  - “关闭本页连接”只关闭本页启动的通信进程，不是停车按钮。
    本页运动任务执行期间不能更改连接；先使用“停止本次”结束任务。
    有其它速度发布源或已观察到非零速度时，也不能配置或关闭连接。
  - 关闭浏览器不会关闭后端连接。退出网页服务时，先结束本页运动任务，
    再释放本页启动的底盘；由其它终端启动的底盘保持运行。
  - 连接失败可查看每车的错误和日志尾部，处理后重新检测并连接。
    日志位于 log/web_connections_*，各车底盘详细日志仍在 log/fleet_base_*。
  - 网页服务只需启动一次，之后每次选车连接可以直接在网页完成。
    该功能不自动启动动捕客户端，也不发送运动指令。

多车发送与停车可靠性（2026-11-14）
  网页每辆车使用独立线程和持久ROS发布器。运动任务结束不会销毁发布器；
  单车慢调用/异常不会阻塞其它车的发送，也不会阻塞HTTP停止接口。
  所选车辆全部完成DDS发现后才开始运动，发现总等待上限5秒。
  速度任务共享截止时间；CSV 任务使用共同起点和每车各自的结束时间。
  停止覆盖尚未发送的运动目标；不排队补发旧任务。
  已进入中间件的阻塞调用无法在Python里强制撤销；恢复后先发零，未处理完前不接受新任务。
  任务调度按单调时钟扣除发送耗时，错过的周期不会密集补发；不承诺硬实时同步。

  停车使用独立20Hz频率，至少持续1秒；首次成功发布后计时，发送异常逐车保留。
  发布成功/ DDS ACK 都不是车端收到并执行的回执。DDS ACK超时不再被忽略。
  网页在最多3秒的核对窗口内要求：
    - 首次零速度发布之后新接收的里程计；数据年龄不超过0.3秒；
    - 至少3个不同接收样本，近零状态跨越至少0.2秒；
    - |v| <= 0.005m/s、|w| <= 0.02rad/s，且持续零速发送窗口已结束。
  老的0、断流、无效数值、仍在运动或发布阻塞都不能给出本次近零反馈确认。
  “里程计停车反馈已确认”只表示满足上述观测准则，不是物理急停或固件ACK。
  当前依据本机接收时间与序号，尚不验证车端源时间戳或多源消息真伪；
  实物验证仍须确认编码器/里程计的非零响应与实际运动对应，不能仅看停止时的0。
  某车停车反馈未确认时，固定速度模式禁止该车新任务；CSV 轨迹模式仅提示风险。
  结束后再次停止会重新发送零速并采集新的反馈，不只修改任务状态。
  配置或关闭底盘连接前先释放已空闲的本页控制发布器，再核验其它控制来源。

  SDK的set_velocities仍是同步逐车调用；网页的线程隔离由MotionChannels实现。
  stop_repeated/run_for/close改善主机端重发，不保证阻塞的中间件调用在期限内返回。
  用户提供的车端工程已检出到 /home/ros-car/leap_ros_mcu_driver，来源为
  https://github.com/czu963889306-dev/leap_ros_mcu_driver 。该源码原先没有cmd_vel超时。
  车端补丁为micro-ROS速度加入500ms有效期，由20ms控制循环独立检查，
  并上报版本与cmd_vel_watchdog状态；必须编译、刷入控制板后才会在实车生效。
  网络彻底中断、控制电脑退出或进程故障时，主机重发不能替代车端旧速度失效。
  当前实车尚未刷入本次补丁；本轮没有进行实车运动测试或改变网络参数。
  完整ESP-IDF v5.5.3构建已通过，目标ESP32-S3/4MB Flash；应用镜像1,111,552字节。
  镜像、分组件地址、SHA256与构建记录：log/firmware_motion_safety_build/。
  源码补丁：log/firmware_motion_safety.patch；固件版本字段为2.2.1-motion-safety。
  刷写前核对实际板型与现有分区，备份配置；不能默认从0地址写整包覆盖NVS。
  修复固件下连续速度命令需要定期刷新，推荐沿用20Hz；单次命令不会无限保持。
  本地接收/应用计数用于排查，不是发送端序号，也不是轮子已执行的确认。
  标准Twist没有发送时间戳，固件超时从车端接收时起算，不能判断网络中滞留报文的原始年龄。

动捕位置接入（2026-11-14）
  已由现场移动核对：car04（10.1.1.214）对应刚体 car_04。
  config/fleet.json 的该车 mocap_binding_confirmed=true 记录这个人工确认；
  其它车辆仍需逐车核对。绑定确认不等于刚体坐标轴已与车体对齐。

  保持已有 VRPN 客户端运行；只需要一份客户端，不要重复启动。
  动捕默认 ROS 域 0，底盘仍分别处于各自域（例如 car04 在域 4）。
  网页后台分别订阅两者，直接汇总，不桥接或改写底盘定位话题。
  如果尚未启动任何动捕客户端，可另开终端运行：
    ./read_fleet_positions.sh --start-vrpn
  已有客户端时只查看坐标：
    ./read_fleet_positions.sh

  更新软件后重新构建 leap_fleet，并在当前任务结束后重启网页服务：
    source /opt/ros/humble/setup.bash
    colcon build --packages-select leap_fleet
    ./start_fleet_console.sh
  先在原网页服务终端 Ctrl+C，再执行最后一条；底盘和动捕终端保持运行。
  根配置 mocap_domain_id 指定动捕域，缺省为 0；网页可用 --mocap-domain 覆盖。
  每辆车 pose_topic 指定位姿话题，例如 /vrpn_mocap/car_04/pose。

  网页分别显示底盘状态和动捕状态，并提供二维 XY 位置图、XYZ、刚体航向、
  坐标系、数据年龄及绑定是否核对。位置单位米，航向显示为度。
  位置图保持 XY 等比例，按 frame 分开显示；无坐标的车辆不会画在原点。
  过期位置明确标记，不视为实时位置；后台断开时显示旧状态。
  动捕缺失不会禁止现有的手动速度任务，也不会自动产生停车或规划策略。

  “数据新鲜”只表示最近 1 秒内收到合法位姿，不代表已确认有效跟踪。
  发布者多于 1 时页面提示重复数据源；请在原终端保留一个 VRPN 客户端。
  新的 ROS 时间戳或重复发布不能单独证明动捕软件重新识别成功。
  姿态来自动捕刚体：yaw 是其原始 +X 轴在世界 XY 平面的方向，
  并未自动校正成车头方向。接入闭环算法前需要确认 +X 指向车头、+Z 朝上，
  以及刚体原点相对车辆控制点的固定偏移。旧同域轨迹脚本不能直接套用。

独立 Python 位姿读取（只读，不会发送速度或零速度）
  import math
  import time
  from leap_fleet import Mocap

  with Mocap(cars=["car04"]) as mocap:
      for _ in range(100):
          pose = mocap.get("car04")
          if pose["status"] == "fresh":
              p = pose["position"]
              print(p["x"], p["y"], p["z"],
                    math.degrees(pose["yaw_rad"]), pose["frame_id"])
          else:
              print(pose["status"], pose["age_s"], pose["error"])
          time.sleep(0.1)

  Mocap(config=..., cars=..., domain_id=0, stale_after=1.0) 可显式配置；
  不传 cars 时读取全部配置车辆。构造对象不初始化 ROS，open 或 with 才连接。
  get(car_id) 返回该车状态副本，snapshot() 返回以车辆名为键的所有状态副本。
  状态 fresh/stale/missing/invalid/error 分别代表最近有效数据、数据过期、
  尚无数据/未配话题、收到非法位姿、接收接口异常。
  非法位姿会清空位置和航向，避免继续使用旧坐标。四元数通过检查后归一化。
  返回值包含 position、orientation、yaw_rad（弧度）、frame_id、topic、
  domain_id、age_s、received_at_unix、source_stamp、publishers、error、
  binding_confirmed。age_s 使用本机单调时钟，与源时间戳独立。
  主动算法可在自己的循环中同时读取 Mocap 并调用 Fleet；Mocap 本身不执行控制。

单个 CSV 控制多车时间轨迹（2026-11-14）
  在本机网页 http://127.0.0.1:8765/ 的任务设置中切换到“CSV 轨迹”。
  一个文件包含多辆车，按 car_id 自动匹配配置，不需要逐车上传。
  格式示例（世界坐标示意，使用前需按实际场地修改）：

    car_id,t,x,y
    04,0,1.00,0.50
    04,5,1.20,0.50
    04,10,1.40,0.50
    06,0,1.00,1.50
    06,5,1.20,1.50
    06,10,1.40,1.50

  car_id 支持 4、04、car04，并映射到配置中的 car04。未知编号拒绝导入。
  t 是相对任务共同起点的秒数；每车从 0 开始、严格递增且至少两个点。
  x/y 是共用动捕世界坐标系内的位置，单位米；不同车辆的行可以交错。
  重复同车同时间点、NaN/Inf、列数错误等会拒绝，并提示原因。
  文件要求 UTF-8（允许 BOM），表头严格为 car_id,t,x,y；最多 8 MiB、
  100000 个采样点，每车最长 3600 秒，坐标绝对值最多 100 米。
  旧 HTR 动捕导出的多段表头 CSV 不能直接用于这个新入口，需转换成上述四列。

  操作流程：
    1. 先在连接管理中检测并连接车辆，并保持动捕实时采集。
    2. 在 CSV 模式导入文件，查看彩色路线、车号匹配、起终点和总时长。
    3. 在轨迹车辆列表勾选本次参与车；可只执行文件中的一辆或几辆。
    4. 可选点击“检查轨迹状态（仅提示）”，查看起点、朝向、连接及动捕来源。
       风险提示不会阻止启动，不需要勾选确认；也可以跳过检查直接执行。
       导入、检查都不会自动移车到起点。
    5. 点击开始执行。各车使用统一任务时钟，按自己的最后时刻结束并停车。
       “停止本次”和“全车发送零速度”继续可用；结束后查看逐车停车反馈。

  尚无 CSV 时，可在网页用已选车辆当前动捕位置和方向生成直线 CSV。
  默认前进 0.5 米、20 秒，包含起停过程；生成后下载并导入预览，不会启动运动。
  也可由 Python 生成文件，以下调用仅计算字符串、没有 ROS 或车辆通信：

    from leap_fleet.trajectory import straight_csv, arc_csv, parse_csv
    text = straight_csv("car04", start=(1.0, 0.5), end=(1.5, 0.5), duration=20)
    # 多车文件保留一个表头，拼接其它车辆返回文本的第二行及以后。
    paths = parse_csv(text, known_cars=["car04"])
    reference = paths.tracks["car04"].sample(2.5)

  执行语义：
    后台按原时间轴线性插值 XY，以切线为前进方向，结合动捕或推算位姿计算 v/w。
    默认正向行驶；只有 t/x/y 的文件不能表达指定车头方向的倒车或原地转向。
    相邻重复位置表示停留。文件的原时间和坐标均保留，不做 RDP 简化，
    不因落后而延长时间，也不排队补发过期采样。末行时间到达后发送零速度。
    完成表示轨迹时间已执行且停车反馈通过，不代表每个点均精确到达。
    多车共用的是电脑端时钟，现有 Twist 通信不提供车端硬实时同步。
    已取消 0.15 m/s 的固定线速度上限；检查、生成和实际发送均不以此限速。
    线速度由 CSV 时间与位置、当前定位跟踪误差计算，仍按前进行驶，
    保留线加速度 0.10 m/s²、|w|<=0.6 rad/s、角加速度 1.2 rad/s² 的约束。
    Python control_velocity 的 max_v 默认为 None，算法可显式传入正数设置限速。
    取消固定限速不会修改 CSV 时间或坐标；折角过急的轨迹可能跟不上，
    制作文件时应安排足够的转弯时间。控制器与生成器可替换为后续实验算法。

  检查结果分为 warnings（风险提示）和 errors（当前无法执行的原因）。
  距起点>0.10 m、朝向偏差>0.35 rad、未核对绑定、未确认静止、停车反馈未确认、
  动捕来源/速度发布者重复、订阅端数量、frame 名称不同或姿态倾斜均只提示，
  不阻止启动；运行中位姿跳变或出现其它控制来源也只记录提示，继续执行。
  开始时需要可用的实时 XY 与航向（动捕年龄<=0.3秒）及已就绪的控制通道，
  缺少初始输入时无法启动。运行中的外部状态变化不再中止本次整组轨迹。
  动捕中断时，每车基于最后有效位姿与近期 v/w 指令推算当前位置，
  沿原任务时间轴继续执行；收到可用动捕位姿后自动切回，不重启任务、不补跑时间。
  动捕多来源、底盘失联等运行中异常只记风险提示；发送失败会继续尝试，
  不因这类外部状态触发整组停车。通信恢复前无法保证车端收到电脑发出的命令。
  网页逐车显示“动捕跟踪”或“动捕中断·推算继续（时长）”；恢复后清除当前中断标记，
  掉线/恢复历史仍在事件记录。等待或不可用的定位输入单独显示，不冒充正常动捕。
  动捕卡片和位置图仍显示实际接收数据状态，推算继续不表示动捕数据新鲜或实车位置准确。
  文件无效、所选车没有轨迹、任务/连接正在占用资源时会说明当前无法启动的原因。
  CSV 每车仍有独立结束时间，到时发送零速度并收集逐车停车反馈；
  “停止本次”和“全车发送零速度”优先撤销当前执行，恢复数据不会自动重启。
  CSV 发送线程持续使用最近指令，不启用 300 ms 未更新即停车的电脑端租约。
  上述继续策略仅用于 CSV 轨迹；固定速度任务及 Fleet 接口的现有截止/停车语义不变。
  电脑端到时停车与手动停止不构成断网/电脑退出后的车端停车保证；车辆固件仍未刷写。

  本机 HTTP 接口（仍需有效 Host、同源请求及 X-CSRF-Token）：
    GET  /api/trajectory              获取当前完整预览；/api/state 只附带轻量摘要。
    POST /api/trajectory/import       {"filename":"fleet.csv","csv":"car_id,t,x,y\n..."}
    POST /api/trajectory/check        {"trajectory_id":"导入返回的id","cars":["car04"]}
    POST /api/trajectory/run          与 check 相同字段；无需先调用 check。
    POST /api/trajectory/generate     {"cars":["car04"],"distance":0.5,"duration":20}
    POST /api/stop                    沿用现有 {"scope":"job"} 或 {"scope":"all"}。
  导入保存在当前服务内存中；重新导入会替换，服务重启后需要重新导入。
  operator_confirmed 布尔字段为旧客户端兼容保留，可省略，不作为执行门槛。
  新文件校验失败后，旧文件的任务 ID 失效；运动任务执行期间不允许替换文件。
