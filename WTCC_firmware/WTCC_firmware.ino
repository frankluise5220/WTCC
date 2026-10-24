/* v5.21 · WTCC 换水控制器固件 · 硬件 WTCC-ESP32-RLY4-V1
 * ⚠ 两个版本号各管一段，都要改：
 *    · 软件版本 = 下面的 #define FW_VER        → 改固件逻辑就动它
 *    · 硬件型号 = 后面的 HW_MODEL / HW_REV 宏   → 换板子/改电路才动
 *    改完把【本行】同步改掉。第 1 行与 FW_VER 不一致 = 改漏了。
 * 硬件：DOIT ESP32-32E N4 模组（ESP32-WROOM-32，Xtensa LX6 双核 240MHz，4MB Flash）
 *       + CH340 USB-UART（外部桥，非原生 USB）+ AMS1117-3.3 + SP3485 RS485 + 4×SRD-05VDC 继电器 + 4×PC817 光耦
 *       Type-C 供电，TPS54331 升压至 5V
 * 本固件把这块成品板的原厂 Modbus 从机功能，与 v4.x 的换水控制器逻辑合并：
 *   既做 4 路继电器 + 4 路光耦的主控，又保留 Modbus RTU 从机（地址1/115200/8N1）
 *   → 原来的 DAM 调试软件、RS485 上位机、HA 的 Modbus 接入全部继续可用，无需废弃
 *
 * 保留 v4.9 全部逻辑：单次执行 + 开机启动 + 定时计划(10条) + 执行对象 + 超时保护
 *      + 传感器冗余 + 手动互斥三选一 + 热点/家网自动切换 + MQTT 自动发现 + 记忆网页
 * 相比 v4.9 的变化：
 *   1) 平台从 ESP32-C3 SuperMini 换回经典 ESP32（引脚完全不同，重新映射）
 *   0) v5.1 新增人机交互：GPIO14 状态灯 + BOOT 键短按/长按
 *      v5.0 曾误把 GPIO2 当"红 LED"驱动，已改正（GPIO2 原理图无网络标注且是 strapping 脚）
 *   0) v5.2 重大修正：浮球与遥控改用板载光耦输入（GPIO25/26/27/33）。
 *      v5.0/v5.1 称"光耦会硬联动抢占继电器、必须悬空"是误判——
 *      该联动是原厂固件的软件行为，PIN1~PIN4 与继电器驱动网络 SWT1~SWT4 在原理图无任何交汇，
 *      刷本固件后联动随之消失。改用光耦获得电气隔离 + 每路状态灯。
 *      注意：光耦输入侧【无板上电源】（回路 = INx - 4.7K - PC817 LED - GND_IN），
 *            属无源输入，须由 JP2 引出的 5V 经外部开关构成回路，详见配套文档。
 *      新增 rcTask()：光耦#3=排水遥控、#4=上水遥控（按下沿触发、互斥保护）。
 *      修正 mbOptoBits() 位序 bug（v5.1 及之前 bit0~bit3 与说明书定义相反）。
 *   0) v5.3 输入用途调整：光耦#3(IN3) 由"排水遥控"改为【浸水传感器】，
 *      光耦#4(IN4) 改为【单键遥控】，以击数区分动作：1击排水/2击上水/3击换水/长按急停。
 *      浸水传感器为最高优先级安全信号：确认浸水后强制全关并【自锁】，
 *      传感器变干不会自动解除（避免无人值守时启停循环），须人工复位。
 *      新增 leakTask() / leakReset()；startRun() 与 man() 均加了锁定守卫。
 *      MQTT 新增漏水报警(moisture)与复位按钮；网页新增漏水保护卡片。
 *   2) 恢复 K4 备用输出（该板有 4 路继电器，v4.8 曾因自研 PCB 版面删除）
 *   3) 新增 Modbus RTU 从机（功能码 01/02/05/0F），使成品板的 RS485 能力不被烧写丢失
 *   4) 继电器极性以实测为准：光耦级反相，digitalWrite(LOW)=吸合（沿用 v4.7 实测结论）
 *
 * ---- 本板 GPIO 分配（依据原理图 ESP32_Relay_X4_Modbus_Shell_V1.1 实测网标）----
 *   继电器 #1~#4 : GPIO23 / GPIO5 / GPIO4 / GPIO13   （板载 SS8050 驱动 + SRD 线圈）
 *   光耦输入 #1~#4: GPIO25 / GPIO26 / GPIO27 / GPIO33 （PC817，4.7K限流，无源输入，低电平有效）
 *   RS485        : GPIO19=TX(GPIO32=DE) / GPIO18=RX，经 SP3485 → 排针 JP1/JP2
 *   浮球/遥控    : GPIO17 / GPIO21 / GPIO16（排针引出，带内部上拉，干触点直接接）
 *                  GPIO36 / GPIO39 作冗余备用浮球（仅输入无上拉，需外接 10K）
 *   板载 LED     : GPIO14 为唯一软件可控状态灯；继电器/光耦/串口各有硬件自驱指示灯
 *   板载按键     : BOOT(GPIO0，可作功能键) / EN(纯硬件复位，软件不可控)
 *   保留空出     : GPIO12/15（strapping）、GPIO34/35（仅输入无上拉）
 *
 * ---- 关键电气约束（换水应用必须遵守，否则会误动作）----
 *   a) 光耦输入与继电器【不直连】：IN1~IN4 → PC817 → GPIO25/26/27/33，
 *      与继电器驱动网络 SWT1~SWT4 无任何交汇点，二者只能经 MCU 相通（详见下方 v5.2 说明）。
 *      厂家说的"光耦输入可控制继电器"是【出厂固件的软件行为】，刷本固件后不复存在；
 *      水位判定完全由本固件逻辑决定，光耦只负责把开关量安全地送进 GPIO。
 *   b) 继电器为常开(NO)/常闭(NC)/公共(COM)干接点，与主控电气隔离，
 *      可直接串接 220V 交流水泵与电磁阀线圈，无需中间驱动。
 *   c) 本板继电器为低电平吸合，且上电/复位瞬间 GPIO 为高阻 →
 *      必须【先 pinMode(OUTPUT)，再写释放电平(HIGH)】；顺序反了引脚会停在默认低电平，
 *      导致上电四路全吸合（2026-10-20 实测踩过），见 setup()。
 *
 * 主题：esp-pump/floatA floatB leak relay1..4 boot t0..t5 progress ip diag cmd
 *       （/set 为控制；cmd 支持 start/drain/fill/leak_reset 等）
 */
#include <WiFi.h>
#include <WebServer.h>
#include <EEPROM.h>
#include <ESPmDNS.h>
#include <time.h>
#include <Update.h>        // v5.7 网页 OTA：把上传的 .bin 写进备用 APP 分区
#include <ArduinoOTA.h>    // v5.7 IDE 无线烧录：端口列表里出现 "WTCC at 192.168.x.x"
#include <HTTPClient.h>    // v5.10 URL 升级：从 http(s) 地址直接拉 .bin
#include <WiFiClientSecure.h>  // v5.10 https 下载（setInsecure，见 otaFromUrl 注释）
#define MQTT_MAX_PACKET_SIZE 1024
#include <PubSubClient.h>
#if MQTT_MAX_PACKET_SIZE < 1024
#error "MQTT_MAX_PACKET_SIZE must be >= 1024 (copy the #define line above #include <PubSubClient.h>)"
#endif

/* 版本号规则：每次改动必须动这里。
 *   FW_VER   —— 语义版本（软件），改动累计到一定程度进一位
 *   FW_BUILD —— 编译时间戳，由编译器自动填，**不用手改**
 * 页眉会同时显示两者（如 "v5.4 · Oct 20 2026 22:40"），
 * 所以"到底烧进去没有"看时间戳即可，不必再靠版本号猜。
 * 硬件型号另有 HW_MODEL / HW_REV（见文件前部「硬件身份」一节），不在这里。
 * ⚠ 改 FW_VER 时，把【文件第 1 行】的版本号一起改 —— 那行是给人一眼看版本用的。 */
#define FW_VER   "v5.21"
#define FW_BUILD __DATE__ " " __TIME__

/* ======== v5.9：网页改成页签切换（控制 / 设置 / 记录 / 系统）========
 * 起因：卡片从 8 张涨到 13 张，手机上要滚好几屏才能摸到「流程执行」那几个按钮。
 * 做法：顶部 sticky 页签，显隐由 body[data-tab] + 每张卡的 data-pane 决定，
 *   **不搬 DOM** —— 每页内部的卡片顺序仍按源码先后，加新卡片只要写对 data-pane。
 *   停留页存 localStorage，下次打开手机浏览器还停在原页。
 * 分组：控制 = 运行状态/漏水/流程/手动；设置 = 开机启动/水位判定/时间参数/定时计划；
 *       记录 = 最近动作/BOOT 键表；系统 = 网络/MQTT/固件升级。
 * 顺带两处：① BOOT 键功能表原来嵌在「网络设置」卡片里，拆成独立卡片挪到「记录」页；
 *   ② 漏水锁定后给「控制」页签打红点，否则用户停在别的页完全看不到警告。
 */

/* ======== v5.9 同时修掉一个藏了很久的 EEPROM 布局 bug ========
 * cfgT0/T1/T2/T4/T5 原来声明成 `unsigned int` —— ESP32 上这是 **4 字节**，
 * 而 save()/setup() 的地址是按 **2 字节**一格排的（0/2/5、221/223/225），于是互相踩：
 *     put(221,T0) 写 221~224 → 高半被 put(223,T4) 覆盖
 *     put(223,T4) 写 223~226 → 高半被 put(225,T5) 覆盖
 *     put(225,T5) 写 225~228 → 顺手盖掉 227 的 planCount
 * 读回来全是天文数字，被范围校验打回默认值。
 * 症状：**T4 改成 1500，页面显示 1500，一重启又变回 120**。
 * T0/T1/T2 同样中招，只是默认值 5/20/20 恰好常用，一直没人发现。
 * 修法：改 uint16_t（2 字节），地址表严格对齐；历史写入的低 2 字节仍然有效。
 * 详见 cfgT1 声明处的注释。⚠️ 修复后 T4 需要重新设一次（EEPROM 里已被写成 120）。
 */

/* ======== v5.14：网页交互三项微调 ========
 * 1) 「停止/待机」文案统一：之前空闲时 phase 显示「停止」（manSt 默认值）、progress 显示「待机」，
 *    两个词表达同一个「无动作」状态，语义重复。现将 manSt 默认值与手动全关统一为「待机」，
 *    phase 与 progress 在空闲时都显示「待机」；GAP（换水流程里的排水-上水间隙）仍保留「停止」，
 *    因为那是流程中的一个真实阶段（progress 显示「排水停止中」）。
 * 2) 水位判定卡片的两段说明（极性说明 + 3 秒宽限说明）从常驻 tip 改到悬浮层：标题栏加「?」信息按钮，
 *    点开才显示，避免卡片被长文字撑高。
 * 3) WiFi「保存并连接/断开」、MQTT「保存并启用/停用/重发配置」的按钮合并到同一行。
 *    ⚠ 上面第 1 条的写法有后遗症，v5.15 已改：把两个位置都改成「待机」后，
 *      首页变成同一个词上下写两遍，仍是重复。详见 v5.15 说明。
 */

/* ======== v5.15：彻底消除状态文案重复 + 去掉重复的版本行 ========
 * 1) progressText() 的 default 分支不再复述 manSt（phase 大字已经在显示了）。
 *    空闲 → 「当前没有任务」；手动正转/反转/阀开 → 「手动…中」；
 *    漏水锁定 → 「已强制关闭全部输出」；已到位未启动 → 「信号已到位，请检查接线或极性」。
 *    这样 phase 说「是什么状态」，progress 说「补充信息」，分工不再重叠。
 * 2) 删掉状态卡片底部那行「固件 vX · 编译 …」（#meta）—— 系统页「固件升级」卡片的
 *    「当前版本」（#fwCur）已经在显示同一串信息。漏水状态仍由 #leakCard + 页签红点负责。
 * 3) 水位判定的术语从「通电 / 断电」改成「通路 / 断开」——「通电」容易被读成"给板子供电"，
 *    而这里说的是信号端子有没有电压。改动覆盖：状态行（当前/规则）、切换 toast、
 *    悬浮层说明、事件日志。⚠ 不覆盖「备用 通电」（那是 K4 继电器线圈得电）
 *    和「断电重启」（那是设备供电），两者语境不同。
 * 4) 运行来源可见：新增全局 runSrc，记录本次运行/手动操作是谁发起的
 *    （WEB 网页 / MQTT / BTN 按钮 / RC 遥控 / TIME 定时 / BOOT 上电 / MB Modbus）。
 *    /state 增 "src" 字段，网页把它拼进 progress 小字。
 *    待机/结束时不显示（JS 用 active 判断），避免残留上一轮的来源。
 *    ⚠ evSrc 只在一瞬间有效（mqttCb 里临时切走又切回），所以必须单独留一份。
 * 5) 流程名后缀收窄：progress 后面那个「· 排水/上水/换水」原来无差别拼，
 *    导致「排水开始 · 排水」这种同词重复，手动时还会带出上一次流程的残留名字
 *    （如「手动正转中 · 换水」）。现改为【只在自动流程 + 换水时】显示：
 *    排水/上水流程 progress 本身已含「排水…」「上水…」；换水流程才需要标出整体任务，
 *    否则「排水开始」「上水开始」看不出是同一轮换水的哪一段。
 *    /state 为此新增 "auto" 字段（1=自动流程在跑，0=待机或手动）。
 */

/* ======== v5.16：状态行的读取顺序改成「来源 → 流程 → 阶段」========
 * 用户要求：流程唤起时流程名要排在前面，形如「MQTT.换水.泵反转」「网页.泵正转」。
 * 原来 v5.15 是「阶段 · 换水 · 来源」，从里往外读，眼睛得倒着扫；改成从外往里：
 *   换水流程  MQTT.换水.排水开始     ← 谁发起、哪条流程、走到哪一步
 *   排水/上水 网页.排水开始          ← 阶段名已含「排水」，不再补流程名
 *   手动操作  网页.手动正转中        ← 没有流程，只有来源和动作
 *   待机/结束 当前没有任务           ← 无来源无流程，保持原样
 * 分隔符用英文点号（用户指定的写法），三段之间无空格，一行放得下。
 * 同时把 /state 的 "auto" 收紧：原来只判 st != IDLE，导致 END（流程刚结束）
 *   和漏水锁定（强制全关）也被当成"自动流程在跑"，会挂出「网页.换水.已强制关闭全部输出」
 *   这种胡说。现改为 st != IDLE && st != END && !leakLatch。
 */

/* ======== v5.17：流程名挪到大字上，小字只留「来源.设备状态」========
 * 用户要求：「后面跟的是设备状态，泵、阀的状态，不是流程状态」，
 *   接着又定「可以把流程名放在大字上」。于是两行彻底分开：
 *     大字(26px) 换水 / 排水 / 上水 / 手动 / 待机 / 漏水保护   ← 在干什么
 *     小字(13px) MQTT.泵反转 / 网页.泵正转 / 当前没有任务       ← 谁发起的 + 泵阀什么状态
 * 好处：设备状态（泵反转）只在 progress 出现一次，不再和 stName() 的大字撞车；
 *   阶段细节（排水开始/上水开始）从状态卡退出，要查去看「最近动作」事件日志。
 * /state 为此新增 dev（泵阀物理状态）、tmo（超时标志）、ended（刚跑完），
 *   phase 改由新的 runPhase() 产出；stName() 保留给串口诊断用。
 */

/* ======== v5.18：大字改成「换水中」，小字写成「设备+动作+原因」========
 * 用户定稿：
 *   大字  换水中 / 排水中 / 上水中 / 换水完成 / 手动 / 待机 / 漏水保护
 *   小字  排水泵反转（下水位触发/超时）· MQTT      ← 阶段文案 + 末尾挂来源
 * 关键修正（按状态机核对过）：
 *   1) 「排水泵正转停止」这个状态不存在 —— 下水位到位或 T4 超时后是【直接切反转】，
 *      pumpOut(0,1,0)，泵一刻没停。所以文案挂在「反转」上，不是「正转停止」。
 *   2) 「排水结束(T2)」只在换水流程（obj=0）有 —— 纯排水流程反转完直接 END。
 *   3) 括号里的原因保留并列写法（用户要的），真超时由卡片转橙 + 事件日志区分。
 * 实现：阶段文案统一由 progressText() 产出（网页和 MQTT esp-pump/progress 共用一份），
 *   网页只负责在末尾补「 · 来源」，不再自己拼设备名。
 */

/* ======== v5.19：手动关闭设备后给回执，不再一律「当前没有任务」========
 * 用户反馈：手动开阀再关阀，状态卡显示「当前没有任务」，看不出到底关没关。
 * 以前 man(0,0,0) 一律把 manSt 写成「待机」，网页只能显示「当前没有任务」；
 * 而且关阀、关泵、关备用三种操作在界面上长得一模一样。
 * 现在在 pumpOut 之前先读实际输出，据此写回执：
 *   阀开着 → 「阀关闭」  泵转着 → 「泵停止」  阀泵都开 → 「全部停止」
 *   本来就没开 → 仍「待机」，不硬造回执
 * manAux(false) 同理，别的都没开时写「备用关闭」。
 * 对应地：
 *   · runPhase() 把这些回执也归到「手动」大字，否则大字会先掉回「待机」
 *   · 网页的 showSrc 加上 `phase=='手动'`，回执才挂得上「 · 网页」
 *   · 流程跑完回 IDLE 时把 manSt/runSrc 复位，免得流程结束后还挂着上一轮回执
 */

/* ======== v5.21：给板子一个固定的硬件型号 ========
 * 需求：软件版本号（FW_VER）只说明固件是哪一版，说明不了跑在哪块板子上。
 *   以后可能同时有几块外观相近的板子（换主控、换继电器、加路数），
 *   光看固件版本分不出来，所以要一个硬件型号。
 * 定义（文件前部「硬件身份」一节）：
 *   HW_MODEL "WTCC-ESP32-RLY4"   ← 板型：WTCC 项目 + ESP32 主控 + 4 路继电器
 *   HW_REV   "V1"                ← 硬件改版，动一次电路进一位
 *   WTCC_BOARD_TAG = HW_MODEL "-" HW_REV  ← OTA 校验串，拼出来仍是 "WTCC-ESP32-RLY4-V1"
 * ⚠ 拼接值必须与旧版逐字相同，否则手头已导出的旧 .bin 会被拒绝写入。
 * 可见位置：
 *   · 文件第 1 行（人一眼看版本）
 *   · 串口启动横幅 [BOOT] vX build=… hw=WTCC-ESP32-RLY4-V1
 *   · 网页「系统」页固件升级卡片「硬件型号」一行（/state 的 hwm / hwr 字段）
 *   · HA 设备页（MQTT discovery 的 device.model / hw_version / sw_version）
 * 规则：改固件逻辑只动 FW_VER；只有换板子 / 改电路才动 HW_MODEL 或 HW_REV。
 */

/* ======== v5.20：给 HA 的结果信号（done / last）========
 * 起因：用户 HA 自动化里有一步「等待换水完成信号 · 60 分钟超时」，等的实体
 *   `huan_shui_complete` 在固件里根本不存在 —— 固件只有 sensor.progress 的中文文本。
 *   拿文本做判断非常脆：光今天就改了三轮措辞（排水开始 → 排水泵正转 → 排水结束（20 秒））。
 * 新增两个：
 *   binary_sensor 流程完成  ← esp-pump/done   ON=最近一次流程正常跑完（st==END）
 *   sensor        上次结果  ← esp-pump/last   「换水完成」/「换水被拒：水位已到位」/「上水超时（T5）」
 * done 是【锁存】的：跑完置 ON 并一直保持，直到下次启动、手动接管才清 OFF。
 *   不这么做的话，HA 只有 END 那 5 秒窗口能抓到 ON，网络稍慢就漏了。
 * 启动被拒（漏水锁定 / 水位已到位）也会写进 last，自动化可以立刻发现没跑起来，
 *   不用干等 60 分钟超时。
 */

/* ======== v5.9 水位输入卡片重做：两排独立开关 ========
 * 用户要求：第一排「排水中止信号（IN1）」、第二排「上水中止信号（IN2）」，
 *   每排右边一个拨动开关，默认「通电时停止」，可拨成「断电时停止」，开关左侧显示当前信号。
 * 于是把 IN1 也从编译期宏 SEN_A_TRIG_LOW 升级成运行期变量 senAPol（EEPROM 309），
 *   与 IN2 的 senBPol 完全对称，状态机与 MQTT floatA 都改用变量。
 * 措辞统一成「通电时停止 / 断电时停止」，不再用「有信号=到位」那套绕口的说法。
 */

/* ======== v5.12：升级只校验硬件，不校验软件版本号 ========
 * 需求：允许刷任何版本的固件（升级 / 降级 / 同版本重刷都放行），但必须挡住刷错芯片的包。
 * 可用的硬件标识只有镜像头（esp_image_header_t，前 24 字节）：
 *   offset 0      magic   必须是 0xE9
 *   offset 12~13  chip_id 小端 uint16，0x0000=ESP32
 * esp_app_desc_t（文件 offset 0x20 起）里的 version 字段【不比较】—— 这就是"不校验软件版本号"。
 * 另外：app_desc 的 project_name 实测是 "arduino-lib-builder" 而非工程名，不能当身份判据。
 * 两条通道都改成「先校验后写」：上传通道在 UPLOAD_FILE_WRITE 攒够 14 字节再 Update.begin()；
 *   URL 通道先读 14 字节校验，通过后 Update.write() 头部再继续流式下载。
 * 校验不通过就一个字节都不落 flash，原因原样回网页 + 写事件日志。
 */

/* ======== v5.11：两种极性的「保持」语义分开处理 ========
 * 起因：允许「通电时停止 / 断电时停止」之后，两种极性的语义其实是相反的 ——
 *   pol=1 通电时停止：到位 = 信号出现。浮球一旦闭合就停在那儿，是**边沿**事件，
 *                    所以"信号闪一下"最多误停一次，出现即停是对的。
 *   pol=0 断电时停止：到位 = 信号消失。反过来，阀/泵能工作是因为**信号持续存在**，
 *                    信号是"许可证"。而"持续存在"这个条件天然会波动（接触不良、
 *                    水面晃、线松一下），旧逻辑一丢信号就 st=END，一次抖动废掉整轮换水。
 * 改法一（宽限）：pol=0 时判定到"到位"不立刻停，连续丢失超过 HOLD_GRACE_MS(3 秒)
 *   才确认；期间信号恢复就撤销宽限、流程继续。FWD 与 VALVE_ON 都套用，共用一个 holdOk()。
 *   pol=1 完全不走宽限，行为与 v5.10 一致（holdOk 直接返回 true）。
 * 改法二（启动预检）：启动瞬间若对应中止信号已处于"到位"，不再静默跳转，而是拒绝启动
 *   并在事件日志/manSt 里写清是哪一路。这直接兜住一个真实陷阱 ——
 *   某路输入没接线时光耦输出侧上拉，GPIO 恒读 HIGH，此时把该路拨成"断电时停止"
 *   就等于"永久到位"，现象是"点一下就说完成了"，看起来像程序 bug，实际是接线/极性。
 *   注：换水/排水启动时若下水位已到位也会被拒（此时该用「上水」而不是「换水」）。
 */

/* ======== v5.10：固件升级支持填网址 ========
 * 除了网页选 .bin 上传，再开一条 GET /update_url?url=... ，板子自己去下载并写入。
 * 适合把 .bin 丢在内网服务器/NAS/图床上，手机端不用每次找文件。
 * 实现：HTTPClient 流式下载 → Update.write() 分块写 flash，1024 字节一包，边下边写，
 *   不占大块 RAM（整包缓冲会 OOM）。循环里 delay(1) 喂狗，3 分钟没下完就放弃。
 * https 走 WiFiClientSecure + setInsecure()：跳过证书校验，家庭内网够用，
 *   但不可信网络上等于门户大开（中间人能换固件），tip 里已写明。
 * 同步执行：下载期间 WebServer 被占住，/state 轮询会超时，属正常，页面已先停轮询。
 */

/* ======== v5.8：IN2 判定极性改为运行期可配 ========
 * 起因：同一颗浮球翻转 180° 安装后，磁簧与磁铁的相对位置就反了 ——
 *   「下位（下垂）」变成闭合、「上位（上浮）」变成断开，与固件原本的假设
 *   （常开 NO：水满才闭合）正好相反。现象是上水一开阀就收到「已到位」，0.3 秒内关阀。
 * 做法：把编译期宏 SEN_B_TRIG_LOW 升级成运行期变量 senBPol，存 EEPROM(308)，掉电保留；
 *   网页新增「水位输入判定」卡片，一个开关切换「有信号=到位 / 无信号=到位」，
 *   并同时显示 IN2 的【原始信号】和换算后的【判定结果】——现场不用万用表也能定性。
 *   实测值会随 /state 的 in1raw / in2raw / in2pol 一起下发。
 * 注：IN1 下水位仍走编译期宏 SEN_A_TRIG_LOW，需要同样可配再另说。
 */

/* ======== v5.3k：网页交互两处硬伤修复 ========
 * 1) 【点了像没反应】T0~T5 的 +/− 走 /cmd 修改成功后，页面数字要等下一次 5 秒轮询
 *    才刷新。人手点一下、5 秒内看不到变化，就会判定"改不动"。
 *    修法：cmd() 收到响应后立刻 loadCfg()，数字当场就变。
 * 2) 【页面看不到时间】原界面根本没有当前时间，定时计划却依赖系统时钟，
 *    用户无法确认板子时钟对不对。修法：/state 增加 now 字段，页面常显当前时间；
 *    NTP 未同步时明确显示"未同步"，不再让人猜。
 * 另：ioSnap 增加 trig 字段，便于串口确认"上电自动运行"开关到底开没开。
 */

/* ======== v5.3i：串口命令行 + 全链路诊断（只为排障，不改控制逻辑）========
 * 起因：用户反馈"网页上时间参数改不动、按正转/反转继电器没声音"。
 *   两个症状同源，可能是三层中任意一层：(a) 浏览器没发出 /cmd；(b) 固件收到但被
 *   leakLatch(漏水锁定) 拒绝；(c) 固件执行了但硬件没吸合。
 *   仅靠网页界面无法区分这三层 —— 故加两条独立观察通道：
 *   1) 【串口命令行】在串口直接敲 fwd / rev / stop / t0up … 即等价于网页点按钮，
 *      完全绕开 HTTP 与浏览器，可独立验证"固件+硬件"这一层。
 *   2) 【命令回显】HTTP 每次 /cmd 都往串口打一行 [HTTP]，
 *      用户点网页按钮时看串口有没有这行，即可判断请求到底有没有到固件。
 */
#define SERIAL_CLI 1

/* ======== 热点保活宽限期（v5.3h 新增）========
 * 现象：配好家网后，v5.3f/g 的"连上家网就关热点"逻辑会在开机约 5 秒后执行
 *   softAPdisconnect()，热点 FS_TWCC 随即从手机列表消失。
 *   此时板子其实在家网里正常干活，但用户既搜不到热点、也不知道家网 IP
 *   —— 表现为"热点凭空没了"。
 * 对策：开机后 AP_GRACE_MS 这段时间内，无论 STA 是否已连上家网，热点一律保留。
 *   这样刚上电 / 刚烧完固件一定能刷到 FS_TWCC 并进 192.168.4.1；
 *   宽限期过后才恢复"连上家网即关热点"的省资源策略。
 * 想让热点永久常驻：把 AP_GRACE_MS 改成 0xFFFFFFFF 即可。
 */
#define AP_GRACE_MS 1800000UL    // 30 分钟（排障期临时放宽，便于直接连热点实测 HTTP）

// ==== v5.3m 诊断用全局量（必须声明在所有使用它的函数之前）====
unsigned long httpCmdN = 0, httpStateN = 0, httpCfgN = 0, httpRootN = 0;
unsigned long loopLastMs = 0, loopMaxGap = 0;
unsigned long lastHttpMs = 0;
char          lastHttpCmd[28] = "";
unsigned long selfOkN = 0, selfFailN = 0, selfRtt = 0;
unsigned long lastDiag = 0;
unsigned long mqttBlkMs = 0;   // MQTT connect() 对 loop 的最大阻塞时长
/* ---- ESP32-32E 引脚映射（经典 ESP32，与 C3 完全不兼容）---- */
#define RLY1 23        // 继电器#1 泵正转（驱动级反相：LOW=吸合）
#define RLY2 5         // 继电器#2 泵反转
#define RLY3 4         // 继电器#3 电磁阀
#define RLY4 13        // 继电器#4 备用（v5.0 恢复）

/* ======== 继电器吸合电平 ========
 * 1 = GPIO 输出 LOW 时吸合（本板实测结论）
 * 0 = GPIO 输出 HIGH 时吸合
 *
 * 【2026-10-20 结论修正 —— 之前把方向搞反了】
 * 一度误判为「高电平吸合」，改宏后四路依旧全亮，说明问题不在极性，而在初始化顺序：
 * 原 setup() 把 digitalWrite 写在 pinMode(OUTPUT) 之前，ESP32 在引脚仍是 INPUT 时
 * 写的电平不会保持，切到 OUTPUT 后引脚停在默认低电平；本板是低电平吸合，
 * 于是无论宏取 0 还是 1，上电都是四路全吸合。
 * 现在 setup() 已改为「先 pinMode(OUTPUT)，再写释放电平」，宏恢复为 1。
 * 若日后换板真遇到高电平吸合的型号，把这里改 0 即可，其余代码不用动。
 */
#define RLY_ACTIVE_LOW 1

/* ======== TKS-M8 单键电平泵模式（v5.3n 新增）========
 * 场景：外挂通奇思 TKS-M8 正反转模块，且为「P1-M3 单键电平型」——
 *   K1 有信号=正转、无信号=反转、K2 未用。这种模块一旦得电就永远在转
 *   （不是正转就是反转），所以停机必须靠【切断模块供电】实现。
 *
 * 继电器分工（按实际接线定死，勿改）：
 *   #1(RLY1) = 供电开关，串在 TKS-M8 的 DC+ 供电回路上（吸合=得电）
 *   #2(RLY2) = K1 信号，接 TKS-M8 的 K1 脚（吸合=K1有信号=正转）
 *
 * 状态翻译（pumpOut）：
 *   正转 = 供电吸合 + 信号吸合  → #1=1 #2=1
 *   反转 = 供电吸合 + 信号断开  → #1=1 #2=0
 *   停机 = 供电断开            → #1=0 #2=0
 *
 * TKS_SINGLE_KEY = 1 时启用本模式；0 时保持旧的双键逻辑（#1=正转/#2=反转）。
 * 界面/网页/MQTT 语义完全不动，只改底层继电器的落点。
 */
#define TKS_SINGLE_KEY 1
#define OPT1 25       // 光耦输入 #1 → 下水位浮球
#define OPT2 26       // 光耦输入 #2 → 上水位浮球
#define LEAK_IN 27     // 光耦输入 #3 → 浸水传感器（漏水/溢水检测，安全信号）
#define RC_IN  33      // 光耦输入 #4 → 单键遥控（1击排水 / 2击上水 / 3击换水 / 长按急停）
#define SEN_A OPT1    // 下水位
#define SEN_B OPT2    // 上水位

/* ============================================================
 *  【输入触发逻辑总配置】IN1~IN4 四路光耦输入，怎么算「触发」
 * ============================================================
 * 硬件背景（务必先读懂，否则下面的 0/1 会改反）：
 *   四路光耦输入都配置为 INPUT_PULLUP（GPIO 内部上拉）。因此：
 *     · 光耦【不导通】→ 输出侧被上拉 → 固件读到 HIGH(1)
 *     · 光耦【导通】  → 输出侧被拉低 → 固件读到 LOW(0)
 *   所以「读到 LOW」=「光耦有电流导通」，「读到 HIGH」=「光耦断开」。
 *
 * 那「短接 INx+GND 到底能不能触发」？
 *   取决于你板子光耦输入是哪种走法（固件注释 v5.2 已说明）：
 *     A 法（自带偏置/上拉）：短接 INx 与 GND_IN 即有电流 → 光耦导通 → 读 LOW
 *     B 法（无源，需外部供电）：必须【5V → 传感器 → INx】才有电流；
 *        单纯短接 INx+GND_IN 无电流，光耦不导通。
 *   这是硬件差异，固件改不了；请先用万用表/网页「输入状态」实测你的板子是 A 还是 B。
 *
 * 下面每个 *_TRIG_LOW 决定「哪种电平算触发」：
 *   = 1  → 读 LOW（光耦导通）算触发   ← 常开 NO 浮球、干接点闭合型传感器
 *   = 0  → 读 HIGH（光耦断开）算触发  ← 常闭 NC 浮球、反向输出 DO 的模块
 *
 * ⚠️ 安全提醒：IN3 是【浸水漏水保护】，极性改反会直接失去漏水保护！
 *   改完任何一路极性，务必【实测验证】：短接/断开该路，看网页「下水位/上水位/
 *   浸水」是否按预期变化，确认无误再投入使用。尤其是 IN3，宁可多测一遍。
 * ------------------------------------------------------------
 */
#define SEN_A_TRIG_LOW 1   // IN1 排水中止【仅作默认值】。v5.9 起与 IN2 一样改为运行期可配，
                           // 网页「水位输入判定」里拨开关即可，掉电存 EEPROM(309)。
#define SEN_B_TRIG_LOW 1   // IN2 上水位【仅作默认值】。v5.8 起改为运行期可配：
                           // 网页「水位输入判定」卡片里随时切换，存 EEPROM 掉电保留。
                           // 本宏只在 EEPROM 里没有有效值时兜底，改它需要重新编译。
                           // 判定口诀：把浮球摆到【水满】姿态，网页应显示「到位」；反了就切换。
#define SEN_CONFIRM_MS 30 // 到位确认时间（消抖）。机械浮球 30ms 够；
                          // 电容式贴壁探头容易受水流/挂珠影响，建议设 300~1000。
#define HOLD_GRACE_MS 3000 // v5.11：断电时停止（pol=0）的「信号丢失宽限」。
                          // pol=0 时信号是"许可证"——阀/泵靠"信号持续存在"才工作，
                          // 而这个条件天然会波动（接触不良、水面晃、线松一下）。
                          // 所以判定到"到位"后不立刻停，连续丢失超过本时长才确认；
                          // 期间信号恢复就取消宽限、继续跑。
                          // pol=1（通电时停止）不走宽限：那是"出现即到位"的边沿语义。
                          // 改大 → 更能扛抖动，但真到位后泵/阀会多跑这么久，注意别干转。
#define USE_REDUNDANT 0   // 是否启用"备用/冗余"水位输入（GPIO36/39）。
                          // 强烈建议保持 0！GPIO34~39 是仅输入脚且【没有内部上拉】，
                          // 不接线时悬空电平随机漂移，一旦漂成低电平会被误判为"水位到达"，
                          // 导致排水/上水流程提前中止。要用冗余必须先给这两脚外接 10K 上拉，再置 1。
                          // 更省事的做法：把 RED_A/RED_B 改成排针上带内部上拉的空闲脚（如 GPIO17/21），再置 1。
#define RED_A 36      // 备用下水位（仅在 USE_REDUNDANT=1 时参与判断）
#define RED_B 39      // 备用上水位
#define REMOTE_IN 16  // 排针备用遥控（干触点闭合沿触发换水，与光耦#3/#4 二选一即可）

/* ======== 硬件身份（v5.21）========
 * FW_VER 只说明"固件是哪一版"，说明不了"跑在哪块板子上"。
 * 以后可能同时存在几块外观相近的板子（换主控、换继电器、加路数），
 * 光看固件版本分不出来，所以给板子一个固定型号。
 *   HW_MODEL —— 板型，刻丝印 / 写进说明书，不随改版变
 *   HW_REV   —— 硬件改版，动一次电路进一位（V1 → V2）
 *   WTCC_BOARD_TAG —— 两者拼接，OTA 校验用：只接受带同样标记的 .bin
 * ⚠ 拼接结果必须保持 "WTCC-ESP32-RLY4-V1" 逐字不变 —— 手头已导出的旧 bin
 *   带的是这个标记，改字面值会让它们被拒绝写入。
 * ⚠ 换板子时改这两行再重编：旧板固件就写不进新板（这正是目的），反之亦然。
 * ⚠ 定义必须在文件前部：宏没有前置声明兜底，/state 的 btag 字段会先引用它。
 *   详细说明见 otaVerifyImage 之后那段 v5.13 注释。 */
#define HW_MODEL "WTCC-ESP32-RLY4"
#define HW_REV   "V1"
#define WTCC_BOARD_TAG HW_MODEL "-" HW_REV
#define OTA_TAG_MAX 64

/* ======== 为什么浮球与遥控改用光耦输入（v5.2 重要修正）========
 * v5.0/v5.1 曾建议"光耦必须悬空、浮球改走 GPIO17/21（排针）"，理由是说明书第四节
 * 称"光耦输入会硬联动抢占继电器"。该建议建立在误判之上，现更正：
 *
 *   那条联动是【出厂固件的软件行为】，不是硬件连线。
 *   原理图网络名追踪证据：
 *     IN1~IN4（光耦输入端 JP2）→ IC7/IC8(PC817) → 网络名 PIN1~PIN4
 *     PIN1~PIN4 在全图只出现于两处：光耦输出侧、U1 的 GPIO25/26/27/33
 *     继电器驱动网络 SWT1~SWT4 与之【无任何交汇点】，二者只能经由 MCU 相通
 *   ⇒ 物理上光耦输出只能进 GPIO，绝无可能绕过 MCU 直达继电器线圈。
 *   ⇒ 烧录本固件后，原厂那段联动代码已被完全替换，联动随之消失。
 *
 *   注：若日后刷回厂家原固件，联动会立刻复活——届时浮球/遥控接在光耦上的行为会变。
 *
 * 改用光耦的收益（远优于直连 GPIO 排针）：
 *   1) 光电隔离：浮球线常拉数米至十几米，隔离输入抗干扰、防地环路的能力远超直连
 *   2) 输入侧串 4.7K 限流（非上拉），且回路内无板上电源：
 *      干触点须一端接 JP2 的 5V、另一端接 INx，并先把 GND 与 GND_IN 短接
 *   3) 每路有独立指示灯（LED7~LED10），现场排查免费可视化
 *   4) JP2 引出 5V / GND / GND_IN / IN1~IN4：IN1~IN4 共用 GND_IN 作公共端，
 *      并非"4 路 × 2 脚各自独立"；接线前请实测是 A 法(自带偏置) 还是 B 法(需外部供电)
 *   5) 释放 GPIO17/21/16 等排针脚备用
 * ============================================================ */
#define MB_DE 32       // RS485 收发方向：HIGH=发送（原理图网络名 RE，RE#/DE 短接同控）
#define MB_TX 19       // UART1 TX → SP3485 DI
#define MB_RX 18       // UART1 RX ← SP3485 RO
/* 系统状态指示灯：全板**唯一**一颗软件可控的 LED
 * 原理图证据：U1 第 13 脚(IO14) 的网络名即为 "LED2_IO14"，确认 GPIO14 驱动一颗 LED。
 *
 * 其余 LED 都是硬件自动的，与本固件无关、也无需任何代码：
 *   LED1/LED2/LED5/LED6 四颗红灯，分别并联在继电器 #1~#4 线圈两端，线圈通电即亮。
 *   即便固件跑飞、看门狗复位、WiFi 断开，它们仍如实反映继电器是否真的在通电
 *   ——这是现场排查故障时比网页状态更可信的依据。
 *   LED3/4/7/8/9/10 归属 USB/电源/光耦区域，同样硬件自驱。
 *
 * 注：曾误把 GPIO2 当作"红 LED"驱动，已在 v5.0 修正移除。
 *   GPIO2 在原理图上无任何网络标注（IO2/IO15 整图未出现网络名，大概率悬空），
 *   且它是 strapping 脚（上电须为低），在固件里输出高低电平有启动模式风险。
 */
#define LED_STATUS 14
#define LED_ACTIVE_HIGH 1   // 点亮电平：1=高电平点亮，0=低电平点亮。
                            // 实物极性未知（LED 经限流电阻接 VCC 还是 GND 未知）。
                            // 若发现"该亮时不亮 / 该灭时常亮"，把此值改成 0 重烧即可。
#define MAX_PLAN 10

/* ======== 浸水传感器（IN3 / GPIO27）参数 ========
 * 触发逻辑与 IN1/IN2 相同：光耦 INPUT_PULLUP，读 LOW=光耦导通、读 HIGH=断开。
 * LEAK_TRIG_LOW = 1：光耦【导通】(读到 LOW) 判为"浸水"（绝大多数浸水传感器/干接点报警器如此）
 * LEAK_TRIG_LOW = 0：光耦【断开】(读到 HIGH) 判为"浸水"（少数模块 DO 反向输出，改此值）
 *   注意「短接 IN3+GND 能否触发」同样取决于板子是 A 法(自带偏置)还是 B 法(需 5V→IN3)，
 *   与 IN1/IN2 一致，见「输入触发逻辑总配置」。
 * LEAK_CONFIRM_MS：连续浸水多久才认定。防溅水、结露、抖动误报，默认 1.5 秒。
 *
 * 触发后的行为是【自锁】：强制全关输出 + 禁止一切启动，且传感器变干也不会自动解除。
 * 为什么要自锁：若水干了就自动恢复，现场无人时会出现"启停反复横跳"，
 * 泵在漏水-恢复-再漏水之间循环，比一直停着危险得多。必须人工确认后复位。
 */
#define LEAK_TRIG_LOW    1
#define LEAK_CONFIRM_MS  1500UL

/* ======== 单键遥控（IN4 / GPIO33）参数 ========
 * 触发逻辑与 IN1~IN3 不同：遥控是【沿触发】，不是电平触发——
 *   固件认「读 LOW = 按下、读 HIGH = 松开」，在按下/松开【边沿】上计数，
 *   因此干触点按钮（点动/瞬态，按住通、松开断）天然适配，无需极性开关。
 *   若用 433MHz 接收板，务必把接收板设成【点动/瞬态】模式，别设自锁，
 *   否则一次按下被持续读 LOW，会误判成长按急停。
 * 击数判定：松开后若 RC_WIN_MS 内没有下一次按下，就按累计击数执行。
 * RC_LONG_MS：按住超过这个时长直接判为长按（急停 + 解除漏水锁定），不再计击数。
 */
/* 漏水锁定时是否连 K4「备用输出」一起禁止：
 *   1 = 一起锁（推荐，"浸水 = 停止所有动作"，与触发瞬间全关 K1~K4 的行为一致）
 *   0 = K4 不受漏水影响（仅当 K4 接的是与漏水完全无关的负载时才设 0）
 */
#define LEAK_LOCK_AUX 1

#define RC_WIN_MS    600UL    // 连击窗口：两次按下的间隔上限
#define RC_LONG_MS  2500UL    // 长按阈值

/* 板载按键：原理图 x820-910 区域 SW1=EN(10K上拉+1uF RC，纯硬件复位，软件不可控)
 *                            SW2=BOOT → GPIO0(10K 上拉 R6)
 *   EN  ：按下即硬件复位整块板，不经 MCU，固件无法拦截
 *   BOOT：GPIO0，运行时可作功能键；启动时按住会进下载模式（正常，不影响使用） */
#define BTN_BOOT 0

/* Modbus RTU 从站参数（必须与原厂一致，否则上位机连不上） */
#define MB_SLAVE_ID 1
#define MB_BAUD 115200
HardwareSerial mbSerial(1);

/* ⚠️ 这五个必须是 uint16_t，**不能写 unsigned int**。
 * ESP32 上 sizeof(unsigned int) == 4，而 save()/setup() 里的 EEPROM 布局是按 2 字节一格排的
 * （0 / 2 / 5、221 / 223 / 225），用 4 字节类型会互相踩：
 *     put(0,   T1) 写 0~3    → 高半被 put(2, T2) 覆盖
 *     put(2,   T2) 写 2~5    → 高半被 put(5, wifiMode) 覆盖
 *     put(221, T0) 写 221~224 → 高半被 put(223, T4) 覆盖
 *     put(223, T4) 写 223~226 → 高半被 put(225, T5) 覆盖
 *     put(225, T5) 写 225~228 → 顺手盖掉 227 的 planCount
 * 读回来就成了天文数字，被下面的范围校验打回默认值 —— 现象正是
 * 「明明改成 1500，保存成功、页面上也显示 1500，一重启又变回 120」。
 * T0/T1/T2 也有同样的毛病，只是它们默认值 5/20/20 恰好是常用的，一直没人发现。
 * v5.9 修正为 uint16_t（2 字节），布局与地址表严格对齐，历史写入的低 2 字节依然有效。 */
uint16_t cfgT1 = 20, cfgT2 = 20, cfgT0 = 5, cfgT4 = 120, cfgT5 = 120;
byte trig = 1;                    // 开机启动：0=是(上电自动执行1次) 1=否(上电待机)，默认关=否
byte objGlobal = 0;               // 0换水 1排水 2上水（上电/遥控默认对象）
/* v5.9：两路水位的「中止信号」判定极性，运行期可配（网页「水位输入判定」）。
 *   1 = 通电时停止（光耦导通 / GPIO 读 LOW  → 常开 NO 浮球：水满才闭合）
 *   0 = 断电时停止（光耦断开 / GPIO 读 HIGH → 常闭 NC 浮球：水低就闭合）
 * 之所以从宏改成变量：同一款浮球换个安装方向（翻转 180°）磁簧与磁铁的相对位置就反了，
 * 现场没法改代码重烧，只能靠网页拨。掉电存 EEPROM：IN1→309，IN2→308。
 * 对应关系：IN1 管排水（泵正转什么时候停），IN2 管上水（阀什么时候关）。 */
byte senAPol = SEN_A_TRIG_LOW;   // IN1 排水中止信号
byte senBPol = SEN_B_TRIG_LOW;   // IN2 上水中止信号
byte planCount = 0;
byte wifiMode = 0;
byte mqttEn = 0;
uint16_t mqttPort = 1883;
char ssid[33] = { 0 };
char pass[65] = { 0 };
char mqttHost[16] = { 0 };
char mqttUser[32] = { 0 };
char mqttPass[64] = { 0 };

// 计划结构：en, type(0周 1隔N天), param(周位图 或 N), hour, min, obj, refDay(2B)
byte planEn[MAX_PLAN], planType[MAX_PLAN], planParam[MAX_PLAN], planHour[MAX_PLAN], planMin[MAX_PLAN], planObj[MAX_PLAN];
uint16_t planRef[MAX_PLAN];

enum { IDLE, FWD, REV, T4REV, GAP, VALVE_ON, T5TO, END } st = IDLE;
bool leakNow  = false;   // 浸水传感器实时状态（true=浸水）
bool leakLatch = false;  // 漏水保护自锁：true 时禁止一切启动，必须人工复位
const char* manSt = "待机";       // 手动状态显示（空闲/无动作，与 stName 的待机统一，不再用「停止」）
byte runObj = 0;                  // 当前执行对象
bool bootRun = false;             // 上电运行是否已执行
bool lastRemote = HIGH;           // 外接遥控输入上次电平（沿检测）
long lastTrigMin = -1;            // 定时触发分钟去重
bool timeSynced = false;
unsigned long tmr = 0, lastWiFiRetry = 0, lastMqttTry = 0, lastPub = 0, lastDisp = 0;
/* v5.11：断电时停止（pol=0）的信号宽限状态。
 * lost = 已经判定"到位"但还在宽限期内（阀/泵继续跑）；t0 = 宽限起点。
 * 两路分开记，互不干扰：A 对应 IN1/下水位（排水），B 对应 IN2/上水位（上水）。 */
bool holdALost = false, holdBLost = false;
unsigned long holdAT0 = 0, holdBT0 = 0;
byte dispubFail = 0;
bool apOff = false;              // 连上家网后热点已关
  unsigned long apOffAt = 0;       // 热点关闭时刻
  unsigned long lastApCheck = 0;    // 热点自愈巡检计时（每 5 秒核对一次）
  unsigned long lastNetLog = 0;     // v5.3h：串口网络诊断打印计时（每 3 秒一行）
WebServer server(80);
WiFiClient mqttNet;
PubSubClient mqtt(mqttNet);

/* 继电器实际状态（影子寄存器）：bit0=#1 泵正转 / bit1=#2 泵反转 / bit2=#3 电磁阀 / bit3=#4 备用
 * 驱动级为 SS8050 反相，故 digitalWrite(LOW)=吸合；读取继电器真实状态需回读引脚（见 relayOn） */
byte relayState = 0;

/* 唯一的继电器写入口：所有吸合/释放都必须走这里，极性只由 RLY_ACTIVE_LOW 决定，
 * 这样换板子时只改一个宏，不用满地找 digitalWrite。 */
inline void rlyWrite(int pin, bool on) {
#if RLY_ACTIVE_LOW
  digitalWrite(pin, on ? LOW : HIGH);
#else
  digitalWrite(pin, on ? HIGH : LOW);
#endif
}
// 引脚电平对应的"吸合"读数，供 ioSnap 打印实况用
#define RLY_ON_LEVEL (RLY_ACTIVE_LOW ? LOW : HIGH)

/* ================= 事件日志（v5.6）=================
 * 目的：事后能回答"刚才到底是谁触发了什么"——网页按钮、MQTT、BOOT 键、遥控、
 *       定时计划、Modbus、安全保护，每一条都留痕，不用再靠猜。
 *
 * 实现：32 条环形缓冲，只存在 RAM，**重启即清空**（不写 EEPROM，避免擦写磨损）。
 *       每条格式 "来源|内容"，例如 "BTN|启动 换水" / "WEB|手动 阀开"。
 * 出口：① 串口实时打印 [EV ] 行；② GET /log 返回 JSON；③ 网页「最近动作」卡片。
 * 时间：联网校时后显示真实时钟（HH:MM:SS），未校时显示开机秒数（+123s）。
 */
#define EV_MAX 32
#define EV_LEN 64
struct EvRec { uint32_t ms; uint32_t ep; char txt[EV_LEN]; };
EvRec evBuf[EV_MAX];
byte evHead = 0, evCnt = 0;
const char* evSrc = "WEB";   // 当前动作来源：WEB/MQTT/BTN/RC/TIME/BOOT/MB/RUN/SAFE
/* v5.15：本次运行/手动操作的发起者。evSrc 只在动作发生的那一瞬间有效
 * （mqttCb 里临时切走又切回），函数返回后就丢了；这里把它留住，供 /state 下发给网页。
 * 空串 = 还没跑过任何流程，或已手动全关回到待机。
 *
 * 来源分两类，别混：
 *   【启动流程】TIME 定时 / BOOT 上电 / BTN 短按按钮 / RC 遥控干接点 / MQTT cmd start
 *              → 大字会显示「换水中/排水中/上水中」，后缀只是补充说明是谁点的
 *   【直接控设备】WEB 网页按钮 / MQTT relay1~4/set / MB Modbus 写线圈
 *              → 大字显示「手动」，后缀才告诉你具体是哪个通道
 * 所以「手动」不等于「网页手动」，三条路都会进手动态。 */
const char* runSrc = "";

/* ======== v5.20：给 HA 的「完成 / 结果」信号 ========
 * 原来 HA 只能靠 sensor.progress 的中文文案判断跑完没有 —— 文案改一次自动化就废一次
 * （光今天就从「排水开始」改成「排水泵正转」又改成「排水结束（20 秒）」）。
 *   done : 最近一次流程是否【正常】跑完（st==END）。ON 会一直保持，直到下次启动或手动操作，
 *          所以 HA 不用去抢 END 那 5 秒窗口，慢慢响应也不会漏。
 *   last : 上次结果 / 被拒原因，纯文本，给人看。
 * ⚠ done 必须在「新一轮启动」和「手动接管」时清掉，否则上一轮的 ON 会一直挂着。 */
bool doneFlag = false;
char lastResult[48] = "";
bool endHandled = false;    // END/T5TO 块里只结算一次，避免每轮 loop 都覆盖 lastResult

void evLog(const char* src, const char* msg) {
  EvRec& r = evBuf[evHead];
  r.ms = millis();
  r.ep = timeSynced ? (uint32_t)time(nullptr) : 0;
  snprintf(r.txt, EV_LEN, "%s|%s", src, msg);
  evHead = (evHead + 1) % EV_MAX;
  if (evCnt < EV_MAX) evCnt++;
  Serial.printf("[EV ] %s\r\n", r.txt);
}
void evLog(const char* src, const String& msg) { evLog(src, msg.c_str()); }

bool relayOn(int pin) {
  if (pin == RLY1) return (relayState & 0x01) != 0;
  if (pin == RLY2) return (relayState & 0x02) != 0;
  if (pin == RLY3) return (relayState & 0x04) != 0;
  if (pin == RLY4) return (relayState & 0x08) != 0;
  return false;
}
// 原子写：一次设置全部 4 路，保证互斥不会中间态导致泵反转与正转同时吸合
void applyRelays(bool f, bool r, bool v, bool a) {
  byte want = 0;
  if (f) want |= 0x01;
  if (r) want |= 0x02;
  if (v) want |= 0x04;
  if (a) want |= 0x08;
  if (want == relayState) return;
  // 先全部写入目标电平再切输出方向，避免切换瞬间出现全吸合
  rlyWrite(RLY1, (want & 0x01) != 0);
  rlyWrite(RLY2, (want & 0x02) != 0);
  rlyWrite(RLY3, (want & 0x04) != 0);
  rlyWrite(RLY4, (want & 0x08) != 0);
  relayState = want;
}
// 三参版供自动状态机使用：只改泵/阀三路，保留 K4 备用继电器的既有状态，
// 避免自动流程（如排水→上水）把手动打开的备用输出意外清掉。
void setOut(int f, int r, int v) { applyRelays(f, r, v, relayOn(RLY4)); }
void setOutAll(int f, int r, int v, int a) { applyRelays(f, r, v, a); }

/* TKS-M8 单键电平泵：把「正转/反转/停」翻译成「供电+信号」两路继电器落点。
 * 参数仍是上层语义（f=正转 r=反转），本函数内部换算成 TKS 需要的物理电平：
 *   #1=供电(RLY1)  #2=K1信号(RLY2)
 * 阀 v 原样透传给 #3。 */
void pumpOut(bool f, bool r, bool v) {
#if TKS_SINGLE_KEY
  bool pwr = (f || r);   // 正转或反转都要供电；停=断电
  bool sig = f;          // 正转=K1有信号(吸合)，反转=K1无信号(断开)
  applyRelays(pwr, sig, v, relayOn(RLY4));
#else
  applyRelays(f, r, v, relayOn(RLY4));
#endif
}

// 水位「是否到位」的原语。activeLow=1：光耦导通（GPIO 读 LOW）算到位；0 则相反。
// 极性按路作为参数传入，所以 IN1 接贴壁探头（低电平型）、IN2 接机械浮球也能混着用。
inline bool lvlActive(int p, bool activeLow) {
  bool low = (digitalRead(p) == LOW);   // 光耦导通 → 输出侧被拉低
  return activeLow ? low : !low;
}
bool trigS(int p, bool activeLow) {
  if (!lvlActive(p, activeLow)) return false;
  delay(SEN_CONFIRM_MS);                // 确认时间：水面波动 / 触点回跳 / 电容探头抖动
  return lvlActive(p, activeLow);
}
bool trigAny(int p1, int p2, bool activeLow) {   // 主 + 备用，任一到位即触发
#if USE_REDUNDANT
  if (!lvlActive(p1, activeLow) && !lvlActive(p2, activeLow)) return false;
  delay(SEN_CONFIRM_MS);
  return lvlActive(p1, activeLow) || lvlActive(p2, activeLow);
#else
  (void)p2;                             // 冗余未启用：只认主输入，备用脚完全不参与
  return trigS(p1, activeLow);
#endif
}
/* v5.11：「到位」是否确认生效。
 * 两种极性的"保持"语义本来就不同，所以停机判定不能一刀切：
 *   pol=1 通电时停止 —— 信号出现【就是】到位事件（浮球闭合后不会自己松开），出现即停，不宽限。
 *   pol=0 断电时停止 —— 信号是"允许工作"的许可证，要求【持续存在】。这种条件天然会波动，
 *                       所以丢失后先给 HOLD_GRACE_MS 宽限，连续丢失够久才确认到位；
 *                       期间恢复就撤销宽限，流程继续跑（否则一次接触不良就废掉整轮换水）。
 * lost/t0 是调用方传进来的状态变量（A/B 两路各一套）。
 * 返回 true = 确认到位，调用方可以停机/跳转。 */
bool holdOk(byte pol, bool trig, bool& lost, unsigned long& t0, const char* what) {
  if (!trig) {
    if (lost) { lost = false; evLog("RUN", String(what) + "信号恢复，继续运行"); }
    return false;
  }
  if (pol != 0) return true;                       // 通电时停止：出现即到位
  if (!lost) {
    lost = true; t0 = millis();
    evLog("RUN", String(what) + "信号丢失，宽限 " + String(HOLD_GRACE_MS / 1000) + " 秒");
    return false;
  }
  return (millis() - t0 >= (unsigned long)HOLD_GRACE_MS);
}

const char* onoff(bool b) { return b ? "ON" : "OFF"; }
const char* bootName() { return trig == 0 ? "是" : "否"; }   // 开机启动：是/否
const char* objName(byte o) { return o == 0 ? "换水" : o == 1 ? "排水" : "上水"; }
const char* stName() {
  switch (st) {
    case FWD: return "正转"; case REV: return "反转"; case T4REV: return "超时反转";
    case GAP: return "停止"; case VALVE_ON: return "阀开"; case T5TO: return "超时关阀";
    case END: return "结束"; default: return manSt;
  }
}
// 泵语义状态（0停/1正转/2反转）。progressText() 要用它，而它的定义在本文件后半部分，
// 故在此提前声明 —— 单键模式下 #1/#2 是「供电/信号」，不能直接拿 relayOn 判断正反转。
int pumpState();

// 运行状态：给 HA/网页的阶段名。v5.18 起改写成「设备 + 动作 + 原因」的口语化说法，
// 用户要求保留「（下水位触发/超时）」这种并列写法 —— 同一位置可能由两种原因进入，
// 具体是哪个由卡片颜色和事件日志区分（超时会转橙）。
const char* progressText() {
  switch (st) {
    case FWD: return "排水泵正转";
    /* REV 与 T4REV 共用一个说法：泵都是反转，区别只在「下水位触发」还是「T4 超时」。
     * 并列写出来是因为用户希望一眼看到这个位置的两种可能；真超时了卡片会转橙。 */
    case REV:
    case T4REV: return "排水泵反转（下水位触发/超时）";
    /* GAP 只在换水流程里出现（纯排水流程反转完直接 END，没有 T2 停顿）。
     * 它的含义是「排水段完成，停 T2 秒再开阀」，不是整个流程结束。
     * v5.19：这里直接把 T2 的秒数写出来，不要显示参数名「T2」—— 用户看的是实际等待时间。 */
    case GAP: {
      static char gapBuf[28];
      snprintf(gapBuf, sizeof(gapBuf), "排水结束（%u 秒）", (unsigned)cfgT2);
      return gapBuf;
    }
    case VALVE_ON: return "上水阀打开";
    case T5TO: return "上水阀停止（超时）";
    /* END 两种来路：纯排水流程反转完（obj=1），或换水/上水流程上水位到位。 */
    case END: return runObj == 1 ? "排水结束" : "上水阀停止（上水位触发）";
    /* st==IDLE：手动控制或空闲。这里按【实际继电器状态】判断，不能只看 manSt ——
     * manSt 是动作发生那一刻写下的标签，之后可能和实际输出脱节：
     * 例：开着 K4 备用时 manSt 仍是「待机」，只看 manSt 会错报成「当前没有任务」；
     *     点「停止」后若 K4 没被带走，同样会错。 */
    default: {
      if (leakLatch) return "已强制关闭全部输出";
      if (strcmp(manSt, "已到位未启动") == 0) return "信号已到位，请检查接线或极性";
      int p = pumpState();
      if (p == 1) return "泵正转";
      if (p == 2) return "泵反转";
      if (relayOn(RLY3)) return "阀打开";
      if (relayOn(RLY4)) return "备用开";
      /* v5.19：全关之后仍然给出「刚才关了什么」的回执，而不是一句「当前没有任务」。
       * 顺序放在实际状态判断之后 —— 有输出时以实际状态为准，没输出才看回执。 */
      if (strcmp(manSt, "全部停止") == 0) return "全部停止";
      if (strcmp(manSt, "阀关闭") == 0) return "阀关闭";
      if (strcmp(manSt, "泵停止") == 0) return "泵停止";
      if (strcmp(manSt, "备用关闭") == 0) return "备用关闭";
      return "当前没有任务";
    }
  }
}

/* 前置声明：下面的 LED/按键/遥控任务会用到 startRun()，而其定义在本文件后半部分。
 * Arduino IDE 会自动补 Prototype，但 PlatformIO 或把 .ino 改成 .cpp 时会编译失败，故显式声明。 */
void startRun(byte o);
void startRun(byte o, const char* src);   // src=动作来源，写进事件日志
String ioSnap();      // v5.3i：诊断快照（漏水锁定 / 输入电平 / 继电器引脚实况）
void serialCli();     // v5.3i：串口命令行
void leakReset();                             // btnTask() 在前、leakReset() 定义在后，需显式声明
void leakReset(const char* src);

/* v5.11 补：别指望 Arduino 的自动原型生成 —— 它靠 ctags，而 ctags 不认识 rawliteral
 * 这种原始字符串：handleRoot() 里那段 500 行的 HTML 会让 ctags 把每个双引号当成字符串结束，
 * 从此之后整个文件的解析都是错位的，后面定义的函数能不能拿到隐式原型纯看运气
 * （v5.8 能过、v5.11 报 httpSelfTest 未声明，就是 HTML 长度变了、错位落点跟着变）。
 * 规矩：凡是"先用后定义"的函数，一律在这里显式声明，不要依赖 IDE。 */
void save();           // wifiFactoryReset() 在 L646 调它，save() 定义在后面
void httpSelfTest();   // doCmd 的 selftest 分支调它，定义在 handleRoot() 的大 rawliteral 之后

/* ================= v5.17：状态卡片两行各管一件事 =================
 * 用户定的分工：
 *   大字(26px) = 流程名 / 手动 / 待机 / 漏水保护  ——「在干什么」
 *   小字(13px) = 来源.设备状态                    ——「谁让它干的」「泵阀现在什么状态」
 * 于是：
 *   换水流程 MQTT 触发  →  大字「换水」 / 小字「MQTT.泵反转」
 *   手动正转 网页点击    →  大字「手动」 / 小字「网页.泵正转」
 *   待机               →  大字「待机」 / 小字「当前没有任务」
 * v5.16 及以前大字放的是 stName()（正转/反转/阀开），和小字末段的设备状态撞车；
 * 现在大字让给流程名，设备状态只在 progress 里出现一次。 */

// 大字：在干什么。自动流程 → 「换水中 / 排水中 / 上水中」，跑完 → 「…完成」；
// 手动有输出 → 「手动」；否则「待机」。用户原话：「大字不就是流程名吗，比如换水中」。
const char* runPhase() {
  if (leakLatch) return "漏水保护";
  if (st != IDLE) {
    bool done = (st == END);
    if (runObj == 0) return done ? "换水完成" : "换水中";
    if (runObj == 1) return done ? "排水完成" : "排水中";
    return done ? "上水完成" : "上水中";
  }
  if (relayOn(RLY1) || relayOn(RLY2) || relayOn(RLY3) || relayOn(RLY4)) return "手动";
  /* v5.19：刚手动关掉设备时仍算「手动」，好让状态卡显示「阀关闭」这类回执，
   * 而不是立刻掉回笼统的「待机 / 当前没有任务」。真正回待机要等下一次上电或流程结束。 */
  if (strcmp(manSt, "全部停止") == 0 || strcmp(manSt, "阀关闭") == 0 ||
      strcmp(manSt, "泵停止") == 0 || strcmp(manSt, "备用关闭") == 0) return "手动";
  return "待机";
}

// 小字末段：泵阀此刻的物理状态。多路同时动作时用 + 连接。
String devText() {
  byte p = pumpState();
  String s = "";
  if (p == 1) s = "泵正转";
  else if (p == 2) s = "泵反转";
  if (relayOn(RLY3)) { if (s.length()) s += "+"; s += "阀开"; }
  if (relayOn(RLY4)) { if (s.length()) s += "+"; s += "备用开"; }
  if (!s.length()) s = "全部停止";
  return s;
}

/* ================= 人机交互：一颗状态灯 + 一个 BOOT 键 =================
 * 继电器的 4 颗红灯(LED1/2/5/6)是硬件自驱的，固件完全管不着，也不需要管：
 * 它们并联在线圈两端，反映"继电器是否真的通电"，固件跑飞时依然可信。
 * 这里是唯一软件可控的那颗 GPIO14 灯，用来反映**系统状态**。 */
void ledSet(bool on) { digitalWrite(LED_STATUS, LED_ACTIVE_HIGH ? on : !on); }

#define BTN_STOP_MS      3000UL   // BOOT 长按 3s：急停 + 解除漏水锁定
#define BTN_RESTART_MS   6000UL   // BOOT 长按 6s：松手即重启（保留全部配置）
#define BTN_WIFIRESET_MS 10000UL  // BOOT 超长按 10s：网络复位（清 WiFi 重开热点）+ 重启
#define WIFI_RESET_CLEARS_MQTT 0  // 网络复位时是否一并清掉 MQTT 配置：1=清，0=保留（默认）
// 0=无提示 1=急停窗口(3~6s) 2=重启窗口(6~10s)，LED 用不同闪速区分
byte btnWarn = 0;


void ledTask() {
  static unsigned long last = 0;
  static bool lv = false;
  bool wifiOk = (wifiMode == 1 && WiFi.status() == WL_CONNECTED);
  bool busy = (st != IDLE) || (relayState != 0);
  unsigned long period;
  if (leakLatch)                 period = 60;       // 漏水：极急闪，优先级最高
  else if (btnWarn == 2)         period = 70;       // 6~10s：快速闪 = 现在松手就重启
  else if (btnWarn == 1)         period = 120;      // 3~6s：中速闪 = 已急停，继续按住可重启/复位
  else if (st == T4REV || st == T5TO) period = 100;  // 超时保护触发：急促闪烁
  else if (busy)                 period = 0;        // 流程运行中：常亮
  else if (!wifiOk)              period = 200;      // 未连 WiFi：快闪 = 等配网
  else                           period = 1000;     // 待机在线：慢闪 = 心跳
  if (period == 0) { ledSet(true); return; }
  if (millis() - last >= period) { last = millis(); lv = !lv; ledSet(lv); }
}

// 统一的热点启动入口。
// 起因：v5.3e 及之前有 5 处直接调 `WiFi.softAP("FS_TWCC")`，**从不检查返回值**，也没有任何重试。
//   一旦某次没起来（例如刚被 `softAPdisconnect(true)` 清掉 AP 模式位、驱动还没缓过来），
//   热点就静默消失，且再无恢复机会 —— 只能断电重启。
// 读 core 3.3.11 源码：`softAP()` → `AP.begin()` → `WiFi.enableAP(true)` → mode(currentMode | WIFI_MODE_AP)，
//   理论上能自己补回模式位，所以失败概率不高；**失败后无人兜底**才是真问题。这里做三件事：
//   ① 先显式 `WiFi.mode(WIFI_AP_STA)` 把模式钉死；② 检查返回值；③ 失败延迟重试一次。
// 注意不能拿纯 `WIFI_AP` 兜底 —— 那会把 STA 一起关掉，家网也断了。
bool startAP() {
  WiFi.mode(WIFI_AP_STA);                 // 关键是这一步：补回 AP 模式位
  delay(120);
  if (WiFi.softAP("FS_TWCC")) return true;
  delay(300);
  return WiFi.softAP("FS_TWCC");          // 再试一次；还失败就交给 5 秒后的自愈巡检重试
}

// 网络出厂复位：只清 WiFi（SSID/密码/wifiMode）并重开配网热点 FS_TWCC，
// 保留 T0~T5、定时计划与互斥/保护逻辑 —— 换路由器或改 WiFi 密码时不用重新配一堆参数。
void wifiFactoryReset() {
  evLog("BTN", "10s 网络复位：清 WiFi，重开热点");
  applyRelays(0, 0, 0, 0);                 // 先把输出全放掉，别带着负载重启
  st = IDLE; leakLatch = false;
  manSt = "WiFi 已复位";
  ssid[0] = 0; pass[0] = 0; wifiMode = 0;  // 只动网络三字段
#if WIFI_RESET_CLEARS_MQTT
  mqttEn = 0; mqttHost[0] = 0; mqttUser[0] = 0; mqttPass[0] = 0;
#endif
  save();
  for (byte i = 0; i < 6; i++) { ledSet(true); delay(100); ledSet(false); delay(100); }  // 急闪 6 下提示
  WiFi.disconnect();
  startAP(); apOff = false;
  delay(200);
  ESP.restart();                           // 干净重启，重新走一遍配网流程
}

// BOOT 键（GPIO0）四档：
//   短按 <3s       执行一次换水
//   按住 >=3s      急停：强制停止全部输出 + 解除漏水锁定（立即执行，不用松手）
//   6~10s 之间松手 重启：保留 WiFi/计划/MQTT 全部配置
//   按住 >=10s     网络复位：清 WiFi 重开热点 FS_TWCC，然后重启
// 6s 档特意做成「松手触发」：若一按到 6s 就立即重启，就永远按不到 10s 的复位档。
void btnTask() {
  static bool lastLv = HIGH;
  static unsigned long pressAt = 0;
  static bool stopDone = false;            // 本次按住是否已执行过急停
  bool now = digitalRead(BTN_BOOT);
  if (lastLv == HIGH && now == LOW) {                // 按下沿
    delay(30);                                        // 去抖
    if (digitalRead(BTN_BOOT) == LOW) { pressAt = millis(); stopDone = false; btnWarn = 0; }
    else now = HIGH;
  } else if (lastLv == LOW && now == HIGH) {          // 松开沿
    unsigned long held = millis() - pressAt;
    if (!stopDone && held < BTN_STOP_MS) {
      startRun(0, "BTN");                             // 短按：开始换水
    } else if (held >= BTN_RESTART_MS) {
      manSt = "按键重启";                             // 6~10s 松手：只重启，不动配置
      evLog("BTN", "6s 松手：重启（配置保留）");
      Serial.println("[BTN] 6s 松手重启");
      delay(50);
      ESP.restart();
    }
    stopDone = true;
    btnWarn = 0;
  } else if (now == LOW) {                            // 持续按住
    unsigned long held = millis() - pressAt;
    if (held >= BTN_STOP_MS) {
      if (!stopDone) { leakReset("BTN"); manSt = "急停"; stopDone = true; }  // 急停全关 + 解锁漏水
      btnWarn = (held >= BTN_RESTART_MS) ? 2 : 1;
    }
    if (held >= BTN_WIFIRESET_MS) wifiFactoryReset();  // 10s：清 WiFi + 重启
  }
  lastLv = now;
}

/* ================= 浸水传感器（IN3）：安全信号，优先级最高 =================
 * 接线：传感器/开关须串在 JP2 的 5V 与 IN3 之间（GND_IN 先与 GND 短接），
 *      不可仅短接 IN3 与 GND_IN——输入侧无源，那样没有电流、光耦不导通。
 *      传感器本身必须是干接点(浮球/继电器)；纯探针式阻值过大，实测无法触发。
 * 触发：连续浸水 LEAK_CONFIRM_MS 后认定（防溅水/结露误报）。
 * 动作：立即强制全关 K1~K4，并置 leakLatch 自锁。
 *
 * 自锁的意义（务必理解）：传感器变干【不会】自动解除。
 * 设想不自锁的后果 —— 地面积水被抽干或自然蒸发后系统自动恢复，
 * 然后又开始上水、再次漏水…… 在无人值守时形成启停循环，比一直停机危险得多。
 * 所以必须由人到现场确认、排除漏水后再手动复位。
 *
 * 复位途径（四选一）：
 *   1) 板载 BOOT 键长按 ≥3 秒
 *   2) 遥控键长按 ≥RC_LONG_MS
 *   3) 网页「复位并解除锁定」按钮
 *   4) MQTT 发送 esp-pump/cmd = leak_reset
 * 注：传感器仍处于浸水状态时复位会立刻再次触发（这是对的）。
 */
void leakTask() {
  static unsigned long wetSince = 0;
  bool wet = (digitalRead(LEAK_IN) == LOW);
#if !LEAK_TRIG_LOW
  wet = !wet;                       // 反向输出的模块
#endif
  leakNow = wet;

  if (!wet) { wetSince = 0; return; }        // 干燥：计时清零（已锁定则保持锁定）
  if (leakLatch) return;                     // 已锁定：不再重复动作
  if (wetSince == 0) { wetSince = millis(); return; }
  if (millis() - wetSince < LEAK_CONFIRM_MS) return;

  // 确认浸水：强制全关 + 自锁
  leakLatch = true;
  st = IDLE;
  evLog("SAFE", "浸水确认：强制全关并锁定");
  applyRelays(0, 0, 0, 0);   // 注意：不可先写 relayState=0，否则 applyRelays 判定
  manSt = "漏水保护";        // want==relayState 提前 return，继电器根本不会断开
}

/* ================= 单键遥控（IN4）：以击数区分动作 =================
 * 一个输入要控制三个流程，靠的是"按几下"：
 *     1 击 = 排水      2 击 = 上水      3 击 = 换水      长按 = 急停 + 复位漏水锁定
 * 判定方法：每次松开都累加计数，若 RC_WIN_MS 内没有下一次按下，就按累计击数执行。
 *
 * 【重要前提】若用 433MHz 接收板，其输出必须设为【点动 / 瞬态模式】(momentary/jog)，
 *   即"按住才导通、松开即断开"。若设成自锁模式（按一下翻转并保持），
 *   一次按下会被持续读到 LOW，超过 RC_LONG_MS 后判成长按 → 误触发急停。
 *   点动/自锁通常由接收板上的跳线或焊点选择，接线前务必确认。
 */
void rcTask() {
  static bool lastLv = HIGH;
  static byte cnt = 0;
  static unsigned long pressAt = 0;
  static unsigned long lastRel = 0;
  static bool longDone = false;      // 本次按下已判为长按，松开时不再计击数
  static bool pending = false;       // 有待执行的击数

  bool now = digitalRead(RC_IN);

  // —— 按下沿 ——
  if (lastLv == HIGH && now == LOW) {
    delay(30);                                  // 去抖
    if (digitalRead(RC_IN) == LOW) { pressAt = millis(); longDone = false; }
    else now = HIGH;                            // 抖动，忽略
  }

  // —— 长按判定（仍按住且未处理过）——
  if (now == LOW && !longDone && millis() - pressAt >= RC_LONG_MS) {
    longDone = true;
    cnt = 0; pending = false;
    leakLatch = false;                          // 长按同时解除漏水锁定
    st = IDLE;
    applyRelays(0, 0, 0, 0);
    manSt = "急停";
    evLog("RC", "长按 2.5s：急停 + 解除漏水锁定");
  }

  // —— 松开沿：累计击数 ——
  if (lastLv == LOW && now == HIGH) {
    if (!longDone) { cnt++; pending = true; lastRel = millis(); }
  }

  // —— 连击窗口结束：按击数执行 ——
  if (pending && millis() - lastRel >= RC_WIN_MS) {
    byte n = cnt; cnt = 0; pending = false;
    if (n == 1)      startRun(1, "RC");         // 排水
    else if (n == 2) startRun(2, "RC");         // 上水
    else if (n >= 3) startRun(0, "RC");         // 换水
  }

  lastLv = now;
}

// 供 BOOT 长按 / 网页 / MQTT 复用：解除漏水锁定并全关
void leakReset() { leakReset("WEB"); }
void leakReset(const char* src) {
  leakLatch = false;
  st = IDLE;
  applyRelays(0, 0, 0, 0);
  manSt = "已复位";
  evLog(src, "急停：全关 + 解除漏水锁定");
}

/* ================= Modbus RTU 从机（保持成品板 RS485 能力不被烧写丢失）=================
 * 与原厂协议完全一致：地址1 / 115200 / 8N1，功能码 01/02/05/0F
 *   0x01 读线圈   → 继电器 #1~#4 状态
 *   0x02 读离散输入 → 光耦 #1~#4 状态（本板光耦未接，恒为 0 = 无输入）
 *   0x05 写单线圈 → 控制单个继电器
 *   0x0F 写多线圈 → 批量控制
 * 上位机/DAM 调试软件因此可继续使用；本固件的网页与 MQTT 逻辑不受影响。
 */
uint16_t mbCrc16(const uint8_t* d, uint16_t n);
void mbHandle(uint8_t* f, uint8_t n, uint8_t fc);
uint16_t mbCrc16(const uint8_t* d, uint16_t n) {
  uint16_t c = 0xFFFF;
  for (uint16_t i = 0; i < n; i++) {
    c ^= d[i];
    for (uint8_t b = 0; b < 8; b++) c = (c & 1) ? (c >> 1) ^ 0xA001 : c >> 1;
  }
  return c;
}
void mbSendBytes(uint8_t* d, uint8_t n) {
  digitalWrite(MB_DE, HIGH);            // 切发送
  delayMicroseconds(200);               // 建立建立时间
  mbSerial.write(d, n);
  mbSerial.flush();                     // 等移位彻底完成
  delayMicroseconds(200);
  digitalWrite(MB_DE, LOW);             // 切回接收
  delay(2);                             // 帧间静默 ≥3.5 字符
}
// 读光耦输入（Modbus 0x02）。说明书定义 Bit0~Bit3 依次对应光耦 #1~#4。
// v5.1 及之前此处位序写反了（把 GPIO27 当 bit0），本版按说明书修正。
byte mbOptoBits() {
  byte v = 0;
  if (digitalRead(OPT1) == LOW) v |= 0x01;
  if (digitalRead(OPT2) == LOW) v |= 0x02;
  if (digitalRead(LEAK_IN) == LOW) v |= 0x04;   // bit2 = 光耦#3 = 浸水传感器
  if (digitalRead(RC_IN)   == LOW) v |= 0x08;   // bit3 = 光耦#4 = 遥控按钮
  return v;
}
void mbApplyCoilMask(byte mask) {
  // 与网页/状态机共享同一继电器状态，并中断自动流程（等同手动操作语义）
#if TKS_SINGLE_KEY
  // 单键模式：bit0/bit1 仍按「正转/反转」语义传入，由 pumpOut 翻译成供电+信号
  pumpOut(mask & 0x01, mask & 0x02, mask & 0x04);
#else
  applyRelays(mask & 0x01, mask & 0x02, mask & 0x04, mask & 0x08);
#endif
  st = IDLE;
  const char* m = "停止";
  if (mask & 0x01) m = "正转";
  else if (mask & 0x02) m = "反转";
  else if (mask & 0x04) m = "阀开";
  else if (mask & 0x08) m = "备用开";
  manSt = m;
  /* v5.18：Modbus 写线圈也是一次「直接控制设备」的动作，来源要标成 MB，
   * 否则状态卡会挂空来源或残留上一次的（网页/MQTT）。
   * mask=0 是全关，等于回待机，来源一并清空。 */
  runSrc = mask ? "MB" : "";
  evLog("MB", String("线圈写 mask=0x") + String(mask, HEX) + " → " + m);
}
void mbPoll() {
  static uint8_t buf[256];
  static uint8_t idx = 0;
  while (mbSerial.available()) {
    int b = mbSerial.read();
    if (b < 0) break;
    if (idx == 0 && b != MB_SLAVE_ID) continue;      // 只处理本机地址，静默丢弃其它站
    buf[idx++] = (uint8_t)b;
    if (idx >= 2) {
      uint8_t fc = buf[1];
      uint8_t need = 0;
      if (fc == 0x01 || fc == 0x02) need = 8;        // 地址+功能+起始2+数量2+CRC2
      else if (fc == 0x05) need = 8;
      else if (fc == 0x0F) need = 9;                 // +数据字节数(此处固定1)
      if (need && idx >= need) { mbHandle(&buf[0], need, fc); idx = 0; }
      else if (need && idx > need) idx = 0;          // 超长帧 → 重同步
    }
    if (idx >= 250) idx = 0;
  }
}
void mbHandle(uint8_t* f, uint8_t n, uint8_t fc) {
  uint16_t rxCrc = (uint16_t)f[n - 2] | ((uint16_t)f[n - 1] << 8);
  if (mbCrc16(f, n - 2) != rxCrc) return;            // CRC 错静默丢弃（RTU 标准行为）
  uint16_t addr = ((uint16_t)f[2] << 8) | f[3];
  if (addr >= 4) return;                             // 越界地址忽略
  uint8_t r[64]; uint8_t rl = 0;
  if (fc == 0x01 || fc == 0x02) {
    r[0] = MB_SLAVE_ID; r[1] = fc; r[2] = 0x01;
    r[3] = (fc == 0x01) ? relayState : mbOptoBits();
    rl = 4;
  } else if (fc == 0x05) {
    bool on = (f[4] == 0xFF && f[5] == 0x00);
    byte m = relayState;
    if (on) m |= (1 << addr); else m &= ~(1 << addr);
    mbApplyCoilMask(m);
    memcpy(r, f, 6);                                  // 功能码05 原样回显
    rl = 6;
  } else if (fc == 0x0F) {
    uint16_t qty = ((uint16_t)f[4] << 8) | f[5];
    uint8_t bcount = f[6];
    byte m = 0;
    for (uint8_t i = 0; i < bcount && i < 2; i++) {
      byte d = f[7 + i];
      for (uint8_t b = 0; b < 8; b++) if (d & (1 << b)) {
        uint16_t a = addr + i * 8 + b;
        if (a < 4) m |= (1 << a);
      }
    }
    (void)qty;
    mbApplyCoilMask(m);
    memcpy(r, f, 6);                                  // 回显 地址/起始/数量
    rl = 6;
  } else return;                                      // 不支持的功能码
  uint16_t c = mbCrc16(r, rl);
  r[rl++] = c & 0xFF;                                 // CRC 低字节在前
  r[rl++] = c >> 8;
  mbSendBytes(r, rl);
}

// ---- 字符串处理：UTF-8 安全拷贝 + JSON 转义 ----
// 最后一次保存时被丢弃的字节数（>0 = 输入太长，缓冲区装不下）
unsigned int lastTrimBytes = 0;

// 把 src 拷进定长缓冲区，但绝不把一个多字节字符切成两半。
//
// 为什么需要它：String::toCharArray(dst, n) 是按【字节】截断的，UTF-8 里一个汉字占 3 字节。
//   当剩余空间凑巧停在某个汉字的第 2 / 第 3 字节上时，剩下的 1~2 个字节既不是合法 UTF-8，
//   也拼不回汉字 —— 浏览器和 JSON 解析器只能把它显示成 U+FFFD，
//   也就是你看到的「黑方块里面一个问号」。
//   注意这不是"编码格式搞错了"，而是字节层面把一个汉字切成了残骸。
// 做法：先按字节算出截断位置，再向前回退，直到落在字符边界上（ASCII 或多字节首字节）。
unsigned int utf8Cpy(char* dst, const String& src, unsigned int bufsize) {
  if (bufsize == 0) return 0;
  unsigned int len = src.length();
  unsigned int cut = bufsize - 1;                 // 留一个字节给结尾 '\0'
  if (cut > len) cut = len;
  // cut 处如果是一个 UTF-8 续字节(10xxxxxx)，说明把一个字符切开了 -> 回退到字符起点
  while (cut > 0 && ((unsigned char)src.charAt(cut) & 0xC0) == 0x80) cut--;
  for (unsigned int i = 0; i < cut; i++) dst[i] = src.charAt(i);
  dst[cut] = 0;
  return len - cut;                               // 返回被丢弃的字节数
}

// JSON 字符串转义：SSID / MQTT 用户名里如果出现引号或反斜杠，会直接把 JSON 结构撑破
String jesc(const char* p) {
  String o;
  while (*p) {
    char c = *p++;
    if (c == '"' || c == '\\') { o += '\\'; o += c; }
    else if (c == '\n') o += "\\n";
    else if ((unsigned char)c < 0x20) { /* 控制字符直接丢弃 */ }
    else o += c;
  }
  return o;
}

void save() {
  EEPROM.put(0, cfgT1); EEPROM.put(2, cfgT2); EEPROM.put(5, wifiMode);
  EEPROM.put(6, ssid); EEPROM.put(39, pass);
  EEPROM.put(104, mqttEn); EEPROM.put(105, mqttHost); EEPROM.put(121, mqttPort);
  EEPROM.put(123, mqttUser); EEPROM.put(155, mqttPass);
  EEPROM.put(219, trig); EEPROM.put(220, objGlobal);
  EEPROM.put(221, cfgT0); EEPROM.put(223, cfgT4); EEPROM.put(225, cfgT5);
  EEPROM.put(227, planCount);
  EEPROM.put(308, senBPol);        // v5.8：IN2 判定极性（228~307 被定时计划占用，从 308 起空闲）
  EEPROM.put(309, senAPol);        // v5.9：IN1 判定极性
  for (int i = 0; i < MAX_PLAN; i++) {
    int a = 228 + i * 8;
    EEPROM.put(a, planEn[i]); EEPROM.put(a + 1, planType[i]); EEPROM.put(a + 2, planParam[i]);
    EEPROM.put(a + 3, planHour[i]); EEPROM.put(a + 4, planMin[i]); EEPROM.put(a + 5, planObj[i]);
    EEPROM.put(a + 6, planRef[i]);
  }
  EEPROM.commit();
}
void planShiftDel(int idx) {
  for (int i = idx; i < MAX_PLAN - 1; i++) {
    planEn[i] = planEn[i + 1]; planType[i] = planType[i + 1]; planParam[i] = planParam[i + 1];
    planHour[i] = planHour[i + 1]; planMin[i] = planMin[i + 1]; planObj[i] = planObj[i + 1]; planRef[i] = planRef[i + 1];
  }
  planEn[MAX_PLAN - 1] = 0; planType[MAX_PLAN - 1] = 0; planParam[MAX_PLAN - 1] = 0;
  planHour[MAX_PLAN - 1] = 0; planMin[MAX_PLAN - 1] = 0; planObj[MAX_PLAN - 1] = 0; planRef[MAX_PLAN - 1] = 0;
  planCount--;
  save();
}
int dayNum() { return (int)(time(nullptr) / 86400); }

// 计算某计划「下一次执行」的日期时间文本（供前端展示）。
// 返回空串 = 无法计算（时间未同步 / 数据无效）。
String nextRunText(int idx) {
  if (!timeSynced) return "";
  if (idx < 0 || idx >= planCount) return "";
  time_t now = time(nullptr);
  int h = planHour[idx], m = planMin[idx];
  char buf[20];
  if (planType[idx] == 0) {   // 按周：从今天起往后找第一个命中的星期
    int wbit = planParam[idx];
    if (wbit == 0) return "";
    for (int d = 0; d < 8; d++) {
      time_t t = now + (time_t)d * 86400;
      struct tm lt; localtime_r(&t, &lt);
      int wd = lt.tm_wday;                    // 0=周日
      int w = (wd == 0) ? 6 : wd - 1;         // 0=周一
      if (wbit & (1 << w)) {
        snprintf(buf, sizeof(buf), "%02d-%02d %02d:%02d", lt.tm_mon + 1, lt.tm_mday, h, m);
        return String(buf);
      }
    }
    return "";
  } else {                   // 隔N天：从基准日 planRef 往后找第一个命中
    int N = planParam[idx];
    if (N <= 0 || planRef[idx] <= 0) return "";
    int today = dayNum();
    for (int d = 0; d < N + 1; d++) {
      int dn = today + d;
      if ((dn - planRef[idx]) % N == 0) {
        time_t t = now + (time_t)d * 86400;
        struct tm lt; localtime_r(&t, &lt);
        snprintf(buf, sizeof(buf), "%02d-%02d %02d:%02d", lt.tm_mon + 1, lt.tm_mday, h, m);
        return String(buf);
      }
    }
    return "";
  }
}

String stateText() {
  String s = "{";
  s += "\"phase\":\"" + String(runPhase()) + "\"";   // v5.17：大字改成流程名/手动/待机
  s += ",\"progress\":\"" + String(progressText()) + "\"";
  s += ",\"dev\":\"" + devText() + "\"";             // v5.17：泵阀物理状态，小字末段用
  s += ",\"tmo\":" + String((st == T4REV || st == T5TO) ? 1 : 0);   // 超时（T4 反转 / T5 关阀）
  s += ",\"ended\":" + String(st == END ? 1 : 0);    // 流程刚跑完，还没被下一次操作覆盖
  s += ",\"src\":\"" + String(runSrc) + "\"";   // v5.15：本次运行/手动操作的发起者（网页/定时/按钮/遥控/MQTT/上电）
  /* v5.15：1=自动流程在跑，0=待机/手动/已结束/漏水锁定。
   * 网页靠它决定要不要显示流程名 —— 手动时 runObj 还是上一次流程的值，
   * 不区分的话会显示成「手动正转中 · 换水」这种错乱组合。
   * v5.16：把 END 和 leakLatch 也排除。END 时 st 虽不是 IDLE，但流程已结束、
   *        runObj 已是过去式；漏水锁定会强制全关，此时再挂「换水」更是胡说。 */
  s += ",\"auto\":" + String((st != IDLE && st != END && !leakLatch) ? 1 : 0);
  s += ",\"obj\":" + String(runObj);
  s += ",\"r1\":" + String(relayOn(RLY1) ? 1 : 0);
  s += ",\"r2\":" + String(relayOn(RLY2) ? 1 : 0);
  s += ",\"r3\":" + String(relayOn(RLY3) ? 1 : 0);
  s += ",\"r4\":" + String(relayOn(RLY4) ? 1 : 0);
  // 泵的「语义状态」：0=停 1=正转 2=反转。单键模式(TKS_SINGLE_KEY)下 #1/#2 是
  // 供电/信号，不能直接用 r1/r2 判断泵在正转还是反转，故单独给出语义字段给网页用。
  s += ",\"pump\":" + String(pumpState());
  // v5.8：两路水位的【原始信号】+ IN2 当前判定极性。
  // inXraw=1 表示光耦导通（有信号，GPIO 读 LOW）；与网页「到位」不是一回事，
  // 到位与否还要经过 in2pol 换算 —— 排查极性时看这个，别看到位。
  s += ",\"in1raw\":" + String(digitalRead(SEN_A) == LOW ? 1 : 0);
  s += ",\"in2raw\":" + String(digitalRead(SEN_B) == LOW ? 1 : 0);
  s += ",\"in1pol\":" + String(senAPol);
  s += ",\"in2pol\":" + String(senBPol);
  s += ",\"boot\":" + String(trig == 0 ? 1 : 0);
  s += ",\"leak\":" + String(leakNow ? 1 : 0);      // 浸水传感器实时状态
  s += ",\"leakLat\":" + String(leakLatch ? 1 : 0); // 漏水保护是否处于锁定
  s += ",\"wifi\":" + String((wifiMode == 1 && WiFi.status() == WL_CONNECTED) ? 1 : 0);
  s += ",\"ip\":\"" + String((wifiMode == 1 && WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString().c_str() : "") + "\"";
  bool hasAP = (WiFi.getMode() & WIFI_AP) != 0;          // 诊断用：热点到底起来没有
  s += ",\"ap\":" + String(hasAP ? 1 : 0);
  s += ",\"apip\":\"" + String(hasAP ? WiFi.softAPIP().toString().c_str() : "") + "\"";
  s += ",\"mqtt\":" + String(mqtt.connected() ? 1 : 0);
  s += ",\"mqttEn\":" + String(mqttEn ? 1 : 0);   // 前端区分：0=未启用 1=已启用(连不连看 mqtt 字段)
  // v5.3k：把板子当前时间直接给页面，省得定时计划准不准全靠猜
  char tb[32]; tb[0] = 0;
  if (timeSynced) {
    time_t n = time(nullptr);
    struct tm t; localtime_r(&n, &t);
    snprintf(tb, sizeof(tb), "%04d-%02d-%02d %02d:%02d:%02d",
             t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec);
  }
  s += ",\"now\":\"" + String(tb) + "\"";
  s += ",\"ver\":\"" + String(FW_VER) + "\"";
  s += ",\"btag\":\"" + String(WTCC_BOARD_TAG) + "\"";   // 完整板卡标记（= 型号-改版），OTA 认这个串
  /* v5.21：硬件身份分两个字段下发，网页拼成「WTCC-ESP32-RLY4 · V1」。
   * 不直接拆 btag 是因为型号里本身就带连字符，拆不干净。 */
  s += ",\"hwm\":\"" + String(HW_MODEL) + "\"";
  s += ",\"hwr\":\"" + String(HW_REV) + "\"";
  s += ",\"build\":\"" + String(FW_BUILD) + "\"";
  s += ",\"pubfail\":" + String(dispubFail);
  s += "}";
  return s;
}
String cfgJson() {
  String s = "{\"t0\":" + String(cfgT0) + ",\"t1\":" + String(cfgT1) + ",\"t2\":" + String(cfgT2) +
             ",\"t4\":" + String(cfgT4) + ",\"t5\":" + String(cfgT5) + ",\"trig\":" + String(trig) +
             ",\"obj\":" + String(objGlobal) + ",\"in1pol\":" + String(senAPol) + ",\"in2pol\":" + String(senBPol) + ",\"ssid\":\"" + jesc(ssid) + "\",\"pset\":" + (pass[0] ? 1 : 0) +
             ",\"mhost\":\"" + jesc(mqttHost) + "\",\"muser\":\"" + jesc(mqttUser) + "\",\"mport\":" + String(mqttPort) +
             ",\"men\":" + String(mqttEn) + ",\"mset\":" + (mqttPass[0] ? 1 : 0) + ",\"trim\":" + String(lastTrimBytes) +
             ",\"plans\":[";
  for (int i = 0; i < planCount; i++) {
    if (i) s += ",";
    s += "{\"en\":" + String(planEn[i]) + ",\"type\":" + String(planType[i]) + ",\"param\":" + String(planParam[i]) +
         ",\"h\":" + String(planHour[i]) + ",\"m\":" + String(planMin[i]) + ",\"obj\":" + String(planObj[i]) +
         ",\"next\":\"" + nextRunText(i) + "\"}";
  }
  s += "]}";
  return s;
}

// ---- MQTT 发现（每条≤240B） ----
// 注意：不做 delay 重试。publish 失败（尤其半开连接）时若还 delay 重试，
// 20+ 条 discovery 会把 loop 堵死几十秒，直接拖垮网页。失败就返回，交给上层计数。
bool tryPub(const char* topic, const char* payload) {
  if (mqtt.publish(topic, payload, true)) return true;
  return false;
}
void mqttPubDiscovery() {
  /* v5.21：device 里补上硬件身份 —— HA 设备页会显示型号 / 硬件版本 / 软件版本，
   * 几块板子挂在同一个 HA 下时一眼分得清哪块是 V1、哪块是 V2。
   *   dev  = 长版，配短配置用，带 manufacturer/model/hw_version/sw_version
   *   devS = 短版，配长配置用（select 之类），只带 model —— MQTT_MAX_PACKET_SIZE 只有 1024
   * ⚠ dev 从 66 字节涨到约 153 字节。若哪天网页「系统」页出现 pubfail > 0，
   *   先查是不是这里太长把某条 discovery 顶爆了。 */
  String dev = String(R"rawliteral("device":{"identifiers":["esp-pump"],"name":"换水控制器","manufacturer":"WTCC","model":")rawliteral")
             + HW_MODEL + R"rawliteral(","hw_version":")rawliteral" + HW_REV
             + R"rawliteral(","sw_version":")rawliteral" + FW_VER + R"rawliteral("})rawliteral";
  String devS = String(R"rawliteral("device":{"identifiers":["esp-pump"],"model":")rawliteral")
              + HW_MODEL + R"rawliteral("})rawliteral";
  if (!tryPub("homeassistant/switch/esp-pump/boot/config",
    (String(R"rawliteral({"name":"开机启动","unique_id":"esp-pump_boot","state_topic":"esp-pump/boot","command_topic":"esp-pump/boot/set","payload_on":"ON","payload_off":"OFF",)rawliteral") + devS).c_str())) dispubFail++;
  if (!tryPub("homeassistant/binary_sensor/esp-pump/floatA/config",
    (String(R"rawliteral({"name":"下水位","unique_id":"esp-pump_floatA","state_topic":"esp-pump/floatA",)rawliteral") + dev).c_str())) dispubFail++;
  if (!tryPub("homeassistant/binary_sensor/esp-pump/leak/config",
    (String(R"rawliteral({"name":"漏水报警","unique_id":"esp-pump_leak","state_topic":"esp-pump/leak","device_class":"moisture",)rawliteral") + dev).c_str())) dispubFail++;
  if (!tryPub("homeassistant/binary_sensor/esp-pump/floatB/config",
    (String(R"rawliteral({"name":"上水位","unique_id":"esp-pump_floatB","state_topic":"esp-pump/floatB",)rawliteral") + dev).c_str())) dispubFail++;
  if (!tryPub("homeassistant/switch/esp-pump/relay1/config",
    (String(R"rawliteral({"name":"泵正转","unique_id":"esp-pump_relay1","state_topic":"esp-pump/relay1","command_topic":"esp-pump/relay1/set",)rawliteral") + dev).c_str())) dispubFail++;
  if (!tryPub("homeassistant/switch/esp-pump/relay2/config",
    (String(R"rawliteral({"name":"泵反转","unique_id":"esp-pump_relay2","state_topic":"esp-pump/relay2","command_topic":"esp-pump/relay2/set",)rawliteral") + dev).c_str())) dispubFail++;
  if (!tryPub("homeassistant/switch/esp-pump/relay3/config",
    (String(R"rawliteral({"name":"电磁阀","unique_id":"esp-pump_relay3","state_topic":"esp-pump/relay3","command_topic":"esp-pump/relay3/set",)rawliteral") + dev).c_str())) dispubFail++;
  if (!tryPub("homeassistant/switch/esp-pump/relay4/config",
    (String(R"rawliteral({"name":"备用继电器","unique_id":"esp-pump_relay4","state_topic":"esp-pump/relay4","command_topic":"esp-pump/relay4/set",)rawliteral") + dev).c_str())) dispubFail++;
  if (!tryPub("homeassistant/button/esp-pump/run_full/config",
    (String(R"rawliteral({"name":"换水","unique_id":"esp-pump_run_full","command_topic":"esp-pump/cmd","payload_press":"start",)rawliteral") + dev).c_str())) dispubFail++;
  if (!tryPub("homeassistant/button/esp-pump/leak_reset/config",
    (String(R"rawliteral({"name":"复位漏水保护","unique_id":"esp-pump_leak_reset","command_topic":"esp-pump/cmd","payload_press":"leak_reset",)rawliteral") + dev).c_str())) dispubFail++;
  if (!tryPub("homeassistant/button/esp-pump/run_drain/config",
    (String(R"rawliteral({"name":"排水","unique_id":"esp-pump_run_drain","command_topic":"esp-pump/cmd","payload_press":"drain",)rawliteral") + dev).c_str())) dispubFail++;
  if (!tryPub("homeassistant/button/esp-pump/run_fill/config",
    (String(R"rawliteral({"name":"上水","unique_id":"esp-pump_run_fill","command_topic":"esp-pump/cmd","payload_press":"fill",)rawliteral") + dev).c_str())) dispubFail++;
  /* v5.20：结果信号。让 HA 等「流程完成」时直接看二元状态，不必去匹配中文文案
   * （progress 的措辞改一次，靠文本匹配的自动化就废一次）。用 devS 短版省长度。 */
  if (!tryPub("homeassistant/binary_sensor/esp-pump/done/config",
    (String(R"rawliteral({"name":"流程完成","unique_id":"esp-pump_done","state_topic":"esp-pump/done","icon":"mdi:check-circle",)rawliteral") + devS).c_str())) dispubFail++;
  if (!tryPub("homeassistant/sensor/esp-pump/last/config",
    (String(R"rawliteral({"name":"上次结果","unique_id":"esp-pump_last","state_topic":"esp-pump/last","icon":"mdi:history",)rawliteral") + devS).c_str())) dispubFail++;
  if (!tryPub("homeassistant/number/esp-pump/t0/config",
    (String(R"rawliteral({"name":"T0 上电延时","unique_id":"esp-pump_t0","state_topic":"esp-pump/t0","command_topic":"esp-pump/t0/set","min":0,"max":9999,)rawliteral") + dev).c_str())) dispubFail++;
  if (!tryPub("homeassistant/number/esp-pump/t1/config",
    (String(R"rawliteral({"name":"T1 反转秒数","unique_id":"esp-pump_t1","state_topic":"esp-pump/t1","command_topic":"esp-pump/t1/set","min":1,"max":9999,)rawliteral") + dev).c_str())) dispubFail++;
  if (!tryPub("homeassistant/number/esp-pump/t2/config",
    (String(R"rawliteral({"name":"T2 停后到阀开","unique_id":"esp-pump_t2","state_topic":"esp-pump/t2","command_topic":"esp-pump/t2/set","min":1,"max":9999,)rawliteral") + dev).c_str())) dispubFail++;
  if (!tryPub("homeassistant/number/esp-pump/t4/config",
    (String(R"rawliteral({"name":"T4 泵最长运行","unique_id":"esp-pump_t4","state_topic":"esp-pump/t4","command_topic":"esp-pump/t4/set","min":1,"max":9999,)rawliteral") + dev).c_str())) dispubFail++;
  if (!tryPub("homeassistant/number/esp-pump/t5/config",
    (String(R"rawliteral({"name":"T5 阀最长运行","unique_id":"esp-pump_t5","state_topic":"esp-pump/t5","command_topic":"esp-pump/t5/set","min":1,"max":9999,)rawliteral") + dev).c_str())) dispubFail++;
  if (!tryPub("homeassistant/sensor/esp-pump/progress/config",
    (String(R"rawliteral({"name":"运行状态","unique_id":"esp-pump_progress","state_topic":"esp-pump/progress",)rawliteral") + dev).c_str())) dispubFail++;
  if (!tryPub("homeassistant/sensor/esp-pump/ip/config",
    (String(R"rawliteral({"name":"IP 地址","unique_id":"esp-pump_ip","state_topic":"esp-pump/ip",)rawliteral") + dev).c_str())) dispubFail++;
}
// 泵语义状态：0=停 1=正转 2=反转。单键模式下 #1/#2 是供电/信号，
// 不能直接用 relayOn(RLY1/RLY2) 判断正反转，故集中在此换算，供网页/MQTT 复用。
int pumpState() {
#if TKS_SINGLE_KEY
  bool pwr = relayOn(RLY1), sig = relayOn(RLY2);
  return !pwr ? 0 : (sig ? 1 : 2);
#else
  return relayOn(RLY1) ? 1 : relayOn(RLY2) ? 2 : 0;
#endif
}
void mqttPubStates() {
  if (!mqtt.connected()) return;
  mqtt.publish("esp-pump/floatA", onoff(trigAny(SEN_A, RED_A, senAPol)));
  mqtt.publish("esp-pump/floatB", onoff(trigAny(SEN_B, RED_B, senBPol)));
  // relay1/relay2 按泵语义发布：正转时只 relay1=ON，反转时只 relay2=ON，
  // 单键模式下不再出现「供电+信号都吸合 → 两灯同亮」。
  int ps = pumpState();
  mqtt.publish("esp-pump/relay1", onoff(ps == 1));
  mqtt.publish("esp-pump/relay2", onoff(ps == 2));
  mqtt.publish("esp-pump/relay3", onoff(relayOn(RLY3)));
  mqtt.publish("esp-pump/relay4", onoff(relayOn(RLY4)));
  mqtt.publish("esp-pump/leak", onoff(leakLatch));   // 漏水报警（自锁状态，非瞬时浸水）
  mqtt.publish("esp-pump/boot", onoff(trig == 0));   // 开机启动：ON=是 OFF=否
  mqtt.publish("esp-pump/t0", String(cfgT0).c_str());
  mqtt.publish("esp-pump/t1", String(cfgT1).c_str());
  mqtt.publish("esp-pump/t2", String(cfgT2).c_str());
  mqtt.publish("esp-pump/t4", String(cfgT4).c_str());
  mqtt.publish("esp-pump/t5", String(cfgT5).c_str());
  mqtt.publish("esp-pump/progress", progressText());   // 运行状态（进度阶段名）
  /* v5.20：给 HA 的两个结果信号，别让自动化去匹配中文文案 */
  mqtt.publish("esp-pump/done", onoff(doneFlag));      // ON=最近一次流程正常跑完
  mqtt.publish("esp-pump/last", lastResult);           // 上次结果/被拒原因（文本）
  mqtt.publish("esp-pump/ip", (WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : String("仅热点")).c_str());
  mqtt.publish("esp-pump/diag", (String(FW_VER) + " disc_fail:" + String(dispubFail)).c_str());
}
void mqttConnect() {
  if (!mqttEn || mqttHost[0] == 0) return;
  mqtt.setServer(mqttHost, mqttPort);
  if (mqtt.connect("esp-pump", mqttUser, mqttPass)) {
    mqtt.subscribe("esp-pump/cmd");
    mqtt.subscribe("esp-pump/relay1/set");
    mqtt.subscribe("esp-pump/relay2/set");
    mqtt.subscribe("esp-pump/relay3/set");
    mqtt.subscribe("esp-pump/relay4/set");
    mqtt.subscribe("esp-pump/boot/set");
    mqtt.subscribe("esp-pump/t0/set");
    mqtt.subscribe("esp-pump/t1/set");
    mqtt.subscribe("esp-pump/t2/set");
    mqtt.subscribe("esp-pump/t4/set");
    mqtt.subscribe("esp-pump/t5/set");
    mqttPubDiscovery();
    mqttPubStates();
  }
}

// 启动一次执行（o=0换水 1排水 2上水）
void startRun(byte o) { startRun(o, "RUN"); }
void startRun(byte o, const char* src) {
  if (leakLatch) {
    manSt = "漏水锁定";
    snprintf(lastResult, sizeof(lastResult), "%s被拒：漏水锁定", objName(o));   // v5.20
    evLog(src, String(objName(o)) + " 启动被拒：漏水锁定");
    return;                                       // 漏水保护期间拒绝一切启动
  }
  /* v5.11 启动预检：启动瞬间若对应中止信号已经处于「到位」状态，不静默跳转，
   *   而是直接拒绝启动并把原因写清楚。
   *   典型踩坑：某路输入没接线，光耦输出侧上拉 → GPIO 恒读 HIGH；
   *   此时把该路拨成「断电时停止」，就会一启动就判"到位"，
   *   现象是"点一下就说完成了"，但根因是接线/极性，不是程序。
   *   预检让这种情况当场暴露，而不是伪装成流程跑完。 */
  bool preHit = (o == 2) ? trigAny(SEN_B, RED_B, senBPol) : trigAny(SEN_A, RED_A, senAPol);
  if (preHit) {
    const char* which = (o == 2) ? "上水位(IN2)" : "下水位(IN1)";
    manSt = "已到位未启动";
    snprintf(lastResult, sizeof(lastResult), "%s被拒：水位已到位", objName(o));   // v5.20
    evLog(src, String(objName(o)) + " 未启动：" + which + "当前已处于到位状态，请检查接线或极性");
    return;
  }
  holdALost = false; holdBLost = false;           // 清掉上一轮的宽限标记
  runObj = o; st = IDLE;
  runSrc = src;                                   // v5.15：记住是谁发起的（网页/定时/按钮/遥控/MQTT/上电）
  /* v5.20：新一轮开始 → 清掉上一轮的完成标志，否则 HA 会一直看到 done=ON */
  doneFlag = false; endHandled = false;
  evLog(src, String("启动 ") + objName(o));
  if (o == 2) { st = VALVE_ON; tmr = millis(); pumpOut(0, 0, 1); }
  else { st = FWD; tmr = millis(); pumpOut(1, 0, 0); }
}
// 手动按钮：中断自动流程
void man(int f, int r, int v) {
  if (leakLatch && (f || r || v)) {               // 锁定期间只能关、不能开
    st = IDLE; pumpOut(0, 0, 0); manSt = "漏水锁定";
    evLog(evSrc, "手动操作被拒：漏水锁定");
    return;
  }
  holdALost = false; holdBLost = false;           // 手动接管：宽限状态一并作废
  /* v5.19：全关时记下「关掉的是什么」，给用户一个明确回执。
   * 以前一律写「待机」，网页只能显示「当前没有任务」—— 点「关阀」后看不出到底关没关，
   * 而且关阀、关泵、全关三种操作在界面上长得一模一样。
   * 必须在 pumpOut 之前读实际输出，之后再读就已经全断了。 */
  bool wasValve = relayOn(RLY3);
  bool wasPump  = (pumpState() != 0);
  st = IDLE; pumpOut(f, r, v);
  doneFlag = false;                               // v5.20：手动接管 → 上一轮的「完成」作废
  if (f) manSt = "正转";
  else if (r) manSt = "反转";
  else if (v) manSt = "阀开";
  else if (wasValve && wasPump) manSt = "全部停止";
  else if (wasValve) manSt = "阀关闭";
  else if (wasPump)  manSt = "泵停止";
  else manSt = "待机";                            // 本来就没开，保持待机，不硬造回执
  /* 注：man() 不碰 K4（pumpOut 会保留原值），所以这里不处理「备用关闭」，
   * 那个回执由 manAux(false) 负责。 */
  /* 只有真正回到「待机」才清空来源；有回执（阀关闭/泵停止…）说明刚有一次手动动作，
   * 来源留着，状态卡才能显示「阀关闭 · 网页」。 */
  runSrc = (strcmp(manSt, "待机") == 0) ? "" : evSrc;
  evLog(evSrc, String("手动 ") + manSt);
}
// 备用继电器 K4：单独开关，不影响泵/阀互斥组合
void manAux(bool a) {
#if LEAK_LOCK_AUX
  if (leakLatch && a) {                       // 锁定期间不允许开备用，但关闭始终允许
    applyRelays(0, 0, 0, 0); manSt = "漏水锁定";
    evLog(evSrc, "备用通电被拒：漏水锁定");
    return;
  }
#endif
  applyRelays(relayOn(RLY1), relayOn(RLY2), relayOn(RLY3), a);
  if (!a) {
    /* v5.19：关备用后，如果别的都没开，就给「备用关闭」的回执（原来写「待机」，
     * 状态卡会变成「当前没有任务」，看不出备用到底关没关）。 */
#if TKS_SINGLE_KEY
    // 单键模式：#1=供电 #2=K1信号。正转=供电+信号都吸合，反转=供电吸合+信号断开
    manSt = (relayOn(RLY1) && relayOn(RLY2)) ? "正转"
          : (relayOn(RLY1) && !relayOn(RLY2)) ? "反转"
          : relayOn(RLY3) ? "阀开" : "备用关闭";
#else
    manSt = relayOn(RLY1) ? "正转" : relayOn(RLY2) ? "反转" : relayOn(RLY3) ? "阀开" : "备用关闭";
#endif
  }
  else manSt = "备用开";
  /* v5.18：开关备用也是一次手动动作，来源要记住。v5.19 起关闭时 manSt 是
   * 「备用关闭」而不是「待机」，所以来源一律保留 —— 状态卡要显示「备用关闭 · 网页」。 */
  runSrc = evSrc;
  evLog(evSrc, a ? String("备用 通电") : (String("备用 断电 → ") + manSt));
}

String doCmd(String x) {
  if (x == "start") { startRun(objGlobal, evSrc); }
  else if (x == "drain") { startRun(1, evSrc); }
  else if (x == "fill") { startRun(2, evSrc); }
  else if (x == "fwd") { man(1, 0, 0); }
  else if (x == "rev") { man(0, 1, 0); }
  else if (x == "stop") { man(0, 0, 0); }
  else if (x == "valve_on") { man(0, 0, 1); }
  else if (x == "valve_off") { man(0, 0, 0); }
  else if (x == "aux_on") { manAux(true); }
  else if (x == "aux_off") { manAux(false); }
  else if (x == "leak_reset") { leakReset(evSrc); }
  else if (x == "trig0" || x == "trig1") { trig = x[4] - '0'; save(); }
  // v5.9：两路中止信号的判定极性。xxxpol1=通电时停止（默认）；xxxpol0=断电时停止
  else if (x == "in1pol0" || x == "in1pol1") {
    byte v = (byte)(x[6] - '0');
    if (v != senAPol) {
      senAPol = v; save();
      evLog(evSrc, v ? "IN1 排水中止改为：通路时停止" : "IN1 排水中止改为：断开时停止");
    }
  }
  else if (x == "in2pol0" || x == "in2pol1") {
    byte v = (byte)(x[6] - '0');
    if (v != senBPol) {
      senBPol = v; save();
      evLog(evSrc, v ? "IN2 上水中止改为：通路时停止" : "IN2 上水中止改为：断开时停止");
    }
  }
  else if (x == "obj0" || x == "obj1" || x == "obj2") { objGlobal = x[3] - '0'; save(); }
  else if (x == "mqtt_resend") { mqttPubDiscovery(); }
  else if (x == "wifi_save") {
    String s = server.arg("ssid"); String p = server.arg("pass");
    if (s.length() == 0) { wifiMode = 0; save(); WiFi.disconnect(); startAP(); apOff = false; }
    else {
      lastTrimBytes = utf8Cpy(ssid, s, sizeof(ssid));
      if (p.length() > 0) lastTrimBytes += utf8Cpy(pass, p, sizeof(pass));   // 密码留空=保留旧密码
      wifiMode = 1; save(); WiFi.disconnect(); WiFi.begin(ssid, pass);
    }
    evLog(evSrc, s.length() ? (String("保存 WiFi：") + ssid) : String("断开家网，仅热点"));
  }
  else if (x == "wifi_off") { wifiMode = 0; save(); WiFi.disconnect(); startAP(); apOff = false; evLog(evSrc, "断开家网，仅热点"); }
  else if (x == "ap_restart") { apOff = false; startAP(); evLog(evSrc, "重开热点"); }   // 不改动任何配置，单纯把热点重新拉起来
  else if (x == "mqtt_save") {
    String h = server.arg("host"); String u = server.arg("user");
    String p = server.arg("pass"); String po = server.arg("port");
    h.trim();
    if (h.length() == 0) { mqttEn = 0; save(); mqtt.disconnect(); evLog(evSrc, "停用 MQTT"); return "MQTT 已停用"; }
    // v5.3m：只接收 IPv4 直填；填域名则在此解析一次后改存 IP。
    // 原因：mqttHost 只有 16 字节（域名会被静默截断成垃圾，正是本次故障的成因之一），
    // 且连接阶段的 DNS 解析会把 loop 堵死十几秒，直接拖垮网页。
    IPAddress ip;
    if (!ip.fromString(h.c_str())) {
      if (WiFi.status() == WL_CONNECTED && WiFi.hostByName(h.c_str(), ip)) h = ip.toString();
      else return "ERR 无法解析 '" + h + "'，请直接填 MQTT 服务器 IP";
    }
    if (h.length() > 15) return "ERR MQTT 服务器地址过长，请用 IP";
    lastTrimBytes  = utf8Cpy(mqttHost, h, sizeof(mqttHost));
    lastTrimBytes += utf8Cpy(mqttUser, u, sizeof(mqttUser));
    if (p.length() > 0) lastTrimBytes += utf8Cpy(mqttPass, p, sizeof(mqttPass));   // 密码留空=保留旧密码
    mqttPort = po.toInt(); if (mqttPort == 0) mqttPort = 1883;
    mqttEn = 1; save(); mqtt.disconnect(); mqttConnect();
    evLog(evSrc, String("保存并启用 MQTT：") + mqttHost + ":" + String(mqttPort));
    return String("MQTT 已保存 ") + mqttHost + ":" + String(mqttPort);
  }
  else if (x == "mqtt_off") { mqttEn = 0; save(); mqtt.disconnect(); evLog(evSrc, "停用 MQTT"); }
  else if (x == "selftest") { httpSelfTest(); return "selftest done (see serial)"; }   // v5.3m：按需 HTTP 自检
  else if (x == "t0up") { if (cfgT0 < 9999) cfgT0++; save(); }
  else if (x == "t0dn") { if (cfgT0 > 0)   cfgT0--; save(); }
  else if (x == "tset") {   // 手动直接设值：/cmd?x=tset&k=t1&v=30
    String k = server.arg("k"); int n = server.arg("v").toInt();
    if (k == "t0") { if (n >= 0 && n <= 9999) { cfgT0 = n; save(); } }
    else if (k == "t1") { if (n >= 1 && n <= 9999) { cfgT1 = n; save(); } }
    else if (k == "t2") { if (n >= 1 && n <= 9999) { cfgT2 = n; save(); } }
    else if (k == "t4") { if (n >= 1 && n <= 9999) { cfgT4 = n; save(); } }
    else if (k == "t5") { if (n >= 1 && n <= 9999) { cfgT5 = n; save(); } }
  }
  else if (x == "t1up") { if (cfgT1 < 9999) cfgT1++; save(); }
  else if (x == "t1dn") { if (cfgT1 > 1)   cfgT1--; save(); }
  else if (x == "t2up") { if (cfgT2 < 9999) cfgT2++; save(); }
  else if (x == "t2dn") { if (cfgT2 > 1)   cfgT2--; save(); }
  else if (x == "t4up") { if (cfgT4 < 9999) cfgT4++; save(); }
  else if (x == "t4dn") { if (cfgT4 > 1)   cfgT4--; save(); }
  else if (x == "t5up") { if (cfgT5 < 9999) cfgT5++; save(); }
  else if (x == "t5dn") { if (cfgT5 > 1)   cfgT5--; save(); }
  else if (x == "plan_add") {
    if (planCount >= MAX_PLAN) return stateText();
    int ty = server.arg("type").toInt(), pm = server.arg("param").toInt();
    int hh = server.arg("h").toInt(), mm = server.arg("m").toInt(), oo = server.arg("obj").toInt();
    if (hh < 0 || hh > 23 || mm < 0 || mm > 59 || ty < 0 || ty > 1) return stateText();
    int i = planCount++;
    planEn[i] = 1; planType[i] = ty; planParam[i] = pm; planHour[i] = hh; planMin[i] = mm; planObj[i] = oo;
    planRef[i] = (ty == 1 && timeSynced) ? dayNum() : 0;   // 隔N天基准日=今天
    save();
    evLog(evSrc, String("新增计划 #") + String(i + 1) + " " + objName(oo));
  }
  else if (x == "plan_del") {
    int i = server.arg("idx").toInt();
    if (i >= 0 && i < planCount) { planShiftDel(i); evLog(evSrc, String("删除计划 #") + String(i + 1)); }
  }
  else if (x == "plan_tog") {
    int i = server.arg("idx").toInt(), en = server.arg("en").toInt();
    if (i >= 0 && i < planCount) {
      planEn[i] = en ? 1 : 0; save();
      evLog(evSrc, String(en ? "启用计划 #" : "停用计划 #") + String(i + 1));
    }
  }
  return stateText();
}

// 包一层：把动作来源标成 MQTT，事件日志里才能区分「网页点的」和「HA 点的」
void mqttCbBody(char* topic, byte* payload, unsigned int len);
void mqttCb(char* topic, byte* payload, unsigned int len) {
  const char* prev = evSrc;
  evSrc = "MQTT";
  mqttCbBody(topic, payload, len);
  evSrc = prev;
}

void mqttCbBody(char* topic, byte* payload, unsigned int len) {
  String t = String(topic); String v = "";
  for (unsigned int i = 0; i < len; i++) v += (char)payload[i];
  if (t == "esp-pump/cmd") { doCmd(v); return; }
  if (t == "esp-pump/relay1/set") {
    // 泵正转：ON=正转，OFF=停。注意单键模式下不能读 relayOn(RLY2) 作"反转"，
    // 因为正转时 #2(信号) 也吸合，读它会把"关正转"误判成"反转"。
    int vl = relayOn(RLY3);
    if (v == "ON") man(1, 0, vl);
    else man(0, 0, vl);
    mqttPubStates();   // 立即回报真实状态，避免 HA 乐观更新 2 秒后弹回 OFF
    return;
  }
  if (t == "esp-pump/relay2/set") {
    // 泵反转：ON=反转，OFF=停。
    int vl = relayOn(RLY3);
    if (v == "ON") man(0, 1, vl);
    else man(0, 0, vl);
    mqttPubStates();
    return;
  }
  if (t == "esp-pump/relay3/set") {
    int f = pumpState() == 1 ? 1 : 0, r = pumpState() == 2 ? 1 : 0;
    man(f, r, v == "ON");
    mqttPubStates();
    return;
  }
  if (t == "esp-pump/relay4/set") {
    manAux(v == "ON");
    mqttPubStates();
    return;
  }
  if (t == "esp-pump/boot/set") {
    trig = (v == "ON") ? 0 : 1;   // 开机启动：是=上电自动执行1次，否=上电待机
    save(); return;
  }
  if (t == "esp-pump/t0/set") { int n = v.toInt(); if (n >= 0 && n <= 9999) { cfgT0 = n; save(); } return; }
  if (t == "esp-pump/t1/set") { int n = v.toInt(); if (n >= 1 && n <= 9999) { cfgT1 = n; save(); } return; }
  if (t == "esp-pump/t2/set") { int n = v.toInt(); if (n >= 1 && n <= 9999) { cfgT2 = n; save(); } return; }
  if (t == "esp-pump/t4/set") { int n = v.toInt(); if (n >= 1 && n <= 9999) { cfgT4 = n; save(); } return; }
  if (t == "esp-pump/t5/set") { int n = v.toInt(); if (n >= 1 && n <= 9999) { cfgT5 = n; save(); } return; }
}

void handleRoot() {
  httpRootN++; lastHttpMs = millis();
  const char* html = R"rawliteral(
<!DOCTYPE html><html lang="zh"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>泵控</title><style>
:root{--bg:#eef2f7;--card:#fff;--ink:#0f172a;--muted:#64748b;--line:#e2e8f0;--blue:#2563eb;--green:#16a34a;--red:#dc2626;--amber:#d97706;--slate:#475569;--cyan:#0891b2}
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:-apple-system,"Segoe UI","PingFang SC","Microsoft YaHei",sans-serif;background:linear-gradient(180deg,#eef2f7,#e6edf5);color:var(--ink);min-height:100vh;padding:14px 12px 40px}
.wrap{max-width:480px;margin:0 auto}
header{text-align:center;margin-bottom:12px}
/* v5.21：版本号挪到标题右侧。baseline 对齐 —— 数字没有下伸部，看起来就是「底边与标题齐平」；
 * 用 flex-end 反而会因为 h1 行盒的 descender 而显得偏低。 */
header h1{font-size:20px;display:flex;align-items:baseline;justify-content:center;gap:6px}
header h1 .hver{font-size:12px;font-weight:500;color:var(--muted)}
header .sub{font-size:12px;color:var(--muted);margin-top:3px}
.card{background:var(--card);border-radius:14px;padding:14px;margin:10px 0;box-shadow:0 1px 3px rgba(15,23,42,.08),0 8px 24px rgba(15,23,42,.05)}
/* v5.8：顶部页签。原来 13 张卡片全堆一页，手机上要滚好几屏才能摸到按钮。
 * 改成 4 页切换后，控制页只剩「状态/流程/手动」，一屏基本看完。
 * 显示规则用 body[data-tab] 配合卡片上的 data-pane 属性做，不改 DOM 顺序，
 * 所以卡片在各自页内仍按源码里的先后排列，不用手工搬来搬去。
 * 注意：.tabbar/.tb 是新类名，避免和定时计划那套 .tabs/.tab 撞车。 */
.tabbar{display:flex;gap:6px;margin:2px 0 10px;position:sticky;top:0;z-index:30;padding:8px 0;background:linear-gradient(180deg,#eef2f7 74%,rgba(238,242,247,0))}
.tabbar .tb{flex:1;text-align:center;padding:9px 2px;border-radius:10px;background:#fff;border:1px solid var(--line);font-size:13px;font-weight:600;color:var(--muted);cursor:pointer;transition:background .15s,color .15s;box-shadow:0 1px 2px rgba(15,23,42,.06);position:relative}
/* 漏水锁定后「控制」页签打红点：否则用户停在设置页会看不到警告 */
.tabbar .tb.warn::after{content:"";position:absolute;top:3px;right:5px;width:7px;height:7px;border-radius:50%;background:#dc2626;box-shadow:0 0 0 2px #fff}
.tabbar .tb.on{background:var(--blue);border-color:var(--blue);color:#fff;box-shadow:0 2px 8px rgba(37,99,235,.35)}
.tabbar .tb:active{filter:brightness(.95)}
body[data-tab="0"] .card:not([data-pane="0"]),
body[data-tab="1"] .card:not([data-pane="1"]),
body[data-tab="2"] .card:not([data-pane="2"]),
body[data-tab="3"] .card:not([data-pane="3"]){display:none}
.card h3{font-size:14px;color:#334155;display:flex;align-items:center;gap:7px;margin-bottom:10px}
.card h3::before{content:"";width:4px;height:14px;background:var(--blue);border-radius:2px}
.cardbadge{display:inline-flex;align-items:center;gap:5px;font-size:11px;font-weight:600;padding:2px 9px;border-radius:999px;margin-left:auto;white-space:nowrap}
.cardbadge i{width:7px;height:7px;border-radius:50%;background:#94a3b8;flex:none}
.cardbadge.ok{background:#dcfce7;color:#166534}.cardbadge.ok i{background:#16a34a}
.cardbadge.warn{background:#fef9c3;color:#854d0e}.cardbadge.warn i{background:#eab308}
.cardbadge.err{background:#fee2e2;color:#991b1b}.cardbadge.err i{background:#dc2626}
.cardbadge.off{background:#f1f5f9;color:#64748b}.cardbadge.off i{background:#94a3b8}
.status{border-radius:12px;padding:16px 14px;color:#fff;transition:background .2s}
.status.idle{background:#475569}
.status.run{background:linear-gradient(135deg,#16a34a,#0d9488)}
.status.warn{background:linear-gradient(135deg,#d97706,#dc2626)}
.status .ph{font-size:26px;font-weight:800;letter-spacing:1px}
.status .pg{font-size:13px;opacity:.9;margin-top:4px}
.dots{display:grid;grid-template-columns:repeat(4,1fr);gap:8px;margin-top:12px}
.dot{background:rgba(255,255,255,.16);border-radius:9px;padding:8px 6px;text-align:center;font-size:12px}
.dot b{display:block;width:9px;height:9px;border-radius:50%;margin:0 auto 4px;background:rgba(255,255,255,.35)}
.dot.on b{background:#fbbf24;box-shadow:0 0 6px #fbbf24}
.meta{font-size:11px;opacity:.75;margin-top:9px}
.btn{border:none;border-radius:10px;padding:12px 8px;font-size:15px;color:#fff;cursor:pointer}
.row{display:flex;gap:8px;flex-wrap:wrap}.row .btn{flex:1 1 0;min-width:0}
.g{background:var(--green)}.r{background:var(--red)}.b{background:var(--blue)}.y{background:var(--amber)}.c{background:var(--cyan)}
.btn.act{outline:3px solid #fde047;outline-offset:2px;filter:saturate(1.15) brightness(1.06);box-shadow:0 0 14px rgba(253,224,71,.55)}
.objrow{display:flex;align-items:center;gap:10px;background:#f8fafc;border:1px solid var(--line);border-radius:10px;padding:8px 10px;margin:6px 0}
.flowrow{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin:8px 0}
.fbtn{position:relative;height:72px;border:none;border-radius:11px;cursor:pointer;display:flex;align-items:center;justify-content:center;gap:3px;font-size:22px;color:#fff;transition:filter .15s}
.fbtn:active{filter:brightness(.9)}
.fbtn .nm{font-size:12.5px;font-weight:700}
.fbtn .tip{position:absolute;bottom:78px;left:50%;transform:translateX(-50%);white-space:nowrap;background:#0f172a;color:#dbeafe;padding:6px 9px;border-radius:8px;font-size:11px;opacity:0;transition:opacity .15s;pointer-events:none;z-index:5}
.fbtn:hover .tip,.fbtn:active .tip{opacity:1}
.f-cw{background:var(--cyan)}.f-dr{background:var(--blue)}.f-fl{background:var(--green)}
.fbtn.act{outline:3px solid #fde047;outline-offset:2px;filter:saturate(1.2) brightness(1.12);box-shadow:0 0 14px rgba(253,224,71,.55)}
.objrow .nm{flex:1;font-size:14px;color:#334155;font-weight:600}
.objrow .nm small{display:block;color:var(--muted);font-size:11px;font-weight:400;margin-top:1px}
.objrow .go{background:var(--cyan);font-size:14px;padding:11px 20px;border-radius:9px;flex:none}
.seg{display:flex;gap:6px}
.seg .btn{flex:1;background:#eef2f7;color:#334155;font-size:14px;padding:10px 4px;border:1px solid var(--line)}
.seg .btn.on{background:var(--blue);color:#fff;border-color:var(--blue)}
.dim{opacity:.4;pointer-events:none}
.opt{display:flex;align-items:center;gap:9px;background:#f8fafc;border:1.5px solid var(--line);border-radius:10px;padding:9px 10px;margin:6px 0;cursor:pointer;font-size:13px;color:#334155}
.opt input{flex:none;width:16px;height:16px;margin:0;accent-color:var(--blue)}
.opt span{flex:1;min-width:0}
.opt small{display:block;color:var(--muted);font-size:11.5px;margin-top:2px;line-height:1.45}
.opt.on{border-color:var(--blue);background:#eff6ff}
.bootrow{display:flex;align-items:center;gap:12px}
.bootrow .txt{flex:1}
.bootrow .txt b{font-size:15px}
.bootrow .txt small{display:block;color:var(--muted);font-size:11.5px;margin-top:2px}
.stepper{display:flex;align-items:center;gap:8px;margin:7px 0}
.stepper .lab{width:122px;font-size:13px;color:#334155}
.stepper .val{flex:1;text-align:center;font-family:Consolas,monospace;font-size:17px;font-weight:600;color:var(--blue)}
.stepper input.val{min-width:0;width:92px;flex:none;height:34px;border:1px solid var(--line);border-radius:8px;background:#fff;color:var(--blue);text-align:center;font-family:Consolas,monospace;font-size:16px;font-weight:600;outline:none}
.stepper input.val:focus{border-color:var(--blue);box-shadow:0 0 0 2px rgba(59,130,246,.15)}
.stepper button{width:44px;height:34px;border:none;border-radius:8px;background:var(--amber);color:#fff;font-size:18px;cursor:pointer}
input{width:100%;height:40px;font-size:14px;border:1px solid var(--line);border-radius:9px;padding:0 10px;margin:4px 0}
.mini{display:flex;gap:6px}.mini input{width:31%}.mini .btn{flex:1;padding:11px;font-size:14px}
.scanrow{display:flex;gap:6px}.scanrow input{flex:1}.scanrow .btn{flex:none;padding:11px 14px;font-size:13px}
.scanlist{margin:8px 0;border:1px solid var(--line);border-radius:10px;overflow:hidden}
.scanlist .si{display:flex;align-items:center;gap:8px;padding:9px 12px;border-bottom:1px solid #eef2f7;cursor:pointer;font-size:13px}
.scanlist .si:last-child{border-bottom:0}
.scanlist .si:hover{background:#f8fafc}
.scanlist .si .nm{flex:1;color:#1e293b;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.scanlist .si .db{color:#94a3b8;font-size:11px;flex:none}
.scanlist .si .lock{color:#94a3b8;flex:none;font-size:11px}
.dow{display:flex;gap:5px;margin:7px 0}
.tabs{display:flex;gap:4px;margin-bottom:-1px;position:relative;z-index:1}
.tabs .tab{flex:1;padding:10px 4px;text-align:center;font-size:14px;font-weight:600;color:#64748b;border:1.5px solid #cbd5e1;border-bottom:none;border-radius:10px 10px 0 0;background:#cbd5e1;cursor:pointer;transition:background .15s,color .15s}
.tabs .tab.on{background:#f1f5f9;color:#1e293b;border-color:#cbd5e1;font-weight:700;position:relative;margin-bottom:-1px;padding-bottom:11px;z-index:2}
.tabs .tab.on::after{content:"";position:absolute;left:0;right:0;bottom:-1px;height:2px;background:#f1f5f9}
.tabbody{display:none}.tabbody.on{display:block;height:50px}
.subcard{border:1.5px solid #cbd5e1;border-radius:0 0 12px 12px;padding:12px;margin-top:0;background:#f1f5f9;height:215px;overflow:hidden}
.dow button{flex:1;height:36px;border:1px solid var(--line);background:#f8fafc;color:#334155;border-radius:9px;font-size:13px;cursor:pointer}
.dow button.on{background:var(--blue);color:#fff;border-color:var(--blue)}
.lbl{font-size:12.5px;color:#334155;margin:10px 0 5px}
/* v5.21：极窄屏（≤340px）下水位状态行会折行，中文默认任意断字，会把「未触发」劈成「未触/发」。
 * 把「· 已触发/未触发」整体包进 .nw，只在它前面断行，不拆词。 */
.nw{white-space:nowrap}
.tip{font-size:11.5px;color:var(--muted);margin-top:8px;line-height:1.6}
/* 信息按钮（卡片标题右侧，点开说明悬浮层） */
.infobtn{margin-left:auto;width:22px;height:22px;border-radius:50%;border:1px solid var(--line);background:#f8fafc;color:var(--blue);font-size:13px;font-weight:700;line-height:1;cursor:pointer;flex:none;display:inline-flex;align-items:center;justify-content:center}
.infobtn:active{background:#eef2f7}
/* 说明悬浮层 */
.ovl{position:fixed;inset:0;background:rgba(15,23,42,.45);z-index:100;display:none;align-items:center;justify-content:center;padding:20px;overflow:auto}
.ovl.show{display:flex}
/* v5.18：悬浮层内容变长（运行与触发 + 水位判定两节），小屏要能滚，否则底部按钮点不到 */
.pop{background:#fff;border-radius:14px;max-width:92%;width:420px;padding:18px 18px 14px;box-shadow:0 10px 40px rgba(15,23,42,.3);max-height:calc(100vh - 40px);overflow:auto;-webkit-overflow-scrolling:touch}
.pop h4{font-size:15px;color:#0f172a;margin:0 0 10px;display:flex;align-items:center;gap:7px}
.pop h4.mt{margin-top:16px;padding-top:14px;border-top:1px solid var(--line)}   /* 悬浮层里第二个小节 */
.pop .pbody{font-size:12.5px;color:#334155;line-height:1.7}
.pop .pbody p{margin:0 0 10px}
.pop .pbody b{color:#0f172a}
.pop .pclose{width:100%;margin-top:6px}
/* BOOT 键功能表（网络设置卡片底部） */
.kbt{width:100%;border-collapse:collapse;margin-top:10px;font-size:11.5px;line-height:1.5}
.kbt th{font-weight:600;color:var(--muted);text-align:left;padding:4px 6px;background:#f8fafc;border-bottom:1px solid var(--line)}
.kbt td{padding:6px;border-bottom:1px solid var(--line);vertical-align:top;color:#334155}
.kbt tr:last-child td{border-bottom:0}
.kbt td:first-child{white-space:nowrap;font-weight:600;color:var(--ink);width:66px}
.kbt .fn{color:#0f172a;font-weight:600}
/* 事件日志（v5.6）：最近动作列表 */
.evlist{max-height:230px;overflow-y:auto;background:#f8fafc;border:1px solid var(--line);border-radius:10px;padding:6px 8px}
.evrow{display:flex;gap:8px;align-items:flex-start;padding:4px 0;border-bottom:1px dashed #e2e8f0;font-size:12px;line-height:1.5}
.evrow:last-child{border-bottom:0}
.evt{flex:none;width:56px;color:var(--muted);font-variant-numeric:tabular-nums;font-size:11px;padding-top:1px}
.evx{flex:1;color:#334155;word-break:break-all}
.evx b{font-weight:600;color:#0f172a}
/* 固件升级（v5.7） */
.fwbox{background:#f8fafc;border:1px solid var(--line);border-radius:10px;padding:10px}
.fwrow{display:flex;justify-content:space-between;align-items:center;font-size:12px;color:var(--muted)}
.fwrow b{color:var(--ink);font-weight:600}
.fwbar{height:6px;background:#e2e8f0;border-radius:3px;overflow:hidden;margin:8px 0 0}
.fwbar i{display:block;height:100%;width:0;background:#2563eb;transition:width .2s}
.plan{display:flex;align-items:center;gap:9px;background:#f8fafc;border:1px solid var(--line);border-radius:10px;padding:9px 10px;margin:6px 0}
.plan .txt{flex:1;font-size:13px;color:#334155}
.plan .txt small{display:block;color:var(--muted);font-size:11px;margin-top:1px}
.plan .del{width:30px;height:30px;border:none;border-radius:8px;background:#fee2e2;color:var(--red);font-size:16px;cursor:pointer;flex:none}
.sw{width:44px;height:24px;border-radius:20px;background:#cbd5e1;position:relative;cursor:pointer;flex:none}
.sw::after{content:"";position:absolute;top:2px;left:2px;width:20px;height:20px;border-radius:50%;background:#fff;transition:left .15s;box-shadow:0 1px 2px rgba(0,0,0,.25)}
.sw.on{background:var(--green)}.sw.on::after{left:22px}
#toast{position:fixed;left:50%;bottom:26px;transform:translateX(-50%) translateY(16px);background:#0f172a;color:#dbeafe;font-size:13px;padding:9px 16px;border-radius:10px;opacity:0;transition:opacity .25s,transform .25s;pointer-events:none;z-index:99;max-width:88%;text-align:center;box-shadow:0 4px 14px rgba(15,23,42,.25)}
#toast.show{opacity:1;transform:translateX(-50%) translateY(0)}
</style></head><body data-tab="0">
<div id="toast"></div>
<div class="wrap"><header><h1>换水控制器<span class="hver" id="hdrVer">v-</span></h1>
<div class="sub">单次执行 · 不循环</div></header>
<div class="tabbar">
<div class="tb on" onclick="setTab(0)">控制</div><div class="tb" onclick="setTab(1)">设置</div><div class="tb" onclick="setTab(2)">记录</div><div class="tb" onclick="setTab(3)">系统</div>
</div>
<div class="card" data-pane="1"><h3>开机启动</h3>
<div class="bootrow"><div class="txt"><b id="bootTxt">关闭</b><small id="bootSub">上电后保持待机，不会自动执行</small></div>
<div class="sw" id="bootSw" onclick="cmdT('trig0','开机启动已开启')"></div></div>
<p class="tip">定时计划有启用即生效；MQTT / 网页按钮 / 外接遥控随时可用。</p></div>
<div class="card" data-pane="1"><h3>水位输入判定<button class="infobtn" onclick="showInfo('infopop')" title="查看说明">?</button></h3>
<div class="bootrow"><div class="txt"><b>排水中止信号（IN1）</b><small id="in1Now">正在读取…</small></div>
<div class="sw on" id="in1Sw"></div></div>
<div class="bootrow" style="margin-top:16px"><div class="txt"><b>上水中止信号（IN2）</b><small id="in2Now">正在读取…</small></div>
<div class="sw on" id="in2Sw"></div></div></div>
<div class="card" data-pane="0"><h3>运行状态</h3>
<div class="status idle" id="status"><div class="ph" id="phase">待机</div><div class="pg" id="progress">当前没有任务</div>
<div class="dots">
<div class="dot" id="d1"><b></b>K1 泵正转</div><div class="dot" id="d2"><b></b>K2 泵反转</div>
<div class="dot" id="d3"><b></b>K3 电磁阀</div><div class="dot" id="d4"><b></b>K4 备用</div></div>
<div class="meta" id="clock">--</div></div></div>
<div class="card" data-pane="0" id="leakCard" style="display:none"><h3>漏水保护</h3>
<div class="status warn"><div class="ph">检测到浸水 · 已锁定</div><div class="pg" id="leakTip">已强制关闭全部输出</div></div>
<div class="row" style="margin-top:8px"><button class="btn r" onclick="cmd('leak_reset')">复位并解除锁定</button></div>
<p class="tip">传感器仍浸水时复位会立即再次触发；复位前请先到现场排除漏水。也可长按板载 BOOT 键或遥控键复位。</p></div>
<div class="card" data-pane="0"><h3>流程执行</h3>
<div class="flowrow">
<button class="fbtn f-cw" id="fb-cw" onclick="flowToggle(0,'start')">&#8635;<span class="nm">换水</span><span class="tip">正转→下水位/T4→反转T1→停→阀开→上水位/T5→关阀</span></button>
<button class="fbtn f-dr" id="fb-dr" onclick="flowToggle(1,'drain')">&#8595;<span class="nm">排水</span><span class="tip">正转→下水位/T4→反转T1→停</span></button>
<button class="fbtn f-fl" id="fb-fl" onclick="flowToggle(2,'fill')">&#8593;<span class="nm">上水</span><span class="tip">阀开→上水位/T5→关阀</span></button></div>
<p class="tip">点流程开始；再次点击正在运行的流程即停止。</p></div>
<div class="card" data-pane="0"><h3>手动控制</h3>
<div class="row"><button class="btn g" id="hb-fwd" onclick="ptg('fwd')">泵 正转</button>
<button class="btn b" id="hb-rev" onclick="ptg('rev')">泵 反转</button></div>
<div class="row" style="margin-top:6px"><button class="btn b" id="hb-valve" onclick="vtg()">阀 打开</button></div>
<div class="row" style="margin-top:6px"><button class="btn y" id="hb-aux" onclick="atg()">备用 通电</button></div>
<p class="tip">再点一次运行中的按钮即停止；手动操作会中断自动流程。</p></div>
<div class="card" data-pane="1"><h3>时间参数</h3>
<div class="stepper"><span class="lab">T0 上电延时（秒）</span><button onclick="cmd('t0dn')">−</button><input class="val" id="v0" type="number" inputmode="numeric" data-k="t0" onchange="tset(this)"><button onclick="cmd('t0up')">+</button></div>
<div class="stepper"><span class="lab">T1 泵反转（秒）</span><button onclick="cmd('t1dn')">−</button><input class="val" id="v1" type="number" inputmode="numeric" data-k="t1" onchange="tset(this)"><button onclick="cmd('t1up')">+</button></div>
<div class="stepper"><span class="lab">T2 泵停→阀开（秒）</span><button onclick="cmd('t2dn')">−</button><input class="val" id="v2" type="number" inputmode="numeric" data-k="t2" onchange="tset(this)"><button onclick="cmd('t2up')">+</button></div>
<div class="stepper"><span class="lab">T4 泵最长运行（秒）</span><button onclick="cmd('t4dn')">−</button><input class="val" id="v4" type="number" inputmode="numeric" data-k="t4" onchange="tset(this)"><button onclick="cmd('t4up')">+</button></div>
<div class="stepper"><span class="lab">T5 阀最长运行（秒）</span><button onclick="cmd('t5dn')">−</button><input class="val" id="v5" type="number" inputmode="numeric" data-k="t5" onchange="tset(this)"><button onclick="cmd('t5up')">+</button></div>
<p class="tip">T4：泵正转最长，下水位没来超时也反转T1再停；T5：阀开最长，上水位没来强制关阀。</p></div>

<div class="card" data-pane="1"><h3>定时计划</h3>
<div class="lbl">定时计划（有启用即生效，最多 10 条；需连网校时）</div>
<div id="plans"></div>
<div class="lbl" style="margin-top:14px">新增计划 · 重复方式（二选一）</div>
<div class="tabs" id="ptabs">
<div class="tab on" data-p="0">按周重复</div><div class="tab" data-p="1">隔N天</div></div>
<div class="subcard">
<div class="tabbody on" id="pweek"><div class="dow">
<button>一</button><button>二</button><button>三</button><button>四</button><button>五</button><button>六</button><button>日</button></div></div>
<div class="tabbody" id="pdays"><div class="stepper">
<span class="lab">隔 N 天（N=）</span><button id="ndn">−</button><span class="val" id="vn">2</span><button id="nup">+</button></div></div>
<div class="lbl" style="margin-top:12px">执行内容 & 时间</div>
<div class="seg" id="poseg">
<button class="btn on" data-o="0">换水</button><button class="btn" data-o="1">排水</button><button class="btn" data-o="2">上水</button></div>
<div class="mini" style="margin-top:8px">
<input id="ph" placeholder="时(0-23)" value="08"><input id="pm" placeholder="分(0-59)" value="00">
<button class="btn g" onclick="addPlan()">+ 添加计划</button></div>
</div>
<p class="tip">按周每周重复；隔N天保存当天算第1次。均需连网校时。</p></div>
<div class="card" data-pane="3"><h3>网络设置<span class="cardbadge" id="wifiBadge"><i></i>--</span></h3>
<div class="scanrow"><input id="ssid" placeholder="家庭WiFi名称" list="histSsid"><button class="btn b" id="scanBtn" onclick="wifiScan()">扫描</button></div>
<div id="scanlist" style="display:none"></div>
<datalist id="histSsid"></datalist><input id="pass" type="password" placeholder="WiFi密码（已设置则留空不改）">
<div class="row" style="margin-top:8px"><button class="btn b" onclick="wifiSave()">保存并连接 WiFi</button><button class="btn r" onclick="cmdT('wifi_off','已断开，仅热点')">断开（打开热点）</button></div>
<p class="tip">保存后板子会短暂断连再重连。改了 SSID 后旧 IP 会失效，用 <b>esp-pump.local</b> 访问更稳。</p></div>
<div class="card" data-pane="3"><h3>MQTT 设置<span class="cardbadge" id="mqttBadge"><i></i>--</span></h3>
<input id="mhost" placeholder="MQTT服务器IP" list="histMhost"><datalist id="histMhost"></datalist><input id="mport" placeholder="端口（默认1883）">
<input id="muser" placeholder="MQTT用户名"><input id="mpass" type="password" placeholder="MQTT密码（已设置则留空不改）">
<div class="row" style="margin-top:8px"><button class="btn b" onclick="mqttSave()">保存并启用</button><button class="btn r" onclick="cmdT('mqtt_off','已停用 MQTT')">停用</button><button class="btn y" onclick="cmdT('mqtt_resend','已重新发布 MQTT 配置')">重发配置</button></div></div>
<div class="card" data-pane="2"><h3>最近动作<span class="cardbadge off" id="evBadge"><i></i>0 条</span></h3>
<div class="evlist" id="evlist"><div class="tip" style="margin:2px 0">暂无记录</div></div>
<div class="row" style="margin-top:8px"><button class="btn r" onclick="evClear()">清空记录</button></div>
<p class="tip">按时间倒序显示网页 / MQTT / BOOT 键 / 遥控 / 定时计划 / Modbus 触发的动作，最多 32 条。只存在内存，重启后清空。</p></div>
<div class="card" data-pane="2"><h3>BOOT 键功能</h3>
<p class="tip" style="margin-top:0"><b>先认键</b>：板子底边、<b>Type-C 口旁边并排两颗小按钮</b>。<b>左边那颗（丝印 IO0）是 BOOT</b> —— 就是本表讲的键；右边那颗（丝印 EN）是 RESET，纯硬件复位，固件不接管，按一下等于断电重启。两颗按钮靠得很近，别按错。</p>
<table class="kbt">
<tr><th>按住 BOOT（左边 IO0）</th><th>功能</th></tr>
<tr><td>&lt; 3 秒</td><td><span class="fn">开始一次换水</span></td></tr>
<tr><td>3 秒</td><td><span class="fn">急停</span> · 断开全部输出，并解除漏水锁定<br><span style="color:#64748b">状态灯中速闪</span></td></tr>
<tr><td>6 秒</td><td><span class="fn">松手重启</span> · 保留 WiFi、计划、MQTT 全部配置<br><span style="color:#64748b">状态灯快速闪</span></td></tr>
<tr><td>10 秒</td><td><span class="fn">恢复出厂</span> · 清空 WiFi 配置，重开热点 FS_TWCC 并重启<br><span style="color:#64748b">状态灯急闪 6 下（计划与 MQTT 保留）</span></td></tr>
</table>
<p class="tip">先正常上电再按住。<b>切勿按住 BOOT 上电</b>，那样会进入下载模式。清空 WiFi 后，手机连 <b>FS_TWCC</b> 重新配网。</p></div>
<div class="card" data-pane="3"><h3>固件升级<span class="cardbadge off" id="fwBadge"><i></i>v-</span></h3>
<div class="fwbox">
<div class="fwrow"><span>当前版本</span><b id="fwCur">-</b></div>
<div class="fwrow"><span>硬件型号</span><b id="fwHw">-</b></div>
<div class="fwbar"><i id="fwbar"></i></div>
<input type="file" id="fwfile" accept=".bin" style="width:100%;font-size:12px;margin:8px 0 0">
<button class="btn b" id="fwbtn" style="width:100%;margin-top:8px" onclick="fwUpload()">上传并升级</button>
</div>
<div class="lbl" style="margin-top:14px">或从网址升级</div>
<input id="fwurl" placeholder="http://192.168.5.10/wtcc.bin" autocomplete="off" spellcheck="false">
<button class="btn b" id="fwurlbtn" style="width:100%;margin-top:6px" onclick="fwFromUrl()">从网址下载并升级</button>
<p class="tip">选 Arduino「项目 → 导出已编译的二进制文件」生成的 <b>.bin</b>（约 1MB）。升级前会自动停止全部输出，写完板子自动重启，约 30~60 秒。<b>升级过程中不要断电、不要开泵。</b>
网址支持 http 和 https；https 不校验证书（家庭内网够用，别在不可信网络上用）。下载期间页面会短暂失联，属正常。<br>
写入前会校验这个 .bin 是不是 <b>ESP32</b> 固件（看镜像头第 12 字节的芯片号），还要核对 .bin 里的<b>板卡标记</b>与上面「硬件型号」是否一致 —— 给别的板子编的固件（哪怕也是 ESP32）同样拒绝写入。<b>不比对软件版本号</b> —— 升级、降级、同版本重刷都可以。</p></div>
<script>
var N=2,pobj=0,ptype=0;
/* v5.8 页签：控制 / 设置 / 记录 / 系统。
 * 显示与否由 body[data-tab] 配合卡片的 data-pane 属性决定（见 <style> 里的规则），
 * 不搬 DOM，所以每页内部的卡片顺序还是源码里的先后。
 * 停留页记进 localStorage，下次打开手机浏览器还停在原页 —— 日常只看控制页的人不会被弹走。 */
function setTab(n){
  if(n<0||n>3)n=0;
  document.body.setAttribute('data-tab',n);
  var bs=document.querySelectorAll('.tabbar .tb');
  for(var i=0;i<bs.length;i++)bs[i].classList.toggle('on',i===n);
  try{localStorage.setItem('wtcc_tab',String(n))}catch(e){}
}
(function(){
  var n=0;
  try{n=parseInt(localStorage.getItem('wtcc_tab')||'0',10)}catch(e){}
  if(isNaN(n)||n<0||n>3)n=0;
  setTab(n);
})();
function histAdd(k,v){if(!v)return;var a=JSON.parse(localStorage.getItem(k)||'[]');a=a.filter(function(x){return x!=v});a.unshift(v);localStorage.setItem(k,JSON.stringify(a.slice(0,8)))}
function histFill(k,id){var a=JSON.parse(localStorage.getItem(k)||'[]'),d=document.getElementById(id);d.innerHTML='';a.forEach(function(x){var o=document.createElement('option');o.value=x;d.appendChild(o)})}
// v5.3k：命令返回后立即重载配置，T0~T5 数字当场变化，不再等下一次 5 秒轮询
function cmd(x){fetch('/cmd?x='+x).then(r=>r.text()).then(function(s){upd(s);loadCfg();})}
function toast(m){var t=document.getElementById('toast');t.textContent=m;t.classList.add('show');clearTimeout(toast.t);toast.t=setTimeout(function(){t.classList.remove('show')},3000)}
// 说明悬浮层：点信息按钮显示，点遮罩/按钮关闭
function showInfo(id){var o=document.getElementById(id);if(o)o.classList.add('show')}
function hideInfo(id){var o=document.getElementById(id);if(o)o.classList.remove('show')}
// 秒数手动输入：失焦/回车时把值发回固件（/cmd?x=tset&k=t1&v=30）
function tset(el){var k=el.getAttribute('data-k'),v=parseInt(el.value);if(isNaN(v)){loadCfg();return;}
  fetch('/cmd?x=tset&k='+k+'&v='+v).then(r=>r.text()).then(function(s){upd(s);loadCfg();});}
function wifiSave(){var s=document.getElementById('ssid').value;histAdd('hist_ssid',s);toast('正在保存并连接 WiFi…');
  fetch('/cmd?x=wifi_save&ssid='+encodeURIComponent(s)+'&pass='+encodeURIComponent(document.getElementById('pass').value))
    .then(r=>r.text()).then(function(x){upd(x);toast(s?'WiFi 已保存，正在连接家网':'已断开，仅热点')})
    .catch(function(){toast('WiFi 保存中，连接会短暂断开，稍后刷新页面')})}
// WiFi 扫描：点「扫描」列出附近 WiFi，点选自动填入 SSID 输入框
function wifiScan(){var b=document.getElementById('scanBtn'),l=document.getElementById('scanlist');
  b.textContent='扫描中…';b.disabled=true;l.style.display='none';
  fetch('/wifi_scan').then(r=>r.json()).then(function(list){
    b.textContent='扫描';b.disabled=false;
    if(!list.length){l.innerHTML='<div class="tip" style="margin:6px 2px">未发现 WiFi</div>';l.style.display='';return;}
    var h='<div class="scanlist">';
    list.forEach(function(w){h+='<div class="si" onclick="pickWifi(this)" data-ssid="'+w.ssid.replace(/"/g,'&quot;')+'"><span class="nm">'+w.ssid+'</span>'+(w.enc?'<span class="lock">🔒</span>':'')+'<span class="db">'+w.rssi+'dBm</span></div>';});
    h+='</div>';
    l.innerHTML=h;l.style.display='';
  }).catch(function(){b.textContent='扫描';b.disabled=false;toast('扫描失败，请重试');});}
function pickWifi(el){document.getElementById('ssid').value=el.getAttribute('data-ssid');
  document.getElementById('scanlist').style.display='none';
  toast('已选 WiFi：'+el.getAttribute('data-ssid')+'，请填密码后保存');}
function mqttSave(){var h=document.getElementById('mhost').value;histAdd('hist_mhost',h);toast('正在保存并启用 MQTT…');
  fetch('/cmd?x=mqtt_save&host='+encodeURIComponent(h)+'&port='+encodeURIComponent(document.getElementById('mport').value)+'&user='+encodeURIComponent(document.getElementById('muser').value)+'&pass='+encodeURIComponent(document.getElementById('mpass').value))
    .then(r=>r.text()).then(function(x){upd(x);toast(h?'MQTT 已保存并启用':'已停用 MQTT')})
    .catch(function(){toast('MQTT 保存失败，请重试')})}
function cmdT(x,m){cmd(x);toast(m)}
var cur={r1:0,r2:0,r3:0,r4:0,pump:0,obj:0,running:false};
function ptg(d){if((d=='fwd'&&cur.pump==1)||(d=='rev'&&cur.pump==2))cmd('stop');else cmd(d)}
// 流程按钮：运行中点击=停止；未运行点击=启动
function flowToggle(obj,startCmd){if(cur.running&&cur.obj==obj){cmd('stop');}else{cmd(startCmd);}}
function vtg(){cmd(cur.r3?'valve_off':'valve_on')}
function atg(){cmd(cur.r4?'aux_off':'aux_on')}
function setSw(id,on){var e=document.getElementById(id);if(e)e.classList.toggle('on',!!on)}
function setSeg(id,on){var b=document.getElementById(id);if(b)b.classList.toggle('on',!!on)}
function upd(s){
  if(typeof s=='string'){try{s=JSON.parse(s)}catch(e){return}}
  /* hasOut = 真的有输出或自动流程在跑 → 决定卡片颜色（绿）。
   * 手动关掉设备后全无输出，卡片应该回灰，但小字仍要显示「阀关闭 · 网页」，
   * 所以「要不要挂来源」用另一个条件 showSrc 判断。 */
  var hasOut=s.r1||s.r2||s.r3||s.r4||s.auto==1;
  var box=document.getElementById('status');
  box.className='status '+(hasOut?'run':'idle');
  if(s.tmo)box.className='status warn';
  var lc=document.getElementById('leakCard');
  if(lc)lc.style.display=s.leakLat?'':'none';
  // 分页后漏水卡片只在「控制」页，别的页看不到 → 在控制页签打红点兜底
  var tb0=document.querySelectorAll('.tabbar .tb')[0];
  if(tb0)tb0.classList.toggle('warn',!!s.leakLat);
  if(s.leakLat){box.className='status warn';
    document.getElementById('leakTip').textContent=s.leak?'传感器当前仍浸水中':'传感器已干燥，可复位';}
  document.getElementById('phase').textContent=s.phase;
  /* v5.18 两行分工（用户定稿）：
   *   大字 = 换水中 / 排水中 / 上水中 / 换水完成 / 手动 / 待机 / 漏水保护
   *   小字 = 阶段文案 + 来源，例：排水泵反转（下水位触发/超时）· MQTT
   * 阶段文案由固件的 progressText() 给（已经含设备+动作+原因），
   * 网页只负责在尾巴上补一个来源，不再自己拼设备名 —— 免得两边各写一套对不上。 */
  var srcName={WEB:'网页',MQTT:'MQTT',BTN:'按钮',RC:'遥控',TIME:'定时',BOOT:'上电',MB:'Modbus',SAFE:'保护'};
  var suf = srcName[s.src] ? ' · '+srcName[s.src] : '';
  /* 有输出 / 自动流程 / 刚跑完 / 刚手动关过设备（phase='手动'）都要挂来源；
   * 纯待机（phase='待机'）不挂，避免残留上一次的来源。 */
  var showSrc = hasOut || s.ended || s.phase=='手动';
  var txt;
  if (s.leakLat)        txt = s.progress;                  // 已强制关闭全部输出
  else if (showSrc)     txt = s.progress + suf;            // 运行中 / 刚跑完 / 手动回执
  else                  txt = s.progress;                  // 待机、已到位未启动 等提示
  document.getElementById('progress').textContent=txt;
  for(var i=1;i<=4;i++){var d=document.getElementById('d'+i);if(d)d.classList.toggle('on',!!s['r'+i])}
  // 单键模式：d1/d2 按泵语义状态点亮，而非 #1/#2 物理吸合（正转时供电+信号都吸，两灯不能同亮）
  var _d1=document.getElementById('d1');if(_d1)_d1.classList.toggle('on',s.pump==1);
  var _d2=document.getElementById('d2');if(_d2)_d2.classList.toggle('on',s.pump==2);
  cur.r1=s.r1;cur.r2=s.r2;cur.r3=s.r3;cur.r4=s.r4;cur.pump=s.pump||0;
  cur.obj=s.obj||0;
  var running = !s.ended && s.phase!='待机' && s.phase!='漏水保护';
  cur.running=running;
  // v5.15：状态卡片里那行「固件 vX · 编译 …」删掉了 —— 系统页「固件升级」卡片已经显示
  // 当前版本（#fwCur），同一串信息没必要在首页再写一遍。
  // 固件升级卡片（v5.7）：显示当前版本，徽章同步
  var _fc=document.getElementById('fwCur');if(_fc)_fc.textContent=s.ver+' · '+(s.build||'-');
  // v5.21：标题右侧的版本号（页眉一眼可见，不用翻到「系统」页）
  var _hv=document.getElementById('hdrVer');if(_hv)_hv.textContent=s.ver||'v-';
  // v5.21：硬件型号 —— 板型 + 改版。与软件版本分开显示，换板子时一眼能对上。
  var _fh=document.getElementById('fwHw');
  if(_fh)_fh.textContent=(s.hwm||'-')+(s.hwr?' · '+s.hwr:'');
  var _fb=document.getElementById('fwBadge');if(_fb)_fb.innerHTML='<i></i>'+s.ver;
  // WiFi / MQTT 状态徽章（分别放在「网络设置」「MQTT 设置」卡片标题旁）
  var setBdg=function(id,txt,cls){var e=document.getElementById(id);if(!e)return;
    e.className='cardbadge '+(cls||'off');e.innerHTML='<i></i>'+txt;};
  if(s.wifi){setBdg('wifiBadge','已连接 · '+(s.ip||''),'ok');}
  else if(s.ap){setBdg('wifiBadge','热点模式','warn');}
  else{setBdg('wifiBadge','未连接','err');}
  if(!s.mqttEn){setBdg('mqttBadge','未启用','off');}
  else if(s.mqtt){setBdg('mqttBadge','已连接','ok');}
  else{setBdg('mqttBadge','未连接','err');}
  var hb=function(id,on,runTxt,stopTxt){var b=document.getElementById(id);if(!b)return;b.classList.toggle('act',!!on);b.textContent=on?runTxt:stopTxt};
  hb('hb-fwd',cur.pump==1,'泵 停止','泵 正转');
  hb('hb-rev',cur.pump==2,'泵 停止','泵 反转');
  hb('hb-valve',s.r3,'阀 关闭','阀 打开');
  hb('hb-aux',s.r4,'备用 断开','备用 通电');
  // 流程按钮高亮：自动流程运行中时，对应按钮点亮
  var fbCw=document.getElementById('fb-cw'),fbDr=document.getElementById('fb-dr'),fbFl=document.getElementById('fb-fl');
  if(fbCw)fbCw.classList.toggle('act',running&&s.obj==0);
  if(fbDr)fbDr.classList.toggle('act',running&&s.obj==1);
  if(fbFl)fbFl.classList.toggle('act',running&&s.obj==2);
  var ck=document.getElementById('clock');
  if(ck)ck.textContent=s.now?('当前时间 '+s.now):'系统时间未同步（等待 NTP 校时）';
  // v5.9：两排独立 —— 标题固定写死，small 显示「当前信号 + 当前规则」，右侧开关只管规则。
  // 文字颜色反映的是【原始信号】（通路绿 / 断开灰），不是判定结果，别混。
  /* v5.21：补一个「已触发 / 未触发」。
   * 起因：用户看到「当前：通路 · 规则：断开时停止」以为配错了 —— 直觉上「通路了怎么还不停」。
   * 其实规则说的是【哪一侧算到位】，跟此刻信号是什么无关。把这个结论直接算出来写在后面，
   * 就不用再靠人脑推。判定与固件一致：pol=1 → 通路即触发；pol=0 → 断开即触发（另有 3 秒宽限）。
   * v5.21 补：IN1 与 IN2 的【动作】不一样，不能共用「停止」两个字 ——
   *   IN1 触发是「排水正转段结束、泵立刻反转」（不是停泵），IN2 触发才是真的停止（关阀）。
   *   所以两行的规则文案分开写。 */
  var raw1=s.in1raw?1:0, pol1=s.in1pol?1:0;
  var e1n=document.getElementById('in1Now');
  if(e1n){
    var hit1=pol1?raw1:!raw1;
    e1n.innerHTML='<span class="nw">当前：'+(raw1?'通路':'断开')+'</span>'
      +'<span class="nw"> · 规则：'+(pol1?'通路时泵反转':'断开时泵反转')+'</span>'
      +'<span class="nw"> · '+(hit1?'已触发':'未触发')+'</span>';
    e1n.style.color=raw1?'#16a34a':'#94a3b8';
  }
  var raw2=s.in2raw?1:0, pol2=s.in2pol?1:0;
  var e2n=document.getElementById('in2Now');
  if(e2n){
    var hit2=pol2?raw2:!raw2;
    e2n.innerHTML='<span class="nw">当前：'+(raw2?'通路':'断开')+'</span>'
      +'<span class="nw"> · 规则：'+(pol2?'通路时停止':'断开时停止')+'</span>'
      +'<span class="nw"> · '+(hit2?'已触发':'未触发')+'</span>';
    e2n.style.color=raw2?'#16a34a':'#94a3b8';
  }
}
function loadCfg(){fetch('/cfg').then(r=>r.json()).then(function(c){
  // 时间参数：用户正在编辑该输入框时跳过回显，避免每5秒覆盖掉正在输入的值
  var setVal=function(id,v){var e=document.getElementById(id);if(!e)return;
    if(document.activeElement===e)return;e.value=v;};
  setVal('v0',c.t0);setVal('v1',c.t1);setVal('v2',c.t2);setVal('v4',c.t4);setVal('v5',c.t5);
  var on=c.trig==0;setSw('bootSw',on);
  document.getElementById('bootTxt').textContent=on?'开启':'关闭';
  document.getElementById('bootSub').textContent=on?'断电重启后延时 T0 秒自动执行一次':'上电后保持待机，不会自动执行';
  // v5.9：两路中止信号极性回显（1=通路时停止 → 开关在右/绿）
  setSw('in1Sw',c.in1pol==1);
  setSw('in2Sw',c.in2pol==1);
  // 注意：SSID/MQTT 等「一次性配置」不在这里回显——它们只在页面首次加载时回显一次，
  // 否则每 5 秒把旧值覆盖回输入框，用户改不动（这正是"选中 SSID 又被改回"的根因）。
  renderPlans(c.plans);
})}
// 首次加载回显：SSID/MQTT 等「一次性配置」只在页面打开时回显一次，
// 之后不再被周期 loadCfg 覆盖（否则用户改了配置会被旧值顶掉）。
function initCfg(){fetch('/cfg').then(r=>r.json()).then(function(c){
  var ss=document.getElementById('ssid');if(c.ssid)ss.value=c.ssid;
  document.getElementById('pass').placeholder=c.pset?'已设置（留空=不修改）':'WiFi密码';
  var mh=document.getElementById('mhost');if(c.mhost)mh.value=c.mhost;
  document.getElementById('mport').value=c.mport||'1883';
  var mu=document.getElementById('muser');if(c.muser)mu.value=c.muser;
  document.getElementById('mpass').placeholder=c.mset?'已设置（留空=不修改）':'MQTT密码';
})}
var dwSel={};
function renderPlans(pl){
  var names=['一','二','三','四','五','六','日'],b=document.getElementById('plans');b.innerHTML='';
  if(!pl.length){b.innerHTML='<div class="tip" style="margin:2px 0">还没有计划，下面添加</div>';return;}
  pl.forEach(function(p,i){
    var d=document.createElement('div');d.className='plan';
    var txt=p.type==0?('每周 '+dowName(p.param)+' '+(p.h<10?'0':'')+p.h+':'+(p.m<10?'0':'')+p.m):('每'+p.param+'天 '+(p.h<10?'0':'')+p.h+':'+(p.m<10?'0':'')+p.m);
    var objName=['换水','排水','上水'][p.obj];
    var nextTxt=(p.next&&p.next!=='')?('下次 '+p.next):'';
    d.innerHTML='<div class="sw'+(p.en?' on':'')+'" onclick="tog('+i+','+(p.en?0:1)+')"></div><div class="txt">'+txt+' · '+objName+'<small>'+(p.en?'已启用':'已停用')+(nextTxt?' · '+nextTxt:'')+'</small></div><button class="del" onclick="delp('+i+')">×</button>';
    b.appendChild(d);
  });
  function dowName(m){var s='',ns=['一','二','三','四','五','六','日'];for(var i=0;i<7;i++)if(m&(1<<i))s+=ns[i];return s||'未设';}
}
function tog(i,e){fetch('/cmd?x=plan_tog&idx='+i+'&en='+e).then(r=>r.text()).then(function(){loadCfg()})}
function delp(i){fetch('/cmd?x=plan_del&idx='+i).then(r=>r.text()).then(function(){loadCfg()})}
function addPlan(){
  var h=parseInt(document.getElementById('ph').value)||0,m=parseInt(document.getElementById('pm').value)||0;
  var param=ptype==0?dowBits():N;
  if(ptype==0 && param==0){toast('请先勾选要重复的星期');return;}   // 按周必须至少选一个星期
  fetch('/cmd?x=plan_add&type='+ptype+'&param='+param+'&h='+h+'&m='+m+'&obj='+pobj).then(r=>r.text()).then(function(){loadCfg()});
}
function dowBits(){var m=0;document.querySelectorAll('#pweek .dow button').forEach(function(x,i){if(x.classList.contains('on'))m|=1<<i});return m}
document.querySelectorAll('#ptabs .tab').forEach(function(x){x.onclick=function(){ptype=+x.dataset.p;
  document.querySelectorAll('#ptabs .tab').forEach(function(y){y.classList.toggle('on',y===x)});
  document.getElementById('pweek').classList.toggle('on',ptype==0);
  document.getElementById('pdays').classList.toggle('on',ptype==1);}});
document.querySelectorAll('#poseg .btn').forEach(function(x){x.onclick=function(){pobj=+x.dataset.o;
  document.querySelectorAll('#poseg .btn').forEach(function(y){y.classList.toggle('on',y===x)})}});
document.querySelectorAll('#pweek .dow button').forEach(function(x){x.onclick=function(){x.classList.toggle('on')}});
document.getElementById('nup').onclick=function(){if(N<99)N++;document.getElementById('vn').textContent=N};
document.getElementById('ndn').onclick=function(){if(N>1)N--;document.getElementById('vn').textContent=N};
document.getElementById('bootSw').onclick=function(){var on=this.classList.contains('on');cmdT(on?'trig1':'trig0',on?'开机启动已关闭':'开机启动已开启')};
// v5.9：两路中止信号极性。开关在右（on）=通路时停止，点一下切到断开时停止
document.getElementById('in1Sw').onclick=function(){
  var on=this.classList.contains('on');
  cmdT(on?'in1pol0':'in1pol1',on?'IN1 改为断开时停止':'IN1 改为通路时停止');};
document.getElementById('in2Sw').onclick=function(){
  var on=this.classList.contains('on');
  cmdT(on?'in2pol0':'in2pol1',on?'IN2 改为断开时停止':'IN2 改为通路时停止');};
function poll(){fetch('/state').then(r=>r.json()).then(upd)}
var tPoll=setInterval(poll,1500);poll();
var tCfg=setInterval(loadCfg,5000);
histFill('hist_ssid','histSsid');
histFill('hist_mhost','histMhost');
loadCfg();      // 周期刷新：时间参数/boot/plans（不含 SSID/MQTT 输入框）
initCfg();      // 首次回显：SSID/MQTT 输入框（仅此一次，之后不被周期覆盖）
/* 事件日志（v5.6）：每 4 秒拉一次 /log，倒序渲染（最新在最上面） */
function evEsc(s){return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;')}
function evRender(list){
  var d=document.getElementById('evlist'),b=document.getElementById('evBadge');
  if(!list||!list.length){d.innerHTML='<div class="tip" style="margin:2px 0">暂无记录</div>';b.innerHTML='<i></i>0 条';return;}
  var h='';
  for(var i=list.length-1;i>=0;i--){
    var e=list[i],p=String(e.x).split('|');
    h+='<div class="evrow"><span class="evt">'+evEsc(e.t)+'</span><span class="evx"><b>'+evEsc(p[0])+'</b> '+evEsc(p.slice(1).join('|'))+'</span></div>';
  }
  d.innerHTML=h;b.innerHTML='<i></i>'+list.length+' 条';
}
function evLoad(){fetch('/log').then(function(r){return r.json()}).then(evRender).catch(function(){})}
function evClear(){fetch('/log?clear=1').then(function(r){return r.json()}).then(evRender)}
evLoad();var tEv=setInterval(evLoad,4000);
/* 固件升级（v5.7）：XHR 上传 .bin，带进度条；成功后板子自己重启 */
// 上传期间停掉轮询：WebServer 同一时刻只服务一个连接，轮询会拖慢甚至打断上传
function fwPollStop(){clearInterval(tPoll);clearInterval(tCfg);clearInterval(tEv)}
function fwPollStart(){tPoll=setInterval(poll,1500);tCfg=setInterval(loadCfg,5000);tEv=setInterval(evLoad,4000)}
function fwUpload(){
  var f=document.getElementById('fwfile').files[0];
  if(!f){toast('请先选择 .bin 文件');return;}
  if(!/\.bin$/i.test(f.name)){toast('只能上传 .bin 文件');return;}
  if(!confirm('确定升级到 '+f.name+' ？\n升级期间设备会停止运行，约 30~60 秒。'))return;
  var bar=document.getElementById('fwbar'),btn=document.getElementById('fwbtn');
  var fd=new FormData();fd.append('firmware',f,f.name);
  var xhr=new XMLHttpRequest();
  btn.disabled=true;btn.textContent='正在升级…';bar.style.width='0%';
  fwPollStop();
  xhr.open('POST','/update');
  xhr.upload.onprogress=function(e){if(e.lengthComputable)bar.style.width=(e.loaded/e.total*100).toFixed(1)+'%'};
  xhr.onload=function(){
    var ok=xhr.responseText.indexOf('OK')===0;
    bar.style.width='100%';
    if(ok){btn.textContent='升级成功，重启中…';toast('升级成功，板子正在重启');}
    else{btn.disabled=false;btn.textContent='上传并升级';fwPollStart();toast('升级失败：'+xhr.responseText);}
  };
  xhr.onerror=function(){btn.disabled=false;btn.textContent='上传并升级';fwPollStart();toast('上传中断，检查网络后重试');};
  xhr.send(fd);
}
/* v5.10：从网址升级。板子自己去下载，手机不用拿着 .bin。
 * 这是个同步请求 —— 下载多久就等多久，期间 /state 轮询会超时，所以先 fwPollStop()。
 * 进度条拿不到真实百分比（HTTP 是一次请求），改成走一段停住，等结果。 */
function fwFromUrl(){
  var el=document.getElementById('fwurl'),u=(el.value||'').trim();
  if(!u){toast('请先填写 .bin 的完整网址');el.focus();return;}
  if(!/^https?:\/\//i.test(u)){toast('网址要以 http:// 或 https:// 开头');el.focus();return;}
  if(!confirm('从下面地址升级固件？\n\n'+u+'\n\n升级期间设备会停止运行，不要断电。'))return;
  var btn=document.getElementById('fwurlbtn'),bar=document.getElementById('fwbar');
  btn.disabled=true;btn.textContent='正在下载并升级…';
  if(bar){bar.style.width='0%';var p=0;bar.t=setInterval(function(){p=Math.min(p+4,88);bar.style.width=p+'%'},900);}
  fwPollStop();
  toast('正在下载，约 30~60 秒…');
  fetch('/update_url?url='+encodeURIComponent(u))
    .then(function(r){return r.text()})
    .then(function(t){
      if(bar&&bar.t){clearInterval(bar.t);bar.t=null;}
      if(bar)bar.style.width='100%';
      if(t.indexOf('OK')===0){btn.textContent='升级成功，重启中…';toast('升级成功，板子正在重启');}
      else{btn.disabled=false;btn.textContent='从网址下载并升级';fwPollStart();toast('升级失败：'+t);}
    })
    .catch(function(){
      if(bar&&bar.t){clearInterval(bar.t);bar.t=null;}
      btn.disabled=false;btn.textContent='从网址下载并升级';
      toast('连接中断，稍后刷新页面看版本号');
    });
}
</script>
<div class="ovl" id="infopop" onclick="if(event.target===this)hideInfo('infopop')"><div class="pop">
<h4>运行与触发</h4>
<div class="pbody">
<p>上电后保持<b>待机</b>，不会自动执行 —— 要跑流程，必须由下面几种方式之一发起。</p>
<p><b>定时计划</b>有启用即生效，不用另外选模式；<b>MQTT / 网页按钮 / 外接遥控</b>随时可用，不受待机限制。</p>
<p>流程跑完或超时后会停留 5 秒显示结果，然后自动回待机。板载 <b>BOOT 键短按</b>也能启动一次换水。</p>
<p><b>BOOT 键在哪</b>：板子底边、<b>Type-C 口旁边并排两颗小按钮</b> —— <b>左边那颗（丝印 IO0）是 BOOT</b>，右边那颗（丝印 EN）是 RESET。本页说的「BOOT 键」都指左边这颗；右边那颗是纯硬件复位，固件管不到，按一下等于断电重启。长按 BOOT 另有三档功能，见「记录」页的 BOOT 键功能表。</p>
</div>
<h4 class="mt">水位中止信号</h4>
<div class="pbody">
<p>开关在<b>右（绿）＝通路时触发</b>：浮球闭合、端子有信号才算到位；在<b>左（灰）＝断开时触发</b>：浮球断开、端子没信号才算到位。</p>
<p><b>IN1（排水）触发后泵立刻反转</b> —— 排水正转段到此结束，泵马上换向把管路里的剩水抽回，<b>不是停泵</b>；<b>IN2（上水）触发后关阀停止</b>。</p>
<p>浮球翻转 180° 安装会让极性反过来，在这里拨一下即可，不用重烧固件。切换立即生效并写入 EEPROM。</p>
<p><b>通路</b> = 浮球闭合，端子上有 5V 驱动光耦；<b>断开</b> = 浮球断开，端子上无电压。</p>
<p><b>断开时触发</b>多一条保护：信号丢失后不会立刻动作，要连续丢 3 秒才认账，中途恢复就继续跑 —— 因为这种接法下信号是「允许工作」的许可证，接触不良、水面晃一下不该让整轮换水报废。代价是真到位后阀/泵会多跑 3 秒。<b>通路时触发</b>没有这条，信号一出现立刻动。</p>
</div>
<button class="btn b pclose" onclick="hideInfo('infopop')">知道了</button>
</div></div>
</div></body></html>)rawliteral";
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");   // v5.3m：禁止浏览器缓存，避免"看到的是旧页面"
  server.send(200, "text/html", html);
}
// v5.3m：HTTP 层可观测性 —— 请求计数 / 最近一条命令 / loop 阻塞时长。
// 变量声明放在文件前部（见 AP_GRACE_MS 之后），因为 Arduino 只自动补函数原型、
// 不会前置声明全局变量。
// 板子自己访问自己的 HTTP 服务：完全绕开手机/电脑，验证 Web 服务层是否还活着
void httpSelfTest() {
  IPAddress cands[3];
  int n = 0;
  if (WiFi.status() == WL_CONNECTED) cands[n++] = WiFi.localIP();
  cands[n++] = WiFi.softAPIP();          // STA 自连若被协议栈拒绝，退而用热点口再试
  cands[n++] = IPAddress(127, 0, 0, 1);  // 回环口，验证 WebServer 是否真的在 listen
  unsigned long t0 = millis();
  bool conn = false;
  String resp;
  IPAddress used;
  for (int i = 0; i < n && !conn; i++) {
    WiFiClient c;
    c.setTimeout(600);
    if (c.connect(cands[i], 80)) {
      conn = true; used = cands[i];
      c.print(String("GET /state HTTP/1.0\r\nHost: ") + cands[i].toString() + "\r\n\r\n");
      unsigned long t1 = millis();
      while (millis() - t1 < 900) {
        while (c.available()) { resp += (char)c.read(); if (resp.length() > 160) break; }
        if (resp.indexOf("\r\n\r\n") >= 0) break;
        delay(5);
      }
      c.stop();
    }
  }
  selfRtt = millis() - t0;
  bool ok = conn && resp.startsWith("HTTP/");
  if (ok) selfOkN++; else selfFailN++;
  String rs = resp.substring(0, 32);
  rs.replace("\r", " "); rs.replace("\n", " ");
  Serial.printf("[SELF] conn=%d ip=%s rtt=%lums ok=%lu fail=%lu resp='%s'\r\n",
                (int)conn, used.toString().c_str(), selfRtt, selfOkN, selfFailN, rs.c_str());
}

void handleCmd() {
  String x = server.arg("x");
  httpCmdN++; lastHttpMs = millis();
  snprintf(lastHttpCmd, sizeof(lastHttpCmd), "%s", x.c_str());
  Serial.printf("[HTTP] cmd='%s'\r\n", x.c_str());
  String r = doCmd(x);
  Serial.print(String("[HTTP] -> ") + ioSnap() + "\r\n");   // 执行后的实况，一眼看出是否被拒绝
  server.send(200, "text/plain", r);
}
void handleState() { httpStateN++; lastHttpMs = millis(); server.sendHeader("Cache-Control", "no-store"); server.send(200, "text/plain", stateText()); }
void handleCfg()   { httpCfgN++;   lastHttpMs = millis(); server.sendHeader("Cache-Control", "no-store"); server.send(200, "application/json", cfgJson()); }

/* 事件日志查询：GET /log  →  [{ "t":"10:31:02", "x":"BTN|启动 换水" }, ...]（最旧→最新）
 *                     /log?clear=1 → 清空后返回空数组
 * 时间字段：已联网校时给真实时钟；未校时给开机秒数 "+123s"。 */
void handleLog() {
  lastHttpMs = millis();
  if (server.hasArg("clear")) { evHead = 0; evCnt = 0; }
  String s = "[";
  for (byte i = 0; i < evCnt; i++) {
    byte idx = (evHead + EV_MAX - evCnt + i) % EV_MAX;   // 从最旧读到最新
    EvRec& r = evBuf[idx];
    char tb[16];
    if (r.ep) {
      time_t t = (time_t)r.ep; struct tm tt; localtime_r(&t, &tt);
      snprintf(tb, sizeof(tb), "%02d:%02d:%02d", tt.tm_hour, tt.tm_min, tt.tm_sec);
    } else {
      snprintf(tb, sizeof(tb), "+%lus", (unsigned long)(r.ms / 1000UL));
    }
    if (i) s += ",";
    s += "{\"t\":\"" + String(tb) + "\",\"x\":\"" + jesc(r.txt) + "\"}";
  }
  s += "]";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", s);
}
// WiFi 扫描：返回 JSON 数组 [{"ssid":"xxx","rssi":-45,"enc":1}, ...]，按信号强度降序。
// 供前端「扫描附近 WiFi」下拉选择用，避免手敲 SSID 出错。
void handleWifiScan() {
  httpCmdN++; lastHttpMs = millis();
  int n = WiFi.scanNetworks();
  String out = "[";
  int shown = 0;
  for (int i = 0; i < n && shown < 20; i++) {
    String ss = WiFi.SSID(i);
    if (ss.length() == 0) continue;            // 隐藏 SSID 的跳过
    if (shown) out += ",";
    out += "{\"ssid\":\"" + jesc(ss.c_str()) + "\",\"rssi\":" + String(WiFi.RSSI(i)) +
           ",\"enc\":" + String(WiFi.encryptionType(i) != WIFI_AUTH_OPEN ? 1 : 0) + "}";
    shown++;
  }
  out += "]";
  WiFi.scanDelete();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", out);
}

// ==== v5.3m：MQTT 配置健壮性 ====
// 实测本板 EEPROM 里躺着历史脏数据：men=255、port=65535、host 末尾粘着密码 'fubin123'。
// 固件原本只写 if(mqttEn)，把 255 当成"已启用"，于是每次 connect 先去解析那个不存在的域名，
// Be DNS 超时把 loop 堵死约 9.5 秒；WebServer 是纯 loop 驱动的，结果是
// 网页能打开、但任何点击（/cmd）和自动轮询（/state）都大批超时 —— 用户看到的就是"点了没反应"。
bool mqttHostLooksValid(const char* h) {
  size_t n = strlen(h);
  if (n == 0 || n > 15) return false;
  bool hasDot = false;
  for (size_t i = 0; i < n; i++) {
    char c = h[i];
    if (c == '.') { hasDot = true; continue; }
    bool ok = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') ||
              (c >= 'A' && c <= 'Z') || c == '-' || c == '_';
    if (!ok) return false;
  }
  return hasDot;
}

void sanitizeMqttCfg() {
  bool changed = false;
  if (mqttEn != 0 && mqttEn != 1) { mqttEn = 0; changed = true; }
  if (mqttPort == 0 || mqttPort == 65535) { mqttPort = 1883; changed = true; }   // 0xFFFF = EEPROM 未写入值
  if (!mqttHostLooksValid(mqttHost)) {
    // 主机名是垃圾（含不可打印字符/超长，实测尾部粘着密码）→ 整组清空，
    // 否则 /cfg 会把乱码回吐到页面输入框，又变成"黑方块加问号"。
    mqttHost[0] = 0; mqttUser[0] = 0; mqttPass[0] = 0; changed = true;
  }
  if (mqttEn == 1 && mqttHost[0] == 0) { mqttEn = 0; changed = true; }
  if (changed) { save(); Serial.printf("[MQTT] 已清理非法配置 en=%d port=%u\r\n", (int)mqttEn, (unsigned)mqttPort); }
}

/* ======== v5.7 在线升级（OTA）========
 * 两条通道，互相独立：
 *   ① 网页 POST /update 上传 .bin —— 任何浏览器都能用，手机也行，适合交付后维护
 *   ② ArduinoOTA —— IDE「工具→端口」出现 "WTCC at 192.168.x.x"，点上传即可，适合自己调试
 *
 * 共同约束：**开始写入前强制全关输出**。
 *   OTA 期间 WebServer 被上传请求占满，网页命令进不来，继电器会一直保持最后状态；
 *   如果泵正在转，整个升级过程（30~60 秒）它就会一直转。所以先停机再收文件。
 *
 * 前提：分区表必须有两个 APP 分区（Default 4MB with spiffs / Minimal SPIFFS 都满足）。
 *   当前固件能编过 → 必然 ≤1280KB → 两个分区各放得下，OTA 可用。
 * 注意：分区表本身不能 OTA 修改，将来换分区方案仍需插 USB 刷一次。
 */
void otaBegin() {
  // 主机名必须与 setup() 里的 MDNS.begin("esp-pump") 保持一致：
  // ArduinoOTA.begin() 内部会再调一次 MDNS.begin(_hostname)，名字不同会把
  // esp-pump.local 改掉，导致原来能用的 mDNS 名字失效。
  ArduinoOTA.setHostname("esp-pump");
  ArduinoOTA.onStart([]() {
    applyRelays(0, 0, 0, 0);
    evLog("OTA", "无线升级开始，输出已全关");
  });
  ArduinoOTA.onEnd([]() {
    evLog("OTA", "无线升级完成，即将重启");
  });
  ArduinoOTA.onError([](ota_error_t e) {
    char b[40];
    snprintf(b, sizeof(b), "无线升级失败 code=%u", (unsigned)e);
    evLog("OTA", b);
  });
  ArduinoOTA.begin();
  Serial.println("[OTA] ArduinoOTA ready, host=esp-pump");
}

/* ======== v5.12：升级前只校验「硬件」，不校验「软件版本号」 ========
 * 需求：允许刷任何版本的固件（升级 / 降级 / 同版本重刷都放行），但必须挡住刷错芯片的包。
 * .bin 里现成的硬件标识只有镜像头（esp_image_header_t，前 24 字节）这两处：
 *   offset 0      magic     必须是 0xE9，否则根本不是 ESP 应用镜像
 *   offset 12~13  chip_id   小端 uint16：0x0000=ESP32 / 0x0002=S2 / 0x0005=C3 / 0x0009=S3 …
 * esp_app_desc_t（从文件 offset 0x20 起）里那个 version 字段【明确不比较】——
 *   这就是"不校验软件版本号"的落地，别在这里加版本比对逻辑。
 * 实测样本（项目里厂商的 MODBUS.ino.bin）：magic=0xE9、chip_id=0x0000、app_desc magic=0xABCD5432；
 *   它的 project_name 是 "arduino-lib-builder" 而非工程名，所以 project_name 不能当身份判据，
 *   真正可靠的只有 chip_id。
 * 返回空串=通过；否则返回中文原因（原样回给网页显示）。 */
const char* chipName(uint16_t id) {
  switch (id) {
    case 0x0000: return "ESP32";
    case 0x0002: return "ESP32-S2";
    case 0x0005: return "ESP32-C3";
    case 0x0009: return "ESP32-S3";
    case 0x000C: return "ESP32-C2";
    case 0x000D: return "ESP32-C6";
    case 0x0010: return "ESP32-H2";
    case 0x0012: return "ESP32-P4";
    default:     return "未知芯片";
  }
}
String otaVerifyImage(const uint8_t* h, size_t n) {
  if (n < 14) return "文件太小，读不到镜像头，可能不是固件";
  if (h[0] != 0xE9) return "不是 ESP 应用镜像（首字节不是 0xE9），可能选错了文件";
  uint16_t chip = (uint16_t)h[12] | ((uint16_t)h[13] << 8);
  if (chip != 0x0000)
    return String("这是给 ") + chipName(chip) + " 编的固件，本板是 ESP32，已拒绝写入";
  return "";
}

/* ======== v5.13：板卡标记 —— 只认本板子编出来的固件 ========
 * 光校验芯片型号不够：同为 ESP32，把别的板子的固件刷进来照样能跑错逻辑。
 * 于是在固件里埋一个明文字符串 WTCC_BOARD_TAG，升级时在收到的 .bin 里找它：
 *   找得到 = 同源固件，放行；找不到 = 放弃写入，板子继续跑原来的固件。
 *
 * 为什么不用 app_desc 的 project_name？实测厂商固件里它填的是 "arduino-lib-builder"
 *   （构建系统传的 PROJECT_NAME），换套工具链就变样，拿它当身份判据会把我们自己锁在外面。
 *
 * 标记串是 const 字符串、属 .rodata，链进 DROM 段；DROM 是 .bin 的【第一个段】：
 *   文件布局 = 镜像头 24 字节 + 段头 8 字节 + DROM 数据。段头 offset 28~31 是该段长度。
 *   所以扫完整个 DROM 还没见到标记，就能提前判负，不必把整个文件下完。
 *
 * ⚠ 换硬件 / 换板子时，改文件前部「硬件身份」那两行（HW_MODEL / HW_REV）即可：
 *   标记串由它们拼出来，新板固件带新标记、旧板固件带旧标记，互相当陌生固件，谁也刷不进谁。
 *   ⚠ 拼出来的字面值不要随手改 —— 手头已导出的旧 .bin 带的是 "WTCC-ESP32-RLY4-V1"，
 *     改字面值会让这些 bin 全部被拒绝（想刷旧固件就只能走串口）。
 * ⚠ 判负要 abort()：abort() 不切启动分区，板子仍从原分区正常启动，只是 OTA 分区被写脏，
 *   下次 esp_ota_begin 会重新擦。所谓"破釜沉舟"（就是想刷别的固件）请走串口烧写。
 */

/* 跨数据块找标记串。
 * feed() 一次收一小块（上传 1436 字节一包 / URL 1024 字节一包），标记串可能正好跨在两包之间，
 * 所以用「已匹配前缀长度」这个状态量续着走，不需要把整块缓存下来。 */
struct OtaTagScan {
  uint8_t tag[OTA_TAG_MAX];
  size_t  tagLen;
  size_t  matched;      // 当前已匹配上的前缀长度
  bool    found;
  bool    exhausted;    // 已扫完整个 DROM 还没找到 → 可以判负了
  size_t  scanned;      // 已扫过的 DROM 字节数
  size_t  dromLen;      // DROM 段长度（从段头 offset 28~31 取）
  size_t  headSeen;     // 已过掉的镜像头+段头字节数（32）

  void reset() {
    const char* t = WTCC_BOARD_TAG;
    tagLen = strlen(t);
    if (tagLen > OTA_TAG_MAX) tagLen = OTA_TAG_MAX;
    memcpy(tag, t, tagLen);
    matched = 0; found = false; exhausted = false;
    scanned = 0; dromLen = 0; headSeen = 0;
  }
  // 完整扫完 DROM 仍未命中 → 判负条件成立
  bool reject() const { return exhausted && !found; }

  void feed(const uint8_t* d, size_t n) {
    if (found || exhausted) return;
    size_t i = 0;
    while (i < n && headSeen < 32) {                 // 先扣镜像头 24 + 段头 8
      uint8_t b = d[i];
      if (headSeen >= 28) dromLen |= ((size_t)b) << (8 * (headSeen - 28));
      headSeen++; i++;
    }
    for (; i < n; i++) {
      uint8_t b = d[i];
      if (b == tag[matched]) {
        matched++;
        if (matched == tagLen) { found = true; return; }
      } else if (b == tag[0]) {
        matched = 1;                                  // 不匹配的这个字符本身可能是新前缀的开头
      } else {
        matched = 0;
      }
      if (++scanned >= dromLen && dromLen > 0) { exhausted = true; return; }
    }
  }
};
OtaTagScan otaTag;

String otaTagRejectMsg() {
  return String("未检出板卡标记 ") + WTCC_BOARD_TAG + "（可能不是给这块板子编的固件），已放弃写入";
}

// 网页上传的头部校验状态：UPLOAD_FILE_WRITE 会回调多次，要跨包累积够 14 字节才能判
static uint8_t otaHdr[64];
static size_t  otaHdrN = 0;
static bool    otaHdrOk = false;
static bool    otaRejected = false;
static String  otaRejectMsg;

// 网页 OTA：一次上传会回调多次，按 START / WRITE / END 三段推进
void handleUpdateUpload() {
  HTTPUpload& up = server.upload();
  if (up.status == UPLOAD_FILE_START) {
    applyRelays(0, 0, 0, 0);               // 先停机，别带着泵升级
    st = IDLE;
    otaHdrN = 0; otaHdrOk = false; otaRejected = false; otaRejectMsg = "";
    otaTag.reset();                        // 标记扫描状态：跨 UPLOAD_FILE_WRITE 多次回调续着走
    evLog("OTA", "网页升级开始，输出已全关");
    Serial.printf("[OTA] upload start: %s\r\n", up.filename.c_str());
    // 这里【故意不】调 Update.begin()：要先看头部是不是本硬件的固件，
    // 校验不过就一个字节都不往 flash 写，避免把 OTA 分区写脏。
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (otaRejected) return;
    bool wrOk = true;
    if (!otaHdrOk) {
      size_t room = sizeof(otaHdr) - otaHdrN;
      size_t take = up.currentSize < room ? up.currentSize : room;
      memcpy(otaHdr + otaHdrN, up.buf, take);
      otaHdrN += take;
      if (otaHdrN < 14) return;                      // 头还没凑齐，等下一包
      otaRejectMsg = otaVerifyImage(otaHdr, otaHdrN);
      if (otaRejectMsg.length()) {
        otaRejected = true;
        Serial.printf("[OTA] 拒绝：%s\r\n", otaRejectMsg.c_str());
        evLog("OTA", "网页升级被拒：" + otaRejectMsg);
        return;
      }
      otaHdrOk = true;
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { Update.printError(Serial); return; }
      wrOk = (Update.write(otaHdr, otaHdrN) == otaHdrN);
      otaTag.feed(otaHdr, otaHdrN);                 // 头部这 14 字节也参与标记扫描
      size_t rest = 0;                               // 这一包缓冲后还剩多少没写
      if (take < up.currentSize) {
        rest = up.currentSize - take;
        if (wrOk) {
          wrOk = (Update.write(up.buf + take, rest) == rest);
          otaTag.feed(up.buf + take, rest);
        }
      }
      if (!wrOk || otaTag.reject()) goto otaWrTail;  // 同一套收尾：置拒绝标志 + abort
      return;
    }
    wrOk = (Update.write(up.buf, up.currentSize) == up.currentSize);
    otaTag.feed(up.buf, up.currentSize);
    if (!wrOk || otaTag.reject()) goto otaWrTail;
    return;

  otaWrTail:
    if (otaTag.reject()) {
      otaRejectMsg = otaTagRejectMsg();
      Serial.printf("[OTA] 拒绝：%s\r\n", otaRejectMsg.c_str());
      evLog("OTA", "网页升级被拒：" + otaRejectMsg);
    } else {
      otaRejectMsg = String("写入失败：") + Update.errorString();
      Update.printError(Serial);
    }
    otaRejected = true;
    if (Update.isRunning()) Update.abort();         // 不切启动分区，板子照旧从原分区启动
    return;
  } else if (up.status == UPLOAD_FILE_END) {
    if (otaRejected) return;
    if (!otaHdrOk) { otaRejected = true; otaRejectMsg = "文件太小，不是有效的固件"; return; }
    if (otaTag.reject()) {                          // 全下完依然没见到标记 → 放弃
      otaRejectMsg = otaTagRejectMsg();
      otaRejected = true;
      Serial.printf("[OTA] 拒绝：%s\r\n", otaRejectMsg.c_str());
      evLog("OTA", "网页升级被拒：" + otaRejectMsg);
      if (Update.isRunning()) Update.abort();
      return;
    }
    if (Update.end(true)) {
      Serial.printf("[OTA] upload ok, %u bytes\r\n", (unsigned)up.totalSize);
      evLog("OTA", "网页升级写入完成，即将重启");
    } else {
      Update.printError(Serial);
      evLog("OTA", "网页升级写入失败");
    }
  }
}
void handleUpdateDone() {
  if (otaRejected) {                        // 校验没过：把原因原样回给网页，不重启
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", otaRejectMsg);
    otaRejected = false;
    return;
  }
  bool ok = !Update.hasError();
  server.sendHeader("Connection", "close");
  server.send(200, "text/plain", String(ok ? "OK" : Update.errorString()));
  if (ok) { delay(300); ESP.restart(); }    // 留一点时间让响应发出去
}

/* ======== v5.10：从 URL 升级 ========
 * 场景：.bin 放在内网服务器 / NAS / 图床，不想每次都拿手机去选文件。
 * 支持 http 和 https；https 用 setInsecure() 跳过证书校验 —— 家庭内网够用，
 *   但别在不可信网络上用：中间人可以把任意固件塞进来。
 * 返回空串=成功，否则返回失败原因（原样回给网页显示）。
 * 注意：下载期间 WebServer 被这一个请求占住，/state 轮询会超时，属正常现象。 */
String otaFromUrl(const String& url) {
  if (!url.startsWith("http://") && !url.startsWith("https://"))
    return "只支持 http:// 或 https:// 开头";

  // 用 new/delete 而不是栈对象，是为了把析构顺序攥在手里：
  //   HTTPClient 析构时会对 _client 调 stop()，所以必须【先】析构 HTTPClient、【后】释放 TLS client。
  //   反过来先 delete secure，HTTPClient 析构时就是 use-after-free。
  //   WiFiClientSecure 约 40KB，放栈上会直接把 loopTask 的 8KB 栈撑爆，只能走堆。
  HTTPClient* http = new HTTPClient();
  if (!http) return "内存不足";
  WiFiClientSecure* secure = nullptr;
  String err;

  http->setTimeout(20000);
  http->setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);   // 图床/短链常见 302

  bool begun;
  if (url.startsWith("https://")) {
    secure = new WiFiClientSecure();
    if (!secure) { delete http; return "内存不足，无法建立 TLS 连接"; }
    secure->setInsecure();
    begun = http->begin(*secure, url);
  } else {
    begun = http->begin(url);
  }

  if (!begun) {
    err = "地址格式错误";
  } else {
    int code = http->GET();
    if (code != HTTP_CODE_OK) {
      err = String("HTTP ") + String(code);
    } else {
      int len = http->getSize();          // -1 = 服务器没给 Content-Length
      // 先停机再写 flash：理由同上传通道，别带着泵升级
      applyRelays(0, 0, 0, 0);
      st = IDLE;
      otaTag.reset();                     // v5.13：板卡标记扫描状态归零（跨块续着走）
      evLog("OTA", "URL 升级开始，输出已全关");
      Serial.printf("[OTA] url=%s len=%d\r\n", url.c_str(), len);

      /* v5.12：先把 14 字节镜像头读下来校验硬件，通过了才 Update.begin()。
       * 校验不过就一个字节都不写 flash —— 别把 OTA 分区写脏，回头还得拆下来烧串口。 */
      WiFiClient* stream = http->getStreamPtr();
      uint8_t hdr[14];
      size_t hn = 0;
      unsigned long th = millis();
      while (hn < sizeof(hdr) && http->connected() && millis() - th < 15000UL) {
        size_t avail = stream->available();
        if (avail) {
          size_t want = sizeof(hdr) - hn;
          if (avail < want) want = avail;
          size_t got = stream->readBytes(hdr + hn, want);
          if (got == 0) break;
          hn += got;
        } else {
          delay(1);                                    // 喂狗
        }
      }
      String hwErr = (hn < sizeof(hdr)) ? String("文件太小，读不到镜像头，可能不是固件")
                                        : otaVerifyImage(hdr, hn);
      if (hwErr.length()) {
        err = hwErr;
        Serial.printf("[OTA] 拒绝：%s\r\n", hwErr.c_str());
        evLog("OTA", "URL 升级被拒：" + hwErr);
      } else if (!((len > 0) ? Update.begin((size_t)len) : Update.begin(UPDATE_SIZE_UNKNOWN))) {
        err = String("无法开始写入：") + Update.errorString();
        evLog("OTA", "URL 升级失败：无法开始写入");
      } else if (Update.write(hdr, hn) != hn) {
        err = String("写入失败：") + Update.errorString();
        Update.abort();
        evLog("OTA", "URL 升级写入失败");
      } else {
        otaTag.feed(hdr, hn);                          // v5.13：头部 14 字节也参与标记扫描
        uint8_t buf[1024];
        size_t total = hn;                             // 头部这 14 字节也算进总量
        unsigned long t0 = millis();
        while (http->connected() && (len < 0 || total < (size_t)len)) {
          size_t avail = stream->available();
          if (avail) {
            size_t want = avail > sizeof(buf) ? sizeof(buf) : avail;
            size_t got = stream->readBytes(buf, want);
            if (got == 0) break;
            otaTag.feed(buf, got);                     // 边下边扫标记
            if (Update.write(buf, got) != got) {
              err = String("写入失败：") + Update.errorString();
              Update.abort();
              evLog("OTA", "URL 升级写入失败");
              break;
            }
            total += got;
            if (otaTag.reject()) {                     // DROM 扫完仍无标记 → 提前判负
              err = otaTagRejectMsg();
              Serial.printf("[OTA] 拒绝：%s\r\n", err.c_str());
              evLog("OTA", "URL 升级被拒：" + err);
              Update.abort();                          // 不切启动分区，照旧从原分区启动
              break;
            }
          } else {
            if (!http->connected()) break;
            delay(1);                                  // 喂狗 + 让出 CPU，别触发 WDT
            if (millis() - t0 > 180000UL) {            // 3 分钟还没下完就放弃
              err = "下载超时（超过 3 分钟）";
              Update.abort();
              evLog("OTA", "URL 升级超时");
              break;
            }
          }
        }
        if (err.length() == 0) {
          if (otaTag.reject()) {                       // 正常收尾前最后再查一次
            err = otaTagRejectMsg();
            Serial.printf("[OTA] 拒绝：%s\r\n", err.c_str());
            evLog("OTA", "URL 升级被拒：" + err);
            Update.abort();
          } else if (len > 0 && total != (size_t)len) {
            err = String("下载不完整：") + String(total) + "/" + String(len) + " 字节";
            Update.abort();
            evLog("OTA", "URL 升级下载不完整");
          } else if (!Update.end(true)) {
            err = String("收尾失败：") + Update.errorString();
            evLog("OTA", "URL 升级收尾失败");
          } else {
            evLog("OTA", String("URL 升级完成 ") + String(total) + " 字节，即将重启");
          }
        }
      }
    }
    http->end();
  }

  delete http;                 // 先让 HTTPClient 析构（它会 stop 掉 client）
  if (secure) delete secure;   // 再释放 TLS client
  return err;
}

void handleUpdateUrl() {
  String u = server.arg("url");
  u.trim();
  if (u.length() < 8) { server.send(200, "text/plain", "请填写完整的 .bin 网址"); return; }
  Serial.printf("[HTTP] update_url '%s'\r\n", u.c_str());
  String err = otaFromUrl(u);
  if (err.length() == 0) {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", "OK");
    delay(300);
    ESP.restart();
  } else {
    server.send(200, "text/plain", err);
  }
}

void setup() {
  // ① 最优先：把 4 路继电器锁死为释放态。
  //    顺序是【先 pinMode(OUTPUT)，再写释放电平】——这点很关键：
  //    若反过来（先 digitalWrite 再 pinMode），ESP32 在引脚仍为 INPUT 时写的电平
  //    不会保持，切到 OUTPUT 后引脚停在默认低电平；本板低电平吸合，
  //    结果就是上电四路全吸合，且改极性宏也毫无效果（2026-10-20 实测踩过）。
  //    pinMode 那一瞬间会有一个微秒级低脉冲，但继电器吸合需 5~10ms，来不及动作。
  pinMode(RLY1, OUTPUT); pinMode(RLY2, OUTPUT);
  pinMode(RLY3, OUTPUT); pinMode(RLY4, OUTPUT);
  rlyWrite(RLY1, false); rlyWrite(RLY2, false);
  rlyWrite(RLY3, false); rlyWrite(RLY4, false);
  relayState = 0;

  EEPROM.begin(512);
  EEPROM.get(0, cfgT1); EEPROM.get(2, cfgT2); EEPROM.get(5, wifiMode);
  EEPROM.get(6, ssid); EEPROM.get(39, pass);
  EEPROM.get(104, mqttEn); EEPROM.get(105, mqttHost); EEPROM.get(121, mqttPort);
  EEPROM.get(123, mqttUser); EEPROM.get(155, mqttPass);
  sanitizeMqttCfg();      // v5.3m：清理历史脏配置，防止非法 MQTT 把 loop 拖死
  EEPROM.get(219, trig); EEPROM.get(220, objGlobal);
  EEPROM.get(221, cfgT0); EEPROM.get(223, cfgT4); EEPROM.get(225, cfgT5);
  EEPROM.get(227, planCount);
  EEPROM.get(308, senBPol);       // v5.8：IN2 判定极性（0xFF=从未写过，下面校验会兜底成默认值）
  EEPROM.get(309, senAPol);       // v5.9：IN1 判定极性
  for (int i = 0; i < MAX_PLAN; i++) {
    int a = 228 + i * 8;
    EEPROM.get(a, planEn[i]); EEPROM.get(a + 1, planType[i]); EEPROM.get(a + 2, planParam[i]);
    EEPROM.get(a + 3, planHour[i]); EEPROM.get(a + 4, planMin[i]); EEPROM.get(a + 5, planObj[i]);
    EEPROM.get(a + 6, planRef[i]);
  }
  if (cfgT1 < 1 || cfgT1 > 9999) cfgT1 = 20;
  if (cfgT2 < 1 || cfgT2 > 9999) cfgT2 = 20;
  if (cfgT0 > 9999) cfgT0 = 5;
  if (cfgT4 < 1 || cfgT4 > 9999) cfgT4 = 120;
  if (cfgT5 < 1 || cfgT5 > 9999) cfgT5 = 120;
  if (trig > 1) trig = 1;   // v4.0 起只有 0=是 1=否；v4.3 起默认关=1（旧固件 trig 2/3/4 归 1）
  if (objGlobal > 2) objGlobal = 0;
  if (planCount > MAX_PLAN) planCount = 0;
  if (senBPol > 1) senBPol = SEN_B_TRIG_LOW;   // 0xFF/脏数据 → 回落默认（有信号=到位）
  if (senAPol > 1) senAPol = SEN_A_TRIG_LOW;
  // 继电器引脚已在 setup() 第一行锁死为释放态（HIGH），此处不再重复设置。
  pinMode(SEN_A, INPUT_PULLUP); pinMode(SEN_B, INPUT_PULLUP);
  // 注意：GPIO36/39（VP/VN）是 input-only 脚，芯片内部【没有】上/下拉电阻，
  // 传 INPUT_PULLUP 不但无效，还会在串口打印 "gpio_pullup_en(): GPIO number error"。
  // v5.3h 改为纯 INPUT：真要用冗余水位时，外部自己加 10K 上拉。
  pinMode(RED_A, INPUT); pinMode(RED_B, INPUT);
  pinMode(REMOTE_IN, INPUT_PULLUP);
  pinMode(LEAK_IN, INPUT_PULLUP); pinMode(RC_IN, INPUT_PULLUP);   // 光耦 #3 浸水传感器 / #4 单键遥控
  pinMode(MB_DE, OUTPUT); digitalWrite(MB_DE, LOW);   // 485 默认接收
  pinMode(LED_STATUS, OUTPUT); ledSet(false);         // 系统状态灯
  pinMode(BTN_BOOT, INPUT_PULLUP);                    // BOOT 功能键（GPIO0，板上有 10K 上拉）
  // Modbus 从机：恢复成品板的 RS485 能力（原厂协议：地址1 / 115200 / 8N1）
  mbSerial.begin(MB_BAUD, SERIAL_8N1, MB_RX, MB_TX);
  pumpOut(0, 0, 0);
  Serial.begin(115200);                 // USB-UART(CH340) 调试串口
  delay(50);
  Serial.printf("[BOOT] %s build=%s hw=%s ready (DOIT ESP32-32E, Modbus slave addr 1 @115200)\r\n",
                FW_VER, FW_BUILD, WTCC_BOARD_TAG);
  evLog("BOOT", String(FW_VER) + " 启动 · " + WTCC_BOARD_TAG);

  mqtt.setCallback(mqttCb);
  // v5.3m 关键修复：PubSubClient 默认 socket 超时 15 秒。MQTT 服务器不可达时，
  // 每次 connect() 会把 loop 堵死十几秒，而 loop 每 5 秒就重试一次 —— 结果
  // WebServer（完全依赖 loop 驱动）几乎抢不到 CPU，网页所有点击全部超时。
  // 压到 2 秒即可让网页保持响应，代价只是 MQTT 重连更频繁地重试。
  mqtt.setSocketTimeout(2);
  WiFi.mode(WIFI_AP_STA);
  bool apok = startAP();
  // v5.3h 启动诊断：把"热点到底起来没有 / 配的哪个家网"直接打在串口上，
  // 以后再出现"搜不到 FS_TWCC"就不必猜 —— 看这一行即可。
  Serial.printf("[BOOT] apStart=%d apIP=%s wifiMode=%d ssid='%s'\r\n",
                (int)apok, WiFi.softAPIP().toString().c_str(), (int)wifiMode, ssid);
  if (wifiMode == 1 && ssid[0] != 0) {
    WiFi.begin(ssid, pass);
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000UL) delay(200);
    if (WiFi.status() == WL_CONNECTED) {
      MDNS.begin("esp-pump");
      configTime(8 * 3600, 0, "ntp.aliyun.com", "pool.ntp.org");   // 东八区 NTP 校时
    }
  }
  server.on("/", handleRoot);
  server.on("/cmd", handleCmd);
  server.on("/state", handleState);
  server.on("/cfg", handleCfg);
  server.on("/wifi_scan", handleWifiScan);
  server.on("/log", handleLog);
  server.on("/update", HTTP_POST, handleUpdateDone, handleUpdateUpload);   // v5.7 网页 OTA
  server.on("/update_url", handleUpdateUrl);                               // v5.10 从网址升级
  server.begin();
  otaBegin();                                                              // v5.7 IDE 无线烧录
  if (mqttEn) mqttConnect();
  // v5.3i：上电基线快照 —— 确认开机瞬间继电器到底有没有意外吸合
  Serial.print(String("[BOOT] io=") + ioSnap() + "\r\n");
}

void loop() {
  // v5.3m：loop 阻塞时长统计。若 >1000ms，说明有代码卡住主循环（WebServer 全靠 loop 驱动）
  unsigned long nowLoop = millis();
  if (loopLastMs) { unsigned long g = nowLoop - loopLastMs; if (g > loopMaxGap) loopMaxGap = g; }
  loopLastMs = nowLoop;
  server.handleClient();
  ArduinoOTA.handle();   // v5.7：处理 IDE 无线烧录请求（无请求时开销可忽略）
  serialCli();       // v5.3i：串口命令行（排障通道，绕开 HTTP 直接下命令）
  mbPoll();          // Modbus RTU 从机：保持成品板 RS485 上位机可用
  ledTask();         // 系统状态灯（唯一的软件可控 LED）
  btnTask();         // 板载 BOOT 键：短按换水 / 长按(3s)急停 / 超长按(10s)网络复位
  leakTask();        // 浸水传感器（IN3）：最高优先级安全保护
  rcTask();          // 单键遥控（IN4）：1击排水 / 2击上水 / 3击换水 / 长按急停
  if (mqttEn && mqttHost[0] != 0) {
    if (!mqtt.connected() && millis() - lastMqttTry > 30000UL) {
      lastMqttTry = millis();
      unsigned long mt = millis();
      mqttConnect();
      unsigned long md = millis() - mt;
      if (md > mqttBlkMs) mqttBlkMs = md;      // v5.3m：MQTT 连接尝试对 loop 的阻塞时长
    }
    else mqtt.loop();
    if (mqtt.connected() && millis() - lastPub > 10000UL) { lastPub = millis(); mqttPubStates(); }
    // discovery 只在 mqttConnect() 成功时发布一次（HA 会缓存），不再周期重发——
    // 之前每 60 秒重发 20+ 条，每条同步 publish 失败还要 delay 重试，半开连接下会把 loop 堵死几十秒。
  }
  if (wifiMode == 1 && ssid[0] != 0 && WiFi.status() != WL_CONNECTED && millis() - lastWiFiRetry > 30000UL) {
    lastWiFiRetry = millis();
    WiFi.disconnect(); WiFi.begin(ssid, pass);
  }
  // 热点自愈：只要没连上家网，热点就必须在（保证 192.168.4.1 永远可达）；
  // 连上家网才关掉热点省 AP 资源。每 5 秒核对一次真实 Wi-Fi 模式，
  // 遇到 softAP() 偶发起不来的情况能自动补回来 —— 不用重启板子。
  if (millis() - lastApCheck > 5000UL) {
    lastApCheck = millis();
    bool staOk = (WiFi.status() == WL_CONNECTED);
    // v5.3h：开机宽限期内无条件保热点 —— 防止一连上家网就把 FS_TWCC 关掉，
    // 导致用户手机搜不到、又无从得知家网 IP。
    bool grace  = (millis() < AP_GRACE_MS);
    bool wantAP = grace || !(wifiMode == 1 && staOk);   // 宽限期 / 纯热点 / 家网没连上 → 都要开
    bool hasAP  = (WiFi.getMode() & WIFI_AP) != 0;
    if (wantAP && !hasAP) { if (startAP()) apOff = false; }
    else if (!wantAP && hasAP) { WiFi.softAPdisconnect(true); apOff = true; apOffAt = millis(); }
  }
  // v5.3h：每 3 秒一行网络实况，接串口即可判断"热点在不在、家网连没连、IP 多少"
  if (millis() - lastNetLog > 3000UL) {
    lastNetLog = millis();
    bool staOk = (WiFi.status() == WL_CONNECTED);
    Serial.printf("[NET] %lus sta=%d(%s) ip=%s ap=%d apip=%s grace=%d ssid='%s'\r\n",
                  millis() / 1000,
                  (int)staOk, staOk ? WiFi.SSID().c_str() : "-",
                  staOk ? WiFi.localIP().toString().c_str() : "-",
                  (int)((WiFi.getMode() & WIFI_AP) != 0),
                  WiFi.softAPIP().toString().c_str(),
                  (int)(millis() < AP_GRACE_MS),
                  ssid);
    // v5.3i：IO 实况，与 [NET] 同节奏。看 rly 位是否为 1 就知道固件有没有真的拉低引脚
    time_t nowt = time(nullptr);
    Serial.printf("[IO ] %s tsync=%d epoch=%lu t0=%u\r\n",
                  ioSnap().c_str(), (int)timeSynced, (unsigned long)nowt, cfgT0);
  }
  // v5.3m：每 15 秒一次 HTTP 自检 + 请求计数。核心判据：
  //   · 你点网页按钮后，这里的 cmd 计数 +1 且 lastCmd 显示命令名 → 请求确已到达板子
  //   · 计数不动 → 请求没到板子，问题在网络/IP/页面缓存，与固件无关
  //   · [SELF] ok 递增 → Web 服务层健康；持续 fail → 服务层或 socket 资源耗尽
  if (millis() - lastDiag > 15000UL) {
    lastDiag = millis();
    // 注：HTTP 自检已改为按需触发（串口发 selftest），不做周期执行 ——
    // 自连失败时每次要等 TCP SYN 重试（约 3 秒/地址），周期跑会反过来拖慢 loop。
    Serial.printf("[DIAG] req cmd=%lu state=%lu cfg=%lu root=%lu | lastCmd='%s' %lus ago | loopMax=%lums heap=%lu minHeap=%lu\r\n",
                  httpCmdN, httpStateN, httpCfgN, httpRootN,
                  lastHttpCmd[0] ? lastHttpCmd : "(none)",
                  lastHttpMs ? (millis() - lastHttpMs) / 1000UL : 9999UL,
                  loopMaxGap,
                  (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getMinFreeHeap());
    Serial.printf("[MQTT] en=%d host='%s' port=%u conn=%d blk=%lums\r\n",
                  (int)mqttEn, mqttHost, (unsigned)mqttPort,
                  (int)mqtt.connected(), mqttBlkMs);
    mqttBlkMs = 0;
    loopMaxGap = 0;
  }
  if (!timeSynced && wifiMode == 1 && WiFi.status() == WL_CONNECTED && time(nullptr) > 100000) {
    timeSynced = true;
    for (int i = 0; i < planCount; i++)
      if (planType[i] == 1 && planRef[i] == 0 && timeSynced) { planRef[i] = dayNum(); }
    save();
  }

  // 外接遥控/按钮：干触点闭合沿触发一次换水
  {
    bool now = digitalRead(REMOTE_IN);
    if (lastRemote == HIGH && now == LOW) {
      delay(30);
      if (!digitalRead(REMOTE_IN)) startRun(0, "RC");
    }
    lastRemote = now;
  }

  // 开机启动=是：trig=0，T0 延时后执行一次（全局对象）；trig=1 上电待机 不自动执行
  if (trig == 0 && !bootRun && millis() >= cfgT0 * 1000UL) { bootRun = true; startRun(objGlobal, "BOOT"); }

  // 定时计划触发（每秒；有计划启用即生效，无需选择模式）
  if (timeSynced) {
    time_t now = time(nullptr);
    struct tm t; localtime_r(&now, &t);
    int mm = t.tm_hour * 60 + t.tm_min;
    if (mm != lastTrigMin) {
      lastTrigMin = mm;
      int wd = t.tm_wday;                    // 0=周日
      int wbit = wd == 0 ? 6 : wd - 1;
      int dn = dayNum();
      for (int i = 0; i < planCount; i++) {
        if (!planEn[i]) continue;
        bool hit = false;
        if (planType[i] == 0) hit = (planParam[i] & (1 << wbit)) && planHour[i] == t.tm_hour && planMin[i] == t.tm_min;
        else if (planParam[i] > 0 && planRef[i] > 0) hit = ((dn - planRef[i]) % planParam[i] == 0) && planHour[i] == t.tm_hour && planMin[i] == t.tm_min;
        if (hit) { evLog("TIME", String("计划 #") + String(i + 1) + " 到点"); startRun(planObj[i], "TIME"); break; }
      }
    }
  }

  if (st == END || st == T5TO) {               // 结束/超时关阀：展示 5 秒后自动回待机
    /* v5.20：刚进入终态时结算一次结果，发给 HA。
     * done 会一直保持到下次启动或手动操作 —— END 只停 5 秒，让自动化去抢这个窗口太脆。 */
    if (!endHandled) {
      endHandled = true;
      doneFlag = (st == END);                  // 超时(T5TO)不算正常完成
      if (st == END) snprintf(lastResult, sizeof(lastResult), "%s完成", objName(runObj));
      else           snprintf(lastResult, sizeof(lastResult), "上水超时（T5）");
      if (mqtt.connected()) {
        mqtt.publish("esp-pump/done", onoff(doneFlag));
        mqtt.publish("esp-pump/last", lastResult);
      }
    }
    /* v5.19：回待机时把 manSt / runSrc 一并复位 —— 否则上一轮手动留下的「阀关闭」回执
     * 会一直挂着，流程跑完反而显示成「手动 / 阀关闭」，看着像根本没跑过。 */
    if (millis() - tmr >= 5000UL) { st = IDLE; manSt = "待机"; runSrc = ""; }
    return;
  }
  if (st == IDLE) return;                      // 空闲：不跑状态机
  switch (st) {
    case FWD:
      /* v5.11：断电时停止（pol=0）下，信号丢失先给宽限、泵继续正转；
       *        宽限内恢复就撤销，连续丢失超过 HOLD_GRACE_MS 才确认到位跳反转。
       *        pol=1 仍是"出现即到位"，holdOk 直接返回 true，行为与 v5.10 完全一致。 */
      if (holdOk(senAPol, trigAny(SEN_A, RED_A, senAPol), holdALost, holdAT0, "下水位")) {
        holdALost = false;
        evLog("RUN", "下水位到位 → 泵反转 T1");
        st = REV; tmr = millis(); pumpOut(0, 1, 0);
      }
      else if (millis() - tmr >= cfgT4 * 1000UL) {     // 宽限期间 T4 照常兜底
        holdALost = false;
        evLog("RUN", "T4 超时：泵正转超时 → 强制反转");
        st = T4REV; tmr = millis(); pumpOut(0, 1, 0);
      }
      break;
    case REV:
    case T4REV:
      if (millis() - tmr >= cfgT1 * 1000UL) {
        if (runObj == 1) { st = END; tmr = millis(); pumpOut(0, 0, 0); evLog("RUN", "排水完成"); }        // 排水：反转T1后停
        else { st = GAP; tmr = millis(); pumpOut(0, 0, 0); }     // 换水：停T2
      }
      break;
    case GAP:
      if (millis() - tmr >= cfgT2 * 1000UL) {
        holdBLost = false;                              // 进上水前清掉上一轮遗留的宽限标记
        st = VALVE_ON; tmr = millis(); pumpOut(0, 0, 1);
      }
      break;
    case VALVE_ON:
      /* 同 FWD：pol=0 时信号是"许可证"，掉一下不立刻关阀，宽限内恢复就继续上水。 */
      if (holdOk(senBPol, trigAny(SEN_B, RED_B, senBPol), holdBLost, holdBT0, "上水位")) {
        holdBLost = false;
        evLog("RUN", "上水位到位 → 关阀，流程完成");
        st = END; tmr = millis(); pumpOut(0, 0, 0);
      }
      else if (millis() - tmr >= cfgT5 * 1000UL) {     // 宽限期间 T5 照常兜底
        holdBLost = false;
        evLog("RUN", "T5 超时：上水超时 → 强制关阀");
        st = T5TO; tmr = millis(); pumpOut(0, 0, 0);
      }
      break;
    case T5TO:
      break;
  }
}

/* ================= v5.3i 排障工具（不参与任何控制逻辑）================= */

// 一行说清"为什么不动"：漏水锁定？输入电平？继电器引脚到底拉低没有？
// 注意 rly 打印的是【引脚实况】：1 = 引脚处于"吸合电平"（由 RLY_ACTIVE_LOW 决定）。
// 若 rly1=1 却听不到吸合声，问题在硬件/供电；若 rly1=0 且 leakLat=1，则是被漏水锁定拒绝。
String ioSnap() {
  char b[208];
  // rs = 影子寄存器 relayState（固件"认为"的状态，也是网页 r1~r4 显示的值）；
  // rly = 引脚实况 digitalRead==LOW。两者不一致时以 rs 为准判断固件逻辑，
  // 因为 ESP32 的 digitalRead 对纯 OUTPUT 引脚未必反映真实输出电平。
  snprintf(b, sizeof(b),
           "leakLat=%d leak=%d trig=%d in(A,B,Leak,RC)=%d%d%d%d rly=%d%d%d%d rs=0x%02X st=%s",
           (int)leakLatch, (int)leakNow, (int)trig,
           (int)(digitalRead(SEN_A)   == LOW),
           (int)(digitalRead(SEN_B)   == LOW),
           (int)(digitalRead(LEAK_IN) == LOW),
           (int)(digitalRead(RC_IN)   == LOW),
           (int)(digitalRead(RLY1) == RLY_ON_LEVEL),
           (int)(digitalRead(RLY2) == RLY_ON_LEVEL),
           (int)(digitalRead(RLY3) == RLY_ON_LEVEL),
           (int)(digitalRead(RLY4) == RLY_ON_LEVEL),
           (unsigned)relayState,
           stName());
  return String(b);
}

// 串口命令行：敲命令等价于网页按钮，用来绕开浏览器/HTTP 独立验证固件与硬件。
// 常用：fwd / rev / stop / valve_on / aux_on / leak_reset / t0up / t1up / ?
void serialCli() {
#if SERIAL_CLI
  static String line = "";
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line.trim();
      if (line.length()) {
        if (line == "?") {
          Serial.print(F("[CLI] cmd: fwd rev stop valve_on valve_off aux_on aux_off "
                         "leak_reset start drain fill t0up t0dn t1up t1dn t4up t4dn "
                         "t5up t5dn wifi_off ap_restart\r\n"));
        } else {
          Serial.print(String("[CLI] '") + line + "' -> ");
          doCmd(line);                       // 与网页 /cmd 走同一条 doCmd
          Serial.print(ioSnap());
          Serial.printf(" T=%u/%u/%u/%u/%u\r\n", cfgT0, cfgT1, cfgT2, cfgT4, cfgT5);
        }
      }
      line = "";
    } else if (line.length() < 64) {
      line += c;
    }
  }
#endif
}
