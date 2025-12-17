import 'dart:async';
import 'dart:typed_data';
import 'dart:math'; // 🆕 新增：用於計算 max
import 'package:flutter/material.dart';
import 'package:flutter_reactive_ble/flutter_reactive_ble.dart';
import 'package:permission_handler/permission_handler.dart';
import 'package:intl/intl.dart';

// ==========================================
// 1. 設定與 STM32 協調好的 UUID 與參數
// ==========================================
const String TARGET_DEVICE_NAME = "BikeLocker";

final Uuid SERVICE_UUID = Uuid.parse("0000ffe0-0000-1000-8000-00805f9b34fb");

final Uuid CHAR_LOCK_UUID = Uuid.parse("0000ffe1-0000-1000-8000-00805f9b34fb"); // Control
final Uuid CHAR_HISTORY_UUID = Uuid.parse("0000ffe2-0000-1000-8000-00805f9b34fb"); // History
final Uuid CHAR_SPEED_UUID = Uuid.parse("0000ffe3-0000-1000-8000-00805f9b34fb"); // Speed
final Uuid CHAR_CALORIE_UUID = Uuid.parse("0000ffe4-0000-1000-8000-00805f9b34fb"); // Calorie

final _ble = FlutterReactiveBle();

void main() {
  runApp(const BikeLockerApp());
}

class BikeLockerApp extends StatelessWidget {
  const BikeLockerApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'BikeLocker',
      theme: ThemeData(
        primarySwatch: Colors.indigo,
        useMaterial3: true,
      ),
      home: const BikeHomePage(),
    );
  }
}

class BikeHomePage extends StatefulWidget {
  const BikeHomePage({super.key});

  @override
  State<BikeHomePage> createState() => _BikeHomePageState();
}

class _BikeHomePageState extends State<BikeHomePage> {
  StreamSubscription<DiscoveredDevice>? _scanSub;
  StreamSubscription<ConnectionStateUpdate>? _connSub;

  StreamSubscription<List<int>>? _lockNotifySub;
  StreamSubscription<List<int>>? _historyNotifySub;
  StreamSubscription<List<int>>? _speedNotifySub;
  StreamSubscription<List<int>>? _calorieNotifySub;

  DiscoveredDevice? _connectedDevice;
  bool _isConnected = false;
  bool _isScanning = false;
  String _statusText = "準備就緒";

  String _lockState = "未知";
  double _currentSpeed = 0.0;
  double _burntCalories = 0.0;
  List<String> _historyLogs = [];

  // 🆕 新增：用於批次處理歷史紀錄的緩衝區與計時器
  Timer? _historyBatchTimer;
  final List<int> _tempHistoryValues = [];

  @override
  void initState() {
    super.initState();
    _initPermissions();
  }

  @override
  void dispose() {
    _disconnect();
    super.dispose();
  }

  Future<void> _initPermissions() async {
    await [
      Permission.bluetooth,
      Permission.location,
      Permission.bluetoothScan,
      Permission.bluetoothConnect,
    ].request();
  }

  // ==========================================
  // 2. 掃描與連線邏輯
  // ==========================================
  void _startScan() {
    setState(() {
      _isScanning = true;
      _statusText = "正在搜尋 $TARGET_DEVICE_NAME...";
      _historyLogs.clear();
    });

    _scanSub?.cancel();
    _scanSub = _ble.scanForDevices(withServices: []).listen((device) {
      if (device.name == TARGET_DEVICE_NAME) {
        print("找到裝置: ${device.name} (${device.id})");
        _scanSub?.cancel();
        _connectToDevice(device);
      }
    }, onError: (e) {
      setState(() {
        _statusText = "掃描錯誤: $e";
        _isScanning = false;
      });
    });
  }

  void _connectToDevice(DiscoveredDevice device) {
    setState(() {
      _isScanning = false;
      _statusText = "正在連線...";
    });

    _connSub = _ble.connectToDevice(
      id: device.id,
      connectionTimeout: const Duration(seconds: 10),
    ).listen((state) {
      if (state.connectionState == DeviceConnectionState.connected) {
        setState(() {
          _isConnected = true;
          _connectedDevice = device;
          _statusText = "已連線";
        });

        // 🛠️ 修正：加入 500ms 延遲，確保連線穩定後再讀取
        Future.delayed(const Duration(milliseconds: 500), () {
          if (mounted) { // 確保頁面還在
            _subscribeToAllCharacteristics(device.id);
          }
        });

      } else if (state.connectionState == DeviceConnectionState.disconnected) {
        _handleDisconnect();
      }
    }, onError: (e) {
      setState(() => _statusText = "連線失敗: $e");
      _handleDisconnect();
    });
  }

  void _handleDisconnect() {
    setState(() {
      _isConnected = false;
      _connectedDevice = null;
      _statusText = "已斷線";
      _lockState = "未知";
      _currentSpeed = 0.0;
      _burntCalories = 0.0;
    });
    _lockNotifySub?.cancel();
    _historyNotifySub?.cancel();
    _speedNotifySub?.cancel();
    _calorieNotifySub?.cancel();
    // 🆕 清除計時器與緩衝區
    _historyBatchTimer?.cancel();
    _tempHistoryValues.clear();
  }

  Future<void> _disconnect() async {
    await _connSub?.cancel();
    _handleDisconnect();
  }

  // ==========================================
  // 3. 訂閱通知與數據解析 (核心邏輯)
  // ==========================================
  Future<void> _subscribeToAllCharacteristics(String deviceId) async { // ✅ 已加入 async

    // 1. Lock Control (FFE1)
    final lockChar = QualifiedCharacteristic(
        serviceId: SERVICE_UUID, characteristicId: CHAR_LOCK_UUID, deviceId: deviceId);

    // 🛠️ 修正：主動讀取 + 詳細除錯 Log
    try {
      print("🚀 準備讀取初始鎖定狀態..."); // Debug Log
      final initialData = await _ble.readCharacteristic(lockChar);
      print("📩 讀取到的原始數據 (Bytes): $initialData"); // Debug Log - 請看這裡印出什麼

      if (initialData.length >= 2) {
        final state = initialData[1];
        setState(() {
          _lockState = (state == 0x01) ? "已上鎖 🔒" : "已解鎖 🔓";
        });
      } else if (initialData.isNotEmpty) {
        // 容錯：如果 nRF Connect 只填了 1 byte (例如 [1])
        final state = initialData[0];
        setState(() {
          _lockState = (state == 0x01) ? "已上鎖 🔒" : "已解鎖 🔓";
        });
      } else {
        print("⚠️ 警告：讀取到的數據為空！請檢查 nRF Connect 設定");
      }
    } catch (e) {
      print("❌ 讀取初始狀態失敗: $e");
    }

    // 繼續訂閱
    _lockNotifySub = _ble.subscribeToCharacteristic(lockChar).listen((data) {
      if (data.length >= 2) {
        final state = data[1];
        setState(() {
          if (state == 0x01) {
            _lockState = "已上鎖 🔒";
            // 上鎖時保留熱量數值，不歸零
          } else {
            _lockState = "已解鎖 🔓";
            // 只有在「解鎖」時才歸零熱量，準備開始新的一次騎乘
            _burntCalories = 0.0;
          }
        });
      } else if (data.length == 1) { // 增加容錯
        final state = data[0];
        setState(() {
          if (state == 0x01) {
            _lockState = "已上鎖 🔒";
          } else {
            _lockState = "已解鎖 🔓";
            _burntCalories = 0.0;
          }
        });
      }
    });

    // 2. History (FFE2) - 🆕 修改為 Uptime 回推邏輯
    final historyChar = QualifiedCharacteristic(
        serviceId: SERVICE_UUID, characteristicId: CHAR_HISTORY_UUID, deviceId: deviceId);

    _historyNotifySub = _ble.subscribeToCharacteristic(historyChar).listen((data) {
      if (data.length >= 4) {
        final bd = ByteData.sublistView(Uint8List.fromList(data));
        // 這裡讀到的是開機秒數 (Uptime Seconds)
        final uptime = bd.getUint32(0, Endian.little);

        // 收集數據到緩衝區
        _tempHistoryValues.add(uptime);

        // 重置/啟動防抖計時器 (等待所有封包到齊)
        _historyBatchTimer?.cancel();
        _historyBatchTimer = Timer(const Duration(milliseconds: 500), _processHistoryBatch);
      }
    });

    // 3. Speed (FFE3)
    final speedChar = QualifiedCharacteristic(
        serviceId: SERVICE_UUID, characteristicId: CHAR_SPEED_UUID, deviceId: deviceId);

    _speedNotifySub = _ble.subscribeToCharacteristic(speedChar).listen((data) {
      if (data.length >= 2) {
        final bd = ByteData.sublistView(Uint8List.fromList(data));
        final rawSpeed = bd.getUint16(0, Endian.little);
        setState(() {
          _currentSpeed = rawSpeed / 10.0;
        });
      }
    });

    // 4. Calorie (FFE4)
    final calChar = QualifiedCharacteristic(
        serviceId: SERVICE_UUID, characteristicId: CHAR_CALORIE_UUID, deviceId: deviceId);

    _calorieNotifySub = _ble.subscribeToCharacteristic(calChar).listen((data) {
      if (data.length >= 2) {
        final bd = ByteData.sublistView(Uint8List.fromList(data));
        final rawCal = bd.getUint16(0, Endian.little);

        // 只有在「已解鎖」狀態下才更新熱量
        // 這樣上鎖後即使 STM32 傳來 0 或其他值，UI 也會保留最後的數據
        if (_lockState.contains("已解鎖")) {
          setState(() {
            _burntCalories = rawCal / 10.0;
          });
        }
      }
    });
  }

  // 🆕 新增：批次處理歷史紀錄
  void _processHistoryBatch() {
    if (_tempHistoryValues.isEmpty) return;

    // 情況 A: 收到 0，代表無異常
    if (_tempHistoryValues.contains(0)) {
      setState(() {
        _historyLogs.insert(0, "✅ 狀態正常 (無異常震動)");
      });
      _tempHistoryValues.clear();
      return;
    }

    // 情況 B: 有異常紀錄
    // 假設最大值是「解鎖當下的 Uptime」(也就是現在)
    int unlockUptime = _tempHistoryValues.reduce(max);
    final now = DateTime.now();
    List<String> newLogs = [];

    // 排序 (從小到大)，確保處理順序
    _tempHistoryValues.sort();

    for (var uptime in _tempHistoryValues) {
      // 過濾掉解鎖當下的那筆紀錄
      if (uptime == unlockUptime) continue;

      // 計算時間差：異常發生在幾秒前
      int diffSeconds = unlockUptime - uptime;
      if (diffSeconds < 0) diffSeconds = 0; // 防呆

      // 回推真實時間
      final eventTime = now.subtract(Duration(seconds: diffSeconds));
      final formatted = DateFormat('yyyy/MM/dd HH:mm:ss').format(eventTime);

      newLogs.add("⚠️ 異常震動: $formatted");
    }

    if (newLogs.isNotEmpty) {
      setState(() {
        // 反轉列表，讓最新的紀錄顯示在最上面
        _historyLogs.insertAll(0, newLogs.reversed);
      });
    }

    // 清空緩衝區
    _tempHistoryValues.clear();
  }

  // ==========================================
  // 4. 發送指令 (Write)
  // ==========================================
  Future<void> _sendCommand(int command) async {
    if (!_isConnected || _connectedDevice == null) return;

    final lockChar = QualifiedCharacteristic(
        serviceId: SERVICE_UUID,
        characteristicId: CHAR_LOCK_UUID,
        deviceId: _connectedDevice!.id
    );

    final data = Uint8List.fromList([command]);

    try {
      await _ble.writeCharacteristicWithResponse(lockChar, value: data);
      String msg = "";
      switch(command) {
        case 0x01: msg = "發送: 上鎖"; break;
        case 0x02: msg = "發送: 解鎖"; break;
        case 0x03: msg = "發送: 尋車響鈴"; break;
      }
      ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text(msg), duration: const Duration(milliseconds: 500)));
    } catch (e) {
      setState(() => _statusText = "指令失敗: $e");
    }
  }

  // ==========================================
  // 5. UI 建構
  // ==========================================
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text("BikeLocker"),
        backgroundColor: Colors.indigo,
        foregroundColor: Colors.white,
        actions: [
          IconButton(
            icon: Icon(_isConnected ? Icons.bluetooth_connected : Icons.bluetooth_disabled),
            onPressed: _isConnected ? _disconnect : null,
          )
        ],
      ),
      body: Column(
        children: [
          Container(
            width: double.infinity,
            padding: const EdgeInsets.all(12),
            color: _isConnected ? Colors.green[100] : Colors.grey[200],
            child: Text(
              _statusText,
              textAlign: TextAlign.center,
              style: TextStyle(
                  color: _isConnected ? Colors.green[800] : Colors.black54,
                  fontWeight: FontWeight.bold
              ),
            ),
          ),
          Expanded(
            child: _isConnected ? _buildDashboard() : _buildConnectScreen(),
          ),
        ],
      ),
      floatingActionButton: !_isConnected
          ? FloatingActionButton.extended(
        onPressed: _isScanning ? null : _startScan,
        icon: _isScanning
            ? const SizedBox(width: 20, height: 20, child: CircularProgressIndicator(color: Colors.white))
            : const Icon(Icons.search),
        label: Text(_isScanning ? "搜尋中..." : "掃描裝置"),
      )
          : null,
    );
  }

  Widget _buildConnectScreen() {
    return Center(
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          Icon(Icons.pedal_bike, size: 100, color: Colors.grey[300]),
          const SizedBox(height: 20),
          const Text("請掃描並連接 BikeLocker", style: TextStyle(color: Colors.grey)),
        ],
      ),
    );
  }

  Widget _buildDashboard() {
    return ListView(
      padding: const EdgeInsets.all(16),
      children: [
        Card(
          elevation: 4,
          child: Padding(
            padding: const EdgeInsets.all(16),
            child: Column(
              children: [
                const Text("鎖定狀態", style: TextStyle(color: Colors.grey)),
                Text(
                  _lockState,
                  style: TextStyle(
                      fontSize: 32,
                      fontWeight: FontWeight.bold,
                      color: _lockState.contains("上鎖") ? Colors.red : Colors.green
                  ),
                ),
                const SizedBox(height: 20),
                Row(
                  mainAxisAlignment: MainAxisAlignment.spaceEvenly,
                  children: [
                    _buildCmdBtn("上鎖", Icons.lock, Colors.red, 0x01),
                    _buildCmdBtn("解鎖", Icons.lock_open, Colors.green, 0x02),
                  ],
                ),
                const SizedBox(height: 10),
                SizedBox(
                  width: double.infinity,
                  child: OutlinedButton.icon(
                    onPressed: () => _sendCommand(0x03),
                    icon: const Icon(Icons.notifications_active),
                    label: const Text("尋車鈴聲 (Ringing)"),
                  ),
                )
              ],
            ),
          ),
        ),
        const SizedBox(height: 16),
        Row(
          children: [
            Expanded(child: _buildInfoCard("目前時速", "${_currentSpeed.toStringAsFixed(1)}", "km/h", Icons.speed)),
            const SizedBox(width: 16),
            Expanded(child: _buildInfoCard("消耗熱量", "${_burntCalories.toStringAsFixed(1)}", "kcal", Icons.local_fire_department)),
          ],
        ),
        const SizedBox(height: 16),
        const Text("  異常震動紀錄", style: TextStyle(fontWeight: FontWeight.bold, fontSize: 16)),
        const SizedBox(height: 8),
        Container(
          height: 200,
          decoration: BoxDecoration(
              border: Border.all(color: Colors.grey.shade300),
              borderRadius: BorderRadius.circular(8),
              color: Colors.grey.shade50
          ),
          child: _historyLogs.isEmpty
              ? const Center(child: Text("暫無異常紀錄", style: TextStyle(color: Colors.grey)))
              : ListView.separated(
            itemCount: _historyLogs.length,
            separatorBuilder: (c, i) => const Divider(height: 1),
            itemBuilder: (context, index) {
              return ListTile(
                leading: const Icon(Icons.warning_amber_rounded, color: Colors.orange),
                title: Text(_historyLogs[index], style: const TextStyle(fontSize: 14)),
                dense: true,
              );
            },
          ),
        ),
      ],
    );
  }

  Widget _buildCmdBtn(String label, IconData icon, Color color, int cmd) {
    return ElevatedButton.icon(
      onPressed: () => _sendCommand(cmd),
      icon: Icon(icon),
      label: Text(label),
      style: ElevatedButton.styleFrom(
        backgroundColor: color,
        foregroundColor: Colors.white,
        padding: const EdgeInsets.symmetric(horizontal: 24, vertical: 12),
      ),
    );
  }

  Widget _buildInfoCard(String title, String value, String unit, IconData icon) {
    return Card(
      elevation: 2,
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          children: [
            Icon(icon, color: Colors.indigo),
            const SizedBox(height: 8),
            Text(title, style: const TextStyle(fontSize: 12, color: Colors.grey)),
            Text(value, style: const TextStyle(fontSize: 24, fontWeight: FontWeight.bold)),
            Text(unit, style: const TextStyle(fontSize: 12, color: Colors.grey)),
          ],
        ),
      ),
    );
  }
}