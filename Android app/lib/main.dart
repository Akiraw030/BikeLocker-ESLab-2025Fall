import 'dart:async';
import 'dart:typed_data';
import 'package:flutter/material.dart';
import 'package:flutter_reactive_ble/flutter_reactive_ble.dart';
import 'package:permission_handler/permission_handler.dart';
import 'package:intl/intl.dart'; // 用於時間格式化，需在 pubspec.yaml 加入 intl

// ==========================================
// 1. 設定與 STM32 協調好的 UUID 與參數
// ==========================================
const String TARGET_DEVICE_NAME = "BikeLocker";

// Base UUID: 0000xxxx-0000-1000-8000-00805F9B34FB
final Uuid SERVICE_UUID = Uuid.parse("0000ffe0-0000-1000-8000-00805f9b34fb");

// Characteristics
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
  // BLE 狀態管理
  StreamSubscription<DiscoveredDevice>? _scanSub;
  StreamSubscription<ConnectionStateUpdate>? _connSub;

  // 數據訂閱流
  StreamSubscription<List<int>>? _lockNotifySub;
  StreamSubscription<List<int>>? _historyNotifySub;
  StreamSubscription<List<int>>? _speedNotifySub;
  StreamSubscription<List<int>>? _calorieNotifySub;

  DiscoveredDevice? _connectedDevice;
  bool _isConnected = false;
  bool _isScanning = false;
  String _statusText = "準備就緒";

  // 應用數據
  String _lockState = "未知"; // 鎖定狀態
  double _currentSpeed = 0.0; // km/h
  double _burntCalories = 0.0; // kcal
  List<String> _historyLogs = []; // 異常紀錄列表

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
    // 請求必要的藍牙與定位權限
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
      _historyLogs.clear(); // 清除舊紀錄
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
        // 連線成功後，訂閱所有特徵值
        _subscribeToAllCharacteristics(device.id);
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
    // 取消所有訂閱
    _lockNotifySub?.cancel();
    _historyNotifySub?.cancel();
    _speedNotifySub?.cancel();
    _calorieNotifySub?.cancel();
  }

  Future<void> _disconnect() async {
    await _connSub?.cancel();
    _handleDisconnect();
  }

  // ==========================================
  // 3. 訂閱通知與數據解析 (核心邏輯)
  // ==========================================
  void _subscribeToAllCharacteristics(String deviceId) {

    // 1. Lock Control (FFE1) - 接收鎖定狀態變化
    final lockChar = QualifiedCharacteristic(
        serviceId: SERVICE_UUID, characteristicId: CHAR_LOCK_UUID, deviceId: deviceId);

    _lockNotifySub = _ble.subscribeToCharacteristic(lockChar).listen((data) {
      // 預期 2 bytes: [Result, State]
      if (data.length >= 2) {
        // Byte 1 is State: 0x01 Locked, 0x00 Unlocked
        final state = data[1];
        setState(() {
          _lockState = (state == 0x01) ? "已上鎖 🔒" : "已解鎖 🔓";
        });
      }
    });

    // 2. Abnormal History (FFE2) - 接收異常時間戳記
    final historyChar = QualifiedCharacteristic(
        serviceId: SERVICE_UUID, characteristicId: CHAR_HISTORY_UUID, deviceId: deviceId);

    _historyNotifySub = _ble.subscribeToCharacteristic(historyChar).listen((data) {
      // 預期 UInt32 (4 bytes), Little Endian
      if (data.length >= 4) {
        final bd = ByteData.sublistView(Uint8List.fromList(data));
        final timestamp = bd.getUint32(0, Endian.little);

        // 轉換 Unix Timestamp 為可讀時間
        final date = DateTime.fromMillisecondsSinceEpoch(timestamp * 1000);
        final formatted = DateFormat('yyyy/MM/dd HH:mm:ss').format(date);

        setState(() {
          // 新增到列表頂端
          _historyLogs.insert(0, "⚠️ 異常震動: $formatted");
        });
      }
    });

    // 3. Speed (FFE3) - 接收速度
    final speedChar = QualifiedCharacteristic(
        serviceId: SERVICE_UUID, characteristicId: CHAR_SPEED_UUID, deviceId: deviceId);

    _speedNotifySub = _ble.subscribeToCharacteristic(speedChar).listen((data) {
      // 預期 UInt16 (2 bytes), Unit: 0.1 km/h
      if (data.length >= 2) {
        final bd = ByteData.sublistView(Uint8List.fromList(data));
        final rawSpeed = bd.getUint16(0, Endian.little);
        setState(() {
          _currentSpeed = rawSpeed / 10.0;
        });
      }
    });

    // 4. Calorie (FFE4) - 接收卡路里
    final calChar = QualifiedCharacteristic(
        serviceId: SERVICE_UUID, characteristicId: CHAR_CALORIE_UUID, deviceId: deviceId);

    _calorieNotifySub = _ble.subscribeToCharacteristic(calChar).listen((data) {
      // 預期 UInt16 (2 bytes), Unit: 0.1 cal (User requirement)
      if (data.length >= 2) {
        final bd = ByteData.sublistView(Uint8List.fromList(data));
        final rawCal = bd.getUint16(0, Endian.little);
        setState(() {
          _burntCalories = rawCal / 10.0;
        });
      }
    });
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

    // Command: 0x01 Lock, 0x02 Unlock, 0x03 Ring
    // 傳送單一 Byte
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
          // 狀態列
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
        // 1. 鎖定狀態與控制
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

        // 2. 騎乘儀表板
        Row(
          children: [
            Expanded(child: _buildInfoCard("目前時速", "${_currentSpeed.toStringAsFixed(1)}", "km/h", Icons.speed)),
            const SizedBox(width: 16),
            Expanded(child: _buildInfoCard("消耗熱量", "${_burntCalories.toStringAsFixed(1)}", "kcal", Icons.local_fire_department)),
          ],
        ),

        const SizedBox(height: 16),

        // 3. 異常紀錄 Log
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