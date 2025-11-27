# A spec for STM and App part bridging

|Data name|UUID (Characteristic)|Properties|Data|Data content|Usage|
|-|-|-|-|-|-|
|Advertisement name|||String|BikeLocker|BLE connection|
|Lock control and bike find||Write, Notify|Bytes UInt8|0x01 Lock, 0x02 Unlock, 0x03 Ringing|For locking action and locking state|
|Abnormal history||Notify|Bytes UInt32|Unix Timestamp|Return abnormal history when unlock|
|Speed||Nofity|Bytes UInt16|0.1km/h|Return current speed|
|Calorie||Notify|Bytes UInt16|0.1cal|Return current burnt calorie|
