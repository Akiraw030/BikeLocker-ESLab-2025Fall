# A spec for STM and App part bridging

|Data name|UUID (Characteristic)|Properties|Data|Data content|Usage|
|-|-|-|-|-|-|
|Advertisement name|ffe0 (service)||String|BikeLocker|BLE connection|
|Lock control and bike find|ffe1|Write, Notify|Bytes UInt8|0x01 Lock, 0x02 Unlock, 0x03 Ringing|For locking action and locking state|
|Abnormal history|ffe2|Notify|Bytes UInt32|Unix Timestamp|Return abnormal history when unlock|
|Speed|ffe3|Nofity|Bytes UInt16|0.1km/h|Return current speed|
|Calorie|ffe4|Notify|Bytes UInt16|0.1cal|Return current burnt calorie|