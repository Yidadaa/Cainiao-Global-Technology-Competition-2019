import 'package:flutter/material.dart';
import 'package:sensors/sensors.dart';
import 'dart:async';
import 'package:camera/camera.dart';

import './utils.dart';

List<CameraDescription> cameras;

Future main() async {
  cameras = await availableCameras();
  runApp(MyApp());
}

class MyApp extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Flutter Demo',
      theme: ThemeData(
        primarySwatch: Colors.blue,
      ),
      home: MyHomePage(title: '菜鸟挑战赛Demo'),
    );
  }
}

class MyHomePage extends StatefulWidget {
  MyHomePage({Key key, this.title}) : super(key: key);

  final String title;

  @override
  _MyHomePageState createState() => _MyHomePageState();
}

class _MyHomePageState extends State<MyHomePage> {
  String gyroscopeData = "";
  String accelerometerData = "";

  bool isRecording = false;
  DateTime startTime = new DateTime.now();
  DateTime currentTime = new DateTime.now();
  String timeText = "0s";

  EventCache gcache = EventCache();
  EventCache acache = EventCache();
  List<CameraImage> imcache;

  CameraController controller;
  Timer _timer;

  @override
  void initState() {
    super.initState();
    gyroscopeEvents.listen((GyroscopeEvent event) {
      if (isRecording) {
        gcache.addEvent(CNEvent(event.x, event.y, event.z));
      }
      int n = gcache.cache.length;
      if (n % 100 == 0) {
        updateText(event2string(event.x, event.y, event.z) + ' '+ gcache.cache.length.toString(), "");
      }
    });
    accelerometerEvents.listen((AccelerometerEvent event) {
      if (isRecording) {
        acache.addEvent(CNEvent(event.x, event.y, event.z));
      }
      int n = acache.cache.length;
      if (n % 100 == 0) {
        updateText("", event2string(event.x, event.y, event.z) + ' '+ acache.cache.length.toString());
      }
    });
    controller = CameraController(cameras[0], ResolutionPreset.medium);
    controller.initialize().then((_) {
      if (!mounted) {
        return;
      }
      setState(() {});
    });
    initTimer();
    print("初始化完毕");
  }

  void updateText(String gyroscopeData_, String accelerometerData_) {
    setState(() {
      if (gyroscopeData_.length > 0) gyroscopeData = gyroscopeData_;
      if (accelerometerData_.length > 0) accelerometerData = accelerometerData_;
    });
  }

  void onRecordButtonPressed() {
    setState(() {
      startTime = new DateTime.now();
      currentTime = new DateTime.now();
      isRecording = !isRecording;
      if (isRecording) {
        timeText = '0s';
        endRecord();
      } else {
        startRecord();
      }
    });
  }

  void startRecord() {
    if (controller.value.isInitialized &&
        !controller.value.isRecordingVideo &&
        !controller.value.isStreamingImages &&
        !controller.value.isTakingPicture) {
      clearCache();
      controller.startImageStream((CameraImage img) {
        imcache.add(img);
      });
    }
  }

  void endRecord() {
    controller.stopImageStream();
    // TODO: 持久化录制的数据，并发送到服务端
  }

  void clearCache() {
    gcache.clear();
    acache.clear();
    imcache.clear();
  }

  void initTimer() {
    const oneSec = const Duration(seconds: 1);
    _timer = new Timer.periodic(oneSec, (Timer timer) {
      setState(() {
        if (isRecording) {
          currentTime = currentTime.add(oneSec);
          timeText = currentTime.difference(startTime).inSeconds.toString() + 's';
        }
      });
    });
  }

  @override
  void dispose() {
    controller?.dispose();
    _timer?.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    if (!controller.value.isInitialized) {
      return Container();
    }
    return Scaffold(
      appBar: AppBar(
        title: Text(widget.title),
      ),
      body: Column(
        children: <Widget>[
          ListTile(
            leading: Text('加速度'),
            trailing: Text(accelerometerData),
          ),
          ListTile(
            leading: Text('陀螺仪'),
            trailing: Text(gyroscopeData),
          ),
          AspectRatio(
              aspectRatio: controller.value.aspectRatio,
              child: CameraPreview(controller)),
          RaisedButton.icon(
            label: Text(isRecording ? timeText : '开始录制'),
            onPressed: onRecordButtonPressed,
            icon: Icon(isRecording ? Icons.stop : Icons.play_arrow),
          )
        ],
      ),
    );
  }
}
