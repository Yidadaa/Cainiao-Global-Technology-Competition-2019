import 'package:flutter/material.dart';
import 'package:sensors/sensors.dart';
import 'dart:async';
import 'package:camera/camera.dart';

import './data-page.dart';
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
  var _scaffoldKey = new GlobalKey<ScaffoldState>();

  String gyroscopeData = "";
  String accelerometerData = "";

  bool isRecording = false;
  DateTime startTime = new DateTime.now();
  DateTime currentTime = new DateTime.now();
  String timeText = "0s";

  EventCache gcache = EventCache();
  EventCache acache = EventCache();
  CNImageCache imcache = CNImageCache();
  String videoPath = '';

  String host = '';

  CameraController controller;
  Timer _timer;

  TextEditingController textController = TextEditingController();

  @override
  void initState() {
    super.initState();
    gyroscopeEvents.listen((GyroscopeEvent event) {
      if (isRecording) {
        gcache.addEvent(CNEvent(event.x, event.y, event.z));
      }
      int n = gcache.cache.length;
      if (n % 100 == 1) {
        updateText(
            event2string(event.x, event.y, event.z) +
                ' ' +
                gcache.cache.length.toString(),
            "");
      }
    });
    accelerometerEvents.listen((AccelerometerEvent event) {
      if (isRecording) {
        acache.addEvent(CNEvent(event.x, event.y, event.z));
      }
      int n = acache.cache.length;
      if (n % 100 == 1) {
        updateText(
            "",
            event2string(event.x, event.y, event.z) +
                ' ' +
                acache.cache.length.toString());
      }
    });
    controller = CameraController(cameras[0], ResolutionPreset.high);
    controller.initialize().then((_) {
      if (!mounted) {
        return;
      }
      setState(() {});
    });
    initTimer();
    print("初始化完毕");
  }

  void updateText(String _gyroscopeData, String _accelerometerData) {
    setState(() {
      if (_gyroscopeData.length > 0) gyroscopeData = _gyroscopeData;
      if (_accelerometerData.length > 0) accelerometerData = _accelerometerData;
    });
  }

  void onRecordButtonPressed() async {
    String hostname = await getHost();
    setState(() {
      startTime = new DateTime.now();
      currentTime = new DateTime.now();
      if (isRecording) {
        timeText = '0s';
        endRecord();
      } else {
        startRecord();
      }
      isRecording = !isRecording;
      host = hostname;
    });
  }

  void startRecord() async {
    if (controller.value.isInitialized &&
        !controller.value.isRecordingVideo &&
        !controller.value.isStreamingImages &&
        !controller.value.isTakingPicture) {
      clearCache();
      // controller.startImageStream((CameraImage img) {
      //   imcache.addImage(CNImage(img));
      // });
      videoPath = await getVideoPath();
      controller.startVideoRecording(videoPath);
    }
  }

  void endRecord() {
    // controller.stopImageStream();
    controller.stopVideoRecording();
    showSnackBar(true, '');
    save2file(acache, gcache, imcache, videoPath).then((file) {
      print(file.video.path);
      showSnackBar(false, file.name);
    });
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
          timeText =
              currentTime.difference(startTime).inSeconds.toString() + 's';
        }
      });
    });
  }

  void showHostDialog(context) {
    showDialog(
        context: context,
        builder: (context) {
          return AlertDialog(
            title: Text('输入Host地址'),
            content: TextField(
              controller: textController,
            ),
            actions: <Widget>[
              FlatButton(
                child: Text('取消'),
                onPressed: () {
                  Navigator.of(context).pop();
                },
              ),
              FlatButton(
                child: Text('确认'),
                onPressed: () {
                  setState(() {
                    host = textController.text;
                  });
                  setHost(host);
                  Navigator.of(context).pop();
                },
              )
            ],
          );
        });
  }

  void showSnackBar(bool isSaving, String filename) {
    final savingSnackBar = SnackBar(
        content: Row(
      children: <Widget>[Text('正在保存文件'), const CircularProgressIndicator()],
    ));

    final doneSnackBar = SnackBar(
      content: Text(filename),
      action: SnackBarAction(
        label: '查看',
        onPressed: () {
          Navigator.push(
              context, MaterialPageRoute(builder: (context) => DataPage()));
        },
      ),
    );
    // _scaffoldKey.currentState.removeCurrentSnackBar();
    _scaffoldKey.currentState.showSnackBar(isSaving ? savingSnackBar : doneSnackBar);
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
      key: _scaffoldKey,
      appBar: AppBar(
        title: Text(widget.title),
        actions: <Widget>[
          IconButton(
            icon: Icon(
              Icons.folder,
              color: Colors.white,
            ),
            onPressed: () {
              Navigator.push(
                  context, MaterialPageRoute(builder: (context) => DataPage()));
            },
          ),
        ],
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
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceAround,
            children: <Widget>[
              RaisedButton.icon(
                label: Text(isRecording ? timeText : '开始录制'),
                onPressed: onRecordButtonPressed,
                icon: Icon(isRecording ? Icons.stop : Icons.play_arrow),
              ),
              RaisedButton.icon(
                label: Text(host.length == 0 ? '未设置地址' : host),
                onPressed: () {
                  showHostDialog(context);
                },
                icon: Icon(Icons.computer),
              )
            ],
          )
        ],
      ),
    );
  }
}
