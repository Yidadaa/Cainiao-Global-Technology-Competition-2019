import 'dart:convert';
import 'dart:async';
import 'dart:io';
import 'package:camera/camera.dart';
import 'package:path_provider/path_provider.dart';
import 'package:date_format/date_format.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:dio/dio.dart';
import 'package:path/path.dart';

String event2string(double x, double y, double z) {
  return [x.toStringAsFixed(8), y.toStringAsFixed(8), z.toStringAsFixed(8)]
      .join(', ');
}

class CNEvent {
  double _x, _y, _z;
  int ts;

  CNEvent(double x, double y, double z) {
    this._x = x;
    this._y = y;
    this._z = z;
    this.ts = new DateTime.now().millisecondsSinceEpoch;
  }
}

class EventCache {
  List<CNEvent> cache = [];

  int addEvent(CNEvent event) {
    cache.add(event);
    return cache.length;
  }

  @override
  String toString() {
    return cache.map((CNEvent e) {
      return [
        e._x.toString(),
        e._y.toString(),
        e._z.toString(),
        e.ts.toString()
      ].join(',');
    }).join('\n');
  }

  void clear() {
    this.cache.clear();
  }
}

class CNImage {
  CameraImage img;
  int ts;

  CNImage(CameraImage _img) {
    img = _img;
    ts = new DateTime.now().millisecondsSinceEpoch;
  }
}

class CNImageCache {
  List<CNImage> cache = [];

  int addImage(CNImage img) {
    cache.add(img);
    return cache.length;
  }

  @override
  String toString() {
    return cache
        .map((cnimage) {
          var img = cnimage.img;
          return [
            [
              cnimage.ts.toString(),
              img.height.toString(),
              img.width.toString(),
              img.planes.length
            ].join(','),
            img.planes
                .map((Plane plane) {
                  return plane.bytes.join(',');
                })
                .toList()
                .join('\n')
          ].join('\n');
        })
        .toList()
        .join('\n');
  }

  void clear() {
    cache.clear();
  }
}

class FileWithInfo {
  File file;
  bool isUploaded = false;
  String name = '';
  File video;

  FileWithInfo(File file) {
    this.file = file;
    this.isUploaded = file.path.contains('_uploaded');
    String videoName = file.path.replaceAll('.data', '').split('_')[2];
    this.video = new File(
        file.path.replaceAll(basename(file.path), '') + videoName + '.mp4');
    this.name = basename(file.path)
        .replaceAll('_uploaded', '')
        .replaceAll(videoName, '');
  }

  void updateLoadStatus(bool newStatus) {
    var name = basename(file.path);
    var path = file.path.replaceAll(name, '');
    name = name.replaceAll('.data', '');
    name = name.replaceAll('_uploaded', '');
    this.isUploaded = newStatus;
    file.rename(path + name + (newStatus ? '_uploaded' : '') + '.data');
  }

  void delete() {
    this.file.delete();
    this.video.delete();
  }
}

Future<FileWithInfo> save2file(EventCache acache, EventCache gcache,
    CNImageCache imcache, String videoPath) async {
  var rootDir = (await getApplicationDocumentsDirectory()).path;
  var now = new DateTime.now();
  String fileName = formatDate(now, [yyyy, '-', mm, '-', dd, '-']) +
      now.millisecondsSinceEpoch.toString();

  videoPath = basename(videoPath).replaceAll('.mp4', '');

  File theFile = new File('$rootDir/$fileName\_$videoPath.data');
  String fileBuffer = [
    ['a', acache.cache.length].join(','),
    acache.toString(),
    ['g', gcache.cache.length].join(','),
    gcache.toString(),
    // ['i', imcache.cache.length].join(','),
    // imcache.toString()
  ].join('\n');
  theFile.openWrite().write(fileBuffer);

  return FileWithInfo(theFile);
}

Future<String> getVideoPath() async {
  var rootDir = (await getApplicationDocumentsDirectory()).path;
  var now = new DateTime.now();
  String fileName =
      base64Encode(utf8.encode(now.millisecondsSinceEpoch.toString())) + '.mp4';
  return '$rootDir/$fileName';
}

Future<List> listFiles() async {
  var dir = await getApplicationDocumentsDirectory();
  return await dir
      .list()
      .where((f) => f.path.endsWith('.data'))
      .map((f) => FileWithInfo(f))
      .toList()
    ..sort((a, b) => a.file.path.compareTo(b.file.path));
}

Future<String> getHost() async {
  var prefs = await SharedPreferences.getInstance();
  return prefs.getString('host');
}

Future setHost(String host) async {
  var prefs = await SharedPreferences.getInstance();
  return prefs.setString('host', host);
}

Future uploadFile(FileWithInfo f, Function cb) async {
  String host = await getHost();
  FormData data = new FormData.from(
      {
        'file': new UploadFileInfo(f.file, basename(f.file.path)),
        'video': new UploadFileInfo(f.video, basename(f.video.path))
      });
  return await Dio().post(host, data: data, onSendProgress: cb);
}
