import 'package:flutter/material.dart';
import './utils.dart';

class DataPage extends StatefulWidget {
  @override
  _DataPageState createState() => _DataPageState();
}

class _DataPageState extends State<DataPage> {
  List<FileWithInfo> cache = [];
  var _scaffoldKey = new GlobalKey<ScaffoldState>();

  String loadingName = '';
  double loadingPercent = 0;

  void showSheet(context, FileWithInfo f) {
    showModalBottomSheet(
        context: context,
        builder: (context) {
          return Container(
              child: Wrap(
            children: <Widget>[
              ListTile(
                  leading: Icon(Icons.info),
                  title: Text("状态：" + (f.isUploaded ? '已上传' : "未上传")),
                  onTap: () {}),
              ListTile(
                  leading: Icon(Icons.file_upload),
                  title: Text("上传此文件"),
                  onTap: () {
                    if (!f.isUploaded) {
                      loadingName = f.name;
                      uploadFile(f, (int cur, int total) {
                        setState(() {
                          loadingPercent = cur/total * 100;
                        });
                        if (cur == total) {
                          loadingName = '';
                          f.updateLoadStatus(true);
                          getFiles();
                          toast('上传完成');
                        }
                      });
                    } else {
                      toast('此文件已上传');
                    }
                    Navigator.of(context).pop();
                  }),
              ListTile(
                leading: Icon(Icons.delete),
                title: Text(
                  "删除此文件",
                ),
                onTap: () async {
                  f.delete();
                  getFiles();
                  Navigator.of(context).pop();
                  toast('已删除');
                },
              ),
            ],
          ));
        });
  }

  void toast(String text) {
    _scaffoldKey.currentState.removeCurrentSnackBar();
    _scaffoldKey.currentState.showSnackBar(SnackBar(
      content: Text(text),
      action: SnackBarAction(
        label: '好的',
        onPressed: () {
          _scaffoldKey.currentState.hideCurrentSnackBar();
        },
      ),
    ));
  }

  void getFiles() async {
    List files = await listFiles();
    setState(() {
      cache = files;
    });
  }

  @override
  void initState() {
    getFiles();
    super.initState();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
        key: _scaffoldKey,
        appBar: AppBar(
          title: Text('已录制文件'),
        ),
        body: ListView(
          children: cache.map((FileWithInfo f) {
            return ListTile(
              leading: Text(f.name),
              trailing: f.name == loadingName ?
                Text(loadingPercent.toStringAsFixed(2) + '%')
                : Text(f.isUploaded ? '已上传' : '未上传'),
              onTap: () => showSheet(context, f),
            );
          }).toList(),
        ));
  }
}
