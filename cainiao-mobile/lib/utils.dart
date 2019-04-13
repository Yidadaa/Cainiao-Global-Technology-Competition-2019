import 'dart:convert';

String event2string (double x, double y, double z) {
  return [x.toStringAsFixed(8), y.toStringAsFixed(8), z.toStringAsFixed(8)].join(', ');
}

class CNEvent {
  double _x, _y, _z;
  int ts;

  CNEvent (double x, double y, double z) {
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