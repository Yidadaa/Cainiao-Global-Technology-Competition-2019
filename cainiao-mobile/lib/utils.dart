String event2string (double x, double y, double z) {
  return [x.toStringAsFixed(8), y.toStringAsFixed(8), z.toStringAsFixed(8)].join(', ');
}