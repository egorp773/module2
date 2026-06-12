// Module2 v1 — manual robot control over WebSocket.
//
// Protocol (matches firmware):
//   out:  "M,<left>,<right>"   // -70..70 each
//   out:  "STOP"
//   out:  "ATTACHMENT_ON" / "ATTACHMENT_OFF"
//   out:  "MOUNT_ON" / "MOUNT_OFF"
//   in:   "STATE,CONNECTED"
//   in:   "BAT_PCT,<pct>"      // 0..100
//
// WiFi target: ESP32 in STA mode on your home router. IP is shown in
// firmware Serial output on boot. Enter it in the host field below.
//
// UI: one differential joystick. Y axis = gas, X axis = steering.
//     Output: left  = clamp((y - x) * 70)
//             right = clamp((y + x) * 70)

import 'dart:async';

import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;
import 'package:web_socket_channel/web_socket_channel.dart';

const int _kEsp32Port = 81;

void main() => runApp(const Module2App());

class Module2App extends StatelessWidget {
  const Module2App({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Module2',
      theme: ThemeData(
        useMaterial3: true,
        colorScheme: ColorScheme.fromSeed(seedColor: Colors.indigo),
      ),
      home: const ControlScreen(),
    );
  }
}

class ControlScreen extends StatefulWidget {
  const ControlScreen({super.key});

  @override
  State<ControlScreen> createState() => _ControlScreenState();
}

class _ControlScreenState extends State<ControlScreen> {
  final _hostController = TextEditingController(text: '192.168.4.1');

  WebSocketChannel? _ws;
  bool _connected = false;
  String _status = 'enter ESP32 IP and tap Connect';
  int? _batteryPct;

  bool _attachmentOn = false;
  bool _mountOn = false;

  int? _outLeft;
  int? _outRight;

  Timer? _moveThrottle;
  Timer? _pingTimer;

  @override
  void dispose() {
    _moveThrottle?.cancel();
    _pingTimer?.cancel();
    _ws?.sink.close();
    _hostController.dispose();
    super.dispose();
  }

  String get _host => _hostController.text.trim();

  Future<void> _connect() async {
    if (_host.isEmpty) {
      setState(() => _status = 'host is empty');
      return;
    }
    setState(() => _status = 'connecting…');
    // Best-effort HTTP ping to wake the server up before opening WS.
    try {
      await http
          .get(Uri.parse('http://$_host:$_kEsp32Port/ping'))
          .timeout(const Duration(seconds: 2));
    } catch (_) {
      // ignore — WS open will report the real error
    }

    final uri = Uri.parse('ws://$_host:$_kEsp32Port/ws');
    try {
      final ch = WebSocketChannel.connect(uri);
      _ws = ch;
      ch.stream.listen(
        _onMessage,
        onDone: _onDisconnect,
        onError: (_) => _onDisconnect(),
        cancelOnError: true,
      );
      setState(() => _status = 'ws opened, waiting for STATE…');
    } catch (e) {
      setState(() => _status = 'ws open failed: $e');
    }
  }

  void _disconnect() {
    _pingTimer?.cancel();
    try {
      _ws?.sink.close();
    } catch (_) {}
    _ws = null;
    setState(() {
      _connected = false;
      _status = 'disconnected (manual)';
    });
  }

  void _onMessage(dynamic raw) {
    if (raw is! String) return;
    setState(() {
      if (raw.startsWith('STATE,CONNECTED')) {
        _connected = true;
        _status = 'connected';
        // Start keepalive ping every 5s — also keeps failsafe from triggering
        // if the user pauses the joystick.
        _pingTimer?.cancel();
        _pingTimer = Timer.periodic(const Duration(seconds: 5), (_) => _send('PING'));
        _send('PING');
      } else if (raw == 'PONG') {
        _connected = true;
        _status = 'connected';
      } else if (raw.startsWith('BAT_PCT,')) {
        _batteryPct = int.tryParse(raw.substring('BAT_PCT,'.length));
      }
    });
  }

  void _onDisconnect() {
    _pingTimer?.cancel();
    _pingTimer = null;
    if (!mounted) return;
    setState(() {
      _connected = false;
      _status = 'disconnected — auto-retry in 2s';
    });
    _ws = null;
    // Retry, but only if the user hasn't changed the host meanwhile.
    Future.delayed(const Duration(seconds: 2), () {
      if (!mounted) return;
      if (_ws == null) _connect();
    });
  }

  void _send(String msg) {
    final ch = _ws;
    if (ch == null) return;
    try {
      ch.sink.add(msg);
    } catch (_) {
      // ignore — onError will fire and we'll reconnect
    }
  }

  // Throttle stick updates to ~30 Hz so we don't flood the WS.
  void _pushMove(int left, int right) {
    _moveThrottle?.cancel();
    _moveThrottle = Timer(const Duration(milliseconds: 33), () {
      _send('M,$left,$right');
    });
  }

  void _onStickChange(Offset norm) {
    const speedCmd = 70; // matches firmware MAX_SPEED_PERCENT
    final double y = -norm.dy; // screen Y grows downward
    final double x = norm.dx;
    double l = (y - x) * speedCmd;
    double r = (y + x) * speedCmd;
    l = l.clamp(-speedCmd.toDouble(), speedCmd.toDouble());
    r = r.clamp(-speedCmd.toDouble(), speedCmd.toDouble());
    final int li = l.round();
    final int ri = r.round();
    if (li == _outLeft && ri == _outRight) return;
    _outLeft = li;
    _outRight = ri;
    _pushMove(li, ri);
  }

  void _onStickRelease() {
    _outLeft = 0;
    _outRight = 0;
    _send('M,0,0');
  }

  void _stop() {
    _outLeft = 0;
    _outRight = 0;
    _send('STOP');
  }

  void _toggleAttachment() {
    _attachmentOn = !_attachmentOn;
    _send(_attachmentOn ? 'ATTACHMENT_ON' : 'ATTACHMENT_OFF');
    setState(() {});
  }

  void _toggleMount() {
    _mountOn = !_mountOn;
    _send(_mountOn ? 'MOUNT_ON' : 'MOUNT_OFF');
    setState(() {});
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Module2 — manual'),
        actions: [
          Padding(
            padding: const EdgeInsets.only(right: 12),
            child: Center(child: _batteryPct == null
                ? const Text('—%')
                : Text('$_batteryPct%')),
          ),
        ],
      ),
      body: Column(
        children: [
          Container(
            width: double.infinity,
            padding: const EdgeInsets.symmetric(vertical: 6, horizontal: 12),
            color: _connected ? Colors.green.shade100 : Colors.red.shade100,
            child: Text(_status, style: const TextStyle(fontSize: 12)),
          ),
          Padding(
            padding: const EdgeInsets.fromLTRB(12, 8, 12, 0),
            child: Row(
              children: [
                Expanded(
                  child: TextField(
                    controller: _hostController,
                    enabled: !_connected,
                    keyboardType: TextInputType.number,
                    decoration: const InputDecoration(
                      labelText: 'ESP32 IP',
                      border: OutlineInputBorder(),
                      isDense: true,
                    ),
                  ),
                ),
                const SizedBox(width: 8),
                FilledButton(
                  onPressed: _connected ? _disconnect : _connect,
                  child: Text(_connected ? 'Disconnect' : 'Connect'),
                ),
              ],
            ),
          ),
          Expanded(
            child: Center(
              child: AspectRatio(
                aspectRatio: 1,
                child: _JoystickPad(
                  onChanged: _onStickChange,
                  onRelease: _onStickRelease,
                ),
              ),
            ),
          ),
          Padding(
            padding: const EdgeInsets.all(16),
            child: Row(
              mainAxisAlignment: MainAxisAlignment.spaceEvenly,
              children: [
                FilledButton.tonalIcon(
                  onPressed: _stop,
                  icon: const Icon(Icons.stop),
                  label: const Text('STOP'),
                ),
                FilledButton.tonalIcon(
                  onPressed: _toggleAttachment,
                  icon: Icon(_attachmentOn ? Icons.link : Icons.link_off),
                  label: const Text('ATTACH'),
                ),
                FilledButton.tonalIcon(
                  onPressed: _toggleMount,
                  icon: Icon(_mountOn ? Icons.camera : Icons.no_photography),
                  label: const Text('MOUNT'),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }
}

/// Simple pad-based joystick. Reports normalized offset from center.
class _JoystickPad extends StatefulWidget {
  const _JoystickPad({required this.onChanged, required this.onRelease});

  final ValueChanged<Offset> onChanged;
  final VoidCallback onRelease;

  @override
  State<_JoystickPad> createState() => _JoystickPadState();
}

class _JoystickPadState extends State<_JoystickPad> {
  Offset _knob = Offset.zero;

  void _setFromLocal(Offset local, Size size) {
    final r = size.shortestSide / 2;
    final c = Offset(r, r);
    Offset p = local - c;
    final dist = p.distance;
    if (dist > r) p = p / dist * r;
    setState(() => _knob = p);
    widget.onChanged(Offset(p.dx / r, p.dy / r));
  }

  @override
  Widget build(BuildContext context) {
    return LayoutBuilder(
      builder: (context, constraints) {
        final size = Size(constraints.maxWidth, constraints.maxHeight);
        final r = size.shortestSide / 2;
        return GestureDetector(
          onPanDown: (d) => _setFromLocal(d.localPosition, size),
          onPanUpdate: (d) => _setFromLocal(d.localPosition, size),
          onPanEnd: (_) {
            setState(() => _knob = Offset.zero);
            widget.onRelease();
          },
          onPanCancel: () {
            setState(() => _knob = Offset.zero);
            widget.onRelease();
          },
          child: Container(
            margin: const EdgeInsets.all(24),
            decoration: BoxDecoration(
              shape: BoxShape.circle,
              color: Colors.grey.shade200,
              border: Border.all(color: Colors.grey.shade400, width: 2),
            ),
            child: Center(
              child: Transform.translate(
                offset: _knob,
                child: Container(
                  width: r * 0.5,
                  height: r * 0.5,
                  decoration: BoxDecoration(
                    shape: BoxShape.circle,
                    color: Colors.indigo.shade400,
                  ),
                  child: const Center(
                    child: Icon(Icons.touch_app, color: Colors.white, size: 28),
                  ),
                ),
              ),
            ),
          ),
        );
      },
    );
  }
}
