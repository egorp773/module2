// Module2 v2 — manual robot control with speed limiter and smooth UX.
//
// Protocol (matches firmware v2):
//   out:  "M,<left>,<right>,<max>"  // -100..100 each, max 10..100
//   out:  "STOP"
//   out:  "ATTACHMENT_ON" / "ATTACHMENT_OFF"
//   out:  "MOUNT_ON" / "MOUNT_OFF"
//   in:   "STATE,CONNECTED"
//   in:   "BAT_PCT,<pct>"           // 0..100
//
// UI: differential joystick (Y = gas, X = steering). Speed slider caps
// the command domain so the robot never accelerates past the chosen %.

import 'dart:async';

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:http/http.dart' as http;
import 'package:web_socket_channel/web_socket_channel.dart';

const int _kEsp32Port = 81;

void main() {
  SystemChrome.setSystemUIOverlayStyle(
    const SystemUiOverlayStyle(
      statusBarColor: Colors.transparent,
      statusBarIconBrightness: Brightness.light,
    ),
  );
  runApp(const Module2App());
}

// ---------- Color palette -----------------------------------------------------

class _Palette {
  static const bg = Color(0xFF0E1116);
  static const surface = Color(0xFF181C22);
  static const surfaceHigh = Color(0xFF222831);
  static const accent = Color(0xFF7AE2B0);   // mint
  static const accentDim = Color(0xFF3E8E6E);
  static const danger = Color(0xFFFF6B6B);
  static const warn = Color(0xFFFFC857);
  static const textPrimary = Color(0xFFE6E9EE);
  static const textSecondary = Color(0xFF8A93A0);
}

// ---------- App root ----------------------------------------------------------

class Module2App extends StatelessWidget {
  const Module2App({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Module2',
      debugShowCheckedModeBanner: false,
      themeMode: ThemeMode.dark,
      darkTheme: ThemeData(
        useMaterial3: true,
        brightness: Brightness.dark,
        scaffoldBackgroundColor: _Palette.bg,
        colorScheme: const ColorScheme.dark(
          primary: _Palette.accent,
          onPrimary: Color(0xFF0E1116),
          secondary: _Palette.accent,
          surface: _Palette.surface,
          onSurface: _Palette.textPrimary,
        ),
        sliderTheme: const SliderThemeData(
          activeTrackColor: _Palette.accent,
          inactiveTrackColor: _Palette.surfaceHigh,
          thumbColor: _Palette.accent,
          overlayColor: Color(0x337AE2B0),
        ),
        textTheme: const TextTheme(
          bodyLarge: TextStyle(color: _Palette.textPrimary),
          bodyMedium: TextStyle(color: _Palette.textSecondary),
          labelLarge: TextStyle(
            color: _Palette.textPrimary,
            fontWeight: FontWeight.w600,
            letterSpacing: 0.4,
          ),
        ),
      ),
      home: const ControlScreen(),
    );
  }
}

// ---------- Control screen ----------------------------------------------------

class ControlScreen extends StatefulWidget {
  const ControlScreen({super.key});

  @override
  State<ControlScreen> createState() => _ControlScreenState();
}

class _ControlScreenState extends State<ControlScreen> {
  static const _defaultHost = '192.168.4.1';

  final _hostController = TextEditingController(text: _defaultHost);

  WebSocketChannel? _ws;
  bool _connected = false;
  String _status = 'enter ESP32 IP and tap Connect';
  int? _batteryPct;

  bool _attachmentOn = false;
  bool _mountOn = false;

  // Stick output
  int _outLeft = 0;
  int _outRight = 0;

  // Speed cap (10..100), sent as 4th arg of M-command
  double _maxSpeed = 50;

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
    try {
      await http
          .get(Uri.parse('http://$_host:$_kEsp32Port/ping'))
          .timeout(const Duration(seconds: 2));
    } catch (_) {}

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
        _pingTimer?.cancel();
        _pingTimer = Timer.periodic(
          const Duration(seconds: 5),
          (_) => _send('PING'),
        );
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
    } catch (_) {}
  }

  void _pushMove(int left, int right, int max) {
    _moveThrottle?.cancel();
    _moveThrottle = Timer(const Duration(milliseconds: 33), () {
      _send('M,$left,$right,$max');
    });
  }

  void _onStickChange(Offset norm) {
    const cap = 100;
    final double y = -norm.dy;
    final double x = norm.dx;
    double l = (y - x) * cap;
    double r = (y + x) * cap;
    l = l.clamp(-cap.toDouble(), cap.toDouble());
    r = r.clamp(-cap.toDouble(), cap.toDouble());
    final int li = l.round();
    final int ri = r.round();
    if (li == _outLeft && ri == _outRight) return;
    _outLeft = li;
    _outRight = ri;
    _pushMove(li, ri, _maxSpeed.round());
  }

  void _onStickRelease() {
    _outLeft = 0;
    _outRight = 0;
    _send('M,0,0,${_maxSpeed.round()}');
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

  // ----- Battery color -----
  Color get _batteryColor {
    final p = _batteryPct;
    if (p == null) return _Palette.textSecondary;
    if (p < 20) return _Palette.danger;
    if (p < 50) return _Palette.warn;
    return _Palette.accent;
  }

  // ----- Build -----

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: _Palette.bg,
      body: SafeArea(
        child: Column(
          children: [
            _StatusBar(
              connected: _connected,
              status: _status,
              batteryPct: _batteryPct,
              batteryColor: _batteryColor,
            ),
            _ConnectionRow(
              hostController: _hostController,
              connected: _connected,
              onConnect: _connect,
              onDisconnect: _disconnect,
            ),
            Expanded(
              child: Padding(
                padding: const EdgeInsets.fromLTRB(20, 12, 20, 12),
                child: Center(
                  child: AspectRatio(
                    aspectRatio: 1,
                    child: _JoystickPad(
                      onChanged: _onStickChange,
                      onRelease: _onStickRelease,
                      maxSpeed: _maxSpeed,
                    ),
                  ),
                ),
              ),
            ),
            _SpeedSlider(
              value: _maxSpeed,
              onChanged: (v) {
                setState(() => _maxSpeed = v);
                // Re-send current stick position with new cap
                if (_outLeft != 0 || _outRight != 0) {
                  _pushMove(_outLeft, _outRight, v.round());
                }
              },
            ),
            _RelayBar(
              onStop: _stop,
              onAttachment: _toggleAttachment,
              attachmentOn: _attachmentOn,
              onMount: _toggleMount,
              mountOn: _mountOn,
            ),
            const SizedBox(height: 8),
          ],
        ),
      ),
    );
  }
}

// ---------- Status bar --------------------------------------------------------

class _StatusBar extends StatelessWidget {
  const _StatusBar({
    required this.connected,
    required this.status,
    required this.batteryPct,
    required this.batteryColor,
  });

  final bool connected;
  final String status;
  final int? batteryPct;
  final Color batteryColor;

  @override
  Widget build(BuildContext context) {
    final dotColor = connected ? _Palette.accent : _Palette.danger;
    return Container(
      width: double.infinity,
      padding: const EdgeInsets.fromLTRB(20, 12, 20, 12),
      color: _Palette.surface,
      child: Row(
        children: [
          Container(
            width: 8,
            height: 8,
            decoration: BoxDecoration(
              color: dotColor,
              shape: BoxShape.circle,
              boxShadow: [
                BoxShadow(
                  color: dotColor.withValues(alpha: 0.6),
                  blurRadius: 6,
                  spreadRadius: 1,
                ),
              ],
            ),
          ),
          const SizedBox(width: 10),
          Expanded(
            child: Text(
              status,
              style: const TextStyle(
                color: _Palette.textSecondary,
                fontSize: 12,
                letterSpacing: 0.3,
              ),
              maxLines: 1,
              overflow: TextOverflow.ellipsis,
            ),
          ),
          _BatteryChip(pct: batteryPct, color: batteryColor),
        ],
      ),
    );
  }
}

class _BatteryChip extends StatelessWidget {
  const _BatteryChip({required this.pct, required this.color});
  final int? pct;
  final Color color;

  @override
  Widget build(BuildContext context) {
    final shown = pct?.toString() ?? '—';
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 5),
      decoration: BoxDecoration(
        color: _Palette.surfaceHigh,
        borderRadius: BorderRadius.circular(20),
        border: Border.all(color: color.withValues(alpha: 0.4), width: 1),
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          Icon(Icons.battery_full, size: 14, color: color),
          const SizedBox(width: 4),
          Text(
            '$shown%',
            style: TextStyle(
              color: color,
              fontSize: 12,
              fontWeight: FontWeight.w600,
              fontFeatures: const [FontFeature.tabularFigures()],
            ),
          ),
        ],
      ),
    );
  }
}

// ---------- Connection row ---------------------------------------------------

class _ConnectionRow extends StatelessWidget {
  const _ConnectionRow({
    required this.hostController,
    required this.connected,
    required this.onConnect,
    required this.onDisconnect,
  });

  final TextEditingController hostController;
  final bool connected;
  final VoidCallback onConnect;
  final VoidCallback onDisconnect;

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.fromLTRB(20, 12, 20, 0),
      child: Row(
        children: [
          Expanded(
            child: TextField(
              controller: hostController,
              enabled: !connected,
              keyboardType: const TextInputType.numberWithOptions(decimal: true),
              style: const TextStyle(
                color: _Palette.textPrimary,
                fontFeatures: [FontFeature.tabularFigures()],
              ),
              decoration: InputDecoration(
                labelText: 'ESP32 IP',
                labelStyle: const TextStyle(color: _Palette.textSecondary),
                filled: true,
                fillColor: _Palette.surface,
                border: OutlineInputBorder(
                  borderRadius: BorderRadius.circular(12),
                  borderSide: BorderSide.none,
                ),
                enabledBorder: OutlineInputBorder(
                  borderRadius: BorderRadius.circular(12),
                  borderSide: const BorderSide(
                    color: _Palette.surfaceHigh,
                    width: 1,
                  ),
                ),
                focusedBorder: OutlineInputBorder(
                  borderRadius: BorderRadius.circular(12),
                  borderSide: const BorderSide(
                    color: _Palette.accent,
                    width: 1.5,
                  ),
                ),
                isDense: true,
                contentPadding: const EdgeInsets.symmetric(
                  horizontal: 14,
                  vertical: 14,
                ),
              ),
            ),
          ),
          const SizedBox(width: 10),
          FilledButton(
            onPressed: connected ? onDisconnect : onConnect,
            style: FilledButton.styleFrom(
              backgroundColor:
                  connected ? _Palette.danger : _Palette.accent,
              foregroundColor: const Color(0xFF0E1116),
              padding: const EdgeInsets.symmetric(
                horizontal: 20,
                vertical: 18,
              ),
              shape: RoundedRectangleBorder(
                borderRadius: BorderRadius.circular(12),
              ),
              textStyle: const TextStyle(
                fontWeight: FontWeight.w700,
                letterSpacing: 0.4,
              ),
            ),
            child: Text(connected ? 'STOP' : 'CONNECT'),
          ),
        ],
      ),
    );
  }
}

// ---------- Joystick ---------------------------------------------------------

class _JoystickPad extends StatefulWidget {
  const _JoystickPad({
    required this.onChanged,
    required this.onRelease,
    required this.maxSpeed,
  });

  final ValueChanged<Offset> onChanged;
  final VoidCallback onRelease;
  final double maxSpeed;

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
            decoration: BoxDecoration(
              shape: BoxShape.circle,
              gradient: const RadialGradient(
                colors: [_Palette.surface, _Palette.bg],
                stops: [0.0, 1.0],
              ),
              border: Border.all(
                color: _Palette.surfaceHigh,
                width: 2,
              ),
              boxShadow: [
                BoxShadow(
                  color: _Palette.accent.withValues(alpha: 0.08),
                  blurRadius: 30,
                  spreadRadius: 5,
                ),
              ],
            ),
            child: Stack(
              alignment: Alignment.center,
              children: [
                // crosshair
                _Crosshair(size: r * 1.6),
                // knob
                AnimatedContainer(
                  duration: const Duration(milliseconds: 80),
                  curve: Curves.linear,
                  width: r * 0.55,
                  height: r * 0.55,
                  transform: Matrix4.translationValues(_knob.dx, _knob.dy, 0),
                  decoration: BoxDecoration(
                    shape: BoxShape.circle,
                    gradient: const RadialGradient(
                      colors: [_Palette.accent, _Palette.accentDim],
                      stops: [0.3, 1.0],
                    ),
                    boxShadow: [
                      BoxShadow(
                        color: _Palette.accent.withValues(alpha: 0.4),
                        blurRadius: 16,
                        spreadRadius: 1,
                      ),
                    ],
                  ),
                  child: const Center(
                    child: Icon(
                      Icons.touch_app,
                      color: Color(0xFF0E1116),
                      size: 32,
                    ),
                  ),
                ),
              ],
            ),
          ),
        );
      },
    );
  }
}

class _Crosshair extends StatelessWidget {
  const _Crosshair({required this.size});
  final double size;

  @override
  Widget build(BuildContext context) {
    return IgnorePointer(
      child: SizedBox(
        width: size,
        height: size,
        child: CustomPaint(painter: _CrosshairPainter()),
      ),
    );
  }
}

class _CrosshairPainter extends CustomPainter {
  @override
  void paint(Canvas canvas, Size size) {
    final paint = Paint()
      ..color = _Palette.surfaceHigh
      ..strokeWidth = 1;
    final c = size.center(Offset.zero);
    canvas.drawLine(
      Offset(c.dx - size.width / 2, c.dy),
      Offset(c.dx + size.width / 2, c.dy),
      paint,
    );
    canvas.drawLine(
      Offset(c.dx, c.dy - size.height / 2),
      Offset(c.dx, c.dy + size.height / 2),
      paint,
    );
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => false;
}

// ---------- Speed slider -----------------------------------------------------

class _SpeedSlider extends StatelessWidget {
  const _SpeedSlider({required this.value, required this.onChanged});
  final double value;
  final ValueChanged<double> onChanged;

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.fromLTRB(20, 0, 20, 12),
      child: Container(
        padding: const EdgeInsets.fromLTRB(16, 8, 16, 8),
        decoration: BoxDecoration(
          color: _Palette.surface,
          borderRadius: BorderRadius.circular(14),
          border: Border.all(color: _Palette.surfaceHigh, width: 1),
        ),
        child: Column(
          children: [
            Row(
              children: [
                const Icon(
                  Icons.speed,
                  color: _Palette.accent,
                  size: 18,
                ),
                const SizedBox(width: 8),
                const Text(
                  'MAX SPEED',
                  style: TextStyle(
                    color: _Palette.textSecondary,
                    fontSize: 11,
                    letterSpacing: 1.2,
                    fontWeight: FontWeight.w600,
                  ),
                ),
                const Spacer(),
                Text(
                  '${value.round()}%',
                  style: const TextStyle(
                    color: _Palette.accent,
                    fontSize: 18,
                    fontWeight: FontWeight.w700,
                    fontFeatures: [FontFeature.tabularFigures()],
                  ),
                ),
              ],
            ),
            SliderTheme(
              data: SliderTheme.of(context).copyWith(
                trackHeight: 4,
                thumbShape:
                    const RoundSliderThumbShape(enabledThumbRadius: 8),
              ),
              child: Slider(
                value: value,
                min: 10,
                max: 100,
                divisions: 18, // 5% steps
                onChanged: onChanged,
              ),
            ),
          ],
        ),
      ),
    );
  }
}

// ---------- Relay bar --------------------------------------------------------

class _RelayBar extends StatelessWidget {
  const _RelayBar({
    required this.onStop,
    required this.onAttachment,
    required this.attachmentOn,
    required this.onMount,
    required this.mountOn,
  });

  final VoidCallback onStop;
  final VoidCallback onAttachment;
  final bool attachmentOn;
  final VoidCallback onMount;
  final bool mountOn;

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 20),
      child: Row(
        children: [
          Expanded(child: _BigButton(
            label: 'STOP',
            icon: Icons.stop_rounded,
            color: _Palette.danger,
            onPressed: onStop,
          )),
          const SizedBox(width: 10),
          Expanded(child: _BigButton(
            label: 'ATTACH',
            icon: attachmentOn ? Icons.link : Icons.link_off,
            color: attachmentOn ? _Palette.accent : _Palette.textSecondary,
            filled: attachmentOn,
            onPressed: onAttachment,
          )),
          const SizedBox(width: 10),
          Expanded(child: _BigButton(
            label: 'MOUNT',
            icon: mountOn ? Icons.videocam : Icons.videocam_off_outlined,
            color: mountOn ? _Palette.accent : _Palette.textSecondary,
            filled: mountOn,
            onPressed: onMount,
          )),
        ],
      ),
    );
  }
}

class _BigButton extends StatelessWidget {
  const _BigButton({
    required this.label,
    required this.icon,
    required this.color,
    required this.onPressed,
    this.filled = false,
  });

  final String label;
  final IconData icon;
  final Color color;
  final VoidCallback onPressed;
  final bool filled;

  @override
  Widget build(BuildContext context) {
    return Material(
      color: filled ? color.withValues(alpha: 0.18) : _Palette.surface,
      borderRadius: BorderRadius.circular(14),
      child: InkWell(
        borderRadius: BorderRadius.circular(14),
        onTap: onPressed,
        splashColor: color.withValues(alpha: 0.2),
        highlightColor: color.withValues(alpha: 0.1),
        child: Container(
          height: 64,
          decoration: BoxDecoration(
            borderRadius: BorderRadius.circular(14),
            border: Border.all(
              color: filled ? color : _Palette.surfaceHigh,
              width: 1.2,
            ),
          ),
          padding: const EdgeInsets.symmetric(horizontal: 8),
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              Icon(icon, color: color, size: 22),
              const SizedBox(height: 4),
              Text(
                label,
                style: TextStyle(
                  color: color,
                  fontSize: 11,
                  fontWeight: FontWeight.w700,
                  letterSpacing: 0.6,
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}
