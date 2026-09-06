import 'package:flutter/material.dart';
import 'screens/connect_screen.dart';

void main() {
  runApp(const AmrControlApp());
}

class AmrControlApp extends StatelessWidget {
  const AmrControlApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'AMR Control',
      debugShowCheckedModeBanner: false,
      theme: ThemeData(
        colorSchemeSeed: const Color(0xFF2563EB),
        useMaterial3: true,
        brightness: Brightness.dark,
      ),
      home: const ConnectScreen(),
    );
  }
}
