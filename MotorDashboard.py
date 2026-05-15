"""
MotorDashboard.py
=================
Project : Distributed DC Motor PID Control with CAN Bus Telemetry
Layer   : Monitoring & Tuning GUI (PyQt6 + PyQtGraph)

Features
--------
- Auto-detect ESP32 COM port (looks for CP210x / CH340 USB-serial adapters).
- Dedicated QThread for serial reading — UI never freezes.
- Live dual-trace graph: Actual RPM (cyan) vs Target RPM (amber) at up to 50 Hz.
- PID tuning panel: Kp / Ki / Kd inputs → sends "P:x,I:y,D:z\n" to ESP32 #2.
- Status bar shows connection state, distance warning, and data rate.
- CSV export of the entire session log with timestamps.

Requirements
------------
    pip install PyQt6 pyqtgraph pyserial

Serial format expected from ESP32 #2
-------------------------------------
    <actual_int>,<target_int>\n        e.g. "1450,1500\n"

Serial format sent to ESP32 #2
-------------------------------
    P:<float>,I:<float>,D:<float>\n    e.g. "P:1.5,I:0.30,D:0.05\n"
"""

import sys
import csv
import time
import datetime
import threading

import serial
import serial.tools.list_ports

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QLineEdit, QComboBox, QGroupBox, QStatusBar,
    QFileDialog, QSizePolicy, QFrame, QGridLayout, QMessageBox,
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QObject, QThread
from PyQt6.QtGui import QFont, QColor, QPalette, QIcon

import pyqtgraph as pg

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
SERIAL_BAUD    = 115_200
PLOT_MAX_POINTS = 500          # Rolling window — keep last N samples
GRAPH_UPDATE_MS = 20           # Redraw timer (50 Hz)
STATUS_UPDATE_MS = 500         # Status-bar refresh

COLOUR_ACTUAL  = "#00e5ff"     # Cyan  — Actual RPM trace
COLOUR_TARGET  = "#ffa726"     # Amber — Target RPM trace
COLOUR_BG      = "#0d1117"     # Near-black background
COLOUR_PANEL   = "#161b22"     # Card background
COLOUR_ACCENT  = "#238636"     # Green accent
COLOUR_WARN    = "#f85149"     # Red / warning

# ---------------------------------------------------------------------------
# Serial Worker  (runs in a QThread)
# ---------------------------------------------------------------------------
class SerialWorker(QObject):
    """
    Reads lines from the serial port in a dedicated thread.
    Emits `newData(actual_rpm, target_rpm)` on each valid line.
    Emits `errorOccurred(message)` on port errors.
    """
    newData       = pyqtSignal(int, int)
    errorOccurred = pyqtSignal(str)

    def __init__(self, port: str, baud: int = SERIAL_BAUD):
        super().__init__()
        self.port     = port
        self.baud     = baud
        self._running = False
        self._ser     = None

    def start(self):
        self._running = True
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=1)
        except serial.SerialException as e:
            self.errorOccurred.emit(f"Cannot open {self.port}: {e}")
            self._running = False
            return

        while self._running:
            try:
                raw = self._ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="ignore").strip()
                parts = line.split(",")
                if len(parts) == 2:
                    actual = int(parts[0])
                    target = int(parts[1])
                    self.newData.emit(actual, target)
            except (serial.SerialException, OSError) as e:
                self.errorOccurred.emit(f"Serial error: {e}")
                self._running = False
            except ValueError:
                pass  # Malformed line — skip silently

    def send(self, text: str):
        """Thread-safe write to serial port."""
        if self._ser and self._ser.is_open:
            try:
                self._ser.write((text + "\n").encode("utf-8"))
            except serial.SerialException:
                pass

    def stop(self):
        self._running = False
        if self._ser and self._ser.is_open:
            self._ser.close()


# ---------------------------------------------------------------------------
# Helper: auto-detect likely ESP32 port
# ---------------------------------------------------------------------------
def auto_detect_esp32_port() -> str | None:
    """
    Scans available COM ports and returns the first one that looks like
    a CP210x, CH340, or FTDI USB-serial adapter (common on ESP32 devkits).
    Returns None if nothing found.
    """
    keywords = ["CP210", "CH340", "FTDI", "USB Serial", "USB-SERIAL", "Silicon Labs"]
    for p in serial.tools.list_ports.comports():
        description = (p.description or "") + (p.manufacturer or "")
        if any(kw.lower() in description.lower() for kw in keywords):
            return p.device
    # Fallback: return first available port
    ports = serial.tools.list_ports.comports()
    return ports[0].device if ports else None


# ---------------------------------------------------------------------------
# Main Window
# ---------------------------------------------------------------------------
class MotorDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MotorDashboard — DC Motor PID Telemetry")
        self.resize(1100, 680)
        self._apply_dark_theme()

        # Data buffers
        self._times   : list[float] = []   # Relative time in seconds
        self._actuals : list[int]   = []   # Actual RPM history
        self._targets : list[int]   = []   # Target RPM history
        self._t0      : float       = time.time()

        # Serial state
        self._worker : SerialWorker | None = None
        self._thread : QThread      | None = None
        self._last_actual = 0
        self._last_target = 0
        self._sample_count = 0
        self._data_rate    = 0.0
        self._rate_window_start = time.time()
        self._rate_window_count = 0

        # Build UI
        self._build_ui()

        # Timers
        self._graph_timer = QTimer()
        self._graph_timer.timeout.connect(self._refresh_plot)
        self._graph_timer.start(GRAPH_UPDATE_MS)

        self._status_timer = QTimer()
        self._status_timer.timeout.connect(self._refresh_status)
        self._status_timer.start(STATUS_UPDATE_MS)

        # Try auto-connect
        self._try_auto_connect()

    # ------------------------------------------------------------------
    # UI Construction
    # ------------------------------------------------------------------
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setSpacing(8)
        root.setContentsMargins(12, 12, 12, 8)

        # Top bar: port selector + connect button
        root.addLayout(self._build_top_bar())

        # Main content: graph | right panel
        content = QHBoxLayout()
        content.setSpacing(10)
        root.addLayout(content, stretch=1)

        # Graph
        content.addWidget(self._build_graph(), stretch=3)

        # Right panel
        right = QVBoxLayout()
        right.setSpacing(10)
        right.addWidget(self._build_pid_panel())
        right.addWidget(self._build_metrics_panel())
        right.addWidget(self._build_log_panel())
        right.addStretch()
        content.addLayout(right, stretch=1)

        # Status bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Disconnected — select a COM port and press Connect")

    def _build_top_bar(self) -> QHBoxLayout:
        bar = QHBoxLayout()

        lbl = QLabel("Port:")
        lbl.setFont(QFont("Courier New", 10))
        bar.addWidget(lbl)

        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(160)
        self._populate_ports()
        bar.addWidget(self.port_combo)

        refresh_btn = QPushButton("⟳")
        refresh_btn.setToolTip("Refresh port list")
        refresh_btn.setFixedWidth(32)
        refresh_btn.clicked.connect(self._populate_ports)
        bar.addWidget(refresh_btn)

        self.connect_btn = QPushButton("Connect")
        self.connect_btn.setFixedWidth(100)
        self.connect_btn.setStyleSheet(f"background:{COLOUR_ACCENT}; color:#fff; font-weight:bold;")
        self.connect_btn.clicked.connect(self._toggle_connection)
        bar.addWidget(self.connect_btn)

        bar.addStretch()

        # Live RPM readouts
        self.lbl_actual = QLabel("Actual: — RPM")
        self.lbl_target = QLabel("Target: — RPM")
        for lbl, col in [(self.lbl_actual, COLOUR_ACTUAL), (self.lbl_target, COLOUR_TARGET)]:
            lbl.setFont(QFont("Courier New", 13, QFont.Weight.Bold))
            lbl.setStyleSheet(f"color:{col};")
            bar.addWidget(lbl)
            bar.addSpacing(20)

        return bar

    def _build_graph(self) -> QWidget:
        pg.setConfigOptions(antialias=True, background=COLOUR_BG)
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setLabel("left",   "RPM")
        self.plot_widget.setLabel("bottom", "Time (s)")
        self.plot_widget.showGrid(x=True, y=True, alpha=0.2)
        
        # --- BAGIAN YANG DIUBAH ---
        self.plot_widget.setYRange(0, 300, padding=0.05)
        # --------------------------
        
        self.plot_widget.addLegend(offset=(10, 10))

        self.curve_actual = self.plot_widget.plot(
            [], [], pen=pg.mkPen(COLOUR_ACTUAL, width=2), name="Actual RPM"
        )
        self.curve_target = self.plot_widget.plot(
            [], [], pen=pg.mkPen(COLOUR_TARGET, width=2, style=Qt.PenStyle.DashLine),
            name="Target RPM"
        )
        return self.plot_widget

    def _build_pid_panel(self) -> QGroupBox:
        box = QGroupBox("PID Tuning")
        box.setStyleSheet(f"QGroupBox {{ background:{COLOUR_PANEL}; border-radius:6px; }}")
        layout = QGridLayout(box)

        self.kp_edit = QLineEdit("1.50")
        self.ki_edit = QLineEdit("0.30")
        self.kd_edit = QLineEdit("0.05")

        for row, (label, widget) in enumerate([
            ("Kp (Proportional):", self.kp_edit),
            ("Ki (Integral):",     self.ki_edit),
            ("Kd (Derivative):",   self.kd_edit),
        ]):
            layout.addWidget(QLabel(label), row, 0)
            widget.setFixedWidth(80)
            widget.setAlignment(Qt.AlignmentFlag.AlignRight)
            layout.addWidget(widget, row, 1)

        send_btn = QPushButton("Send Update ▶")
        send_btn.setStyleSheet(f"background:{COLOUR_ACCENT}; color:#fff; font-weight:bold; padding:6px;")
        send_btn.clicked.connect(self._send_pid)
        layout.addWidget(send_btn, 3, 0, 1, 2)

        return box

    def _build_metrics_panel(self) -> QGroupBox:
        box = QGroupBox("Live Metrics")
        box.setStyleSheet(f"QGroupBox {{ background:{COLOUR_PANEL}; border-radius:6px; }}")
        layout = QVBoxLayout(box)

        self.lbl_error   = QLabel("Error:    — RPM")
        self.lbl_rate    = QLabel("Rate:     — Hz")
        self.lbl_samples = QLabel("Samples:  0")

        for lbl in (self.lbl_error, self.lbl_rate, self.lbl_samples):
            lbl.setFont(QFont("Courier New", 10))
            layout.addWidget(lbl)

        return box

    def _build_log_panel(self) -> QGroupBox:
        box = QGroupBox("Session Log")
        box.setStyleSheet(f"QGroupBox {{ background:{COLOUR_PANEL}; border-radius:6px; }}")
        layout = QVBoxLayout(box)

        export_btn = QPushButton("Export to CSV")
        export_btn.setStyleSheet("padding:6px;")
        export_btn.clicked.connect(self._export_csv)
        layout.addWidget(export_btn)

        clear_btn = QPushButton("Clear Data")
        clear_btn.setStyleSheet("padding:6px;")
        clear_btn.clicked.connect(self._clear_data)
        layout.addWidget(clear_btn)

        return box

    # ------------------------------------------------------------------
    # Port Management
    # ------------------------------------------------------------------
    def _populate_ports(self):
        self.port_combo.clear()
        for p in serial.tools.list_ports.comports():
            self.port_combo.addItem(f"{p.device} — {p.description}", p.device)
        if self.port_combo.count() == 0:
            self.port_combo.addItem("No ports found", "")

    def _try_auto_connect(self):
        detected = auto_detect_esp32_port()
        if detected:
            # Select it in combo
            for i in range(self.port_combo.count()):
                if self.port_combo.itemData(i) == detected:
                    self.port_combo.setCurrentIndex(i)
                    break
            self._connect(detected)

    def _toggle_connection(self):
        if self._worker is None:
            port = self.port_combo.currentData()
            if not port:
                QMessageBox.warning(self, "No Port", "Please select a valid COM port.")
                return
            self._connect(port)
        else:
            self._disconnect()

    def _connect(self, port: str):
        self._t0 = time.time()
        self._rate_window_start = time.time()
        self._rate_window_count = 0

        self._thread = QThread()
        self._worker = SerialWorker(port, SERIAL_BAUD)
        self._worker.moveToThread(self._thread)

        self._thread.started.connect(self._worker.start)
        self._worker.newData.connect(self._on_new_data)
        self._worker.errorOccurred.connect(self._on_serial_error)

        self._thread.start()

        self.connect_btn.setText("Disconnect")
        self.connect_btn.setStyleSheet(f"background:{COLOUR_WARN}; color:#fff; font-weight:bold;")
        self.status_bar.showMessage(f"Connected → {port}  @  {SERIAL_BAUD} baud")

    def _disconnect(self):
        if self._worker:
            self._worker.stop()
        if self._thread:
            self._thread.quit()
            self._thread.wait(2000)
        self._worker = None
        self._thread = None

        self.connect_btn.setText("Connect")
        self.connect_btn.setStyleSheet(f"background:{COLOUR_ACCENT}; color:#fff; font-weight:bold;")
        self.status_bar.showMessage("Disconnected")

    # ------------------------------------------------------------------
    # Data Handling
    # ------------------------------------------------------------------
    def _on_new_data(self, actual: int, target: int):
        """Called from SerialWorker thread via Qt signal — safe to update buffers."""
        t = time.time() - self._t0
        self._times.append(t)
        self._actuals.append(actual)
        self._targets.append(target)

        # Keep rolling window
        if len(self._times) > PLOT_MAX_POINTS:
            self._times   = self._times[-PLOT_MAX_POINTS:]
            self._actuals = self._actuals[-PLOT_MAX_POINTS:]
            self._targets = self._targets[-PLOT_MAX_POINTS:]

        self._last_actual  = actual
        self._last_target  = target
        self._sample_count += 1
        self._rate_window_count += 1

        # Update live RPM labels
        self.lbl_actual.setText(f"Actual: {actual:5d} RPM")
        self.lbl_target.setText(f"Target: {target:5d} RPM")

    def _on_serial_error(self, msg: str):
        self.status_bar.showMessage(f"⚠ {msg}")
        self._disconnect()

    # ------------------------------------------------------------------
    # Plot Refresh
    # ------------------------------------------------------------------
    def _refresh_plot(self):
        if not self._times:
            return
        self.curve_actual.setData(self._times, self._actuals)
        self.curve_target.setData(self._times, self._targets)

    # ------------------------------------------------------------------
    # Status Refresh
    # ------------------------------------------------------------------
    def _refresh_status(self):
        now = time.time()
        elapsed = now - self._rate_window_start
        if elapsed >= 1.0:
            self._data_rate = self._rate_window_count / elapsed
            self._rate_window_count = 0
            self._rate_window_start = now

        error = self._last_target - self._last_actual
        self.lbl_error.setText(f"Error:    {error:+5d} RPM")
        self.lbl_rate.setText( f"Rate:     {self._data_rate:5.1f} Hz")
        self.lbl_samples.setText(f"Samples:  {self._sample_count}")

    # ------------------------------------------------------------------
    # PID Send
    # ------------------------------------------------------------------
    def _send_pid(self):
        try:
            kp = float(self.kp_edit.text())
            ki = float(self.ki_edit.text())
            kd = float(self.kd_edit.text())
        except ValueError:
            QMessageBox.warning(self, "Invalid Input", "Kp, Ki, and Kd must be numeric values.")
            return

        if kp < 0 or ki < 0 or kd < 0:
            QMessageBox.warning(self, "Invalid Input", "PID gains must be ≥ 0.")
            return

        cmd = f"P:{kp:.4f},I:{ki:.4f},D:{kd:.4f}"

        if self._worker:
            self._worker.send(cmd)
            self.status_bar.showMessage(f"PID sent → {cmd}")
        else:
            QMessageBox.warning(self, "Not Connected", "Connect to the ESP32 first.")

    # ------------------------------------------------------------------
    # CSV Export
    # ------------------------------------------------------------------
    def _export_csv(self):
        if not self._times:
            QMessageBox.information(self, "No Data", "No data to export yet.")
            return

        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        default_name = f"motor_session_{timestamp}.csv"

        path, _ = QFileDialog.getSaveFileName(
            self, "Save Session Log", default_name,
            "CSV Files (*.csv);;All Files (*)"
        )
        if not path:
            return

        try:
            with open(path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["Time_s", "Actual_RPM", "Target_RPM", "Error_RPM"])
                for t, a, g in zip(self._times, self._actuals, self._targets):
                    writer.writerow([f"{t:.4f}", a, g, g - a])
            self.status_bar.showMessage(f"Exported {len(self._times)} rows → {path}")
        except OSError as e:
            QMessageBox.critical(self, "Export Failed", str(e))

    # ------------------------------------------------------------------
    # Clear Data
    # ------------------------------------------------------------------
    def _clear_data(self):
        self._times.clear()
        self._actuals.clear()
        self._targets.clear()
        self._sample_count  = 0
        self._t0            = time.time()
        self.curve_actual.setData([], [])
        self.curve_target.setData([], [])
        self.status_bar.showMessage("Data cleared.")

    # ------------------------------------------------------------------
    # Theme
    # ------------------------------------------------------------------
    def _apply_dark_theme(self):
        self.setStyleSheet(f"""
            QMainWindow, QWidget {{
                background-color: {COLOUR_BG};
                color: #c9d1d9;
                font-family: 'Segoe UI', 'SF Pro Text', sans-serif;
                font-size: 11pt;
            }}
            QGroupBox {{
                border: 1px solid #30363d;
                border-radius: 6px;
                margin-top: 10px;
                padding: 8px;
                font-weight: bold;
                color: #8b949e;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                left: 8px;
            }}
            QLineEdit {{
                background: #21262d;
                border: 1px solid #30363d;
                border-radius: 4px;
                padding: 3px 6px;
                color: #e6edf3;
            }}
            QLineEdit:focus {{
                border-color: #388bfd;
            }}
            QPushButton {{
                background: #21262d;
                border: 1px solid #30363d;
                border-radius: 4px;
                padding: 4px 10px;
                color: #c9d1d9;
            }}
            QPushButton:hover {{
                background: #30363d;
                border-color: #8b949e;
            }}
            QComboBox {{
                background: #21262d;
                border: 1px solid #30363d;
                border-radius: 4px;
                padding: 3px 6px;
                color: #e6edf3;
            }}
            QStatusBar {{
                background: {COLOUR_PANEL};
                color: #8b949e;
                font-size: 10pt;
            }}
            QLabel {{
                color: #c9d1d9;
            }}
        """)

    # ------------------------------------------------------------------
    # Cleanup on close
    # ------------------------------------------------------------------
    def closeEvent(self, event):
        self._disconnect()
        event.accept()


# ---------------------------------------------------------------------------
# Entry Point
# ---------------------------------------------------------------------------
def main():
    app = QApplication(sys.argv)
    app.setApplicationName("MotorDashboard")
    app.setApplicationVersion("1.0")

    window = MotorDashboard()
    window.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()