#!/usr/bin/env python3
"""
Robot Firmware Launcher
GUI tool for ESP-IDF build / flash / monitor
Requires: python3-tk, pyserial
"""

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox, filedialog
import subprocess
import threading
import shutil
import os
import glob
import signal
import json

# ── Config ────────────────────────────────────────────────────
ESP_IDF_PATHS = [
    os.path.expanduser("~/esp/esp-idf/export.sh"),
    os.path.expanduser("~/.espressif/v6.0/esp-idf/export.sh"),
    os.path.expanduser("~/.espressif/v5.4/esp-idf/export.sh"),
    os.path.expanduser("~/.espressif/v5.3/esp-idf/export.sh"),
    os.path.expanduser("~/.espressif/v5.2/esp-idf/export.sh"),
]

ESP_TARGETS = [
    "esp32s2", "esp32s3", "esp32c3", "esp32c6",
    "esp32h2", "esp32", "esp32p4"
]

PROJECT_DIR   = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SETTINGS_FILE = os.path.expanduser("~/.robot_launcher.json")

COLORS = {
    "bg":      "#1e1e2e",
    "surface": "#2a2a3e",
    "border":  "#44445a",
    "accent":  "#7c7cff",
    "success": "#50fa7b",
    "warning": "#ffb86c",
    "error":   "#ff5555",
    "info":    "#8be9fd",
    "text":    "#f8f8f2",
    "muted":   "#6272a4",
    "build":   "#bd93f9",
    "flash":   "#50fa7b",
    "monitor": "#8be9fd",
    "all":     "#ff79c6",
    "clean":   "#ffb86c",
    "stop":    "#ff5555",
    "target":  "#ffb86c",
}

# ── Helpers ───────────────────────────────────────────────────
def find_espidf():
    for p in ESP_IDF_PATHS:
        if os.path.exists(p):
            return p
    try:
        result = subprocess.run(
            ["find", os.path.expanduser("~/"), "-name", "export.sh",
             "-path", "*/esp-idf/*", "-maxdepth", "8"],
            capture_output=True, text=True, timeout=10
        )
        for line in result.stdout.strip().split("\n"):
            if line and os.path.exists(line):
                return line
    except Exception:
        pass
    return None

def find_serial_ports():
    ports = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
    return sorted(ports) if ports else ["(no device found)"]

def load_settings():
    try:
        with open(SETTINGS_FILE) as f:
            return json.load(f)
    except Exception:
        return {}

def save_settings(data):
    try:
        with open(SETTINGS_FILE, "w") as f:
            json.dump(data, f, indent=2)
    except Exception:
        pass

def get_sdkconfig_target(project):
    """Read current IDF_TARGET from sdkconfig, returns None if not found."""
    path = os.path.join(project, "sdkconfig")
    if not os.path.isfile(path):
        return None
    try:
        with open(path) as f:
            for line in f:
                if line.startswith('CONFIG_IDF_TARGET='):
                    return line.strip().split('=')[1].strip('"')
    except Exception:
        pass
    return None

# ── Main App ──────────────────────────────────────────────────
class RobotLauncher(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Robot Firmware Launcher")
        self.geometry("980x700")
        self.configure(bg=COLORS["bg"])
        self.resizable(True, True)

        self.process  = None
        self.idf_path = find_espidf()
        self.running  = False
        self.settings = load_settings()

        self._build_ui()
        self._check_idf()
        self.refresh_ports()
        self._load_saved_settings()
        self._refresh_target_status()

        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ── Settings ──────────────────────────────────────────────
    def _load_saved_settings(self):
        if "project" in self.settings:
            self.project_var.set(self.settings["project"])
        if "target" in self.settings:
            self.target_var.set(self.settings["target"])
        if "port" in self.settings:
            ports = find_serial_ports()
            if self.settings["port"] in ports:
                self.port_var.set(self.settings["port"])

    def _save_current_settings(self):
        save_settings({
            "project": self.project_var.get(),
            "target":  self.target_var.get(),
            "port":    self.port_var.get(),
        })

    def _on_close(self):
        self._save_current_settings()
        self.destroy()

    # ── Target status indicator ───────────────────────────────
    def _refresh_target_status(self):
        project = self.project_var.get().strip()
        current = get_sdkconfig_target(project)
        selected = self.target_var.get()
        if current is None:
            self.target_status.config(text="(not configured)", fg=COLORS["muted"])
        elif current == selected:
            self.target_status.config(text=f"✔ active: {current}", fg=COLORS["success"])
        else:
            self.target_status.config(text=f"⚠ active: {current} — click Set Target", fg=COLORS["warning"])

    # ── UI ─────────────────────────────────────────────────────
    def _build_ui(self):
        self.columnconfigure(0, weight=1)
        self.rowconfigure(3, weight=1)

        # ── Header ──
        hdr = tk.Frame(self, bg=COLORS["surface"],
                       highlightthickness=1,
                       highlightbackground=COLORS["border"])
        hdr.grid(row=0, column=0, sticky="ew")
        hdr.columnconfigure(1, weight=1)

        tk.Label(hdr, text=" Robot Firmware Launcher",
                 font=("Courier", 15, "bold"),
                 bg=COLORS["surface"], fg=COLORS["accent"]).grid(
                 row=0, column=0, padx=16, pady=12, sticky="w")

        self.status_label = tk.Label(hdr, text="● Idle",
                 font=("Courier", 11), bg=COLORS["surface"], fg=COLORS["muted"])
        self.status_label.grid(row=0, column=2, padx=16, pady=12, sticky="e")

        # ── Row 1: Project ────────────────────────────────────
        row1 = tk.Frame(self, bg=COLORS["bg"])
        row1.grid(row=1, column=0, sticky="ew", padx=16, pady=(12, 0))
        row1.columnconfigure(1, weight=1)

        tk.Label(row1, text="Project:", font=("Courier", 11),
                 bg=COLORS["bg"], fg=COLORS["muted"]).grid(
                 row=0, column=0, padx=(0, 6), sticky="w")

        self.project_var = tk.StringVar(value=PROJECT_DIR)
        self.project_var.trace_add("write", lambda *_: self._refresh_target_status())
        tk.Entry(row1, textvariable=self.project_var, font=("Courier", 11),
                 bg=COLORS["surface"], fg=COLORS["text"],
                 insertbackground=COLORS["text"],
                 relief="flat", bd=4).grid(
                 row=0, column=1, sticky="ew", padx=(0, 6))

        tk.Button(row1, text="Browse",
                  font=("Courier", 10),
                  bg=COLORS["surface"], fg=COLORS["info"],
                  activebackground=COLORS["border"],
                  relief="flat", bd=0, padx=8, pady=4,
                  cursor="hand2",
                  highlightthickness=1,
                  highlightbackground=COLORS["info"],
                  command=self._browse_project).grid(row=0, column=2)

        # ── Row 2: Target + Port ──────────────────────────────
        row2 = tk.Frame(self, bg=COLORS["bg"])
        row2.grid(row=2, column=0, sticky="ew", padx=16, pady=(8, 0))

        tk.Label(row2, text="Target:", font=("Courier", 11),
                 bg=COLORS["bg"], fg=COLORS["muted"]).grid(
                 row=0, column=0, padx=(0, 6), sticky="w")

        self.target_var = tk.StringVar(value="esp32s2")
        self.target_var.trace_add("write", lambda *_: self._refresh_target_status())
        ttk.Combobox(row2, textvariable=self.target_var,
                     values=ESP_TARGETS,
                     font=("Courier", 11),
                     width=12, state="readonly").grid(
                     row=0, column=1, padx=(0, 6))

        tk.Button(row2, text="Set Target",
                  font=("Courier", 10),
                  bg=COLORS["surface"], fg=COLORS["target"],
                  activebackground=COLORS["border"],
                  relief="flat", bd=0, padx=8, pady=4,
                  cursor="hand2",
                  highlightthickness=1,
                  highlightbackground=COLORS["target"],
                  command=self.cmd_set_target).grid(row=0, column=2, padx=(0, 16))

        self.target_status = tk.Label(row2, text="",
                 font=("Courier", 10), bg=COLORS["bg"], fg=COLORS["muted"])
        self.target_status.grid(row=0, column=3, padx=(0, 32), sticky="w")

        tk.Label(row2, text="Port:", font=("Courier", 11),
                 bg=COLORS["bg"], fg=COLORS["muted"]).grid(
                 row=0, column=4, padx=(0, 6), sticky="w")

        self.port_var = tk.StringVar()
        self.port_menu = ttk.Combobox(row2, textvariable=self.port_var,
                                      font=("Courier", 11),
                                      width=14, state="readonly")
        self.port_menu.grid(row=0, column=5)

        tk.Button(row2, text="↻", font=("Courier", 12),
                  bg=COLORS["surface"], fg=COLORS["info"],
                  relief="flat", bd=0, cursor="hand2",
                  command=self.refresh_ports).grid(row=0, column=6, padx=(6, 0))

        # ── Row 3: Action Buttons ─────────────────────────────
        btn_frame = tk.Frame(self, bg=COLORS["bg"])
        btn_frame.grid(row=3, column=0, sticky="ew", padx=16, pady=(10, 4))
        self.rowconfigure(3, weight=0)

        buttons = [
            ("Build",               COLORS["build"],   self.cmd_build),
            ("Flash",               COLORS["flash"],   self.cmd_flash),
            ("Monitor",             COLORS["monitor"], self.cmd_monitor),
            ("Build+Flash+Monitor", COLORS["all"],     self.cmd_all),
            ("Clean",               COLORS["warning"], self.cmd_clean),
            ("Stop",                COLORS["stop"],    self.cmd_stop),
        ]

        for i, (label, color, cmd) in enumerate(buttons):
            tk.Button(btn_frame, text=label,
                      font=("Courier", 11, "bold"),
                      bg=COLORS["surface"], fg=color,
                      activebackground=COLORS["border"],
                      activeforeground=color,
                      relief="flat", bd=0,
                      padx=14, pady=8,
                      cursor="hand2",
                      highlightthickness=1,
                      highlightbackground=color,
                      command=cmd).grid(row=0, column=i, padx=(0, 8))

        # ── Row 4: Log Output ─────────────────────────────────
        log_frame = tk.Frame(self, bg=COLORS["bg"])
        log_frame.grid(row=4, column=0, sticky="nsew", padx=16, pady=(8, 4))
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(1, weight=1)
        self.rowconfigure(4, weight=1)

        tk.Label(log_frame, text="Output", font=("Courier", 10),
                 bg=COLORS["bg"], fg=COLORS["muted"]).grid(
                 row=0, column=0, sticky="w", pady=(0, 4))

        self.log = scrolledtext.ScrolledText(
            log_frame,
            font=("Courier", 11),
            bg=COLORS["surface"], fg=COLORS["text"],
            insertbackground=COLORS["text"],
            relief="flat", bd=4,
            wrap=tk.WORD,
            state="disabled"
        )
        self.log.grid(row=1, column=0, sticky="nsew")

        self.log.tag_config("error",   foreground=COLORS["error"])
        self.log.tag_config("success", foreground=COLORS["success"])
        self.log.tag_config("warning", foreground=COLORS["warning"])
        self.log.tag_config("info",    foreground=COLORS["info"])
        self.log.tag_config("muted",   foreground=COLORS["muted"])

        # ── Footer ──
        foot = tk.Frame(self, bg=COLORS["surface"],
                        highlightthickness=1,
                        highlightbackground=COLORS["border"])
        foot.grid(row=5, column=0, sticky="ew")
        foot.columnconfigure(0, weight=1)

        self.idf_label = tk.Label(foot, text="",
                 font=("Courier", 10), bg=COLORS["surface"], fg=COLORS["muted"])
        self.idf_label.grid(row=0, column=0, padx=12, pady=6, sticky="w")

        tk.Button(foot, text="Clear log", font=("Courier", 10),
                  bg=COLORS["surface"], fg=COLORS["muted"],
                  relief="flat", bd=0, cursor="hand2",
                  command=self.clear_log).grid(row=0, column=1, padx=12, pady=6)

    # ── Helpers ────────────────────────────────────────────────
    def _check_idf(self):
        if self.idf_path:
            self.idf_label.config(
                text=f"ESP-IDF: {self.idf_path}", fg=COLORS["success"])
            self.log_line(f"ESP-IDF found: {self.idf_path}\n", "success")
        else:
            self.idf_label.config(text="ESP-IDF: NOT FOUND", fg=COLORS["error"])
            self.log_line("ESP-IDF not found. Run setup.sh first.\n", "error")

    def _browse_project(self):
        folder = filedialog.askdirectory(
            title="Select ESP-IDF Project Folder",
            initialdir=self.project_var.get()
        )
        if folder:
            self.project_var.set(folder)
            self._refresh_target_status()
            self._save_current_settings()

    def refresh_ports(self):
        ports = find_serial_ports()
        self.port_menu["values"] = ports
        if ports:
            current = self.port_var.get()
            self.port_var.set(current if current in ports else ports[0])

    def log_line(self, text, tag=None):
        self.log.config(state="normal")
        if tag:
            self.log.insert(tk.END, text, tag)
        else:
            self.log.insert(tk.END, text)
        self.log.see(tk.END)
        self.log.config(state="disabled")

    def clear_log(self):
        self.log.config(state="normal")
        self.log.delete("1.0", tk.END)
        self.log.config(state="disabled")

    def set_status(self, text, color):
        self.status_label.config(text=f"● {text}", fg=color)

    # ── Command Runner ─────────────────────────────────────────
    def _run(self, idf_args, label=None, post_fn=None):
        if not self.idf_path:
            messagebox.showerror("Error", "ESP-IDF not found. Run setup.sh first.")
            return

        if self.running:
            messagebox.showwarning("Busy", "A command is already running. Stop it first.")
            return

        project = self.project_var.get().strip()
        port    = self.port_var.get().strip()

        if not os.path.isdir(project):
            messagebox.showerror("Error", f"Project directory not found:\n{project}")
            return

        port_arg = f"-p {port}" if port and "no device" not in port else ""

        # Unset stale IDF env vars before sourcing — prevents wrong target
        cmd = (
            "unset IDF_PATH IDF_TARGET IDF_PYTHON_ENV_PATH "
            "IDF_TOOLS_EXPORT_CMD OPENOCD_SCRIPTS && "
            f'. "{self.idf_path}" > /dev/null 2>&1 && '
            f"idf.py {port_arg} {idf_args}"
        )

        self.running = True
        self.clear_log()
        self.log_line(f"$ idf.py {port_arg} {idf_args}\n\n", "muted")
        self.set_status(label or "Running...", COLORS["warning"])
        self._save_current_settings()

        def worker():
            self.process = subprocess.Popen(
                ["bash", "-c", cmd],
                cwd=project,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                preexec_fn=os.setsid
            )
            proc = self.process
            try:
                for line in proc.stdout:
                    tag = None
                    low = line.lower()
                    if any(k in low for k in ["error:", "failed", "fatal"]):
                        tag = "error"
                    elif any(k in low for k in ["warning:", "warn:"]):
                        tag = "warning"
                    elif any(k in low for k in ["complete", "success", "done", "verified"]):
                        tag = "success"
                    elif any(k in low for k in ["flash", "monitor", "connecting"]):
                        tag = "info"
                    self.after(0, self.log_line, line, tag)

                proc.wait()
                rc = proc.returncode
                if rc == 0:
                    self.after(0, self.set_status, "Done", COLORS["success"])
                    self.after(0, self.log_line, "\nDone.\n", "success")
                    if post_fn:
                        self.after(0, post_fn)
                elif rc != -15:  # -15 = SIGTERM (user clicked Stop) — not an error
                    self.after(0, self.set_status, "Failed", COLORS["error"])
                    self.after(0, self.log_line, f"\nExited with code {rc}\n", "error")

            except Exception:
                pass  # Swallow race-condition errors from Stop
            finally:
                self.running = False
                self.process = None

        threading.Thread(target=worker, daemon=True).start()

    # ── Commands ───────────────────────────────────────────────
    def cmd_build(self):   self._run("build",               "Building...")
    def cmd_flash(self):   self._run("flash",               "Flashing...")
    def cmd_monitor(self): self._run("monitor",             "Monitoring...")
    def cmd_all(self):     self._run("build flash monitor", "Build+Flash+Monitor...")
    def cmd_clean(self):   self._run("fullclean",           "Cleaning...")

    def cmd_set_target(self):
        target  = self.target_var.get()
        project = self.project_var.get().strip()

        if not os.path.isdir(project):
            messagebox.showerror("Error", f"Project directory not found:\n{project}")
            return

        current = get_sdkconfig_target(project)
        if current == target:
            messagebox.showinfo("Already set",
                f"Target is already {target}. No action needed.")
            return

        confirm = messagebox.askyesno(
            "Set Target",
            f"Set target to {target}?\n\n"
            f"Current: {current or 'none'}\n\n"
            "This will delete build/, sdkconfig, and sdkconfig.old."
        )
        if not confirm:
            return

        # Delete stale artifacts first
        self.clear_log()
        for path in ["build", "sdkconfig", "sdkconfig.old"]:
            full = os.path.join(project, path)
            if os.path.isdir(full):
                shutil.rmtree(full)
                self.log_line(f"Deleted {path}/\n", "warning")
            elif os.path.isfile(full):
                os.remove(full)
                self.log_line(f"Deleted {path}\n", "warning")

        # Run set-target, then refresh status indicator when done
        self._run(
            f"set-target {target}",
            f"Setting target: {target}...",
            post_fn=self._refresh_target_status
        )

    def cmd_stop(self):
        if self.process:
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
                self.log_line("\nStopped.\n", "warning")
                self.set_status("Stopped", COLORS["warning"])
            except Exception as e:
                self.log_line(f"\nStop failed: {e}\n", "error")
        self.running = False
        self.process = None


# ── Entry ──────────────────────────────────────────────────────
if __name__ == "__main__":
    import signal
    app = RobotLauncher()
    signal.signal(signal.SIGINT, lambda *_: app._on_close())
    app.mainloop()
