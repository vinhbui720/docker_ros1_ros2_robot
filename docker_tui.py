#!/usr/bin/env python3
"""
Docker ROS1/ROS2 Robot TUI Manager
  pip install "textual>=0.50"
  python3 docker_tui.py

Keys:
  s  – toggle sidebar
  l  – expand/collapse log panel
  r  – refresh status
  q  – quit
"""
from __future__ import annotations

import asyncio
import os
import subprocess
from datetime import datetime
from pathlib import Path

from rich.text import Text
from textual import on, work
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Container, Horizontal, VerticalScroll
from textual.widgets import (
    Button, DataTable, Footer, Header,
    Input, Label, RadioButton, RadioSet,
    RichLog, Select, Switch, TabbedContent, TabPane,
)

# ── constants ─────────────────────────────────────────────────────────────────
COMPOSE_FILE = Path(__file__).parent / "docker-compose.yml"

ALL_SERVICES: list[str] = [
    "roscore",
    "motoman_controller",
    "motoman_vel_controller",
    "ros1_bridge_traj",
    "ros1_bridge_vel",
    "gandtry_driver",
]

PROFILE_SERVICES: dict[str, list[str]] = {
    "traj_controller": ["roscore", "motoman_controller", "ros1_bridge_traj", "gandtry_driver"],
    "vel_controller":  ["roscore", "motoman_vel_controller", "ros1_bridge_vel", "gandtry_driver"],
}

PROFILES: list[str] = list(PROFILE_SERVICES.keys())


def _get_statuses() -> dict[str, str]:
    """Query `docker compose ps` for all profiles and return {service: status}."""
    try:
        r = subprocess.run(
            [
                "docker", "compose", "-f", str(COMPOSE_FILE),
                "--profile", "*", "ps",
                "--format", "{{.Service}}\t{{.Status}}",
            ],
            capture_output=True, text=True, timeout=8,
        )
        out: dict[str, str] = {}
        for line in r.stdout.splitlines():
            parts = line.split("\t", 1)
            if len(parts) == 2:
                out[parts[0].strip()] = parts[1].strip()
        return out
    except Exception:
        return {}


# ── CSS ───────────────────────────────────────────────────────────────────────
CSS = """\
Screen { layout: vertical; }

#body { height: 1fr; layout: horizontal; }

/* ── sidebar ── */
#sidebar {
    width: 26;
    min-width: 20;
    background: $panel;
    border-right: solid $primary-darken-2;
}

TabbedContent { height: 1fr; }
TabPane { padding: 0 1 1 1; }

.sect { color: $accent; text-style: bold; margin-top: 1; }

RadioSet { border: none; background: transparent; height: auto; margin-bottom: 1; }

Button { width: 100%; margin-bottom: 1; }

#sim-row { height: 3; layout: horizontal; align: left middle; margin-bottom: 1; }
#sim-row Label { width: 6; }

#ip-input   { margin-bottom: 1; }
#svc-select { margin-bottom: 1; }

/* ── main ── */
#main { width: 1fr; layout: vertical; padding-left: 1; }

#status-pane { height: 1fr; min-height: 5; border: solid $primary-darken-2; }
DataTable    { height: 1fr; }

#log-pane { height: 12; border: solid $accent-darken-2; margin-top: 1; }
#log-pane.expanded { height: 1fr; }

#log-toolbar {
    height: 3;
    layout: horizontal;
    background: $surface;
    padding: 0 1;
    align: left middle;
}
#log-toolbar Label { width: 1fr; }
#log-toolbar Button { width: auto; min-width: 6; margin: 0 0 0 1; }

RichLog { height: 1fr; padding: 0 1; }
"""


# ── App ───────────────────────────────────────────────────────────────────────
class RobotDockerTUI(App[None]):
    """TUI manager for the ROS1/ROS2 robot Docker Compose stack."""

    TITLE = "🤖 ROS Docker Manager"
    CSS = CSS
    BINDINGS = [
        Binding("q",      "quit",            "Quit"),
        Binding("r",      "refresh_status",  "Refresh"),
        Binding("s",      "toggle_sidebar",  "Sidebar"),
        Binding("l",      "toggle_log",      "Expand Log"),
        Binding("ctrl+c", "quit",            show=False),
    ]

    _profile: str = "traj_controller"
    _sim: bool = False
    _ip: str = "192.168.1.14"
    _proc: asyncio.subprocess.Process | None = None
    _log_expanded: bool = False

    # ── layout ────────────────────────────────────────────────────────────────
    def compose(self) -> ComposeResult:
        yield Header()

        with Container(id="body"):

            # ── sidebar (tabbed) ──────────────────────────────────────────────
            with Container(id="sidebar"):
                with TabbedContent():

                    with TabPane("Run", id="tab-run"):
                        with VerticalScroll():
                            yield Label("Profile", classes="sect")
                            with RadioSet(id="profile-set"):
                                yield RadioButton("traj_controller", value=True)
                                yield RadioButton("vel_controller")
                            yield Label("Params", classes="sect")
                            with Horizontal(id="sim-row"):
                                yield Label("SIM")
                                yield Switch(id="sim-sw", value=False)
                            yield Input(value="192.168.1.14", placeholder="Robot IP", id="ip-input")
                            yield Label("Actions", classes="sect")
                            yield Button("▶ Start Profile",   id="b-up",      variant="success")
                            yield Button("■ Stop All",        id="b-stop")
                            yield Button("↓ Down+Orphans",    id="b-down",    variant="warning")
                            yield Button("↺ Restart Profile", id="b-restart")

                    with TabPane("Svcs", id="tab-svc"):
                        with VerticalScroll():
                            yield Select(
                                [(s, s) for s in ALL_SERVICES],
                                prompt="Select service…",
                                id="svc-select",
                            )
                            yield Button("▶ Start",   id="b-svc-up",      variant="success")
                            yield Button("■ Stop",    id="b-svc-stop",    variant="error")
                            yield Button("↺ Restart", id="b-svc-restart")
                            yield Button("📜 Logs",   id="b-svc-logs")

                    with TabPane("Sys", id="tab-sys"):
                        with VerticalScroll():
                            yield Button("🔨 Build Image",  id="b-build")
                            yield Button("⬇  Pull Latest",  id="b-pull")
                            yield Button("📊 Stats",        id="b-stats")
                            yield Button("📋 Containers",   id="b-ps")
                            yield Button("🔍 Svc List",     id="b-cfg")
                            yield Button("↻  Refresh",      id="b-ref")
                            yield Button("🗑  Prune",       id="b-prune",  variant="warning")

            # ── main area ────────────────────────────────────────────────────
            with Container(id="main"):
                with Container(id="status-pane"):
                    yield DataTable(id="tbl", zebra_stripes=True)

                with Container(id="log-pane"):
                    with Horizontal(id="log-toolbar"):
                        yield Label(" Log")
                        yield Button("Clr", id="b-clr")
                        yield Button("✕",   id="b-kill", variant="error")
                    yield RichLog(id="log", markup=True, highlight=True, auto_scroll=True)

        yield Footer()

    # ── lifecycle ─────────────────────────────────────────────────────────────
    def on_mount(self) -> None:
        tbl = self.query_one("#tbl", DataTable)
        tbl.add_columns("Service", "Status", "Profile")
        self.action_refresh_status()
        self.set_interval(6, self.action_refresh_status)

    # ── actions ───────────────────────────────────────────────────────────────
    def action_refresh_status(self) -> None:
        self._do_refresh()

    def action_toggle_sidebar(self) -> None:
        sidebar = self.query_one("#sidebar")
        sidebar.display = not sidebar.display

    def action_toggle_log(self) -> None:
        self._log_expanded = not self._log_expanded
        log_pane = self.query_one("#log-pane")
        if self._log_expanded:
            log_pane.add_class("expanded")
        else:
            log_pane.remove_class("expanded")

    @work(thread=True)
    def _do_refresh(self) -> None:
        data = _get_statuses()
        self.call_from_thread(self._apply_statuses, data)

    def _apply_statuses(self, data: dict[str, str]) -> None:
        tbl = self.query_one("#tbl", DataTable)
        tbl.clear()
        for svc in ALL_SERVICES:
            status = data.get(svc, "")
            if "Up" in status or "running" in status.lower():
                cell = Text(f"● {status}", style="bold green")
            elif "Exit" in status or "exited" in status.lower():
                cell = Text(f"● {status}", style="bold red")
            elif not status:
                cell = Text("○ not created", style="dim")
            else:
                cell = Text(f"○ {status}", style="yellow")
            profile_tag = next(
                (p for p, svcs in PROFILE_SERVICES.items() if svc in svcs), "all"
            )
            tbl.add_row(svc, cell, profile_tag)

    # ── widget events ─────────────────────────────────────────────────────────
    @on(RadioSet.Changed, "#profile-set")
    def _on_profile(self, e: RadioSet.Changed) -> None:
        if e.index < len(PROFILES):
            self._profile = PROFILES[e.index]

    @on(Switch.Changed, "#sim-sw")
    def _on_sim(self, e: Switch.Changed) -> None:
        self._sim = e.value

    @on(Input.Changed, "#ip-input")
    def _on_ip(self, e: Input.Changed) -> None:
        self._ip = e.value.strip() or "192.168.1.14"

    # ── button handlers ───────────────────────────────────────────────────────
    @on(Button.Pressed, "#b-up")
    def _up(self) -> None:
        self._run(
            ["docker", "compose", "-f", str(COMPOSE_FILE),
             "--profile", self._profile, "up"],
            env=self._env(),
            label=f"Start [{self._profile}]  SIM={self._sim}  IP={self._ip}",
        )

    @on(Button.Pressed, "#b-stop")
    def _stop(self) -> None:
        self._run(
            ["docker", "compose", "-f", str(COMPOSE_FILE), "--profile", "*", "stop"],
            label="Stop All",
        )

    @on(Button.Pressed, "#b-down")
    def _down(self) -> None:
        self._run(
            ["docker", "compose", "-f", str(COMPOSE_FILE),
             "--profile", "*", "down", "--remove-orphans"],
            label="Down + Remove Orphans",
        )

    @on(Button.Pressed, "#b-restart")
    def _restart(self) -> None:
        self._run(
            ["docker", "compose", "-f", str(COMPOSE_FILE),
             "--profile", self._profile, "restart"],
            env=self._env(),
            label=f"Restart [{self._profile}]",
        )

    @on(Button.Pressed, "#b-ref")
    def _ref(self) -> None:
        self.action_refresh_status()

    @on(Button.Pressed, "#b-svc-up")
    def _svc_up(self) -> None:
        if svc := self._svc():
            self._run(
                ["docker", "compose", "-f", str(COMPOSE_FILE),
                 "--profile", "*", "start", svc],
                env=self._env(), label=f"Start {svc}",
            )

    @on(Button.Pressed, "#b-svc-stop")
    def _svc_stop(self) -> None:
        if svc := self._svc():
            self._run(
                ["docker", "compose", "-f", str(COMPOSE_FILE), "stop", svc],
                label=f"Stop {svc}",
            )

    @on(Button.Pressed, "#b-svc-restart")
    def _svc_restart(self) -> None:
        if svc := self._svc():
            self._run(
                ["docker", "compose", "-f", str(COMPOSE_FILE), "restart", svc],
                label=f"Restart {svc}",
            )

    @on(Button.Pressed, "#b-svc-logs")
    def _svc_logs(self) -> None:
        if svc := self._svc():
            self._run(
                ["docker", "compose", "-f", str(COMPOSE_FILE),
                 "logs", "--tail=200", "-f", svc],
                label=f"Logs: {svc}",
            )

    @on(Button.Pressed, "#b-build")
    def _build(self) -> None:
        self._run(
            ["docker", "compose", "-f", str(COMPOSE_FILE), "build"],
            label="Build Image",
        )

    @on(Button.Pressed, "#b-pull")
    def _pull(self) -> None:
        self._run(
            ["docker", "compose", "-f", str(COMPOSE_FILE), "--profile", "*", "pull"],
            label="Pull Images",
        )

    @on(Button.Pressed, "#b-stats")
    def _stats(self) -> None:
        self._run(
            ["docker", "stats", "--no-stream", "--format",
             "table {{.Name}}\t{{.CPUPerc}}\t{{.MemUsage}}\t{{.NetIO}}"],
            label="Docker Stats",
        )

    @on(Button.Pressed, "#b-ps")
    def _ps(self) -> None:
        self._run(
            ["docker", "ps", "-a", "--format",
             "table {{.Names}}\t{{.Status}}\t{{.Image}}"],
            label="All Containers",
        )

    @on(Button.Pressed, "#b-cfg")
    def _cfg(self) -> None:
        self._run(
            ["docker", "compose", "-f", str(COMPOSE_FILE),
             "--profile", "*", "config", "--services"],
            label="Compose Services",
        )

    @on(Button.Pressed, "#b-prune")
    def _docker_prune(self) -> None:
        self._run(
            ["docker", "system", "prune", "-f"],
            label="Docker System Prune",
        )

    @on(Button.Pressed, "#b-clr")
    def _clr(self) -> None:
        self.query_one("#log", RichLog).clear()

    @on(Button.Pressed, "#b-kill")
    def _kill(self) -> None:
        if self._proc and self._proc.returncode is None:
            self._proc.kill()
            self.query_one("#log", RichLog).write(
                Text("⚠  Process killed by user.", style="bold yellow")
            )

    # ── helpers ───────────────────────────────────────────────────────────────
    def _env(self) -> dict[str, str]:
        env = os.environ.copy()
        env["SIM_MODE"]    = "true" if self._sim else "false"
        env["TARGET_IP"]   = self._ip
        env["MY_UID"]      = str(os.getuid())
        env["MY_GID"]      = str(os.getgid())
        env["VEL_CONTROL"] = "true" if self._profile == "vel_controller" else "false"
        return env

    def _svc(self) -> str | None:
        sel = self.query_one("#svc-select", Select)
        if sel.value is Select.BLANK:
            self.query_one("#log", RichLog).write(
                Text("⚠  No service selected.", style="bold yellow")
            )
            return None
        return str(sel.value)

    def _run(self, cmd: list[str], env: dict | None = None, label: str = "") -> None:
        log = self.query_one("#log", RichLog)
        ts = datetime.now().strftime("%H:%M:%S")
        log.write(Text(f"\n[{ts}] ▶  {label}", style="bold cyan"))
        log.write(Text(f"  $ {' '.join(cmd)}", style="dim"))
        self._stream(cmd, env)

    @work(exclusive=False)
    async def _stream(self, cmd: list[str], env: dict | None = None) -> None:
        log = self.query_one("#log", RichLog)
        try:
            self._proc = await asyncio.create_subprocess_exec(
                *cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.STDOUT,
                env=env,
            )
            assert self._proc.stdout is not None
            async for raw in self._proc.stdout:
                line = raw.decode("utf-8", errors="replace").rstrip()
                if not line:
                    continue
                low = line.lower()
                if any(k in low for k in ("error", "failed", "fatal", "exception")):
                    style = "bold red"
                elif "warn" in low:
                    style = "yellow"
                elif any(k in low for k in ("[info]", "starting", "started", "created", "done")):
                    style = "green"
                else:
                    style = "default"
                log.write(Text(line, style=style))

            await self._proc.wait()
            rc = self._proc.returncode
            if rc == 0:
                log.write(Text("✔  Done (exit 0)", style="bold green"))
            else:
                log.write(Text(f"✖  Failed (exit {rc})", style="bold red"))
        except Exception as exc:
            log.write(Text(f"✖  {exc}", style="bold red"))
        finally:
            self.action_refresh_status()


if __name__ == "__main__":
    RobotDockerTUI().run()