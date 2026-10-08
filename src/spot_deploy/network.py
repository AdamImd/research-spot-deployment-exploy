"""Read the selected route. Never change network configuration or probe a subnet."""

import ipaddress
import json
import socket
import subprocess
from pathlib import Path

from .contracts import ContractError


def wired_route(config):
    if socket.gethostname() != config.expected_host:
        raise ContractError("unexpected control host")
    addresses = socket.getaddrinfo(config.endpoint, 443, socket.AF_INET, socket.SOCK_STREAM)
    ips = sorted({a[4][0] for a in addresses})
    if len(ips) != 1:
        raise ContractError("endpoint must resolve to exactly one IPv4 address")
    ip = ips[0]
    if ipaddress.ip_address(ip).is_loopback:
        raise ContractError("loopback is not a robot endpoint")
    routes = json.loads(
        subprocess.run(
            ["ip", "-j", "route", "get", ip], capture_output=True, text=True, timeout=5, check=True
        ).stdout
    )
    if len(routes) != 1 or routes[0].get("dev") != config.wired_interface:
        raise ContractError("robot route does not use the registered wired interface")
    iface = Path("/sys/class/net") / config.wired_interface
    if not iface.is_dir() or (iface / "wireless").exists():
        raise ContractError("registered interface is absent or wireless")
    if (iface / "operstate").read_text().strip() != "up":
        raise ContractError("wired interface is not up")
    return {
        "endpoint_ip": ip,
        "interface": config.wired_interface,
        "source": routes[0].get("prefsrc"),
        "gateway": routes[0].get("gateway"),
    }
