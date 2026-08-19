"""Lock address-family-neutral NTRIP connection behavior."""

import sys
from pathlib import Path
from unittest.mock import MagicMock, patch


NTRIP_SRC = (
    Path(__file__).resolve().parents[1]
    / "external"
    / "ntrip_client"
    / "src"
)
sys.path.insert(0, str(NTRIP_SRC))

from ntrip_client.ntrip_client import NTRIPClient  # noqa: E402


def test_client_uses_address_family_neutral_connection_helper():
    """DNS resolution may select IPv4, native IPv6, or a DNS64 address."""
    server_socket = MagicMock()
    server_socket.recv.return_value = b"ICY 200 OK\r\n"
    client = NTRIPClient(
        host="caster.example",
        port=2101,
        mountpoint="MOUNT",
        ntrip_version=None,
        username=None,
        password=None,
    )

    with patch(
        "ntrip_client.ntrip_client.socket.create_connection",
        return_value=server_socket,
    ) as create_connection:
        assert client.connect()

    create_connection.assert_called_once_with(
        ("caster.example", 2101), timeout=5
    )
    server_socket.send.assert_called_once()
