import logging
from typing import cast

import pytest
import pytest_asyncio

from gbp import log, GbcClient
from gbp.effects import HeatbeatEcho, OpEnabledEffect


@pytest_asyncio.fixture
async def gbc():
    log.setLevel(logging.DEBUG)

    gbc = GbcClient("ws://localhost:9001/ws")
    try:
        gbc.register(
            HeatbeatEcho(),  # maintain heartbeat with gbc
        )

        await gbc.connect(blocking=False)

        await gbc.run_once(OpEnabledEffect(), lambda op: cast(OpEnabledEffect, op).enable_operation())
    except Exception as e:
        await gbc.close()
        logging.error("Failed to connect to GBC: %s", e)
        pytest.skip("Failed to connect to GBC")

    yield gbc

    await gbc.close()