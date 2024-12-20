import asyncio

import pytest

import glowbuzzer.gbp
from glowbuzzer.gbp import GbcClient, ActivityStreamItem, ACTIVITYTYPE, DwellActivityParams, SoloActivity


@pytest.fixture
def solo():
    yield SoloActivity(0)


@pytest.fixture
def gbc(gbc: GbcClient, solo: SoloActivity):
    # Apply modifications to the gbc instance obtained from the global fixture
    gbc.register(solo)

    yield gbc


@pytest.mark.asyncio
async def test_solo_activity_completes(gbc: GbcClient, solo: SoloActivity):
    """
    Test that a solo activity completes successfully and can be awaited
    """

    gbc.register(glowbuzzer.gbp.MachineStateLogger())

    tag, result = await solo.exec(
        gbc, ActivityStreamItem(activityType=ACTIVITYTYPE.ACTIVITYTYPE_DWELL, dwell=DwellActivityParams(msToDwell=2))
    )
    assert result == True


@pytest.mark.asyncio
async def test_solo_cancel(gbc: GbcClient, solo: SoloActivity):
    """
    Test that a solo activity can be cancelled
    """

    async def run():
        _, dwell = await solo.exec(
            gbc,
            ActivityStreamItem(activityType=ACTIVITYTYPE.ACTIVITYTYPE_DWELL, dwell=DwellActivityParams(msToDwell=5000)),
        )
        return dwell

    # Start dwell in background
    task = asyncio.create_task(run())
    await asyncio.sleep(1)

    # Cancel the dwell
    _, cancellation = await solo.cancel(gbc)
    assert cancellation == True

    # Ensure the dwell task is cancelled
    assert task.result() == False
