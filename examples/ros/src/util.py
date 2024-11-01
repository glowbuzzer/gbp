from action_simple.action import ActionSimple

from gbp.connection import GbcConnectionProvider
from gbp.op import OpEnabledEffect

'''
Simple helper to create a temporary effect to enable operation. Unregisters the effect after complete.
'''


async def enable_operation(controller: GbcConnectionProvider) -> None:
    op = OpEnabledEffect()
    controller.register(op)
    try:
        await op.enable_operation()
    finally:
        controller.unregister(op)


# helper to create feedback message
def feedback_msg(msg):
    feedback = ActionSimple.Feedback()
    feedback.feedback = msg
    return feedback
