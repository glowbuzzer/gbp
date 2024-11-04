from action_simple.action import ActionSimple


def feedback_msg(msg):
    """
    Create a feedback message
    :param msg: The message string
    :return: The message object
    """
    feedback = ActionSimple.Feedback()
    feedback.feedback = msg
    return feedback
