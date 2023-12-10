from collections import deque

from module.config import DEBUG_LEVEL, LOGGIN_POOL_SIZE

previous_message_id = -1
previous_messages = deque(maxlen=LOGGIN_POOL_SIZE)

def log_info(node, message, debug_level, msg_id=-1, allow_repeat=False):
    global previous_message_id, previous_messages

    if DEBUG_LEVEL < debug_level:
        return

    if not allow_repeat and msg_id == -1 and message in previous_messages:
        return

    if not allow_repeat and previous_message_id == msg_id and msg_id != -1:
        return
    
    message_ = f"--- {message}"
    
    
    node.get_logger().info(message_)
    previous_message_id = msg_id
    previous_messages.append(message)