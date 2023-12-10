from collections import deque

from module.config import DEBUG_LEVEL, LOGGING_POOL_SIZE

previous_message_id = -1
previous_messages = deque(maxlen=LOGGING_POOL_SIZE)

def log_info(node, message, debug_level, msg_id=-1, allow_repeat=False):
    """
    
        Простенький фильтр логов
        
        node         - узел рос, который ведет логгирование  

        message      - текст сообщения  

        debug_level  - минимальный уровень дебага для отображения сообщения  

        msg_id       - айди сообщения для отброса дублей (на случай если у нас сообщения не повторяются текстом, но смыслом)  

            Если равно -1, то единицой идентификации становиться сам текст сообщения  

        allow_repeat - разрешить выводить несколько раз подряд одинаковое сообщение  


    """
    global previous_message_id, previous_messages

    if DEBUG_LEVEL < debug_level:
        return

    if not allow_repeat and msg_id == -1 and message in previous_messages:
        if previous_messages[-1] != message:
            previous_messages.append(message)
        return

    if not allow_repeat and previous_message_id == msg_id and msg_id != -1:
        return
    
    message_ = f"--- {message}"
    
    
    node.get_logger().info(message_)
    previous_message_id = msg_id
    previous_messages.append(message)