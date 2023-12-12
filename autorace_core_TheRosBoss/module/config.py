from typing import Literal

# допустимое смещение между центром изображения камеры и центром между обнаруженными желтой и белой линиями. 
OFFSET_BTW_CENTERS = 5
# отношение по высоте для выбора линии уровня между линиями движения
LINES_H_RATIO = 3/4
MAX_LINIEAR_SPEED = 0.25 # максимальная скорость в м/c
ANALOG_CAP_MODE = True
MAXIMUM_ANGLUAR_SPEED_CAP = 1 # если у нас угловая скорость == 0, то линейная максимальна, если угловая >= 1.5 то линейная = 0, в остальном случае угловая скорость преобразуется в коэф линейной

FOLLOW_ROAD_CROP_HALF  : Literal[True, False] = True # обрезать ли изо с перспективы попалам для трека соотвествующей линии (помогает избежать того что у нас может быть линия с двух сторон)
FOLLOW_ROAD_MODE : Literal["BOTH", "YELLOW", "WHITE"] = "WHITE" # какие линии учитываем для слежения за дорогой
WHITE_MODE_CONSTANT = 175 # Для режима белой линии, фиксированный сдвиг от перспективы "несуществующей" желтой линии
YELLOW_MODE_CONSTANT = 620 # Для режима желтой линии, фиксированный сдвиг от перспективы "несуществующей" белой линии
LINE_HISTORY_SIZE = 1 # сколько последних данных хранить о линии в случае ее потери

# Уровни дебага
"""
Уровни дебага
    0: нет его
    1: выводим данные с перспективы
    2: выводим данные с камеры
    3: выводим маски слоев
    4: машинка не поедет никуда
"""
DEBUG_LEVEL : Literal[0, 1, 2, 3, 4] = 2

# # Уровни выполнение задач
# """
# Уровни выполнение задач
# 0: светофор
# 1: перекресток
# 2: обход преград
# 3: парковка
# 4: сбить человека
# 5: туннель
# """
TASK_LEVEL : Literal[0, 1, 2, 3, 4, 5] = 0

# # Статус движения машинки
# """
# Статус движения машинки
# 0: стоять
# 1: движение
# """
STATUS_CAR : Literal[0, 1] = 0

LOGGING_POOL_SIZE = 3