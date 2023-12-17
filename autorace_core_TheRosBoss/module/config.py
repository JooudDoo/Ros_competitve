from typing import Literal

OFFSET_BTW_CENTERS : int = 5 # допустимое смещение между центром изображения камеры и центром между обнаруженными желтой и белой линиями. 
LINES_H_RATIO : float = 3/4 # отношение по высоте для выбора линии уровня между линиями движения
MAX_LINIEAR_SPEED : float = 0.25 # максимальная скорость в м/c
ANALOG_CAP_MODE : bool = True
MAXIMUM_ANGLUAR_SPEED_CAP : float = 1 # если у нас угловая скорость == 0, то линейная максимальна, если угловая >= 1.5 то линейная = 0, в остальном случае угловая скорость преобразуется в коэф линейной

FOLLOW_ROAD_CROP_HALF  : Literal[True, False] = True # обрезать ли изо с перспективы попалам для трека соотвествующей линии (помогает избежать того что у нас может быть линия с двух сторон)
FOLLOW_ROAD_MODE : Literal["BOTH", "YELLOW", "WHITE"] = "WHITE" # какие линии учитываем для слежения за дорогой
WHITE_MODE_CONSTANT : int = 175 # Для режима белой линии, фиксированный сдвиг от перспективы "несуществующей" желтой линии
YELLOW_MODE_CONSTANT : int = 615 # Для режима желтой линии, фиксированный сдвиг от перспективы "несуществующей" белой линии
LINE_HISTORY_SIZE : int = 1 # сколько последних данных хранить о линии в случае ее потери

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

LOGGING_POOL_SIZE : int = 3