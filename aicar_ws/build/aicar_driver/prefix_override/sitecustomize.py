import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yusy/maedongyee/aicar_madongeeee/aicar_ws/install/aicar_driver'
