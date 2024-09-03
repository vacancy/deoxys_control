import os

ROOT_PATH = os.path.dirname(__file__)
if 'DEOXYS_CONFIG_ROOT' in os.environ:
    config_root = os.environ['DEOXYS_CONFIG_ROOT']
    message = f'Using DEOXYS_CONFIG_ROOT: {config_root}'
else:
    config_root = os.path.join(ROOT_PATH, '..', 'config')
    message = f'Using DEFAULT DEOXYS_CONFIG_ROOT: {config_root}'

from deoxys.utils.log_utils import get_deoxys_logger
logger = get_deoxys_logger()
logger.info(message)

if 'PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION' not in os.environ:
    os.environ['PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION'] = 'python'
    logger.info('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION not set, setting to python')

__version__ = "0.1.0"
