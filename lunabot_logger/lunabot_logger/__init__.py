# ANSI color codes
_RESET   = "\033[0m"
_RED     = "\033[1;31m"
_GREEN   = "\033[1;32m"
_YELLOW  = "\033[1;33m"
_CYAN    = "\033[1;36m"


class Logger:
    """
    Wrapper around rclpy's Logger with colored output (matches the C++ lunabot_logger.hpp macros)
    """

    def __init__(self, node_or_logger):
        """Accept either a Node or an rclpy Logger directly."""
        if hasattr(node_or_logger, 'get_logger'):
            self._log = node_or_logger.get_logger()
        else:
            self._log = node_or_logger

    def info(self, msg, **kwargs):
        self._log.info(msg, **kwargs)

    def debug(self, msg, **kwargs):
        self._log.debug(msg, **kwargs)

    def success(self, msg, **kwargs):
        self._log.info(f'{_GREEN}{msg}{_RESET}', **kwargs)

    def action(self, msg, **kwargs):
        self._log.info(f'{_CYAN}{msg}{_RESET}', **kwargs)

    def warning(self, msg, **kwargs):
        self._log.warning(f'{_YELLOW}{msg}{_RESET}', **kwargs)

    def failure(self, msg, **kwargs):
        self._log.error(f'{_RED}{msg}{_RESET}', **kwargs)

    def error(self, msg, **kwargs):
        self._log.error(msg, **kwargs)
