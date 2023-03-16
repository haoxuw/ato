# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

import logging


class ColoredFormatter(logging.Formatter):
    grey = "\x1b[38;21m"
    green = "\x1b[1;32m"
    yellow = "\x1b[0;33m"
    red = "\x1b[0;31m"
    purple = "\x1b[1;35m"
    reset = "\x1b[0m"
    format_template = (
        "[%(asctime)s] {%(pathname)s:%(lineno)d} %(levelname)s - %(message)s"
    )
    colored_format = {
        logging.DEBUG: grey + format_template + reset,
        logging.INFO: green + format_template + reset,
        logging.WARNING: yellow + format_template + reset,
        logging.ERROR: red + format_template + reset,
        logging.CRITICAL: purple + format_template + reset,
    }

    def format(self, record):
        log_fmt = self.colored_format.get(record.levelno)
        formatter = logging.Formatter(log_fmt)
        return formatter.format(record)


logger = logging.getLogger()
logger.setLevel(logging.INFO)  # DEBUG

ch = logging.StreamHandler()
ch.setFormatter(ColoredFormatter())
logger.handlers.clear()
logger.addHandler(ch)
