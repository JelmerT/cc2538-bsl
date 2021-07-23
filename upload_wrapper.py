#!/usr/bin/env python3

import sys
from typing import List
import logging
import time

from cc2538 import flash_main


def main(args: List[str]):
    MAX_RETRIES = 3
    RETRY_INTERVAL_S = 2
    success = False

    for _ in range(MAX_RETRIES):
        try:
            flash_main(args)
        except BaseException as error:
            print(f'{error}')
            time.sleep(RETRY_INTERVAL_S)
        else:
            success = True
            break

    if success:
        logging.info(f'Successfully written firmware: {args}')
    else:
        logging.error(
            f'Unable to write firmware after {MAX_RETRIES} attempts: {args}')


if __name__ == '__main__':
    main(sys.argv[1:])
