#!/usr/bin/env python3

import sys
from typing import List
from collections import namedtuple
import time
import subprocess
import datetime
import json
import os


CoprocessorConfiguration = namedtuple('CoprocessorConfiguration', [
                                      'should_upload', 'serial_port',
                                      'reset_pin', 'backdoor_pin'])


def extract_coprocessor(args: List[str]):
    should_upload = args[0] == 'True' or args[0] == 'true'
    serial_port = args[1]
    reset_pin = args[2]
    backdoor_pin = args[3]

    return CoprocessorConfiguration(
        should_upload, serial_port,
        reset_pin, backdoor_pin)


class CoprocessorResetControl():
    def __init__(self, reset_script_path, coprocessor):
        self._reset_without_backdoor_command = [
            str(reset_script_path),
            str(coprocessor.reset_pin),
            str(coprocessor.backdoor_pin),
            'false'
        ]
        self._reset_with_backdoor_command = [
            str(reset_script_path),
            str(coprocessor.reset_pin),
            str(coprocessor.backdoor_pin),
            'true'
        ]

    def reset_with_backdoor(self):
        subprocess.run(self._reset_with_backdoor_command)

    def reset_without_backdoor(self):
        subprocess.run(self._reset_without_backdoor_command)


def extract_reset_script_path(arg: str):
    if os.path.isfile(arg):
        return arg
    else:
        raise RuntimeError(
            f'Provided reset script path, \'{arg}\', is not a file!')


def extract_firmware_path(arg: str):
    if os.path.isfile(arg):
        return arg
    else:
        raise RuntimeError(
            f'Provided firmware path, \'{arg}\', is not a file!')


def load_initial_response():
    now = datetime.datetime.utcnow()
    current_date_as_string = now.strftime('%m/%d/%Y %H:%M')

    response = {
        'date': current_date_as_string,
        'operations': [
            {
                'description': 'Upload Coprocessor 1 Firmware',
                'is_success': False,
                'details': 'Not Started'
            },
            {
                'description': 'Upload Coprocessor 2 Firmware',
                'is_success': False,
                'details': 'Not Started'
            },
            {
                'description': 'Upload Coprocessor 3 Firmware',
                'is_success': False,
                'details': 'Not Started'
            },
        ],
    }

    return response


def load_uncaught_error_response(error):
    response = load_initial_response()
    response['operations'][0]['is_success'] = False
    response['operations'][0]['details'] = f'Unknown error: {error}'

    return response


class CoprocessorUploader:
    REQUIRED_PARAMETERS_COUNT = 6

    def __init__(self):
        self.parameters_ok = False
        self.firmware_path = None
        self.coprocessors = []
        self.reset_controls = None

        self.response = load_initial_response()

    def load_parameters(self, args: List[str]):
        self.parameters_ok = False

        try:
            self.firmware_path = extract_firmware_path(args[0])
            reset_script_path = extract_reset_script_path(args[1])

            self.coprocessors = [
                extract_coprocessor(args[2:6])]
            self.reset_controls = [
                CoprocessorResetControl(reset_script_path, coprocessor)
                for coprocessor in self.coprocessors
            ]

            self.parameters_ok = True

        except BaseException as error:
            self._response_setup_fail(error)

    def run(self):
        for coprocessor_index in range(len(self.coprocessors)):
            self._run_one_coprocessor(coprocessor_index)

    def reset_all_coprocessors(self):
        for coprocessor_index in range(len(self.coprocessors)):
            self.reset_controls[coprocessor_index].reset_without_backdoor()

    def _run_one_coprocessor(self, coprocessor_index):
        if not self.coprocessors[coprocessor_index].should_upload:
            self._response_coprocessor_not_requested(coprocessor_index)

        else:
            self.reset_controls[coprocessor_index].reset_with_backdoor()
            self._upload_one_coprocessor(coprocessor_index)
            self.reset_controls[coprocessor_index].reset_without_backdoor()

    def _response_setup_fail(self, error):
        self.response['operations'][0]['is_success'] = False
        self.response['operations'][0]['details'] = f'Could not setup upload: {error}'

    def _response_coprocessor_not_requested(self, coprocessor_index):
        self.response['operations'][coprocessor_index]['is_success'] = False
        self.response['operations'][coprocessor_index]['details'] = f'Upload not requested'

    def _response_coprocessor_success(self, coprocessor_index):
        self.response['operations'][coprocessor_index]['is_success'] = True
        self.response['operations'][coprocessor_index]['details'] = 'Success on upload'

    def _response_coprocessor_fail(self, coprocessor_index, details):
        self.response['operations'][coprocessor_index]['is_success'] = False
        self.response['operations'][coprocessor_index]['details'] = f'{details}'

    def _upload_one_coprocessor(self, coprocessor_index):
        MAX_RETRIES = 3
        RETRY_INTERVAL_S = 2
        success = False

        upload_args = f'-e -w -v -p {self.coprocessors[coprocessor_index].serial_port} {self.firmware_path}'.split()
        operation_details = 'Not Started'

        for _ in range(MAX_RETRIES):
            try:
                # import is wrapped in try block to catch import errors

                print(upload_args)
                from cc2538 import flash_main
                flash_main(upload_args)

            except BaseException as error:
                operation_details = f'{error}'
                time.sleep(RETRY_INTERVAL_S)

            else:
                success = True
                break

        if success:
            self._response_coprocessor_success(coprocessor_index)
        else:
            self._response_coprocessor_fail(
                coprocessor_index, operation_details)


if __name__ == '__main__':
    try:
        uploader = CoprocessorUploader()
        params = sys.argv[1:]
        uploader.load_parameters(params)

        uploader.run()
        response = json.dumps(uploader.response)

    except BaseException as unknown_error:
        response = load_uncaught_error_response(unknown_error)

    finally:
        uploader.reset_all_coprocessors()

    # result is echoed to be readable for the calling script
    print(response)
