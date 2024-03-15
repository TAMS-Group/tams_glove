#!/usr/bin/env python3

import time


class Profiler:

    def __init__(self, label, enabled=True, verbose=False):
        if enabled:
            self.label = label
        self.enabled = enabled
        if verbose:
            print("begin", label)

    def __enter__(self):
        if self.enabled:
            self.start = time.time()

    def __exit__(self, exception_type, exception_value, exception_traceback):
        if self.enabled:
            print(self.label, time.time() - self.start)
