"""Windows worker containment using a kill-on-close Job Object.

Create suspended, assign the Job, then resume: no child can escape during
startup. Lease supervisors break away explicitly to preserve independent leases.
"""
import ctypes
from ctypes import wintypes as w
import os
import subprocess


class BasicLimit(ctypes.Structure):
    _fields_ = [('process_time', ctypes.c_int64), ('job_time', ctypes.c_int64),
                ('flags', w.DWORD), ('min_working', ctypes.c_size_t), ('max_working', ctypes.c_size_t),
                ('active_processes', w.DWORD), ('affinity', ctypes.c_size_t),
                ('priority', w.DWORD), ('scheduling', w.DWORD)]


class IOCount(ctypes.Structure):
    _fields_ = [(name, ctypes.c_uint64) for name in ('read_ops', 'write_ops', 'other_ops', 'read', 'write', 'other')]


class ExtendedLimit(ctypes.Structure):
    _fields_ = [('basic', BasicLimit), ('io', IOCount), ('process_memory', ctypes.c_size_t),
                ('job_memory', ctypes.c_size_t), ('peak_process', ctypes.c_size_t), ('peak_job', ctypes.c_size_t)]


class StartupInfo(ctypes.Structure):
    _fields_ = [('cb', w.DWORD), ('reserved', w.LPWSTR), ('desktop', w.LPWSTR), ('title', w.LPWSTR),
                *[(name, w.DWORD) for name in ('x', 'y', 'xsize', 'ysize', 'xchars', 'ychars', 'fill', 'flags')],
                ('show', w.WORD), ('reserved_size', w.WORD), ('reserved_bytes', w.LPVOID),
                ('stdin', w.HANDLE), ('stdout', w.HANDLE), ('stderr', w.HANDLE)]


class ProcessInfo(ctypes.Structure):
    _fields_ = [('process', w.HANDLE), ('thread', w.HANDLE), ('pid', w.DWORD), ('tid', w.DWORD)]


class JobProcess:
    def __init__(self, command):
        self.command, self.returncode = command, None
        self.kernel = ctypes.WinDLL('kernel32', use_last_error=True)
        def bind(name, restype, argtypes):
            function = getattr(self.kernel, name)
            function.restype, function.argtypes = restype, argtypes
            return function
        self.close_handle = bind('CloseHandle', w.BOOL, [w.HANDLE])
        create_job = bind('CreateJobObjectW', w.HANDLE, [w.LPVOID, w.LPCWSTR])
        set_info = bind('SetInformationJobObject', w.BOOL, [w.HANDLE, ctypes.c_int, w.LPVOID, w.DWORD])
        assign = bind('AssignProcessToJobObject', w.BOOL, [w.HANDLE, w.HANDLE])
        resume = bind('ResumeThread', w.DWORD, [w.HANDLE])
        get_std = bind('GetStdHandle', w.HANDLE, [w.DWORD])
        create = bind('CreateProcessW', w.BOOL, [w.LPCWSTR, w.LPWSTR, w.LPVOID, w.LPVOID, w.BOOL,
                                                w.DWORD, w.LPVOID, w.LPCWSTR,
                                                ctypes.POINTER(StartupInfo), ctypes.POINTER(ProcessInfo)])
        self.get_exit = bind('GetExitCodeProcess', w.BOOL, [w.HANDLE, ctypes.POINTER(w.DWORD)])
        self.wait_handle = bind('WaitForSingleObject', w.DWORD, [w.HANDLE, w.DWORD])
        self.terminate_job = bind('TerminateJobObject', w.BOOL, [w.HANDLE, w.UINT])
        self.job, self.handle = create_job(None, None), None
        self.check(self.job)
        limits = ExtendedLimit()
        limits.basic.flags = 0x2000 | 0x800  # KILL_ON_JOB_CLOSE | BREAKAWAY_OK
        try:
            self.check(set_info(self.job, 9, ctypes.byref(limits), ctypes.sizeof(limits)))
            startup, info = StartupInfo(), ProcessInfo()
            startup.cb, startup.flags = ctypes.sizeof(startup), 0x100
            startup.stdin, startup.stdout, startup.stderr = [get_std(value) for value in (-10, -11, -12)]
            environment = dict(os.environ, RSIM_WINDOWS_JOB='1', PYTHONUTF8='1')
            block = ctypes.create_unicode_buffer('\0'.join(f'{key}={value}' for key, value in sorted(environment.items())) + '\0\0')
            cmdline = ctypes.create_unicode_buffer(subprocess.list2cmdline(command))
            self.check(create(None, cmdline, None, None, True, 0x4 | 0x08000000 | 0x400,
                              block, None, ctypes.byref(startup), ctypes.byref(info)))
            self.handle, self.pid = info.process, info.pid
            try:
                self.check(assign(self.job, self.handle))
                if resume(info.thread) == 0xffffffff:
                    raise ctypes.WinError(ctypes.get_last_error())
            except BaseException:
                terminate = bind('TerminateProcess', w.BOOL, [w.HANDLE, w.UINT])
                terminate(self.handle, 1)
                raise
            finally:
                self.close_handle(info.thread)
        except BaseException:
            self.close()
            raise

    @staticmethod
    def check(result):
        if not result:
            raise ctypes.WinError(ctypes.get_last_error())

    def poll(self):
        if self.returncode is None and self.wait_handle(self.handle, 0) == 0:
            code = w.DWORD()
            self.check(self.get_exit(self.handle, ctypes.byref(code)))
            self.returncode = code.value
        return self.returncode

    def wait(self, timeout=None):
        milliseconds = 0xffffffff if timeout is None else int(timeout*1000)
        status = self.wait_handle(self.handle, milliseconds)
        if status == 258:
            raise subprocess.TimeoutExpired(self.command, timeout)
        if status != 0:
            raise ctypes.WinError(ctypes.get_last_error())
        return self.poll()

    def kill(self):
        self.check(self.terminate_job(self.job, 1))

    def close(self):
        if self.job:
            self.close_handle(self.job)
            self.job = None
        if self.handle:
            self.close_handle(self.handle)
            self.handle = None
