import _thread
import threading
import types


# Source: https://theorangeduck.com/page/synchronized-python. Modified, and utilizing RLocks instead.


def synchronized(item):
    """Synchronizes the annotated class, function or attribute across all threads"""

    if type(item) is str:
        decorator = __synchronized_with_attr(item)
        return decorator(item)

    if type(item) is _thread.LockType:
        decorator = __synchronized_with(item)
        return decorator(item)

    else:
        new_lock = threading.RLock()
        decorator = __synchronized_with(new_lock)
        return decorator(item)


def __synchronized_with_attr(lock_name):
    def decorator(method):
        def synced_method(self, *args, **kws):
            lock = getattr(self, lock_name)
            with lock:
                return method(self, *args, **kws)

        return synced_method

    return decorator


def __synchronized_with(lock):
    def synchronized_obj(obj):

        if isinstance(obj, types.FunctionType):

            obj.__lock__ = lock

            def func(*args, **kws):
                with lock:
                    return obj(*args, **kws)

            return func

        elif type(obj) is type:

            orig_init = obj.__init__

            def __init__(self, *args, **kws):
                self.__lock__ = lock
                orig_init(self, *args, **kws)

            obj.__init__ = __init__

            for key in obj.__dict__:
                val = obj.__dict__[key]
                if isinstance(val, types.FunctionType):
                    decorator = __synchronized_with(lock)
                    setattr(obj, key, decorator(val))

            return obj

    return synchronized_obj
