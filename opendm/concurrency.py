from vmem import virtual_memory
import os
import sys
try:
    import Queue as queue
except:
    import queue
import threading
import time
from opendm import log

def get_max_memory(minimum = 5, use_at_most = 0.5):
    """
    :param minimum minimum value to return (return value will never be lower than this)
    :param use_at_most use at most this fraction of the available memory. 0.5 = use at most 50% of available memory
    :return percentage value of memory to use (75 = 75%).
    """
    return max(minimum, (100 - virtual_memory().percent) * use_at_most)

def get_max_memory_mb(minimum = 100, use_at_most = 0.5):
    """
    :param minimum minimum value to return (return value will never be lower than this)
    :param use_at_most use at most this fraction of the available memory. 0.5 = use at most 50% of available memory
    :return value of memory to use in megabytes.
    """
    return max(minimum, (virtual_memory().available / 1024 / 1024) * use_at_most)

def get_total_memory():
    return virtual_memory().total

def parallel_map(func, items, max_workers=1, single_thread_fallback=True):
    """
    Our own implementation for parallel processing
    which handles gracefully CTRL+C and reverts to 
    single thread processing in case of errors
    :param items list of objects
    :param func function to execute on each object
    """
    global error
    error = None

    def process_one(q):
        func(q)

    def worker():
        global error

        while True:
            (num, q) = pq.get()
            if q is None or error is not None:
                pq.task_done()
                break

            try:
                process_one(q)
            except Exception as e:
                error = e
            finally:
                pq.task_done()

    if max_workers > 1:
        use_single_thread = False
        pq = queue.PriorityQueue()
        threads = []
        for i in range(max_workers):
            t = threading.Thread(target=worker)
            t.start()
            threads.append(t)

        i = 1
        for t in items:
            pq.put((i, t))
            i += 1

        def stop_workers():
            for i in range(len(threads)):
                pq.put((-1, None))
            for t in threads:
                t.join()

        # block until all tasks are done
        try:
            while pq.unfinished_tasks > 0:
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("CTRL+C terminating...")
            stop_workers()
            sys.exit(1)

        stop_workers()

        if error is not None and single_thread_fallback:
            # Try to reprocess using a single thread
            # in case this was a memory error
            log.ODM_WARNING("Failed to run process in parallel, retrying with a single thread...")
            use_single_thread = True
    else:
        use_single_thread = True

    if use_single_thread:
        # Boring, single thread processing
        for q in items:
            process_one(q)