import requests
import math
import os
import time
try:
    import queue
except ImportError:
    import Queue as queue
import threading
from pyodm.utils import AtomicCounter
from pyodm.exceptions import RangeNotAvailableError, OdmError
from urllib3.exceptions import ReadTimeoutError

def download(url, destination, progress_callback=None, parallel_downloads=16, parallel_chunks_size=10, timeout=30):
    """Download files in parallel (download accelerator)

    Args:
        url (str): URL to download
        destination (str): directory where to download file. If the directory does not exist, it will be created.
        progress_callback (function): an optional callback with one parameter, the download progress percentage.
        parallel_downloads (int): maximum number of parallel downloads if the node supports http range.
        parallel_chunks_size (int): size in MB of chunks for parallel downloads
        timeout (int): seconds before timing out
    Returns:
        str: path to file
    """
    if not os.path.exists(destination):
        os.makedirs(destination, exist_ok=True)

    try:

        download_stream = requests.get(url, timeout=timeout, stream=True)
        headers = download_stream.headers
        
        output_path = os.path.join(destination, os.path.basename(url))

        # Keep track of download progress (if possible)
        content_length = download_stream.headers.get('content-length')
        total_length = int(content_length) if content_length is not None else None
        downloaded = 0
        chunk_size = int(parallel_chunks_size * 1024 * 1024)
        use_fallback = False
        accept_ranges = headers.get('accept-ranges')

        # Can we do parallel downloads?
        if accept_ranges is not None and accept_ranges.lower() == 'bytes' and total_length is not None and total_length > chunk_size and parallel_downloads > 1:
            num_chunks = int(math.ceil(total_length / float(chunk_size)))
            num_workers = parallel_downloads

            class nonloc:
                completed_chunks = AtomicCounter(0)
                merge_chunks = [False] * num_chunks
                error = None

            def merge():
                current_chunk = 0

                with open(output_path, "wb") as out_file:
                    while current_chunk < num_chunks and nonloc.error is None:
                        if nonloc.merge_chunks[current_chunk]:
                            chunk_file = "%s.part%s" % (output_path, current_chunk)
                            with open(chunk_file, "rb") as fd:
                                out_file.write(fd.read())

                            os.unlink(chunk_file)
                            
                            current_chunk += 1
                        else:
                            time.sleep(0.1)

            def worker():
                while True:
                    task = q.get()
                    part_num, bytes_range = task
                    if bytes_range is None or nonloc.error is not None:
                        q.task_done()
                        break

                    try:
                        # Download chunk
                        res = requests.get(url, stream=True, timeout=timeout, headers={'Range': 'bytes=%s-%s' % bytes_range})
                        if res.status_code == 206:
                            with open("%s.part%s" % (output_path, part_num), 'wb') as fd:
                                bytes_written = 0
                                try:
                                    for chunk in res.iter_content(4096):
                                        bytes_written += fd.write(chunk)
                                except (requests.exceptions.Timeout, requests.exceptions.ConnectionError) as e:
                                    raise OdmError(str(e))
                                
                                if bytes_written != (bytes_range[1] - bytes_range[0] + 1):
                                    # Process again
                                    q.put((part_num, bytes_range))
                                    return

                            with nonloc.completed_chunks.lock:
                                nonloc.completed_chunks.value += 1

                                if progress_callback is not None:
                                    progress_callback(100.0 * nonloc.completed_chunks.value / num_chunks)
                        
                            nonloc.merge_chunks[part_num] = True
                        else:
                            nonloc.error = RangeNotAvailableError()
                    except OdmError as e:
                        time.sleep(5)
                        q.put((part_num, bytes_range))
                    except Exception as e:
                        nonloc.error = e
                    finally:
                        q.task_done()

            q = queue.PriorityQueue()
            threads = []
            for i in range(num_workers):
                t = threading.Thread(target=worker)
                t.start()
                threads.append(t)

            merge_thread = threading.Thread(target=merge)
            merge_thread.start()

            range_start = 0

            for i in range(num_chunks):
                range_end = min(range_start + chunk_size - 1, total_length - 1)
                q.put((i, (range_start, range_end)))
                range_start = range_end + 1

            # block until all tasks are done
            while not all(nonloc.merge_chunks) and nonloc.error is None:
                time.sleep(0.1)

            # stop workers
            for i in range(len(threads)):
                q.put((-1, None))
            for t in threads:
                t.join()
            
            merge_thread.join()

            if nonloc.error is not None:
                if isinstance(nonloc.error, RangeNotAvailableError):
                    use_fallback = True
                else:
                    raise nonloc.error
        else:
            use_fallback = True

        if use_fallback:
            # Single connection, boring download
            with open(output_path, 'wb') as fd:
                for chunk in download_stream.iter_content(4096):
                    downloaded += len(chunk)

                    if progress_callback is not None and total_length is not None:
                        progress_callback((100.0 * float(downloaded) / total_length))

                    fd.write(chunk)
                
    except (requests.exceptions.Timeout, requests.exceptions.ConnectionError, ReadTimeoutError) as e:
        raise OdmError(e)

    return output_path