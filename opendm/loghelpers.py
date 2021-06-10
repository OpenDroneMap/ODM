from shlex import _find_unsafe

def double_quote(s):
    """Return a shell-escaped version of the string *s*."""
    if not s:
        return '""'
    if _find_unsafe(s) is None:
        return s

    # use double quotes, and prefix double quotes with a \
    # the string $"b is then quoted as "$\"b"
    return '"' + s.replace('"', '\\\"') + '"'

def args_to_dict(args):
    args_dict = vars(args)
    result = {}
    for k in sorted(args_dict.keys()):
        # Skip _is_set keys
        if k.endswith("_is_set"):
            continue

        # Don't leak token
        if k == 'sm_cluster' and args_dict[k] is not None:
            result[k] = True
        else:
            result[k] = args_dict[k]
    
    return result