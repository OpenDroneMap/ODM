from opendm import log
from shlex import _find_unsafe
import json
import os

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

def save_opts(opts_json, args):
    try:
        with open(opts_json, "w", encoding='utf-8') as f:
            f.write(json.dumps(args_to_dict(args)))
    except Exception as e:
        log.ODM_WARNING("Cannot save options to %s: %s" % (opts_json, str(e)))

def compare_args(opts_json, args, rerun_stages):
    if not os.path.isfile(opts_json):
        return {}

    try:
        diff = {}

        with open(opts_json, "r", encoding="utf-8") as f:
            prev_args = json.loads(f.read())
        cur_args = args_to_dict(args)

        for opt in cur_args:
            cur_value = cur_args[opt]
            prev_value = prev_args.get(opt, None)
            stage = rerun_stages.get(opt, None)

            if stage is not None and cur_value != prev_value:
                diff[opt] = prev_value
        
        return diff
    except:
        return {}

def find_rerun_stage(opts_json, args, rerun_stages, processopts):
    # Find the proper rerun stage if one is not explicitly set
    if not ('rerun_is_set' in args or 'rerun_from_is_set' in args or 'rerun_all_is_set' in args):
        args_diff = compare_args(opts_json, args, rerun_stages)
        if args_diff:
            if 'split_is_set' in args:
                return processopts[processopts.index('dataset'):], args_diff

            try:
                stage_idxs = [processopts.index(rerun_stages[opt]) for opt in args_diff.keys() if rerun_stages[opt] is not None]
                return processopts[min(stage_idxs):], args_diff
            except ValueError as e:
                print(str(e))
    return None, {}