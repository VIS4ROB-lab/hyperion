import inspect

from functools import partial
from typing import Callable, Any


def get_func_from_potential_partial(maybe_partial: Callable[..., Any]) -> Any:
    return maybe_partial.func if isinstance(maybe_partial, partial) else maybe_partial


def get_fixed_args_from_potential_partial(maybe_partial: Callable[..., Any]) -> list[str]:
    if isinstance(maybe_partial, partial):
        func = maybe_partial.func
        sig = inspect.signature(func)
        fixed_pos_args = maybe_partial.args
        fixed_kwargs = maybe_partial.keywords
        fixed_param_names = list(sig.parameters.keys())[:len(fixed_pos_args)]

        if fixed_kwargs:
            fixed_param_names.extend(fixed_kwargs.keys())

        return fixed_param_names
    else:
        return []


def get_name_from_potential_partial(maybe_partial: Callable[..., Any]) -> str:
    if isinstance(maybe_partial, partial):
        return maybe_partial.func.__name__
    try:
        return maybe_partial.__name__
    except AttributeError:
        return str(maybe_partial)


def get_suffix_from_potential_partial(maybe_partial: Callable[..., Any]):
    if isinstance(maybe_partial, partial):
        if maybe_partial.keywords:
            return '_' + ''.join(f"{value}" for key, value in maybe_partial.keywords.items())
    return ''
