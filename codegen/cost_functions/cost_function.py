import itertools

from pathlib import Path
from symforce import typing as T
from symforce.codegen import Codegen, CppConfig, template_util
from typing import Optional

from codegen.splines.partial import *

CURRENT_DIR = Path(__file__).parent


class FactorCppConfig(CppConfig):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    @classmethod
    def template_dir(cls) -> Path:
        return CURRENT_DIR / "templates"

    def templates_to_render(self, generated_file_name: str) -> T.List[T.Tuple[str, str]]:
        templates = [("FACTOR.hpp.jinja", f"{generated_file_name}.hpp")]
        if self.explicit_template_instantiation_types is not None:
            templates.append(("FACTOR.cpp.jinja", f"{generated_file_name}.cpp"))
        return templates


class CostFunction:
    @classmethod
    def get_factor_name(cls, maybe_partial, prefix: str = None, suffix: str = None) -> str:
        factor_name = get_name_from_potential_partial(maybe_partial)
        prefix = (prefix + '_') if prefix else ''
        suffix = ('_' + suffix) if suffix else ''
        middle = factor_name.split("_factor")[0]
        return prefix + middle + suffix + '_factor'

    @classmethod
    def get_cost_function_name(cls, maybe_partial, custom_name: Optional[str], prefix: str = None,
                               suffix: str = None) -> str:
        factor_name = get_name_from_potential_partial(maybe_partial)
        prefix = (prefix + '_') if prefix else ''
        suffix = ('_' + suffix) if suffix else ''
        middle = factor_name.split("_factor")[0]
        return (prefix + custom_name + suffix if custom_name else prefix + middle + suffix) + '_cost_function'

    @classmethod
    def get_reduced_args(cls, maybe_partial, excluded_args: list[str]) -> list[str]:
        func = get_func_from_potential_partial(maybe_partial)
        all_inputs = func.__code__.co_varnames[:func.__code__.co_argcount]
        return [arg for arg in all_inputs if arg not in excluded_args]

    @classmethod
    def get_optimizable_args(cls, reduced_args, member_args) -> list[str]:
        # Sanity checks.
        for arg in member_args:
            assert arg in reduced_args
        return [arg for arg in reduced_args if arg not in member_args]

    def __init__(self,
                 factor,
                 namespace: str,
                 custom_name: Optional[str] = None,
                 member_args: dict[str: str] = None,
                 specialized_args: Optional[list[str]] = None,
                 excluded_args: list[str] = ['epsilon'],
                 prefix: str = None,
                 suffix: str = None):
        self.factor = factor
        self.namespace = namespace
        self.factor_name = CostFunction.get_factor_name(factor, prefix, suffix)
        self.cost_function_name = CostFunction.get_cost_function_name(factor, custom_name, prefix, suffix)
        fixed_args = get_fixed_args_from_potential_partial(factor)
        self.reduced_args = CostFunction.get_reduced_args(factor, fixed_args + excluded_args)
        self.member_args = member_args if member_args else []
        self.optimizable_args = CostFunction.get_optimizable_args(self.reduced_args, self.member_args)

        if specialized_args:
            for arg in specialized_args:
                assert arg not in self.member_args and arg in self.optimizable_args, \
                    "Specialized argument must not be a member and must be optimizable!"
            self.specialized_args = specialized_args
        else:
            self.specialized_args = []

        permutation_set = set()
        permutation_set.add(tuple(self.optimizable_args))
        for product in itertools.product([False, True], repeat=len(self.optimizable_args)):
            permutation = [self.optimizable_args[j] for j, arg in enumerate(product) if
                           arg or self.optimizable_args[j] not in self.specialized_args]
            if permutation: permutation_set.add(tuple(permutation))
        self.specialized_permutations = sorted([list(args) for args in permutation_set])
        self.relative_dir_to_factors = 'sym'
        self.imports = []

    def generate(self, output_dir: Path, extends_ceres: bool):
        sym_dir = output_dir / 'sym'
        sym_namespace = 'sym_ceres' if extends_ceres else 'sym_hyperion'
        factor = Codegen.function(self.factor,
                                  config=FactorCppConfig(),  # explicit_template_instantiation_types=['double']
                                  name=self.factor_name)

        factor.optimizable_args = self.optimizable_args  # Add extra information to Jinja.
        factor.extends_ceres = extends_ceres  # Add extra information to Jinja.
        factor.generate_function(output_dir=sym_dir, namespace=sym_namespace, skip_directory_nesting=True)
        self.imports.append(factor.name)
        print("Generated:", factor.name, '(Ceres)' if extends_ceres else '(Hyperion)', flush=True)

        for args in self.specialized_permutations:
            factor_with_jacobians = factor.with_jacobians(which_args=args)
            factor_with_jacobians.optimizable_args = self.optimizable_args  # Add extra information to Jinja.
            factor_with_jacobians.extends_ceres = extends_ceres  # Add extra information to Jinja.
            factor_with_jacobians.generate_function(output_dir=sym_dir, namespace=sym_namespace,
                                                    skip_directory_nesting=True)
            self.imports.append(factor_with_jacobians.name)
            print("Generated:", factor_with_jacobians.name, '(Ceres)' if extends_ceres else '(Hyperion)', flush=True)

        cost_function_templates = [("COST_FUNCTION.hpp.jinja", f"{self.cost_function_name}.hpp"),
                                   ("COST_FUNCTION.cpp.jinja", f"{self.cost_function_name}.cpp")]

        templates = template_util.TemplateList()
        for source, dest in cost_function_templates:
            templates.add(
                template_path=source,
                data=dict(factor.common_data(), factor=factor, cost=self),
                config=factor.config.render_template_config,
                template_dir=factor.config.template_dir(),
                output_path=output_dir / dest,
            )
        templates.render()
        print("Generated:", self.cost_function_name, '(Ceres)' if extends_ceres else '(Hyperion)', flush=True)
