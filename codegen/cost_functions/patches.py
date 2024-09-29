import copy

from symforce import ops, jacobian_helpers
from symforce import typing as T
from symforce.codegen import Codegen
from symforce.values import Values


def codegen_with_ceres_jacobians_patch(
        self,
        which_args: T.Sequence[str] = None,
        which_results: T.Sequence[int] = (0,),
        include_results: bool = True,
        name: str = None,
        sparse_jacobians: bool = False,
) -> Codegen:
    """
    Modified from the original Codegen function to make the Jacobians compatible with Ceres.
    """
    if which_args is None:
        which_args = list(self.inputs.keys())

    assert which_args, "Cannot compute a linearization with respect to 0 arguments"

    assert list(sorted(which_results)) == list(which_results), "which_results must be sorted"

    # Get docstring
    docstring_lines = self.docstring.rstrip().split("\n")

    # Make the new outputs
    if include_results:
        outputs = copy.deepcopy(self.outputs)
    else:
        outputs = Values()

        # Copy in results we're not differentiating
        self_outputs_keys = list(self.outputs.keys())
        for i in range(len(self.outputs)):
            if i not in which_results:
                outputs[self_outputs_keys[i]] = self.outputs[self_outputs_keys[i]]

        # Remove return val lines from docstring
        # TODO(aaron): Make this work when some return values have multi-line descriptions
        for i in which_results:
            index_from_back = -len(self.outputs) + i
            del docstring_lines[index_from_back]

    # Add all the jacobians
    input_args = [self.inputs[arg] for arg in which_args]

    all_outputs = list(self.outputs.items())
    all_jacobian_names = []
    for i in which_results:
        result_name, result = all_outputs[i]

        arg_jacobians = jacobian_helpers.tangent_jacobians(result, input_args)

        for arg_name, arg, arg_jacobian in zip(which_args, input_args, arg_jacobians):
            jacobian_name = f"{result_name}_D_{arg_name}"
            outputs[jacobian_name] = arg_jacobian * ops.LieGroupOps.tangent_D_storage(arg)
            all_jacobian_names.append(jacobian_name)

            result_dim = ops.LieGroupOps.tangent_dim(result)
            arg_dim = ops.LieGroupOps.storage_dim(arg)
            docstring_lines.append(
                f"    {jacobian_name}: ({result_dim}x{arg_dim}) jacobian (result_dim x storage_dim) of "
                + f"{result_name} ({result_dim}) wrt arg {arg_name} ({arg_dim}) (row-major)"
            )

    if len(outputs) == 1:
        # If just computing a single jacobian and nothing else, return it instead of output arg
        return_key: T.Optional[str] = list(outputs.keys())[0]
    elif self.return_key is not None and self.return_key in outputs:
        # If still computing the original return value, return that
        return_key = self.return_key
    else:
        return_key = None

    # Cutely pick a function name if not given
    if not name:
        name = self._pick_name_for_function_with_derivatives(
            which_args, include_results, linearization_mode=None
        )

    sparse_matrices = all_jacobian_names if sparse_jacobians else None
    return Codegen(
        name=name,
        inputs=self.inputs,
        outputs=outputs,
        config=self.config,
        return_key=return_key,
        sparse_matrices=sparse_matrices,
        docstring="\n".join(docstring_lines),
    )
