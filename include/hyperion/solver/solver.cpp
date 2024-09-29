/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include "hyperion/solver/solver.hpp"

#include <fmt/format.h>
#include <glog/logging.h>

#include <boost/asio.hpp>

namespace hyperion {

using ThreadPool = boost::asio::thread_pool;

namespace {

template <typename TContainer, typename TIterator = typename TContainer::const_iterator>
auto SplitRange(TContainer& container, std::size_t num_chunks) -> std::vector<std::pair<TIterator, TIterator>> {
  using Ranges = std::vector<std::pair<TIterator, TIterator>>;

  const auto size = static_cast<std::size_t>(container.size());
  const auto length = size / num_chunks;
  auto remain = size % num_chunks;

  Ranges ranges;
  ranges.reserve(num_chunks);
  auto begin = container.begin();
  auto end = container.begin();

  for (std::size_t i = 0; i < std::min(num_chunks, size); ++i) {
    const auto step = length + (remain > 0 ? (--remain, 1) : 0);
    std::advance(end, step);
    ranges.emplace_back(begin, end);
    begin = end;
  }

  return ranges;
}

template <typename TFunctor, typename... TArgs>
auto postTask(ThreadPool& thread_pool, TFunctor functor, TArgs&&... args) {
  using Result = std::invoke_result_t<TFunctor, TArgs&&...>;
  auto promise = std::make_unique<std::promise<Result>>();
  auto result = promise->get_future();
  boost::asio::post(
      thread_pool,
      [promise = std::move(promise),
       f = std::forward<TFunctor>(functor),
       args = std::make_tuple(std::forward<TArgs>(args)...)] {
        if constexpr (!std::is_same_v<Result, void>) {
          promise->set_value(std::apply(f, args));
        } else {
          std::apply(f, args);
        }
      });
  return result;
}

template <typename TFunctor, typename TRange, typename... TArgs>
auto applyInParallelAndWait(
    ThreadPool& thread_pool,
    TFunctor functor,
    const std::vector<TRange>& ranges,
    TArgs&&... args) {
  using Result = std::invoke_result_t<TFunctor, TRange, TArgs&&...>;
  using Futures = std::vector<std::future<Result>>;
  using Results = std::vector<Result>;

  Futures futures;
  const auto num_ranges = ranges.size();
  futures.reserve(num_ranges);
  for (const auto& range : ranges) {
    futures.emplace_back(postTask(thread_pool, functor, range, std::forward<TArgs>(args)...));
  }

  if constexpr (!std::is_same_v<Result, void>) {
    Results results;
    results.reserve(num_ranges);
    for (auto& future : futures) {
      results.emplace_back(future.get());
    }
    return results;
  } else {
    for (auto& future : futures) {
      future.wait();
    }
  }
}

struct NodeLoadReturnType {
  /// Plus operator (in-place).
  /// @param rhs Right-hand side.
  /// @return Sum of this and rhs (in-place).
  auto operator+=(const NodeLoadReturnType& rhs) -> NodeLoadReturnType& {
    num_parameter_blocks += rhs.num_parameter_blocks;
    num_parameters += rhs.num_parameters;
    num_effective_parameters += rhs.num_effective_parameters;
    num_parameter_blocks_reduced += rhs.num_parameter_blocks_reduced;
    num_parameters_reduced += rhs.num_parameters_reduced;
    num_effective_parameters_reduced += rhs.num_effective_parameters_reduced;
    return *this;
  }

  int num_parameter_blocks{0};
  int num_parameters{0};
  int num_effective_parameters{0};
  int num_parameter_blocks_reduced{0};
  int num_parameters_reduced{0};
  int num_effective_parameters_reduced{0};
};

template <typename TRange>
auto loadNodes(const TRange& range, const SolverOptions& options) -> NodeLoadReturnType {
  NodeLoadReturnType result;
  const auto& [begin, end] = range;
  for (auto itr = begin; itr != end; ++itr) {
    const auto& node = Graph::AccessNode(itr);
    const auto& [value, covariance] = Graph::AccessValueWithCovariance(itr);
    const auto& [ambient_dim, tangent_dim] = node->dims();
    const auto& is_variable = !node->isConstant();
    result.num_parameter_blocks += 1;
    result.num_parameters += ambient_dim;
    result.num_effective_parameters += tangent_dim;
    result.num_parameter_blocks_reduced += is_variable;
    result.num_parameters_reduced += is_variable * ambient_dim;
    result.num_effective_parameters_reduced += is_variable * tangent_dim;
    node->load(value, covariance, options.parameter_tolerance);
  }
  return result;
}

template <typename TRange>
auto storeNodes(const TRange& range) -> void {
  const auto& [begin, end] = range;
  for (auto itr = begin; itr != end; ++itr) {
    const auto& [value, covariance] = Graph::AccessValueWithCovariance(itr);
    Graph::AccessNode(itr)->store(value, covariance);
  }
}

struct FactorLoadReturnType {
  /// Plus operator (in-place).
  /// @param rhs Right-hand side.
  /// @return Sum of this and rhs (in-place).
  auto operator+=(const FactorLoadReturnType& rhs) -> FactorLoadReturnType& {
    num_residual_blocks += rhs.num_residual_blocks;
    num_residuals += rhs.num_residuals;
    num_residual_blocks_reduced += rhs.num_residual_blocks_reduced;
    num_residuals_reduced += rhs.num_residuals_reduced;
    fixed_cost += rhs.fixed_cost;
    initial_cost += rhs.initial_cost;
    return *this;
  }

  int num_residual_blocks{0};
  int num_residuals{0};
  int num_residual_blocks_reduced{0};
  int num_residuals_reduced{0};

  Scalar fixed_cost{0};
  Scalar initial_cost{0};
};

template <typename TRange>
auto loadFactors(const TRange& range, const SolverOptions& options) -> FactorLoadReturnType {
  FactorLoadReturnType result;
  const auto& [begin, end] = range;
  for (auto itr = begin; itr != end; ++itr) {
    const auto& initial_cost = (*itr)->load(options.parameter_tolerance, options.factor_step_size);
    const auto& is_constant = (*itr)->isConstant();
    const auto& num_residuals = (*itr)->numResiduals();
    const auto is_variable = !is_constant;
    result.num_residual_blocks += 1;
    result.num_residuals += num_residuals;
    result.num_residual_blocks_reduced += is_variable;
    result.num_residuals_reduced += is_variable * num_residuals;
    result.fixed_cost += is_constant * initial_cost;
    result.initial_cost += initial_cost;
  }
  return result;
}

template <typename TRange>
auto storeFactors(const TRange& range) -> Scalar {
  Scalar final_cost = 0;
  const auto& [begin, end] = range;
  for (auto itr = begin; itr != end; ++itr) {
    final_cost += (*itr)->energy();
  }
  return final_cost;
}

template <typename TNodeRange, typename TFactorRange>
auto loadGraph(
    ThreadPool& thread_pool,
    const std::vector<TNodeRange>& node_ranges,
    const std::vector<TFactorRange>& factor_ranges,
    const SolverOptions& options,
    SolverSummary& summary) -> bool {
  // Load the nodes.
  NodeLoadReturnType node_load_return;
  for (const auto& return_i : applyInParallelAndWait(thread_pool, loadNodes<TNodeRange>, node_ranges, options)) {
    node_load_return += return_i;
  }
  summary.num_parameter_blocks = node_load_return.num_parameter_blocks;
  summary.num_parameters = node_load_return.num_parameters;
  summary.num_effective_parameters = node_load_return.num_effective_parameters;
  summary.num_parameter_blocks_reduced = node_load_return.num_parameter_blocks_reduced;
  summary.num_parameters_reduced = node_load_return.num_parameters_reduced;
  summary.num_effective_parameters_reduced = node_load_return.num_effective_parameters_reduced;

  // Load the factors.
  FactorLoadReturnType factor_load_return;
  for (const auto& return_i : applyInParallelAndWait(thread_pool, loadFactors<TFactorRange>, factor_ranges, options)) {
    factor_load_return += return_i;
  }
  summary.num_residual_blocks = factor_load_return.num_residual_blocks;
  summary.num_residuals = factor_load_return.num_residuals;
  summary.num_residual_blocks_reduced = factor_load_return.num_residual_blocks_reduced;
  summary.num_residuals_reduced = factor_load_return.num_residuals_reduced;
  summary.fixed_cost = factor_load_return.fixed_cost;
  summary.initial_cost = factor_load_return.initial_cost;
  return true;
}

template <typename TNodeRange, typename TFactorRange>
auto storeGraph(
    ThreadPool& thread_pool,
    const std::vector<TNodeRange>& node_ranges,
    const std::vector<TFactorRange>& factor_ranges,
    SolverSummary& summary) -> bool {
  // Store the nodes.
  applyInParallelAndWait(thread_pool, storeNodes<TNodeRange>, node_ranges);

  // Evaluate the  the factors.
  Scalar final_cost = 0;
  for (const auto& result : applyInParallelAndWait(thread_pool, storeFactors<TFactorRange>, factor_ranges)) {
    final_cost += result;
  }

  summary.final_cost = final_cost;
  return true;
}

template <typename TRange>
auto evaluateNodes(const TRange& range, const Scalar& tolerance, const Scalar& step_size) -> int {
  auto num_node_updates = 0;
  const auto& [begin, end] = range;
  for (auto itr = begin; itr != end; ++itr) {
    num_node_updates += Graph::AccessNode(itr)->maybeEvaluate(tolerance, step_size);
  }
  return num_node_updates;
}

template <typename TRange>
auto evaluateFactors(const TRange& range, const Scalar& tolerance, const Scalar& step_size) -> std::tuple<Scalar, int> {
  Scalar cost = 0;
  auto num_factor_updates = 0;
  const auto& [begin, end] = range;
  for (auto itr = begin; itr != end; ++itr) {
    const auto& [updated_i, cost_i] = (*itr)->maybeEvaluate(tolerance, step_size);
    num_factor_updates += updated_i;
    cost += cost_i;
  }
  return {cost, num_factor_updates};
}

template <typename TNodeRange, typename TFactorRange>
auto evaluateIteration(
    ThreadPool& thread_pool,
    const std::vector<TNodeRange>& node_ranges,
    const std::vector<TFactorRange>& factor_ranges,
    const SolverOptions& options,
    SolverSummary& summary) -> bool {
  // Get initial cost.
  const auto initial_cost = summary.final_cost;

  // Evaluate the nodes.
  auto num_node_updates = 0;
  for (const auto& result : applyInParallelAndWait(
           thread_pool,
           evaluateNodes<TNodeRange>,
           node_ranges,
           options.parameter_tolerance,
           options.node_step_size)) {
    num_node_updates += result;
  }

  // Evaluate the factors.
  Scalar iteration_cost = 0;
  auto num_factor_updates = 0;
  for (const auto& result : applyInParallelAndWait(
           thread_pool,
           evaluateFactors<TFactorRange>,
           factor_ranges,
           options.parameter_tolerance,
           options.factor_step_size)) {
    const auto& [cost_i, updated_i] = result;
    num_factor_updates += updated_i;
    iteration_cost += cost_i;
  }

  const auto cost_change = initial_cost - iteration_cost;
  cost_change > 0 ? ++summary.num_successful_iterations : ++summary.num_unsuccessful_iterations;
  summary.final_cost = iteration_cost;

  // Maybe print progress.
  if (options.minimizer_progress_to_stdout) {
    LOG(INFO) << " Iterations: "
              << " Initial cost: " << summary.initial_cost << " Final cost: " << summary.final_cost;
  }

  // Maybe store the graph.
  if (options.update_state_every_iteration) {
    storeGraph(thread_pool, node_ranges, factor_ranges, summary);
  }

  // Return if no updates are detected.
  if (!num_node_updates && !num_factor_updates) {
    summary.termination_type = TerminationType::CONVERGENCE;
    summary.message = "Parameter tolerance reached. No updates detected.";
    return true;
  }

  // Return if function tolerance is reached.
  if (std::abs(cost_change) < options.function_tolerance * initial_cost) {
    summary.termination_type = TerminationType::CONVERGENCE;
    summary.message = fmt::format(
        "Function tolerance reached. |cost_change|/cost: {:.6e} <= {:.6e}",
        std::abs(cost_change) / initial_cost,
        options.function_tolerance);
    return true;
  }

  return false;
}

}  // namespace

auto Solver::Solve(const Graph& graph, const SolverOptions& options, SolverSummary& summary) -> void {
  // Preprocessing.
  const auto start_time = std::chrono::high_resolution_clock::now();

  // Check options.
  if (!options.areValid(summary.message)) {
    LOG(ERROR) << "Terminating: " << summary.message;
    return;
  }

  // Load the graph.
  ThreadPool thread_pool(options.num_threads);
  summary.optimizer_type = options.optimizer_type;
  summary.num_threads_given = options.num_threads;
  summary.num_threads_used = options.num_threads;
  summary.strategy_type = options.strategy_type;
  summary.update_type = options.update_type;
  const auto node_ranges = SplitRange(graph.nodes(), options.num_threads);
  const auto factor_ranges = SplitRange(graph.factors(), options.num_threads);
  const auto status = loadGraph(thread_pool, node_ranges, factor_ranges, options, summary);
  const auto preprocessor_time = std::chrono::high_resolution_clock::now();

  // Optimization.
  if (status) {
    // Setup.
    int iteration = 0;
    auto old_optimizer_time_i = std::chrono::high_resolution_clock::now();

    // Loop.
    while (true) {
      // Break on maximum number of iterations reached.
      if (options.max_num_iterations <= iteration) {
        summary.termination_type = TerminationType::NO_CONVERGENCE;
        summary.message =
            fmt::format("Maximum number of iterations reached. Number of iterations: {}.", iteration);
        break;
      }

      // Iterate.
      const auto converged = evaluateIteration(thread_pool, node_ranges, factor_ranges, options, summary);
      const auto new_optimizer_time_i = std::chrono::high_resolution_clock::now();
      const auto iteration_time_i = std::chrono::duration<Scalar>{new_optimizer_time_i - old_optimizer_time_i}.count();
      const auto cumulative_time_i = std::chrono::duration<Scalar>{new_optimizer_time_i - preprocessor_time}.count();

      // Evaluate max and min time per iteration.
      if (iteration == 0) {
        summary.max_iteration_time_in_seconds = iteration_time_i;
        summary.min_iteration_time_in_seconds = iteration_time_i;
      } else {
        summary.max_iteration_time_in_seconds = std::max(summary.max_iteration_time_in_seconds, iteration_time_i);
        summary.min_iteration_time_in_seconds = std::min(summary.min_iteration_time_in_seconds, iteration_time_i);
      }

      // Evaluate mean time per iteration.
      const auto num_iterations = summary.num_successful_iterations + summary.num_unsuccessful_iterations;
      summary.mean_iteration_time_in_seconds = cumulative_time_i / num_iterations;

      // Break on convergence.
      if (converged) {
        break;
      }

      // Break on timeout.
      if (options.max_optimizer_time_in_seconds < cumulative_time_i) {
        summary.termination_type = TerminationType::NO_CONVERGENCE;
        summary.message = "Solver timed out.";
        break;
      }

      old_optimizer_time_i = new_optimizer_time_i;
      ++iteration;
    }
  }

  // Postprocessing.
  const auto optimizer_time = std::chrono::high_resolution_clock::now();
  storeGraph(thread_pool, node_ranges, factor_ranges, summary);
  const auto total_time = std::chrono::high_resolution_clock::now();

  // Evaluate times.
  summary.preprocessor_time_in_seconds = std::chrono::duration<Scalar>{preprocessor_time - start_time}.count();
  summary.optimizer_time_in_seconds = std::chrono::duration<Scalar>{optimizer_time - preprocessor_time}.count();
  summary.postprocessor_time_in_seconds = std::chrono::duration<Scalar>{total_time - optimizer_time}.count();
  summary.total_time_in_seconds = std::chrono::duration<Scalar>{total_time - start_time}.count();

  LOG(INFO) << summary.fullReport();
}

auto Solver::Solve(const Graph& graph, const SolverOptions& options) -> SolverSummary {
  SolverSummary summary;
  Solve(graph, options, summary);
  return summary;
}

}  // namespace hyperion
