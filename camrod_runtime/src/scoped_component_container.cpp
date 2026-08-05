// HH_260805 - Finalize composed nodes and their DDS context before shared-library teardown.

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <map>
#include <memory>
#include <string>

#include <class_loader/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/component_manager.hpp>

namespace camrod_runtime
{

class ScopedContextComponentManager : public rclcpp_components::ComponentManager
{
public:
  using PluginLoaderMap =
    std::map<std::string, std::unique_ptr<class_loader::ClassLoader>>;

  ScopedContextComponentManager(
    const std::weak_ptr<rclcpp::Executor> & executor,
    const rclcpp::NodeOptions & options,
    const std::weak_ptr<rclcpp::Context> & context)
  : rclcpp_components::ComponentManager(executor, "ComponentManager", options),
    context_(context)
  {
  }

  PluginLoaderMap detach_components_and_release_loaders()
  {
    // HH_260805 - ComponentManager owns the node instances, but the executor
    // also retains their callback groups in its wait set. Detach every node
    // before destroying either the instance or its plugin library so shutdown
    // cannot execute through an unloaded component vtable.
    for (const auto & entry : node_wrappers_) {
      try {
        remove_node_from_executor(entry.first);
      } catch (const std::exception & error) {
        RCLCPP_WARN(
          get_logger(), "component %lu detach during shutdown failed: %s",
          static_cast<unsigned long>(entry.first), error.what());
      }
    }
    node_wrappers_.clear();
    return std::move(loaders_);
  }

protected:
  rclcpp::NodeOptions create_node_options(
    const std::shared_ptr<LoadNode::Request> request) override
  {
    auto context = context_.lock();
    if (!context) {
      throw rclcpp_components::ComponentManagerException(
              "component container context expired during node load");
    }
    return rclcpp_components::ComponentManager::create_node_options(request)
           .context(std::move(context));
  }

private:
  std::weak_ptr<rclcpp::Context> context_;
};

template<typename ExecutorT>
int run_container(int argc, char * argv[])
{
  int exit_code = 0;
  auto context = std::make_shared<rclcpp::Context>();
  context->init(argc, argv);
  const bool installed_signal_handlers = rclcpp::install_signal_handlers();

  rclcpp::ExecutorOptions executor_options;
  executor_options.context = context;

  typename ExecutorT::SharedPtr executor;
  std::shared_ptr<ScopedContextComponentManager> manager;
  ScopedContextComponentManager::PluginLoaderMap plugin_loaders;

  try {
    auto manager_options = rclcpp::NodeOptions()
      .context(context)
      .start_parameter_services(false)
      .start_parameter_event_publisher(false);

#ifdef CAMROD_RUNTIME_MULTITHREADED_EXECUTOR
    manager = std::make_shared<ScopedContextComponentManager>(
      std::weak_ptr<rclcpp::Executor>(), manager_options, context);
    const auto thread_count = static_cast<size_t>(
      manager->get_parameter("thread_num").as_int());
    executor = std::make_shared<ExecutorT>(executor_options, thread_count);
    manager->set_executor(executor);
#else
    executor = std::make_shared<ExecutorT>(executor_options);
    manager = std::make_shared<ScopedContextComponentManager>(
      executor, manager_options, context);
#endif

    executor->add_node(manager);
    executor->spin();
  } catch (const std::exception & error) {
    exit_code = 1;
    RCLCPP_ERROR(
      rclcpp::get_logger("camrod_runtime.scoped_component_container"),
      "component container failed: %s", error.what());
  }

  // The deferred signal thread runs lifecycle pre-shutdown callbacks. Join it
  // before component destruction so Nav2 cleanup cannot race manager teardown.
  if (installed_signal_handlers) {
    rclcpp::uninstall_signal_handlers();
    // A terminal SIGINT reaches the launch process group first; launch then
    // forwards SIGINT to children. Ignore that second delivery after the ROS
    // deferred handler has completed, otherwise the default action reports -2.
    std::signal(SIGINT, SIG_IGN);
    std::signal(SIGTERM, SIG_IGN);
  }

  if (executor && manager) {
    executor->cancel();
    plugin_loaders = manager->detach_components_and_release_loaders();
    try {
      executor->remove_node(manager);
    } catch (const std::exception & error) {
      RCLCPP_WARN(
        manager->get_logger(), "manager removal during shutdown failed: %s", error.what());
    }
  }
  manager.reset();
  executor.reset();

  if (context->is_valid()) {
    context->shutdown("scoped component container stopped");
  }
  // HH_260805 - ExecutorOptions keeps its own shared Context reference after
  // the executor is gone. Release every reference owned by this container;
  // Humble entities may still defer final Context destruction to process exit.
  executor_options.context.reset();
  context.reset();
  // HH_260805 - Keep plugin code mapped through all explicit node, executor,
  // and Context cleanup attempts. main() then avoids the remaining Humble DSO
  // static-destructor phase where delayed DDS workers previously faulted.
  plugin_loaders.clear();
  return exit_code;
}

}  // namespace camrod_runtime

int main(int argc, char * argv[])
{
  int exit_code = 0;
#ifdef CAMROD_RUNTIME_MULTITHREADED_EXECUTOR
  exit_code =
    camrod_runtime::run_container<rclcpp::executors::MultiThreadedExecutor>(argc, argv);
#else
  exit_code =
    camrod_runtime::run_container<rclcpp::executors::SingleThreadedExecutor>(argc, argv);
#endif
  // HH_260805 - ROS 2 Humble entities can retain Context references after the
  // explicit node/executor teardown above. Returning from main then runs DSO
  // static destructors while CycloneDDS workers still reference RMW code.
  // All owned runtime resources are already finalized, so bypass only that
  // unsafe process-exit destructor phase; the kernel reclaims remaining state.
  std::fflush(nullptr);
  std::_Exit(exit_code);
}
