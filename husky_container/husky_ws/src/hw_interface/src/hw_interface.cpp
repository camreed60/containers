#include "hw_interface/hw_interface.hpp"

#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>
#include <chrono>
#include <thread>
#include <csignal>

#define THREAD_ID_TO_C_STR \
  boost::lexical_cast<std::string>(boost::this_thread::get_id()).c_str()

hw_interface::hw_interface(rclcpp::Node::SharedPtr node)
: node_(std::move(node))
, pluginLoader_("hw_interface", "base_classes::base_interface")
{
  auto & logger = node_->get_logger();
  RCLCPP_DEBUG(logger, "Creating ASIO Services");
  interfaceService_ = std::make_shared<boost::asio::io_context>();
  RCLCPP_DEBUG(logger, "Generating Work");
  interfaceWork_    = std::make_shared<boost::asio::io_context::work>(*interfaceService_);

  RCLCPP_DEBUG(logger, "Loading Plugins");
  addInterfacePlugins();

  RCLCPP_DEBUG(logger, "Starting Thread Pool");
  initThreadPool();
  
  RCLCPP_DEBUG(logger, "Starting Interfaces");
  startInterfaces();
}

hw_interface::~hw_interface()
{
  RCLCPP_INFO(node_->get_logger(), "HW Interfaces are stopping.");
  // Shutdown ASIO
  interfaceService_->stop();
  interfaceWorkerGroup_.join_all();

  // Tear down plugins
  for (auto it = interfacePluginVector_.begin(); it != interfacePluginVector_.end(); /* no increment */) {
    RCLCPP_INFO(node_->get_logger(),
      "Destroying Plugin: %s", (*it)->pluginName.c_str());
    (*it)->stopWork();
    it = interfacePluginVector_.erase(it);
  }
}

bool hw_interface::initPlugin(
  const boost::shared_ptr<base_classes::base_interface> & pluginPtr,
  const std::string & pluginName)
{
  pluginPtr->pluginName = pluginName;
  auto & logger = node_->get_logger();
  try {
    RCLCPP_INFO(logger, "Initializing Plugin: %s", pluginName.c_str());
    pluginPtr->initPlugin(node_, interfaceService_);
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(logger, "Pluginlib Exception: %s", ex.what());
    pluginPtr->enabled = false;
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(logger, "STD Exception: %s", ex.what());
    pluginPtr->enabled = false;
  }
  return pluginPtr->enabled;
}

void hw_interface::addInterfacePlugins()
{
  auto & logger = node_->get_logger();
  RCLCPP_DEBUG(logger, "Looking for plugins on parameter 'plugin_names'");
  std::map<std::string,std::string> pluginMap;
  if (!node_->get_parameter("plugin_names", pluginMap)) {
    RCLCPP_ERROR(logger, "No 'plugin_names' parameter found!");
    return;
  }

  for (auto & [classKey, pkgName] : pluginMap) {
    std::string fullClass = pkgName + "::" + classKey;
    RCLCPP_INFO(logger, "Found Plugin Class %s", fullClass.c_str());
    try {
      std::map<std::string,std::string> instanceMap;
      if (node_->get_parameter(classKey, instanceMap)) {
        for (auto & [paramKey, instanceName] : instanceMap) {
          auto plugin = pluginLoader_.createSharedInstance(fullClass);
          if (!plugin || !initPlugin(plugin, instanceName)) {
            RCLCPP_ERROR(logger, "Failed to instantiate %s", fullClass.c_str());
          } else {
            interfacePluginVector_.push_back(plugin);
            RCLCPP_INFO(logger, "Instantiated Plugin: %s", instanceName.c_str());
          }
        }
      } else {
        RCLCPP_ERROR(logger, "No parameter map for '%s'", classKey.c_str());
      }
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(logger, "Plugin load error: %s", ex.what());
    }
  }
}

void hw_interface::initThreadPool()
{
  auto & logger = node_->get_logger();
  RCLCPP_INFO(logger, "Starting Thread Pool with %zu threads",
               interfacePluginVector_.size() * NUM_THREADS_PER_PLUGIN);
  for (size_t i = 0;
       i < interfacePluginVector_.size() * NUM_THREADS_PER_PLUGIN;
       ++i)
  {
    interfaceWorkerGroup_.create_thread(
      boost::bind(&interface_worker::worker, interfaceService_));
  }
}

bool hw_interface::startInterfaces()
{
  auto & logger = node_->get_logger();
  for (auto & plugin : interfacePluginVector_) {
    RCLCPP_INFO(logger, "Starting Plugin Work: %s", plugin->pluginName.c_str());
    plugin->startWork();
  }
  return true;
}

bool hw_interface::stopInterfaces()
{
  auto & logger = node_->get_logger();
  for (auto & plugin : interfacePluginVector_) {
    RCLCPP_INFO(logger, "Stopping Plugin Work: %s", plugin->pluginName.c_str());
    plugin->stopWork();
  }
  return true;
}

void interface_worker::worker(
  boost::shared_ptr<boost::asio::io_context> ioService)
{
  auto logger = rclcpp::get_logger("interface_worker");
  RCLCPP_DEBUG(logger, "Thread <%s>:: Started Worker", THREAD_ID_TO_C_STR);

  while (!ioService->stopped()) {
    try {
      ioService->run();
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(logger, "Exception in IO worker: %s", ex.what());
    }
    RCLCPP_WARN(logger, "Thread <%s>:: Waiting for Work", THREAD_ID_TO_C_STR);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  RCLCPP_INFO(logger, "Thread <%s>:: Stopping Worker", THREAD_ID_TO_C_STR);
}
