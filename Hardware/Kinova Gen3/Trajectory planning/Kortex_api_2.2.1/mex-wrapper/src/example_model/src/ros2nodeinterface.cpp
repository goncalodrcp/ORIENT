// Copyright 2019 The MathWorks, Inc.
// Generated 13-Apr-2021 16:36:26
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4244)
#pragma warning(disable : 4265)
#pragma warning(disable : 4458)
#pragma warning(disable : 4100)
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#pragma GCC diagnostic ignored "-Wredundant-decls"
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wshadow"
#endif //_MSC_VER
#include "rclcpp/rclcpp.hpp"
#include "example_model.h"
#include "ros2nodeinterface.h"
#include <thread>
#include <chrono>
#include <utility>
const std::string SLROSNodeName("example_model");
namespace ros2 {
namespace matlab {
NodeInterface::NodeInterface()
    : mNode()
    , mModel()
    , mExec()
    , mBaseRateSem()
    , mBaseRateThread()
    , mSchedulerThread()
    , mStopSem()
    , mRunModel(true){
  }
NodeInterface::~NodeInterface() {
    terminate();
  }
void NodeInterface::initialize(int argc, char * const argv[]) {
    try {
        //initialize ros2
        // Workaround to disable /rosout topic until rmw_fastrtps supports
        // multiple typesupport implementations for the same topic (https://github.com/ros2/rmw_fastrtps/issues/265)
        std::vector<char *> args(argv, argv + argc);
        char log_disable_rosout[] = "__log_disable_rosout:=true";
        args.push_back(log_disable_rosout);
        rclcpp::init(static_cast<int>(args.size()), args.data());
        //create the Node specified in Model
        std::string NodeName("example_model");
        mNode = std::make_shared<rclcpp::Node>(NodeName);
        mExec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        mExec->add_node(mNode);
        //initialize the model which will initialize the publishers and subscribers
        mModel = std::make_shared<example_modelModelClass>();
		rtmSetErrorStatus(mModel->getRTM(), (NULL));
        mModel->initialize();
        //create the threads for the rates in the Model
        mBaseRateThread = std::make_shared<std::thread>(&NodeInterface::baseRateTask, this);
		mSchedulerThread = std::make_shared<std::thread>(&NodeInterface::schedulerThreadCallback, this);
    }
    catch (std::exception& ex) {
        std::cout << ex.what() << std::endl;
        throw ex;
    }
    catch (...) {
        std::cout << "Unknown exception" << std::endl;
        throw;
    }
}
int NodeInterface::run() {
  if (mExec) {
    mExec->spin();
  }
  mRunModel = false;
  return 0;
}
boolean_T NodeInterface::getStopRequestedFlag(void) {
    #ifndef rtmGetStopRequested
    return (!(rtmGetErrorStatus(mModel->getRTM())
        == (NULL)));
    #else
    return (!(rtmGetErrorStatus(mModel->getRTM())
        == (NULL)) || rtmGetStopRequested(mModel->getRTM()));
    #endif
}
void NodeInterface::stop(void) {
  if (mExec.get()) {
    mExec->cancel();
    if (mNode) {
      mExec->remove_node(mNode);
    }
    while (mExec.use_count() > 1);
  }
}
void NodeInterface::terminate(void) {
    if (mBaseRateThread.get()) {
        mRunModel = false;
        mBaseRateSem.notify(); // break out wait
        mBaseRateThread->join();
        if (mSchedulerThread.get()) {
            mSchedulerThread->join();
            mSchedulerThread.reset();
        }
        mBaseRateThread.reset();
        if (mModel.get()) {
            mModel->terminate();
        }
        mModel.reset();
        mExec.reset();
        mNode.reset();
        rclcpp::shutdown();
    }
}
//
void NodeInterface::schedulerThreadCallback(void)
{
  while (mRunModel) {
        std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::nanoseconds(std::chrono::nanoseconds(2000000000)));
        mBaseRateSem.notify();
    }
}
//Model specific
void NodeInterface::baseRateTask(void) {
  mRunModel = (rtmGetErrorStatus(mModel->getRTM()) ==
              (NULL));
  while (mRunModel) {
    mBaseRateSem.wait();
    if (!mRunModel) break;
    mModel->step();
    mRunModel &= !NodeInterface::getStopRequestedFlag(); //If RunModel and not stop requested
  }
  NodeInterface::stop();
}
}//namespace matlab
}//namespace ros2
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
