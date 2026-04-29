// Copyright (c) 2026 Davide Faconti
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Regression test for the fast-cancel race between
// `RosActionNode::cancelGoal()` and the `rclcpp_action` client's result
// callback.
//
// Background: `cancelGoal()` calls `async_get_result()` and
// `async_cancel_goal()` on the action client. If the server produces a
// terminal result (including one accepted via cancel) before those calls
// run on the BT thread, the client's result-callback has already erased
// the goal from its internal registry and both calls throw
// `rclcpp_action::exceptions::UnknownGoalHandleError`. The exception used
// to propagate out of `halt()` and cause `TreeExecutionServer` to abort
// the outer goal instead of canceling it.

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "btcpp_ros2_interfaces/action/sleep.hpp"

using namespace std::chrono_literals;
using Sleep = btcpp_ros2_interfaces::action::Sleep;

namespace
{

class SleepLeaf : public BT::RosActionNode<Sleep>
{
public:
  SleepLeaf(const std::string& name, const BT::NodeConfig& conf,
            const BT::RosNodeParams& params)
    : BT::RosActionNode<Sleep>(name, conf, params)
  {}

  bool setGoal(Goal& goal) override
  {
    goal.msec_timeout = 10000;  // deliberately long: server must not finish normally
    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult& /*wr*/) override
  {
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> /*fb*/) override
  {
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode /*err*/) override
  {
    return BT::NodeStatus::FAILURE;
  }
};

class FastCancelRaceTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if(!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
  }
  static void TearDownTestSuite()
  {
    if(rclcpp::ok())
    {
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    bt_node_ = std::make_shared<rclcpp::Node>("bt_cancel_race_test");
    server_node_ = std::make_shared<rclcpp::Node>("server_cancel_race_test");

    using GoalHandle = rclcpp_action::ServerGoalHandle<Sleep>;

    action_server_ = rclcpp_action::create_server<Sleep>(
        server_node_, "/sleep",
        [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const Sleep::Goal>) {
          goal_received_ = true;
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](const std::shared_ptr<GoalHandle>) {
          cancel_received_ = true;
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<GoalHandle> goal_handle) {
          // Detach a thread that waits up to ~50 ms for a cancel request,
          // then completes the goal synchronously once it sees one. This
          // is the shape of the race: the server's `canceled()` call on
          // the client side fires the result callback, which erases the
          // goal from `goal_handles_` on the rclcpp_action client
          // registry. If `halt()` -> `cancelGoal()` reaches
          // `async_get_result` / `async_cancel_goal` after that erase,
          // both calls used to throw `UnknownGoalHandleError`.
          std::thread([goal_handle, this]() {
            for(int i = 0; i < 50 && !cancel_received_.load(); ++i)
            {
              std::this_thread::sleep_for(5ms);
            }
            auto result = std::make_shared<Sleep::Result>();
            result->done = false;
            if(cancel_received_.load())
            {
              goal_handle->canceled(result);
            }
            else
            {
              result->done = true;
              goal_handle->succeed(result);
            }
          }).detach();
        });

    exec_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(
        rclcpp::ExecutorOptions(), 2, false, std::chrono::milliseconds(50));
    exec_->add_node(bt_node_);
    exec_->add_node(server_node_);
    spinner_ = std::thread([this]() { exec_->spin(); });
  }

  void TearDown() override
  {
    if(exec_)
    {
      exec_->cancel();
    }
    if(spinner_.joinable())
    {
      spinner_.join();
    }
    exec_.reset();
  }

  BT::BehaviorTreeFactory makeFactory()
  {
    BT::RosNodeParams params;
    params.nh = bt_node_;
    params.server_timeout = 1000ms;
    params.wait_for_server_timeout = 2000ms;
    params.default_port_value = "/sleep";

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<SleepLeaf>("SleepLeaf", params);
    return factory;
  }

  std::shared_ptr<rclcpp::Node> bt_node_;
  std::shared_ptr<rclcpp::Node> server_node_;
  rclcpp_action::Server<Sleep>::SharedPtr action_server_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> exec_;
  std::thread spinner_;

  std::atomic<bool> goal_received_{ false };
  std::atomic<bool> cancel_received_{ false };
};

}  // namespace

// Regression test for the UnknownGoalHandleError fast-cancel race.
// `tree.haltTree()` must not throw when the server completes the cancel
// handshake synchronously from its own thread while the BT thread is
// still reaching the `cancelGoal()` body.
TEST_F(FastCancelRaceTest, HaltDoesNotThrowOnFastCancel)
{
  auto factory = makeFactory();
  auto tree = factory.createTreeFromText(R"(<root BTCPP_format="4">
  <BehaviorTree ID="T">
    <SleepLeaf/>
  </BehaviorTree>
</root>)");

  // Tick until the server has accepted the goal. This ensures
  // `goal_handle_` is populated inside the BT leaf before we halt.
  tree.tickOnce();
  for(int i = 0; i < 100 && !goal_received_.load(); ++i)
  {
    std::this_thread::sleep_for(5ms);
    tree.tickOnce();
  }
  ASSERT_TRUE(goal_received_.load()) << "Server never received the goal; precondition "
                                        "for the race "
                                        "reproducer was not met.";

  // The halt must not leak `UnknownGoalHandleError` (or any other
  // exception). Before the fix, this throws with
  // "Goal handle is not known to this client.".
  EXPECT_NO_THROW(tree.haltTree());

  // The server must have actually received the cancel request — otherwise
  // we'd be testing the non-racing path.
  EXPECT_TRUE(cancel_received_.load());
}
