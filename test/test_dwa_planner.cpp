// #include <gtest/gtest.h>

// TEST(BasicTest, TrueIsTrue) {
//   EXPECT_TRUE(true);
// }

// int main(int argc, char **argv) {
//   ::testing::InitGoogleTest(&argc, argv);
//   return RUN_ALL_TESTS();
// }
#include <gtest/gtest.h>
#include "dwa_planner/dwa_planner.h"
#include "rclcpp/rclcpp.hpp"

class DWAPlannerTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override {
    rclcpp::shutdown();
  }
};

TEST(BasicTest, TrueIsTrue) {
  EXPECT_TRUE(true);
}

TEST_F(DWAPlannerTest, ConstructorInitializesNode) {
  auto node = std::make_shared<DWAPlanner>(rclcpp::NodeOptions());
  ASSERT_NE(node, nullptr);
  EXPECT_EQ(node->get_name(), std::string("dwa_planner_node"));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}