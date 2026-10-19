#include <ackermann_car_gazebo/coverage_path_planner.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace ackermann_car_gazebo
{
namespace
{

PlanOptions defaultOptions()
{
  PlanOptions options;
  options.scan_width = 1.0;
  options.coverage_ratio = 0.8;
  options.turn_radius = 0.5;
  options.turn_step = 0.1;
  options.straight_step = 0.8;
  options.start_connection_weight = 1.0;
  options.exact_connection_limit = 16;
  options.nonadjacent_penalty = 10.0;
  return options;
}

TEST(CoveragePathPlanner, RectangleProducesCoveragePath)
{
  CoveragePathPlanner planner;
  const std::vector<Point2> polygon{{0.0, 0.0}, {4.0, 0.0}, {4.0, 3.0}, {0.0, 3.0}};
  const CoveragePlanResult result = planner.plan(polygon, defaultOptions());

  EXPECT_TRUE(result.success) << result.message;
  EXPECT_FALSE(result.path_points.empty());
  EXPECT_GT(result.num_lines, 0U);
  EXPECT_EQ(result.decomposition_count, 1U);
  EXPECT_NEAR(result.line_spacing, 0.8, 1e-9);
  EXPECT_GT(result.total_path_cost, 0.0);
}

TEST(CoveragePathPlanner, ConcavePolygonUsesDecomposition)
{
  CoveragePathPlanner planner;
  const std::vector<Point2> polygon{
      {0.0, 0.0}, {4.0, 0.0}, {4.0, 1.0}, {1.0, 1.0}, {1.0, 4.0}, {0.0, 4.0}};
  const CoveragePlanResult result = planner.plan(polygon, defaultOptions());

  EXPECT_TRUE(result.success) << result.message;
  EXPECT_FALSE(result.path_points.empty());
  EXPECT_GT(result.decomposition_count, 1U);
  EXPECT_EQ(result.connection_order.size(), result.connection_states.size());
  EXPECT_EQ(result.selected_variant_indices.size(), result.decomposition_count);
}

TEST(CoveragePathPlanner, RobotStartAddsConnector)
{
  CoveragePathPlanner planner;
  PlanOptions options = defaultOptions();
  options.use_robot_start = true;
  options.use_robot_start_heading = false;
  options.robot_start = Point2{-1.0, -1.0};

  const std::vector<Point2> polygon{{0.0, 0.0}, {4.0, 0.0}, {4.0, 3.0}, {0.0, 3.0}};
  const CoveragePlanResult result = planner.plan(polygon, options);

  EXPECT_TRUE(result.success) << result.message;
  ASSERT_FALSE(result.path_points.empty());
  EXPECT_NEAR(result.path_points.front().x, options.robot_start.x, 1e-9);
  EXPECT_NEAR(result.path_points.front().y, options.robot_start.y, 1e-9);
  EXPECT_GE(result.start_connection_cost, 0.0);
  EXPECT_GE(result.start_region_index, 0);
  EXPECT_GE(result.start_variant_index, 0);
}

TEST(CoveragePathPlanner, TurnParametersStillProducePath)
{
  CoveragePathPlanner planner;
  PlanOptions options = defaultOptions();
  options.turn_radius = 0.8;
  options.turn_step = 0.05;
  const std::vector<Point2> polygon{{0.0, 0.0}, {6.0, 0.0}, {6.0, 2.5}, {0.0, 2.5}};
  const CoveragePlanResult result = planner.plan(polygon, options);

  EXPECT_TRUE(result.success) << result.message;
  EXPECT_FALSE(result.path_points.empty());
  EXPECT_GT(result.path_points.size(), result.num_lines * 2U);
}

TEST(CoveragePathPlanner, StraightStepControlsLineSampling)
{
  CoveragePathPlanner planner;
  PlanOptions sparse_options = defaultOptions();
  sparse_options.straight_step = 1.2;
  sparse_options.turn_step = 0.1;
  const std::vector<Point2> polygon{{0.0, 0.0}, {8.0, 0.0}, {8.0, 2.5}, {0.0, 2.5}};

  const CoveragePlanResult sparse_result = planner.plan(polygon, sparse_options);
  ASSERT_TRUE(sparse_result.success) << sparse_result.message;
  ASSERT_GT(sparse_result.path_points.size(), 2U);

  double max_segment_length = 0.0;
  for (std::size_t i = 0; i + 1 < sparse_result.path_points.size(); ++i)
  {
    max_segment_length =
        std::max(max_segment_length, pointDistance(sparse_result.path_points[i], sparse_result.path_points[i + 1]));
  }
  EXPECT_LE(max_segment_length, sparse_options.straight_step + 1.0e-6);

  PlanOptions dense_options = sparse_options;
  dense_options.straight_step = 0.4;
  const CoveragePlanResult dense_result = planner.plan(polygon, dense_options);
  ASSERT_TRUE(dense_result.success) << dense_result.message;
  EXPECT_GT(dense_result.path_points.size(), sparse_result.path_points.size());
}

TEST(CoveragePathPlanner, InvalidPolygonFailsGracefully)
{
  CoveragePathPlanner planner;
  const std::vector<Point2> polygon{{0.0, 0.0}, {1.0, 1.0}};
  const CoveragePlanResult result = planner.plan(polygon, defaultOptions());

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.message.empty());
  EXPECT_TRUE(result.path_points.empty());
}

}  // namespace
}  // namespace ackermann_car_gazebo

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
