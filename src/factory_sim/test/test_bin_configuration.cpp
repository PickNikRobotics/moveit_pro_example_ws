// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <thread>

#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <moveit_pro_behavior/behaviors/core/add_collision_box.hpp>
#include <moveit_pro_behavior/behaviors/core/get_element_of_vector.hpp>
#include <moveit_pro_behavior/behaviors/core/get_size_of_vector.hpp>
#include <moveit_pro_behavior/behaviors/core/log_message.hpp>
#include <moveit_pro_behavior/behaviors/core/transform_pose.hpp>
#include <moveit_pro_behavior_interface/geometry_msgs_string_conversions.hpp>
#include <moveit_pro_behavior_interface/load_from_yaml.hpp>
#include <moveit_pro_behavior_interface/logger.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <moveit_pro_test_utils/ros_test.hpp>
#include <pluginlib/class_loader.hpp>

namespace
{
class TestLogger : public moveit_pro::behavior::LoggerBase
{
public:
  void publishMessage(int32_t, const std::string& message) override
  {
    messages += message + "\n";
  }
  std::string consumeErrorLogBuffer() override
  {
    return messages;
  }
  std::string messages;
};

class BinConfiguration : public moveit_pro::test_utils::RosTest
{
public:
  void SetUp() override
  {
    const auto share = std::filesystem::path(ament_index_cpp::get_package_share_directory("factory_sim"));
    auto pattern = (std::filesystem::temp_directory_path() / "factory-bin-test-XXXXXX").string();
    const auto* temporary = mkdtemp(pattern.data());
    ASSERT_NE(temporary, nullptr);
    directory = temporary;
    configuration = directory / "bins.yaml";
    documents = YAML::LoadAllFromFile(std::string(FACTORY_CONFIG_DIR) + "/bin_poses.yaml");
    saveConfiguration();
    auto logger = std::make_unique<TestLogger>();
    log = logger.get();
    context = std::make_shared<moveit_pro::behaviors::BehaviorContext>(
        std::make_shared<rclcpp::Node>("bin_configuration_test"), std::move(logger), false);
    using namespace moveit_pro::behaviors;
    factory.registerNodeType<LoadMultipleFromYaml<geometry_msgs::msg::PoseStamped>>("LoadPoseStampedVectorFromYaml",
                                                                                    context);
    factory.registerNodeType<GetSizeOfVector>("GetSizeOfVector", context);
    factory.registerNodeType<GetElementOfVector>("GetElementOfVector", context);
    factory.registerNodeType<TransformPose>("TransformPose", context);
    factory.registerNodeType<AddCollisionBox>("AddCollisionBox", context);
    factory.registerNodeType<LogMessage>("LogMessage", context);
    converter_loader = std::make_unique<pluginlib::ClassLoader<SharedResourcesNodeLoaderBase>>(
        "moveit_pro_behavior_interface", "moveit_pro::behaviors::SharedResourcesNodeLoaderBase");
    converters = converter_loader->createSharedInstance("moveit_pro::behaviors::ConverterBehaviorsLoader");
    converters->registerBehaviors(factory, context);
    for (const auto* name : { "validate_bin_pose_values.xml", "load_bin_configuration.xml", "add_bin_rim.xml",
                              "add_bins_to_planning_scene.xml" })
      factory.registerBehaviorTreeFromFile(std::string(FACTORY_OBJECTIVES_DIR) + "/" + name);
    board = BT::Blackboard::create();
    board->set("configuration_file", std::filesystem::relative(configuration, share).string());
    moveit_msgs::msg::PlanningScene initial;
    moveit_msgs::msg::CollisionObject fixture;
    fixture.id = "unrelated_fixture";
    initial.world.collision_objects.push_back(fixture);
    moveit_msgs::msg::AttachedCollisionObject held;
    held.object.id = "held_tool";
    initial.robot_state.attached_collision_objects.push_back(held);
    board->set("planning_scene", initial);
  }
  void TearDown() override
  {
    std::filesystem::remove_all(directory);
  }
  void saveConfiguration()
  {
    std::ofstream output(configuration);
    for (const auto& document : documents)
      output << "---\n" << document << '\n';
  }
  BT::NodeStatus run()
  {
    auto tree = factory.createTree("Add Bins to Planning Scene", board);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    auto status = tree.tickOnce();
    while (status == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() < deadline)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      status = tree.tickOnce();
    }
    tree.haltTree();
    return status;
  }
  moveit_msgs::msg::PlanningScene scene()
  {
    return board->get<moveit_msgs::msg::PlanningScene>("planning_scene");
  }
  geometry_msgs::msg::PoseStamped pose(const std::string& key)
  {
    return board->get<geometry_msgs::msg::PoseStamped>(key);
  }
  std::filesystem::path directory;
  std::filesystem::path configuration;
  std::vector<YAML::Node> documents;
  std::shared_ptr<moveit_pro::behaviors::BehaviorContext> context;
  TestLogger* log;
  std::unique_ptr<pluginlib::ClassLoader<moveit_pro::behaviors::SharedResourcesNodeLoaderBase>> converter_loader;
  std::shared_ptr<moveit_pro::behaviors::SharedResourcesNodeLoaderBase> converters;
  BT::BehaviorTreeFactory factory;
  BT::Blackboard::Ptr board;
};

TEST_F(BinConfiguration, RestoresEightRimsAndPreservesFixturesAndAttachments)
{
  // GIVEN a scene with an unrelated fixture and an attached tool.
  // WHEN the default bin configuration is restored.
  ASSERT_EQ(run(), BT::NodeStatus::SUCCESS) << log->messages;
  // THEN the eight rims and dependent poses match the defaults without disturbing existing objects.
  auto restored = scene();
  ASSERT_EQ(restored.world.collision_objects.size(), 9u);
  EXPECT_EQ(restored.world.collision_objects.front().id, "unrelated_fixture");
  ASSERT_EQ(restored.robot_state.attached_collision_objects.size(), 1u);
  EXPECT_EQ(restored.robot_state.attached_collision_objects.front().object.id, "held_tool");
  for (std::size_t i = 1; i < restored.world.collision_objects.size(); ++i)
    EXPECT_EQ(restored.world.collision_objects[i].primitives.size(), 1u);
  EXPECT_NEAR(pose("pick_guess_pose").pose.position.x, -0.35, 1e-6);
  EXPECT_NEAR(pose("pick_guess_pose").pose.position.y, 0.8, 1e-6);
  EXPECT_NEAR(pose("pick_crop_pose").pose.position.z, 0.4, 1e-6);
  EXPECT_NEAR(pose("drop_pose").pose.position.x, 0.23, 1e-6);
  EXPECT_NEAR(pose("drop_pose").pose.position.y, 0.54, 1e-6);
  EXPECT_NEAR(pose("drop_pose").pose.position.z, 0.65, 1e-6);
  ASSERT_EQ(run(), BT::NodeStatus::SUCCESS) << log->messages;
  EXPECT_EQ(scene().world.collision_objects.size(), 9u);
}

TEST_F(BinConfiguration, MovingBinsMovesRimsAndDependentPoses)
{
  // GIVEN the default bin setup.
  ASSERT_EQ(run(), BT::NodeStatus::SUCCESS) << log->messages;
  const auto previous = scene();
  const auto guess = pose("pick_guess_pose");
  const auto crop = pose("pick_crop_pose");
  const auto drop = pose("drop_pose");
  // WHEN both bins translate 0.4 m along world x.
  documents[0]["pose"]["position"]["x"] = 0.15;
  documents[1]["pose"]["position"]["x"] = 0.65;
  saveConfiguration();
  ASSERT_EQ(run(), BT::NodeStatus::SUCCESS) << log->messages;
  // THEN all rim and dependent target positions translate by the same amount.
  const auto moved = scene();
  ASSERT_EQ(moved.world.collision_objects.size(), previous.world.collision_objects.size());
  for (std::size_t i = 1; i < moved.world.collision_objects.size(); ++i)
    EXPECT_NEAR(moved.world.collision_objects[i].pose.position.x - previous.world.collision_objects[i].pose.position.x,
                0.4, 1e-6);
  EXPECT_NEAR(pose("pick_guess_pose").pose.position.x - guess.pose.position.x, 0.4, 1e-6);
  EXPECT_NEAR(pose("pick_crop_pose").pose.position.x - crop.pose.position.x, 0.4, 1e-6);
  EXPECT_NEAR(pose("drop_pose").pose.position.x - drop.pose.position.x, 0.4, 1e-6);
}

TEST_F(BinConfiguration, RotatingBinsRotatesRimsAndDependentOffsets)
{
  // GIVEN bins with identity orientations.
  for (auto& document : documents)
  {
    document["pose"]["orientation"]["z"] = 0.0;
    document["pose"]["orientation"]["w"] = 1.0;
  }
  saveConfiguration();
  // WHEN the configured bin setup is restored.
  ASSERT_EQ(run(), BT::NodeStatus::SUCCESS) << log->messages;
  // THEN rim and target offsets follow the bin axes.
  const auto restored = scene();
  ASSERT_EQ(restored.world.collision_objects.size(), 9u);
  EXPECT_EQ(restored.world.collision_objects[1].id, "pick_bin/x_positive");
  EXPECT_NEAR(restored.world.collision_objects[1].pose.position.x, 0.0423, 1e-6);
  EXPECT_NEAR(restored.world.collision_objects[1].pose.position.y, 0.6, 1e-6);
  EXPECT_NEAR(pose("pick_guess_pose").pose.position.x, -0.05, 1e-6);
  EXPECT_NEAR(pose("pick_guess_pose").pose.position.y, 0.7, 1e-6);
  EXPECT_NEAR(pose("pick_crop_pose").pose.orientation.w, 1.0, 1e-6);
  EXPECT_NEAR(pose("drop_pose").pose.position.x, 0.19, 1e-6);
  EXPECT_NEAR(pose("drop_pose").pose.position.y, 0.62, 1e-6);
}

TEST_F(BinConfiguration, SubtreeUsesDefaultConfigurationFile)
{
  // GIVEN a caller that omits the configuration_file input.
  factory.registerBehaviorTreeFromText(R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Configured Bin Setup">
        <SubTree ID="Add Bins to Planning Scene" planning_scene="{planning_scene}"
          pick_guess_pose="{guess}" pick_crop_pose="{crop}" drop_pose="{drop}" />
      </BehaviorTree>
    </root>)");
  // WHEN the subtree runs with its declared default.
  auto tree = factory.createTree("Configured Bin Setup", board);
  EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::SUCCESS) << log->messages;
  // THEN the installed default configuration supplies all eight rims.
  EXPECT_EQ(scene().world.collision_objects.size(), 9u);
}

TEST_F(BinConfiguration, MissingFileFailsBeforeSceneChanges)
{
  // GIVEN a missing configuration file.
  std::filesystem::remove(configuration);
  // WHEN setup reads the configuration.
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  // THEN setup fails without inserting bin geometry.
  EXPECT_EQ(scene().world.collision_objects.size(), 1u);
}

TEST_F(BinConfiguration, MalformedYamlFailsBeforeSceneChanges)
{
  // GIVEN a malformed YAML document.
  std::ofstream(configuration) << "pose: [\n";
  // WHEN setup reads the configuration.
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  // THEN setup fails without inserting bin geometry.
  EXPECT_EQ(scene().world.collision_objects.size(), 1u);
}

TEST_F(BinConfiguration, MissingPlaceBinFailsBeforeSceneChanges)
{
  // GIVEN a configuration containing only the pick bin.
  documents.pop_back();
  saveConfiguration();
  // WHEN setup reads the configuration.
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  // THEN setup fails without inserting bin geometry.
  EXPECT_EQ(scene().world.collision_objects.size(), 1u);
}

TEST_F(BinConfiguration, InvalidPlaceOrientationFailsBeforeSceneChanges)
{
  // GIVEN a zero quaternion for the place bin.
  for (const auto* component : { "x", "y", "z", "w" })
    documents[1]["pose"]["orientation"][component] = 0.0;
  saveConfiguration();
  // WHEN setup reads the configuration.
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  // THEN setup fails without inserting bin geometry.
  EXPECT_EQ(scene().world.collision_objects.size(), 1u);
}

TEST_F(BinConfiguration, NonWorldFramesFailBeforeTargetsOrSceneChanges)
{
  // GIVEN existing targets that must survive invalid configuration.
  geometry_msgs::msg::PoseStamped unchanged;
  unchanged.header.frame_id = "targets-not-derived";
  for (const auto* key : { "pick_guess_pose", "pick_crop_pose", "drop_pose" })
    board->set(key, unchanged);
  for (std::size_t bin = 0; bin < documents.size(); ++bin)
  {
    for (const auto* frame : { "map", "" })
    {
      SCOPED_TRACE("bin=" + std::to_string(bin) + ", frame=" + frame);
      // WHEN either bin uses an unsupported frame.
      documents[bin]["header"]["frame_id"] = frame;
      saveConfiguration();
      log->messages.clear();
      EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
      // THEN targets and scene remain intact, and the rejection identifies the configuration.
      EXPECT_EQ(scene().world.collision_objects.size(), 1u);
      EXPECT_EQ(pose("pick_guess_pose"), unchanged);
      EXPECT_EQ(pose("pick_crop_pose"), unchanged);
      EXPECT_EQ(pose("drop_pose"), unchanged);
      EXPECT_NE(log->messages.find(board->get<std::string>("configuration_file")), std::string::npos);
      EXPECT_NE(log->messages.find(std::string(bin == 0 ? "pick frame=" : "place frame=") + frame), std::string::npos);
      EXPECT_NE(log->messages.find("Set both header.frame_id values to world"), std::string::npos);
    }
    documents[bin]["header"]["frame_id"] = "world";
  }
}
TEST_F(BinConfiguration, NonFiniteComponentsFailBeforeTargetsOrSceneChanges)
{
  // GIVEN existing targets and scene objects that must survive invalid bin poses.
  geometry_msgs::msg::PoseStamped unchanged;
  unchanged.header.frame_id = "targets-not-derived";
  for (const auto* key : { "pick_guess_pose", "pick_crop_pose", "drop_pose" })
    board->set(key, unchanged);
  const auto initial_scene = scene();
  const auto non_finite_values = { std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
                                   -std::numeric_limits<double>::infinity() };
  for (std::size_t bin = 0; bin < documents.size(); ++bin)
  {
    for (const auto* group : { "position", "orientation" })
    {
      for (const auto* component : { "x", "y", "z", "w" })
      {
        if (std::string(group) == "position" && std::string(component) == "w")
          continue;
        const auto original = documents[bin]["pose"][group][component].as<double>();
        for (const auto value : non_finite_values)
        {
          SCOPED_TRACE("bin=" + std::to_string(bin) + ", " + group + "." + component + "=" + std::to_string(value));
          // WHEN either bin contains NaN or infinity in a position or quaternion component.
          documents[bin]["pose"][group][component] = value;
          saveConfiguration();
          log->messages.clear();
          EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
          // THEN targets and the complete scene remain unchanged, with a rejection diagnostic.
          EXPECT_EQ(scene(), initial_scene);
          EXPECT_EQ(pose("pick_guess_pose"), unchanged);
          EXPECT_EQ(pose("pick_crop_pose"), unchanged);
          EXPECT_EQ(pose("drop_pose"), unchanged);
          EXPECT_FALSE(log->messages.empty());
          if (std::string(group) == "position")
          {
            EXPECT_NE(log->messages.find(board->get<std::string>("configuration_file")), std::string::npos);
            EXPECT_NE(log->messages.find(bin == 0 ? "pick bin" : "place bin"), std::string::npos);
            EXPECT_NE(log->messages.find("must be finite"), std::string::npos);
          }
        }
        documents[bin]["pose"][group][component] = original;
      }
    }
  }
}
}  // namespace
