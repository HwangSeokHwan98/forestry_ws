#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <urdf/model.h>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <vector>
#include <memory>

class JointStateHandler : public rclcpp::Node {
public:
    JointStateHandler(const std::string &urdf_file, const std::string &base_link, const std::string &end_effector)
        : Node("joint_state_handler") {
        // URDF 파싱 및 KDL 체인 생성
        KDL::Tree kdl_tree;
        if (!kdl_parser::treeFromFile(urdf_file, kdl_tree)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load URDF and parse KDL tree");
            throw std::runtime_error("URDF parsing failed");
        }

        if (!kdl_tree.getChain(base_link, end_effector, kdl_chain)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to extract KDL chain");
            throw std::runtime_error("KDL chain extraction failed");
        }

        // KDL Solver 생성
        auto gravity_vector = KDL::Vector(0.0, 0.0, -9.81);
        dyn_solver = std::make_unique<KDL::ChainDynParam>(kdl_chain, gravity_vector);

        // 초기화
        num_joints = kdl_chain.getNrOfJoints();
        q.resize(num_joints);
        q_dot.resize(num_joints);
        gravity.resize(num_joints);
        coriolis.resize(num_joints);
        mass_matrix.resize(num_joints);

        // JointState 토픽 구독
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&JointStateHandler::jointStateCallback, this, std::placeholders::_1));
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (msg->position.size() != num_joints || msg->velocity.size() != num_joints) {
            RCLCPP_WARN(this->get_logger(), "JointState size mismatch");
            return;
        }

        // 상태 업데이트
        for (size_t i = 0; i < num_joints; ++i) {
            q(i) = msg->position[i];
            q_dot(i) = msg->velocity[i];
        }

        computeDynamics();
    }

    void computeDynamics() {
        // 질량 행렬
        dyn_solver->JntToMass(q, mass_matrix);

        // 중력 벡터
        dyn_solver->JntToGravity(q, gravity);

        // 코리올리 및 원심력
        dyn_solver->JntToCoriolis(q, q_dot, coriolis);

        RCLCPP_INFO(this->get_logger(), "Mass Matrix M(q):");
        for (size_t i = 0; i < num_joints; ++i) {
            for (size_t j = 0; j < num_joints; ++j) {
                std::cout << mass_matrix(i, j) << " ";
            }
            std::cout << std::endl;
        }

        RCLCPP_INFO(this->get_logger(), "Gravity Vector G(q):");
        for (size_t i = 0; i < num_joints; ++i) {
            std::cout << gravity(i) << " ";
        }
        std::cout << std::endl;

        RCLCPP_INFO(this->get_logger(), "Coriolis Forces C(q, q_dot):");
        for (size_t i = 0; i < num_joints; ++i) {
            std::cout << coriolis(i) << " ";
        }
        std::cout << std::endl;
    }

    size_t num_joints;
    KDL::Chain kdl_chain;
    KDL::JntArray q, q_dot, gravity, coriolis;
    KDL::JntSpaceInertiaMatrix mass_matrix;
    std::unique_ptr<KDL::ChainDynParam> dyn_solver;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    if (argc < 4) {
        std::cerr << "Usage: joint_state_handler <urdf_file> <base_link> <end_effector>" << std::endl;
        return 1;
    }

    try {
        auto node = std::make_shared<JointStateHandler>(argv[1], argv[2], argv[3]);
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    rclcpp::shutdown();
    return 0;
}
