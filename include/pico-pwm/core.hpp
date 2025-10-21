
class MainNode()
{
    public:
        MainNode::MainNode();
        bool MainNode::check_agent_alive();

        rcl_node_t node_;
        rclc_support_t support_;
        rclc_allocator_t allocator_;
        rclc_executor_t executor_;
}

