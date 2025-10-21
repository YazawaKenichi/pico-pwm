bool MainNode::check_agent_alive()
{
    const int TIMEOUT_MS = 1000;
    const uint8_t ATTEMPTS = 120;
    rcl_ret_t ret = rmw_uros_ping_agent(TIMEOUT_MS, ATTEMPTS);
    return ret;
}

void init()
{
    rmw_uros_set_custom_transport(true, NULL, pico_serial_transport_open, pico_serial_transport_close, pico_serial_transport_write, pico_serial_transport_read);
    while(RCL_RET_OK != check_agent_alive())
    {
        //! Agent の生存確認
    }
}

int main()
{
    init();

    rclc_support_init(&support_, 0, NULL, &allocator_);
    rclc_node_init_default(&node_, "/pico_node", "", &support);
    rclc_executor_init(&this->executor_, &support_.context, 1, &allocator_);

    while(1)
    {
        rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(100));
    }

    return 0;
}

