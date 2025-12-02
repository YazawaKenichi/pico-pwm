// Agent の生存確認
bool check_agent_alive()
{
    const int TIMEOUT_MS = 1000;
    const uint8_t ATTEMPTS = 120;
    rcl_ret_t ret = rmw_uros_ping_agent(TIMEOUT_MS, ATTEMPTS);
    return (ret == RCL_RET_OK);
}

void init_ros()
{
    rmw_uros_set_custom_transport(true, NULL, pico_serial_transport_open, pico_serial_transport_close, pico_serial_transport_write, pico_serial_transport_read);
    while(!check_agent_alive())
    {
        //! Agent から応答が返ってくるまで待機
        sleep_ms(100);
    }
}


