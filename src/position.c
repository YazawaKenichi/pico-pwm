#include "position.h"

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

#include "std_msgs/msg/float32.h"
#include "std_msgs/msg/bool.h"

float stepper_bef_;
rcl_publisher_t stepper_publisher_;     // 未使用 値をパブリッシュするときに使える用
rcl_subscription_t stepper_subscriber_;
std_msgs__msg__Float32 stepper_msg_;
rcl_publisher_t goal_publisher_;
bool goaled_;

void init_trigger()
{
    gpio_init(TRIGGER_PIN);
    gpio_set_dir(TRIGGER_PIN, GPIO_IN);
    gpio_pull_up(TRIGGER_PIN);
}

bool get_trigger()
{
    return gpio_get(TRIGGER_PIN);
}

void uart_write_float(float value)
{
#if 0
    //! float の型をそのまま TX しようとしたパターン
    //! 問題点 : Arduino 側の受け取りプログラムを書くのが面倒くさくなる
    union { float f; uint8_t b[4]; } conv;
    conv.f = value;
    uart_write_blocking(UART_ID, conv.b, 4);
#endif
    printf("uart_write_float called: %f\n", (double)value);
    char buf[32];
    int len = snprintf(buf, sizeof(buf), "%f\r\n", (float) value);
    uart_write_blocking(UART_ID, (uint8_t *)buf, len);
}

void uart_write_string(char * message_)
{
    uart_write_blocking(UART_ID, (const uint8_t *) message_, strlen(message_));
}

void stepper_callback_(const void * msgin)
{
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *) msgin;
    if(msg == NULL)
    {
        return;
    }
    if(fabsf(stepper_bef_ - msg->data) > 1e-3f)
    {
#if STEPPER_DRIVE
        //! 値の更新があったときだけ実行
        set_step_rate(msg->data);
#endif
#if STEPPER_UART
        // const char *uart_message_ = "Hello, World!\r\n";
        // uart_write_string(uart_message_);
        uart_write_float(msg->data);
#endif
        stepper_bef_ = msg->data;
    }
}

void stepper_timer_callback_(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) timer;
    (void) last_call_time;
    std_msgs__msg__Float32 pub_msg_;
    pub_msg_.data = stepper_bef_;
    rcl_ret_t rc = rcl_publish(&stepper_publisher_, &pub_msg_, NULL);
}

void set_step_rate(float freq_hz)
{
    uint slice_num = pwm_gpio_to_slice_num(STEPPER_PIN);
    if(freq_hz <= 0.0f)
    {
        pwm_set_wrap(slice_num, 0xFFFF);
        pwm_set_chan_level(slice_num, pwm_gpio_to_channel(STEPPER_PIN), 0);
        return ;
    }
    else
    {
        uint32_t wrap = (uint32_t)(clock_get_hz(clk_sys) / freq_hz) - 1;
        if (wrap > 0xFFFF)
        {
            wrap = 0xFFFF;
        }
        pwm_set_wrap(slice_num, wrap);
        pwm_set_chan_level(slice_num, pwm_gpio_to_channel(STEPPER_PIN), wrap / 2);
    }
}

void init_uart()
{
    stdio_init_all();
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    uart_set_fifo_enabled(UART_ID, false);
}

void position_init()
{
#if STEPPER_DRIVE
    ///// ステッピングモータ /////
    //! PWM 周波数と分解能の設定
    pwm_config stepper_cfg = pwm_get_default_config();
    //! 1 分周
    // float clkdiv = (float) clock_get_hz(clk_sys) / (STEPPER_HZ * STEPPER_RESOLUTION);
    pwm_config_set_clkdiv(&stepper_cfg, 1.0f);
    //! PWM ピンの設定
    gpio_set_function(STEPPER_PIN, GPIO_FUNC_PWM);
    uint slice_stepper = pwm_gpio_to_slice_num(STEPPER_PIN);
    pwm_init(slice_stepper, &stepper_cfg, true);
    pwm_set_enabled(slice_stepper, true);
#endif
#if STEPPER_UART
    init_uart();
#endif
    init_trigger();
    init_goal();
}

void init_goal()
{
    goaled_ = true;
}

bool get_goal()
{
    return goaled_;
}

void set_goal(bool tf)
{
    goaled_ = tf;
}

void goal_timer_callback_(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) timer;
    (void) last_call_time;
    std_msgs__msg__Bool pub_msg_;
    pub_msg_.data = get_trigger();
    rcl_ret_t rc = rcl_publish(&goal_publisher_, &pub_msg_, NULL);
}

