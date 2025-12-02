#include "main.h"

#include <math.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/vector3.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"

///// ブラシレスモータ //////

//! 砲台左
uint16_t gun_l_level_;
rcl_publisher_t gun_l_publisher_;
rcl_subscription_t gun_l_subscriber_;
std_msgs__msg__Float32 gun_l_msg_;

//! 砲台右
uint16_t gun_r_level_;
rcl_publisher_t gun_r_publisher_;
rcl_subscription_t gun_r_subscriber_;
std_msgs__msg__Float32 gun_r_msg_;

///// Arduino の UART /////
#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1

#define STEPPER_UART 1

// Agent の生存確認
bool check_agent_alive()
{
    const int TIMEOUT_MS = 1000;
    const uint8_t ATTEMPTS = 120;
    rcl_ret_t ret = rmw_uros_ping_agent(TIMEOUT_MS, ATTEMPTS);
    return (ret == RCL_RET_OK);
}

///// 初期化関数 /////
void init_ros()
{
    rmw_uros_set_custom_transport(true, NULL, pico_serial_transport_open, pico_serial_transport_close, pico_serial_transport_write, pico_serial_transport_read);
    while(!check_agent_alive())
    {
        //! Agent から応答が返ってくるまで待機
        sleep_ms(100);
    }
}

float rs2205_adaptor(float ad)
{
    uint32_t high_time = RESCALE(ad, APPARENTLY_MAX, APPARENTLY_MIN, HIGH_TIME_MAX, HIGH_TIME_MIN);
    //! ↓ すでに -100 ~ 100 でクリッピングされた値を引数にすれば正直ここは素通りするはず
    if(high_time > HIGH_TIME_MAX)
    {
        high_time = HIGH_TIME_MAX;
    }
    if(high_time < HIGH_TIME_MIN)
    {
        high_time = HIGH_TIME_MIN;
    }
    return (100.0f * high_time) / HZ2US(RS2205_PWM_HZ);
}

uint16_t gun_duty2level(float apparently_duty)
{
    //! 上限と下限
    if(apparently_duty < APPARENTLY_MIN)
    {
        apparently_duty = APPARENTLY_MIN;
    }
    if(apparently_duty > APPARENTLY_MAX)
    {
        apparently_duty = APPARENTLY_MAX;
    }
    //! -100 ~ 100 から 実際のデューティ比 5.1% ~ 10.1% に変換
    float realistic_duty = rs2205_adaptor(apparently_duty);
    //! 0 < level < wrap
    return (float) RS2205_PWM_RESOLUTION * (realistic_duty / 100.0f);
}

uint16_t set_gun_pwm(uint16_t pin, float duty)
{
    uint16_t level = gun_duty2level(duty);
    pwm_set_gpio_level(pin, level);
    return level;
}

void gun_l_callback_(const void * msgin)
{
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *) msgin;
    if(msg == NULL)
    {
        return;
    }
    gun_l_level_ = set_gun_pwm(GUN_L_PIN, 1 * msg->data);
}

void gun_r_callback_(const void * msgin)
{
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *) msgin;
    if(msg == NULL)
    {
        return;
    }
    gun_r_level_ = set_gun_pwm(GUN_R_PIN, -1 * msg->data);
}

void gun_l_timer_callback_(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) timer;
    (void) last_call_time;
    std_msgs__msg__Int32 pub_msg_;
    pub_msg_.data = gun_l_level_;
    rcl_publish(&gun_l_publisher_, &pub_msg_, NULL);
}

void gun_r_timer_callback_(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) timer;
    (void) last_call_time;
    std_msgs__msg__Int32 pub_msg_;
    pub_msg_.data = gun_r_level_;
    rcl_publish(&gun_r_publisher_, &pub_msg_, NULL);
}

void init_gun_pwm()
{
    ///// 砲台 /////
    //! PWM 周波数と分解能の設定
    pwm_config gun_cfg = pwm_get_default_config();
    uint32_t wrap = RS2205_PWM_RESOLUTION - 1;
    float clkdiv = (float) clock_get_hz(clk_sys) / (RS2205_PWM_HZ * RS2205_PWM_RESOLUTION);
    pwm_config_set_wrap(&gun_cfg, wrap);
    pwm_config_set_clkdiv(&gun_cfg, clkdiv);
    //! PWM ピンの設定
    gpio_set_function(GUN_L_PIN, GPIO_FUNC_PWM);
    gpio_set_function(GUN_R_PIN, GPIO_FUNC_PWM);
    uint slice_left = pwm_gpio_to_slice_num(GUN_L_PIN);
    uint slice_right = pwm_gpio_to_slice_num(GUN_R_PIN);
    pwm_init(slice_left, &gun_cfg, true);
    pwm_init(slice_right, &gun_cfg, true);
    gun_l_level_ = set_gun_pwm(GUN_L_PIN, 0);
    gun_r_level_ = set_gun_pwm(GUN_R_PIN, 0);
}

rcl_publisher_t pose_publisher_;
rcl_subscription_t pose_subscriber_;
geometry_msgs__msg__Vector3 pose_msg_;

uint16_t roll_level_;
rcl_publisher_t roll_publisher_;
rcl_subscription_t roll_subscriber_;
std_msgs__msg__Int32 roll_msg_;

uint16_t pitch_level_;
rcl_publisher_t pitch_publisher_;
rcl_subscription_t pitch_subscriber_;
std_msgs__msg__Int32 pitch_msg_;

uint16_t yaw_level_;
rcl_publisher_t yaw_publisher_;
rcl_subscription_t yaw_subscriber_;
std_msgs__msg__Int32 yaw_msg_;

uint16_t loading_level_;
rcl_publisher_t loading_publisher_;
rcl_subscription_t loading_subscriber_;
std_msgs__msg__Int32 loading_msg_;

uint16_t servo_deg2level(float degree)
{
    if(degree < SERVO_ANGLE_MIN)
    {
        degree = SERVO_ANGLE_MIN;
    }
    if(degree > SERVO_ANGLE_MAX)
    {
        degree = SERVO_ANGLE_MAX;
    }
    float servo_pulse_length = RESCALE(degree, SERVO_ANGLE_MAX, SERVO_ANGLE_MIN, SERVO_PULSE_LENGTH_MAX, SERVO_PULSE_LENGTH_MIN);
    float servo_duty = 100.0f * servo_pulse_length / (float) HZ2US(SERVO_HZ);
    return SERVO_RESOLUTION * (float) (servo_duty / (float) 100.0f);
}

uint16_t set_servo_pwm(uint32_t pin, float degree)
{
    uint16_t level = servo_deg2level(degree);
    pwm_set_gpio_level(pin, level);
    return level;
}

void pose_callback_(const void * msgin)
{
    const geometry_msgs__msg__Vector3 * msg = (const geometry_msgs__msg__Vector3 *) msgin;
    if(msg == NULL)
    {
        return;
    }
    roll_level_ = set_servo_pwm(ROLL_PIN, msg->x);
    pitch_level_ = set_servo_pwm(PITCH_PIN, msg->y);
    yaw_level_ = set_servo_pwm(YAW_PIN, msg->z);
}

void roll_callback_(const void * msgin)
{
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *) msgin;
    if(msg == NULL)
    {
        return;
    }
    roll_level_ = set_servo_pwm(ROLL_PIN, msg->data);
}

void pose_timer_callback_(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) timer;
    (void) last_call_time;
    geometry_msgs__msg__Vector3 pub_msg_;
    pub_msg_.x = roll_level_;
    pub_msg_.y = pitch_level_;
    pub_msg_.z = yaw_level_;
    rcl_publish(&pose_publisher_, &pub_msg_, NULL);
}

void roll_timer_callback_(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) timer;
    (void) last_call_time;
    std_msgs__msg__Int32 pub_msg_;
    pub_msg_.data = roll_level_;
    rcl_publish(&roll_publisher_, &pub_msg_, NULL);
}

void pitch_callback_(const void * msgin)
{
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *) msgin;
    if(msg == NULL)
    {
        return;
    }
    pitch_level_ = set_servo_pwm(PITCH_PIN, msg->data);
}

void pitch_timer_callback_(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) timer;
    (void) last_call_time;
    std_msgs__msg__Int32 pub_msg_;
    pub_msg_.data = pitch_level_;
    rcl_publish(&pitch_publisher_, &pub_msg_, NULL);
}

void yaw_callback_(const void * msgin)
{
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *) msgin;
    if(msg == NULL)
    {
        return;
    }
    yaw_level_ = set_servo_pwm(YAW_PIN, msg->data);
}

void yaw_timer_callback_(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) timer;
    (void) last_call_time;
    std_msgs__msg__Int32 pub_msg_;
    pub_msg_.data = yaw_level_;
    rcl_publish(&yaw_publisher_, &pub_msg_, NULL);
}

void loading_callback_(const void * msgin)
{
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *) msgin;
    if(msg == NULL)
    {
        return;
    }
    loading_level_ = set_servo_pwm(LOADING_PIN, msg->data);
}

void loading_timer_callback_(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) timer;
    (void) last_call_time;
    std_msgs__msg__Int32 pub_msg_;
    pub_msg_.data = loading_level_;
    rcl_publish(&loading_publisher_, &pub_msg_, NULL);
}

void init_servo_pwm()
{
    ///// 砲塔 /////
    //! PWM 周波数と分解能の設定
    pwm_config servo_cfg = pwm_get_default_config();
    uint32_t wrap = SERVO_RESOLUTION - 1;
    float clkdiv = (float) clock_get_hz(clk_sys) / (SERVO_HZ * SERVO_RESOLUTION);
    pwm_config_set_wrap(&servo_cfg, wrap);
    pwm_config_set_clkdiv(&servo_cfg, clkdiv);

    //! PWM ピンの設定
    gpio_set_function(ROLL_PIN, GPIO_FUNC_PWM);
    uint slice_roll = pwm_gpio_to_slice_num(ROLL_PIN);
    pwm_init(slice_roll, &servo_cfg, true);
    roll_level_ = set_servo_pwm(ROLL_PIN, 90);

    gpio_set_function(PITCH_PIN, GPIO_FUNC_PWM);
    uint slice_pitch = pwm_gpio_to_slice_num(PITCH_PIN);
    pwm_init(slice_pitch, &servo_cfg, true);
    pitch_level_ = set_servo_pwm(PITCH_PIN, 90);

    gpio_set_function(YAW_PIN, GPIO_FUNC_PWM);
    uint slice_yaw = pwm_gpio_to_slice_num(YAW_PIN);
    pwm_init(slice_yaw, &servo_cfg, true);
    yaw_level_ = set_servo_pwm(YAW_PIN, 90);

    gpio_set_function(LOADING_PIN, GPIO_FUNC_PWM);
    uint slice_loading = pwm_gpio_to_slice_num(LOADING_PIN);
    pwm_init(slice_loading, &servo_cfg, true);
    loading_level_ = set_servo_pwm(LOADING_PIN, 90);
}

bool led_state_;
void put_led(bool b)
{
    gpio_put(PICO_DEFAULT_LED_PIN, b);
    led_state_ = b;
}
bool get_led()
{
    return led_state_;
}

//! ステッピング
//! float stepper_position_;
float stepper_bef_;
rcl_publisher_t stepper_publisher_;     // 未使用 値をパブリッシュするときに使える用
rcl_subscription_t stepper_subscriber_;
std_msgs__msg__Float32 stepper_msg_;

void uart_write_float(float value)
{
#if 0
    //! float の型をそのまま TX しようとしたパターン
    //! 問題点 : Arduino 側の受け取りプログラムを書くのが面倒くさくなる
    union { float f; uint8_t b[4]; } conv;
    conv.f = value;
    uart_write_blocking(UART_ID, conv.b, 4);
#endif
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
    put_led(!get_led());
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
    rcl_publish(&stepper_publisher_, &pub_msg_, NULL);
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
}

void init_stepper()
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
}

//! GPIO
void init_gpio()
{
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
}

//! 時間管理系統
struct repeating_timer time_timer_;
uint64_t time_;

bool time_update_callback(struct repeating_timer *t)
{
    time_++;
    return true;
}

void init_time()
{
    time_ = 0;
    add_repeating_timer_us(-1, time_update_callback, NULL, &time_timer_);
}

int main()
{
    init_gpio();
    // init_time();
    init_stepper();
    ////////// micro ROS の処理 //////////
    init_ros();
    init_gun_pwm();
    init_servo_pwm();

    rcl_node_t node_;
    rcl_allocator_t allocator_ = rcl_get_default_allocator();
    rclc_support_t support_;
    rclc_executor_t executor_;

    ///// ノードの作成 /////
    rclc_support_init(&support_, 0, NULL, &allocator_);
    rclc_node_init_default(&node_, "pico_node", "", &support_);
    rclc_executor_init(&executor_, &support_.context, 16, &allocator_);

    //! ステッピングモータのサブスクライバ
    rcl_ret_t rc;
    rc = rclc_subscription_init_default(&stepper_subscriber_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/pico/stepper/position/raw");
    rclc_subscription_init_default(&loading_subscriber_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/pico/loading/deg");
    rclc_executor_add_subscription(&executor_, &loading_subscriber_, &loading_msg_, loading_callback_, ON_NEW_DATA);

    ///// サブスクライバの作成 /////
    //! 砲台左のサブスクライバ
    rclc_subscription_init_default(&gun_l_subscriber_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/pico/gun/left/pwm/duty");
    rclc_executor_add_subscription(&executor_, &gun_l_subscriber_, &gun_l_msg_, gun_l_callback_, ON_NEW_DATA);
    //! 砲台右のサブスクライバ
    rclc_subscription_init_default(&gun_r_subscriber_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/pico/gun/right/pwm/duty");
    rclc_executor_add_subscription(&executor_, &gun_r_subscriber_, &gun_r_msg_, gun_r_callback_, ON_NEW_DATA);
    //! 砲塔サーボのサブスクライバ
#if INTEGRATE
    rclc_subscription_init_default(&pose_subscriber_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "/pico/pose");
    rclc_executor_add_subscription(&executor_, &pose_subscriber_, &pose_msg_, pose_callback_, ON_NEW_DATA);
#else
    rclc_subscription_init_default(&roll_subscriber_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/pico/roll/deg");
    rclc_executor_add_subscription(&executor_, &roll_subscriber_, &roll_msg_, roll_callback_, ON_NEW_DATA);
    rclc_subscription_init_default(&pitch_subscriber_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/pico/pitch/deg");
    rclc_executor_add_subscription(&executor_, &pitch_subscriber_, &pitch_msg_, pitch_callback_, ON_NEW_DATA);
    rclc_subscription_init_default(&yaw_subscriber_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/pico/yaw/deg");
    rclc_executor_add_subscription(&executor_, &yaw_subscriber_, &yaw_msg_, yaw_callback_, ON_NEW_DATA);
#endif
    if ( rc != RCL_RET_OK )
    {
        while (true)
        {
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            sleep_ms(500);
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
            sleep_ms(500);
        }
    }
    rclc_executor_add_subscription(&executor_, &stepper_subscriber_, &stepper_msg_, stepper_callback_, ON_NEW_DATA);

#define PUBLISHER 0
#if PUBLISHER
    ///// パブリッシャの作成 /////
    //! 砲台左のパブリッシャ
    rcl_timer_t gun_l_timer_;
    rclc_publisher_init_default(&gun_l_publisher_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/pico/gun/left/pwm/level");
    rclc_timer_init_default(&gun_l_timer_, &support_, RCL_MS_TO_NS(1000), gun_l_timer_callback_);
    rclc_executor_add_timer(&executor_, &gun_l_timer_);
    //! 砲台右のパブリッシャ
    rcl_timer_t gun_r_timer_;
    rclc_publisher_init_default(&gun_r_publisher_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/pico/gun/right/pwm/level");
    rclc_timer_init_default(&gun_r_timer_, &support_, RCL_MS_TO_NS(1000), gun_r_timer_callback_);
    rclc_executor_add_timer(&executor_, &gun_r_timer_);

    //! 砲塔サーボのパブリッシャ
    rcl_timer_t pose_timer_;
    rcl_timer_t roll_timer_;
    rcl_timer_t pitch_timer_;
    rcl_timer_t yaw_timer_;
    rcl_timer_t loading_timer_;

#if INTEGRATE
    rclc_publisher_init_default(&pose_publisher_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/pico/pose/level");
    rclc_timer_init_default(&pose_timer_, &support_, RCL_MS_TO_NS(1000), pose_timer_callback_);
    rclc_executor_add_timer(&executor_, &pose_timer_);
#else
    rclc_publisher_init_default(&roll_publisher_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/pico/roll/level");
    rclc_timer_init_default(&roll_timer_, &support_, RCL_MS_TO_NS(1000), roll_timer_callback_);
    rclc_executor_add_timer(&executor_, &roll_timer_);
    rclc_publisher_init_default(&pitch_publisher_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/pico/pitch/level");
    rclc_timer_init_default(&pitch_timer_, &support_, RCL_MS_TO_NS(1000), pitch_timer_callback_);
    rclc_executor_add_timer(&executor_, &pitch_timer_);
    rclc_publisher_init_default(&yaw_publisher_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/pico/yaw/level");
    rclc_timer_init_default(&yaw_timer_, &support_, RCL_MS_TO_NS(1000), yaw_timer_callback_);
    rclc_executor_add_timer(&executor_, &yaw_timer_);
#endif //! INTEGRATE

    rclc_publisher_init_default(&loading_publisher_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/pico/loading/level");
    rclc_timer_init_default(&loading_timer_, &support_, RCL_MS_TO_NS(1000), loading_timer_callback_);
    rclc_executor_add_timer(&executor_, &loading_timer_);
    //! ステッピングモータのパブリッシャ
    rcl_timer_t stepper_timer_;
    rclc_publisher_init_default(&stepper_publisher_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/pico/stepper/position/debug");
    rclc_timer_init_default(&stepper_timer_, &support_, RCL_MS_TO_NS(1000), stepper_timer_callback_);
    rclc_executor_add_timer(&executor_, &stepper_timer_);

#endif //! PUBLISHER

    while(true)
    {
        rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(1));
    }

    return 0;
}

