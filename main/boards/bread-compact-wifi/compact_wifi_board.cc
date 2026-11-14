#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/oled_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include "led/single_led.h"
#include "assets/lang_config.h"

#include <esp_log.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <cmath>
#include <algorithm>
#include <sstream>

#ifdef SH1106
#include <esp_lcd_panel_sh1106.h>
#endif

#define TAG "CompactWifiBoard"

class HumanoidRobotController {
private:
    i2c_master_dev_handle_t pca_dev_ = nullptr;
    SemaphoreHandle_t mutex_ = nullptr;
    float current_angles_[9] = {90, 90, 90, 90, 90, 90, 90, 90, 90};
    int servo_offsets_[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    bool is_busy_action_ = false;

    void WriteReg(uint8_t reg, uint8_t val) {
        if (!pca_dev_) return;
        uint8_t data[2] = {reg, val};
        i2c_master_transmit(pca_dev_, data, 2, 100);
    }

    void WritePwm(uint8_t channel, uint16_t on, uint16_t off) {
        if (!pca_dev_ || channel > 15) return;
        uint8_t data[5] = {
            static_cast<uint8_t>(0x06 + 4 * channel),
            static_cast<uint8_t>(on & 0xFF),
            static_cast<uint8_t>(on >> 8),
            static_cast<uint8_t>(off & 0xFF),
            static_cast<uint8_t>(off >> 8)
        };
        i2c_master_transmit(pca_dev_, data, 5, 100);
    }

    void SetRawAngle(uint8_t ch, float angle) {
        if (ch >= 9) return;
        float calibrated = std::clamp(angle + servo_offsets_[ch], 0.0f, 180.0f);
        uint16_t pulse = static_cast<uint16_t>(102 + (calibrated / 180.0f) * (512 - 102));
        WritePwm(ch, 0, pulse);
        current_angles_[ch] = angle;
    }

public:
    void Init(i2c_master_bus_handle_t i2c_bus) {
        mutex_ = xSemaphoreCreateMutex();
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = 0x40,
            .scl_speed_hz = 400 * 1000,
        };
        if (i2c_master_bus_add_device(i2c_bus, &dev_cfg, &pca_dev_) != ESP_OK) {
            ESP_LOGE(TAG, "PCA9685 not found at 0x40");
            return;
        }

        WriteReg(0x00, 0x10);
        vTaskDelay(pdMS_TO_TICKS(5));
        WriteReg(0xFE, 121);
        WriteReg(0x00, 0x20);
        vTaskDelay(pdMS_TO_TICKS(5));

        for (int i = 0; i < 9; i++) {
            SetRawAngle(i, 90.0f);
        }
    }

    void MoveAllSmooth(const float target[9], int duration_ms) {
        if (!pca_dev_) return;
        xSemaphoreTake(mutex_, portMAX_DELAY);
        float start[9];
        for (int i = 0; i < 9; i++) start[i] = current_angles_[i];

        int steps = std::max(1, duration_ms / 20);
        for (int s = 1; s <= steps; s++) {
            float t = static_cast<float>(s) / steps;
            float ease = t * t * (3.0f - 2.0f * t);
            for (int i = 0; i < 9; i++) {
                float a = start[i] + (target[i] - start[i]) * ease;
                SetRawAngle(i, a);
            }
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        xSemaphoreGive(mutex_);
    }

    void SetSingleJoint(int ch, int angle, int duration_ms = 300) {
        if (ch < 0 || ch >= 9) return;
        float target[9];
        for (int i = 0; i < 9; i++) target[i] = current_angles_[i];
        target[ch] = std::clamp(angle, 0, 180);
        MoveAllSmooth(target, duration_ms);
    }

    void StandNeutral(int speed_ms = 400) {
        float neutral[9] = {90, 90, 90, 90, 90, 90, 90, 90, 90};
        MoveAllSmooth(neutral, speed_ms);
    }

    void WaveHand() {
        is_busy_action_ = true;
        float pose1[9] = {20, 45, 90, 90, 90, 90, 90, 90, 75};
        float pose2[9] = {20, 120, 90, 90, 90, 90, 90, 90, 105};
        MoveAllSmooth(pose1, 350);
        for (int i = 0; i < 3; i++) {
            MoveAllSmooth(pose2, 250);
            MoveAllSmooth(pose1, 250);
        }
        StandNeutral(400);
        is_busy_action_ = false;
    }

    void WalkForward(int steps_count = 3) {
        is_busy_action_ = true;
        for (int s = 0; s < steps_count; s++) {
            float tilt_r[9] = {90, 90, 90, 115, 90, 90, 90, 110, 85};
            MoveAllSmooth(tilt_r, 220);
            float step_l[9] = {65, 80, 75, 115, 110, 90, 115, 110, 85};
            MoveAllSmooth(step_l, 260);
            float level1[9] = {65, 80, 75, 90, 110, 90, 115, 90, 90};
            MoveAllSmooth(level1, 200);
            float tilt_l[9] = {70, 80, 75, 70, 110, 90, 115, 65, 95};
            MoveAllSmooth(tilt_l, 220);
            float step_r[9] = {110, 90, 115, 70, 65, 80, 75, 65, 95};
            MoveAllSmooth(step_r, 260);
            float level2[9] = {110, 90, 115, 90, 65, 80, 75, 90, 90};
            MoveAllSmooth(level2, 200);
        }
        StandNeutral(350);
        is_busy_action_ = false;
    }

    void WalkBackward(int steps_count = 2) {
        is_busy_action_ = true;
        for (int s = 0; s < steps_count; s++) {
            float tilt_r[9] = {90, 90, 90, 115, 90, 90, 90, 110, 90};
            MoveAllSmooth(tilt_r, 220);
            float step_l[9] = {110, 90, 110, 115, 70, 80, 65, 110, 90};
            MoveAllSmooth(step_l, 260);
            float level1[9] = {110, 90, 110, 90, 70, 80, 65, 90, 90};
            MoveAllSmooth(level1, 200);
            float tilt_l[9] = {110, 90, 110, 70, 70, 80, 65, 65, 90};
            MoveAllSmooth(tilt_l, 220);
            float step_r[9] = {70, 80, 65, 70, 110, 90, 110, 65, 90};
            MoveAllSmooth(step_r, 260);
        }
        StandNeutral(350);
        is_busy_action_ = false;
    }

    void Turn(bool left, int count = 2) {
        is_busy_action_ = true;
        float dir = left ? 1.0f : -1.0f;
        for (int i = 0; i < count; i++) {
            float p1[9] = {90, 90, static_cast<float>(90 + 25 * dir), 112, 90, 90, static_cast<float>(90 + 25 * dir), 108, static_cast<float>(90 + 20 * dir)};
            float p2[9] = {90, 90, 90, 90, 90, 90, 90, 90, 90};
            MoveAllSmooth(p1, 280);
            MoveAllSmooth(p2, 280);
        }
        is_busy_action_ = false;
    }

    void Dance() {
        is_busy_action_ = true;
        for (int i = 0; i < 3; i++) {
            float d1[9] = {30, 50, 75, 110, 150, 130, 75, 110, 60};
            float d2[9] = {150, 130, 105, 70, 30, 50, 105, 70, 120};
            MoveAllSmooth(d1, 300);
            MoveAllSmooth(d2, 300);
        }
        StandNeutral(350);
        is_busy_action_ = false;
    }

    void NodHead(bool yes) {
        is_busy_action_ = true;
        if (yes) {
            for (int i = 0; i < 2; i++) {
                float p1[9] = {80, 80, 90, 102, 100, 100, 90, 78, 90};
                float p2[9] = {90, 90, 90, 90, 90, 90, 90, 90, 90};
                MoveAllSmooth(p1, 220);
                MoveAllSmooth(p2, 220);
            }
        } else {
            for (int i = 0; i < 2; i++) {
                SetSingleJoint(8, 55, 180);
                SetSingleJoint(8, 125, 180);
            }
            SetSingleJoint(8, 90, 180);
        }
        is_busy_action_ = false;
    }

    void RunModeBehaviorLoop() {
        int tick = 0;
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(350));
            if (is_busy_action_ || !pca_dev_) continue;

            auto state = Application::GetInstance().GetDeviceState();
            tick++;

            if (state == kDeviceStateSpeaking) {
                float r_sh = 75 + (tick % 3) * 10;
                float l_sh = 105 - ((tick + 1) % 3) * 10;
                float neck = 80 + (tick % 4) * 7;
                float talk_pose[9] = {r_sh, 75, 90, 90, l_sh, 105, 90, 90, neck};
                MoveAllSmooth(talk_pose, 300);
            } else if (state == kDeviceStateListening) {
                float listen_pose[9] = {82, 75, 90, 90, 98, 105, 90, 90, 108};
                MoveAllSmooth(listen_pose, 350);
            } else if (state == kDeviceStateIdle && (tick % 25 == 0)) {
                float look = (tick % 50 == 0) ? 65.0f : 115.0f;
                SetSingleJoint(8, static_cast<int>(look), 400);
                vTaskDelay(pdMS_TO_TICKS(600));
                SetSingleJoint(8, 90, 400);
            }
        }
    }
};

class CompactWifiBoard : public WifiBoard {
private:
    i2c_master_bus_handle_t display_i2c_bus_;
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;
    Display* display_ = nullptr;
    Button boot_button_;
    Button touch_button_;
    Button volume_up_button_;
    Button volume_down_button_;
    HumanoidRobotController robot_;

    void InitializeDisplayI2c() {
        i2c_master_bus_config_t bus_config = {
            .i2c_port = (i2c_port_t)0,
            .sda_io_num = DISPLAY_SDA_PIN,
            .scl_io_num = DISPLAY_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &display_i2c_bus_));
    }

    void InitializeSsd1306Display() {
        esp_lcd_panel_io_i2c_config_t io_config = {
            .dev_addr = 0x3C,
            .scl_speed_hz = 400 * 1000,
            .control_phase_bytes = 1,
            .dc_bit_offset = 6,
            .lcd_cmd_bits = 8,
            .lcd_param_bits = 8,
            .on_color_trans_done = nullptr,
            .user_ctx = nullptr,
            .flags = {
                .dc_low_on_data = 0,
                .disable_control_phase = 0,
            },
        };

        ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(display_i2c_bus_, &io_config, &panel_io_));

        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.bits_per_pixel = 1;

// trigger build
