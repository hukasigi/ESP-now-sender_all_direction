#include "SerialConsole.h"
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

double wheel_v1 = 0.0, wheel_v2 = 0.0, wheel_v3 = 0.0;
double wheel_target_1 = 0.0, wheel_target_2 = 0.0, wheel_target_3 = 0.0;

int16_t x_mm  = 0;
int16_t y_mm  = 0;
int16_t theta = 0;

int16_t theta_mrad        = 0; // 受信姿勢[mrad] ミリにして送受信することで、動く
int     target_x_mm       = 0;
int     target_y_mm       = 0;
int16_t target_theta_mrad = 0; // 送信目標姿勢[mrad]
bool    moving            = false;

constexpr double  INTEGRAL_MAX          = 10.0; // int8_t -> double
constexpr int16_t PWM_LIMIT             = 255;
constexpr double  MAX_VX_CMD_MM_S       = 300.0;
constexpr double  MAX_VY_CMD_MM_S       = 300.0; // 追加
constexpr double  NEAR_SPEED_LIMIT_MM_S = 80.0;
constexpr double  POSITION_DEADBAND_MM  = 1.0;
constexpr double  SPEED_DEADBAND_MM_S   = 2.0;
constexpr double  MAX_OMEGA_CMD_RAD_S   = 2.0;  // 追加: 角速度上限[rad/s]
constexpr double  THETA_DEADBAND_RAD    = 0.03; // 追加: 約1.7deg
constexpr double  DEG2RAD               = PI / 180.0;

const double RAD2DEG = 360 / (2 * PI);

constexpr double USER_SIGN = -1.0; // fを前進にしたい場合 -1.0（現状の逆転補正）

void processCommand(const char* input);

double wrapPi(double rad) {
    while (rad > PI)
        rad -= 2.0 * PI;
    while (rad < -PI)
        rad += 2.0 * PI;
    return rad;
}

SerialConsole<32> console(processCommand);

void processCommand(const char* input) {
    if (!input || input[0] == '\0') return;

    if (strcasecmp(input, "stop") == 0) {
        moving         = false;
        wheel_target_1 = wheel_target_2 = wheel_target_3 = 0.0;
        Serial.println("stopped");
        return;
    }

    const double raw  = atof(input + 1);
    const double dist = fabs(raw);

    switch (input[0]) {
    case 'm':
    case 'M': {
        // m<dx>,<dy>,<dtheta_deg> 例: m100,-50,30
        double dx_cmd = 0.0, dy_cmd = 0.0, dtheta_deg = 0.0;
        if (sscanf(input + 1, " %lf%*[ ,]%lf%*[ ,]%lf", &dx_cmd, &dy_cmd, &dtheta_deg) != 3) {
            Serial.println("usage: m<dx>,<dy>,<dth_deg>  e.g. m100,-50,30");
            return;
        }

        target_x_mm       = (int16_t)lround((double)x_mm + USER_SIGN * dx_cmd);
        target_y_mm       = (int16_t)lround((double)y_mm + USER_SIGN * dy_cmd);
        double theta_rad  = theta_mrad / 1000.0;
        double tgt_rad    = wrapPi(theta_rad + dtheta_deg * DEG2RAD);
        target_theta_mrad = (int16_t)lround(tgt_rad * 1000.0);

        moving = true;

        Serial.printf("move dx=%.1f dy=%.1f dth=%.1f deg\n", dx_cmd, dy_cmd, dtheta_deg);
        break;
    }

    default: Serial.println("unknown command: use r/l/f/b/t or m<dx>,<dy>,<dth_deg>, or stop"); break;
    }
}

// 送信先の情報を保持する構造体
// ブロードキャスト宛先を入れるために使っている
esp_now_peer_info_t slave;

// 送信コールバック
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
    // 送信先のmacアドレス
    char macStr[18];
    // snprintf、macアドレス6バイトを人間が読める形式に変換する
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3],
             mac_addr[4], mac_addr[5]);
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// 受信コールバック
void OnDataRecv(const uint8_t* mac_addr, const uint8_t* data, int data_len) {
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3],
             mac_addr[4], mac_addr[5]);

    if (data_len >= 6) {
        x_mm       = (int16_t)((data[0] << 8) | data[1]);
        y_mm       = (int16_t)((data[2] << 8) | data[3]);
        theta_mrad = (int16_t)((data[4] << 8) | data[5]);
    }

    Serial.println();
}

void setup() {
    Serial.begin(115200);

    // ESP-NOW初期化
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() == ESP_OK) {
        Serial.println("ESPNow Init Success");
    } else {
        Serial.println("ESPNow Init Failed");
        ESP.restart();
    }

    // マルチキャスト用Slave登録
    // 構造体を0クリア
    memset(&slave, 0, sizeof(slave));

    // 08:D1:F9:37:42:0C
    // 送信先MACアドレスを設定
    uint8_t peerAddress[] = {0x08, 0xD1, 0xF9, 0x37, 0x42, 0x0C};
    memcpy(slave.peer_addr, peerAddress, 6);

    // チャンネル指定(0はチャンネル自動)
    slave.channel = 0;
    // 暗号化しない
    slave.encrypt = false;

    esp_err_t addStatus = esp_now_add_peer(&slave);
    if (addStatus == ESP_OK) {
        // Pair success
        Serial.println("Pair success");
    }
    // ESP-NOWコールバック登録
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
    // SerialConsole を使用
    console.handleInput();

    uint8_t data[6] = {(uint8_t)((target_x_mm >> 8) & 0xFF),       (uint8_t)(target_x_mm & 0xFF),
                       (uint8_t)((target_y_mm >> 8) & 0xFF),       (uint8_t)(target_y_mm & 0xFF),
                       (uint8_t)((target_theta_mrad >> 8) & 0xFF), (uint8_t)(target_theta_mrad & 0xFF)};

    esp_err_t result = esp_now_send(slave.peer_addr, data, sizeof(data));

    delay(50);
}
