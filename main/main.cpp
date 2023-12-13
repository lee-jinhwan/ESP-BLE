#include <stdio.h>
#include <cmath>

#include <esp_system.h>
#include <esp_wifi.h>
#include <esp_sleep.h>

#include "RTOS.h"
#include "BLEDevice.h"
#include "BLEScan.h"
#include "driver/gpio.h"

#include "config.h"
#include "irk.h"
#include "ws2812_control.h"

#define DISTANCE(x) std::pow(10.0, (RSSI_TX_POWER - (x)) / (RSSI_N * 10.0))

void setup();

void initialize_BLE();
void initialize_LED();

BLEAdvertisedDevice find_my_device(BLEScanResults &results);
BLEAdvertisedDevice find_door(BLEScanResults &results);
void open(BLEAdvertisedDevice &door);
void light_sleep(uint32_t ms);

void on_LEDs(uint32_t *leds);
void off_LEDs();

static BLEAdvertisedDevice NO_DEVICE;

extern "C" void app_main(void)
{
    setup();

    BLEScan* p_BLEScan;
    p_BLEScan = BLEDevice::getScan();
    p_BLEScan->setActiveScan(true);
    p_BLEScan->setInterval(100);
    p_BLEScan->setWindow(100);

    while (true) {
        printf("Scan start ...\n");
        p_BLEScan->clearResults();
        BLEScanResults scanResult = p_BLEScan->start(SCAN_TIME, false);
        printf("Scan finish ...\n");

        // for (int i = 0; i < scanResult.getCount(); i++) {
        //     BLEAdvertisedDevice device = scanResult.getDevice(i);
        //     printf("Device: %s\n", device.toString().c_str());
        // }

        BLEAdvertisedDevice device = find_my_device(scanResult);
        BLEAdvertisedDevice door = find_door(scanResult);

        if (device == NO_DEVICE) {
            printf("light sleep 5s\n");
            off_LEDs();
            light_sleep(5000);
            continue;
        }

        printf("Device: %s\n", device.toString().c_str());
        double distance = DISTANCE(device.getRSSI());
        printf("distance(%d db) = %lf m\n", device.getRSSI(), distance);

        on_LEDs(NUMBER_TO_LED[(int)distance]);

        if (distance > 7) {
            // on_LEDs(RED_LED);
            light_sleep(500);
            continue;
        }

        if (door == NO_DEVICE) {
            // on_LEDs(YELLOW_LED);
            light_sleep(500);
            continue;
        }

        printf("Door: %s\n", door.toString().c_str());
        open(door);
        // on_LEDs(GREEN_LED);
        light_sleep(10000);
    }
}

void setup() {
    esp_wifi_stop();

    esp_sleep_pd_config(ESP_PD_DOMAIN_MAX,          ESP_PD_OPTION_OFF);
#if SOC_PM_SUPPORT_RTC_PERIPH_PD
    // esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH,   ESP_PD_OPTION_OFF);
    // esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
    // esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
#endif

    initialize_BLE();
    initialize_LED();
}

void initialize_BLE() {
    BLEDevice::init("");
    BLEDevice::setPower(ESP_PWR_LVL_N12, ESP_BLE_PWR_TYPE_DEFAULT); 
    BLEDevice::setPower(ESP_PWR_LVL_N12, ESP_BLE_PWR_TYPE_ADV);
    BLEDevice::setPower(ESP_PWR_LVL_P9 , ESP_BLE_PWR_TYPE_SCAN);
    BLEDevice::stopAdvertising();
}

void initialize_LED() {
    gpio_config_t conf  = {};
    conf.intr_type      = GPIO_INTR_DISABLE;
    conf.mode           = GPIO_MODE_OUTPUT;
    conf.pin_bit_mask   = 1ULL << GPIO_NUM_5;
    conf.pull_down_en   = GPIO_PULLDOWN_DISABLE;
    conf.pull_up_en     = GPIO_PULLUP_DISABLE;
    gpio_config(&conf);
    gpio_set_level(GPIO_NUM_5, 0);

    ws2812_control_init();
}

BLEAdvertisedDevice find_my_device(BLEScanResults &results) {
    BLEAdvertisedDevice minDevice = NO_DEVICE;
    for (int i = 0; i < results.getCount(); i++) {
        BLEAdvertisedDevice device = results.getDevice(i);
        for (int j = 0; j < IRK_LIST_NUMBER; j++) {
            if (btm_ble_addr_resolvable(*(device.getAddress().getNative()), irk[j])) {
                if (minDevice == NO_DEVICE || minDevice.getRSSI() < device.getRSSI()) {
                    minDevice = device;
                }
            }
        }
    }
    return minDevice;
}

BLEAdvertisedDevice find_door(BLEScanResults &results) {
    for (int i = 0; i < results.getCount(); i++) {
        BLEAdvertisedDevice device = results.getDevice(i);
        for (int j = 0; j < DOOR_COUNT; j++) {
            if (device.getName() == DOOR[j]) {
                return device;
            }
        }
    }
    return NO_DEVICE;
}

void open(BLEAdvertisedDevice &door) {
    BLEClient* pClient = BLEDevice::createClient();
    pClient->connect(&door);

    if (!pClient->isConnected()) {
        return;
    }

    BLERemoteService* pRemoteService = pClient->getService(SERVICE_UUID);
    if (pRemoteService != nullptr) {
        BLERemoteCharacteristic* pRemoteCharacteristic = pRemoteService->getCharacteristic(CHAR_UUID);
        if (pRemoteCharacteristic != nullptr) {
            pRemoteCharacteristic->writeValue(DOOR_KEY, DOOR_KEY_LEN, false);
        }
    }

    pClient->disconnect();
    FreeRTOS::sleep(1000);
}

void light_sleep(uint32_t ms) {
    esp_sleep_enable_timer_wakeup(ms * 1000);
    esp_light_sleep_start();
}

void on_LEDs(uint32_t *leds) {
    gpio_set_level(GPIO_NUM_5, 1);
    struct led_state state;
    state.leds[20] = leds[ 0];
    state.leds[15] = leds[ 1];
    state.leds[10] = leds[ 2];
    state.leds[ 5] = leds[ 3];
    state.leds[ 0] = leds[ 4];
    state.leds[21] = leds[ 5];
    state.leds[16] = leds[ 6];
    state.leds[11] = leds[ 7];
    state.leds[ 6] = leds[ 8];
    state.leds[ 1] = leds[ 9];
    state.leds[22] = leds[10];
    state.leds[17] = leds[11];
    state.leds[12] = leds[12];
    state.leds[ 7] = leds[13];
    state.leds[ 2] = leds[14];
    state.leds[23] = leds[15];
    state.leds[18] = leds[16];
    state.leds[13] = leds[17];
    state.leds[ 8] = leds[18];
    state.leds[ 3] = leds[19];
    state.leds[24] = leds[20];
    state.leds[19] = leds[21];
    state.leds[14] = leds[22];
    state.leds[ 9] = leds[23];
    state.leds[ 4] = leds[24];
    ws2812_write_leds(state);
}

void off_LEDs() {
    gpio_set_level(GPIO_NUM_5, 0);
}
