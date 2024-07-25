#include "driver/twai.h"
#include "xiaomi_cybergear_driver.h"

// Pins used to connect to CAN bus transceiver:
#define RX_PIN 21
#define TX_PIN 22

// Interval:
#define TRANSMIT_RATE_MS 1000
#define POLLING_RATE_MS 1000

// User-adjustable variables
float target_position = 30.0;  // Target position in degrees
float speed_limit = 30.0;     // Speed limit in degrees per second
float current_limit = 5.0;    // Current limit in amperes

static bool driver_installed = false;
unsigned long previousMillis = 0;  // will store last time a message was sent

uint8_t CYBERGEAR_CAN_ID = 0x01;
uint8_t MASTER_CAN_ID = 0x00;
XiaomiCyberGearDriver cybergear = XiaomiCyberGearDriver(CYBERGEAR_CAN_ID, MASTER_CAN_ID);

void setup() {
  // initialize TWAI (CAN) interface to communicate with Xiaomi CyberGear
  cybergear.init_twai(RX_PIN, TX_PIN, /*serial_debug=*/true);

  cybergear.init_motor(MODE_POSITION);
  cybergear.set_limit_speed(speed_limit);
  cybergear.set_limit_current(current_limit);
  cybergear.enable_motor();

  cybergear.set_position_ref(target_position);

  // TWAI driver is now successfully installed and started
  driver_installed = true;
}

static void handle_rx_message(twai_message_t& message) {
  if (((message.identifier & 0xFF00) >> 8) == CYBERGEAR_CAN_ID){
    cybergear.process_message(message);
  }
}

static void check_alerts(){
  uint32_t alerts_triggered;
  twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));
  twai_status_info_t twai_status;
  twai_get_status_info(&twai_status);

  if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
    Serial.println("Alert: TWAI controller has become error passive.");
  }
  if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
    Serial.println("Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
    Serial.printf("Bus error count: %d\n", twai_status.bus_error_count);
  }
  if (alerts_triggered & TWAI_ALERT_TX_FAILED) {
    Serial.println("Alert: The Transmission failed.");
    Serial.printf("TX buffered: %d\t", twai_status.msgs_to_tx);
    Serial.printf("TX error: %d\t", twai_status.tx_error_counter);
    Serial.printf("TX failed: %d\n", twai_status.tx_failed_count);
  }

  if (alerts_triggered & TWAI_ALERT_RX_DATA) {
    twai_message_t message;
    while (twai_receive(&message, 0) == ESP_OK) {
      handle_rx_message(message);
    }
  }
}

void loop() {
  if (!driver_installed) {
    delay(1000);
    return;
  }
  delay(1000);

  static float pos = 0.0;
  static float inc_val = 1;
  pos += inc_val;
  if (pos > 10.0) inc_val = -1;
  if (pos < -10.0) inc_val = 1;

  XiaomiCyberGearMotionCommand cmd;
  cmd.position = pos;
  /* I dont recommend bellow parematers values - the motor is heavily oscillating */
  cmd.torque = 0.5f;
  cmd.kp = 0.1f;
  cmd.kd = 0.001f;

  cybergear.send_motion_control(cmd);

  check_alerts();

  XiaomiCyberGearStatus cybergear_status = cybergear.get_status();
  Serial.printf("Current - POS:%.2f V:%.2f T:%.2f temp:%d\n", 
                cybergear_status.position, cybergear_status.speed, 
                cybergear_status.torque, cybergear_status.temperature);

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= TRANSMIT_RATE_MS) {
    previousMillis = currentMillis;
    cybergear.request_status();
  }

  delay(100);  // Small delay to prevent flooding the serial monitor
}