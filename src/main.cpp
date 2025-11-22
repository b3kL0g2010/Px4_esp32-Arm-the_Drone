#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
extern "C" {
  #include "common/mavlink.h"
}

// ===== Config =====
#define MAVLINK_UART Serial2
#define MAVLINK_BAUD 921600
#define UART_TX_PIN 17
#define UART_RX_PIN 16

const char* WIFI_SSID = "PX4-ESP32-PWM";
const char* WIFI_PASS = "px4esp32pwm";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ===== Global channel values =====
int last_ch1 = 0, last_ch2 = 0, last_ch3 = 0, last_ch4 = 0;
uint32_t last_override_ms = 0;
uint16_t rc_inputs[8] = {0};

// ===== HTML UI =====
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="utf-8"/>
<title>PX4 PWM Control</title>
<style>
body { background:#0b0f14; color:#e8eef3; font-family:sans-serif; padding:20px; }
.panel { background:#111; padding:15px; margin:15px 0; border-radius:8px; }
label { display:block; margin:12px 0; }
input[type=range] { width:300px; }
button { background:#121820; color:#e8eef3; border:1px solid #203042;
         padding:10px 20px; border-radius:8px; cursor:pointer; margin:5px; }
button:hover { background:#162130; }
#log { background:#222; color:#8affc1; padding:10px; height:150px; overflow:auto; }
</style></head><body>
<h1>PX4 PWM Control</h1>
<div class="panel">
  <h2>Override Sliders</h2>
  <label>CH1 (Roll): <input type="range" id="ch1" min="-1000" max="1000" value="0"></label>
  <label>CH2 (Pitch): <input type="range" id="ch2" min="-1000" max="1000" value="0"></label>
  <label>CH3 (Throttle): <input type="range" id="ch3" min="0" max="1000" value="500"></label>
  <label>CH4 (Yaw): <input type="range" id="ch4" min="-1000" max="1000" value="0"></label>
</div>
<div class="panel">
  <h2>RC Input (from PX4)</h2>
  <div>CH1: <span id="rc1">--</span></div>
  <div>CH2: <span id="rc2">--</span></div>
  <div>CH3: <span id="rc3">--</span></div>
  <div>CH4: <span id="rc4">--</span></div>
</div>
<div class="panel">
  <h2>Controls</h2>
  <button id="arm">ARM</button>
  <button id="disarm">DISARM</button>
</div>
<div class="panel">
  <h2>Logs</h2>
  <div id="log"></div>
</div>
<script>
const ws = new WebSocket(`ws://${location.host}/ws`);
function sendOverride() {
  const ch1 = parseInt(document.getElementById('ch1').value);
  const ch2 = parseInt(document.getElementById('ch2').value);
  const ch3 = parseInt(document.getElementById('ch3').value);
  const ch4 = parseInt(document.getElementById('ch4').value);
  ws.send(JSON.stringify({cmd:'rc', ch1, ch2, ch3, ch4}));
}
['ch1','ch2','ch3','ch4'].forEach(id=>{
  document.getElementById(id).addEventListener('input', sendOverride);
});
document.getElementById('arm').onclick = ()=>ws.send(JSON.stringify({cmd:'arm'}));
document.getElementById('disarm').onclick = ()=>ws.send(JSON.stringify({cmd:'disarm'}));
ws.onmessage = e => {
  const div = document.getElementById('log');
  div.innerHTML += e.data + "<br>";
  div.scrollTop = div.scrollHeight;
};
async function pollTelemetry() {
  try {
    const res = await fetch('/telemetry');
    const t = await res.json();
    document.getElementById('rc1').textContent = t.rc1;
    document.getElementById('rc2').textContent = t.rc2;
    document.getElementById('rc3').textContent = t.rc3;
    document.getElementById('rc4').textContent = t.rc4;
  } catch(e){}
}
setInterval(pollTelemetry, 300);
</script></body></html>
)rawliteral";

// ===== MAVLink send =====
void send_manual_control(int x, int y, int z, int r) {
  mavlink_message_t msg;
  mavlink_manual_control_t mc = {};
  mc.target = 1;
  mc.x = x;
  mc.y = y;
  mc.z = z;
  mc.r = r;
  mc.buttons = 0;
  mavlink_msg_manual_control_encode(255, 200, &msg, &mc);
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  MAVLINK_UART.write(buf, len);
}

void send_arm(bool arm) {
  mavlink_message_t msg;
  mavlink_command_long_t cmd = {};
  cmd.target_system = 1;
  cmd.target_component = 1;
  cmd.command = MAV_CMD_COMPONENT_ARM_DISARM;
  cmd.param1 = arm ? 1.0f : 0.0f;
  mavlink_msg_command_long_encode(255, 200, &msg, &cmd);
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  MAVLINK_UART.write(buf, len);
}

// ===== MAVLink receive =====
void parse_mavlink() {
  while (MAVLINK_UART.available() > 0) {
    uint8_t c = MAVLINK_UART.read();
    static mavlink_message_t msg;
    static mavlink_status_t status;
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      if (msg.msgid == MAVLINK_MSG_ID_RC_CHANNELS) {
        mavlink_rc_channels_t rc;
        mavlink_msg_rc_channels_decode(&msg, &rc);
        rc_inputs[0] = rc.chan1_raw;
        rc_inputs[1] = rc.chan2_raw;
        rc_inputs[2] = rc.chan3_raw;
        rc_inputs[3] = rc.chan4_raw;
      }
    }
  }
}

// ===== WebSocket handler =====
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      String s; s.reserve(len);
      for (size_t i=0;i<len;i++) s += (char)data[i];
      DynamicJsonDocument doc(256);
      DeserializationError err = deserializeJson(doc, s);
      if (!err) {
        const char* cmd = doc["cmd"];
        if (strcmp(cmd,"rc")==0) {
          last_ch1 = doc["ch1"] | 0;
          last_ch2 = doc["ch2"] | 0;
          last_ch3 = doc["ch3"] | 500;
          last_ch4 = doc["ch4"] | 0;
          char msg[128];
          snprintf(msg,sizeof(msg),"Updated sliders ch1=%d ch2=%d ch3=%d ch4=%d",last_ch1,last_ch2,last_ch3,last_ch4);
          client->text(msg);
        } else if (strcmp(cmd,"arm")==0) {
          send_arm(true);
          client->text("ARM command sent");
        } else if (strcmp(cmd,"disarm")==0) {
          send_arm(false);
          client->text("DISARM command sent");
        }
      }
    }
  }
}

String telemetry_json() {
  String s = "{";
  s += "\"rc1\":" + String(rc_inputs[0]) + ",";
  s += "\"rc2\":" + String(rc_inputs[1]) + ",";
  s += "\"rc3\":" + String(rc_inputs[2]) + ",";
  s += "\"rc4\":" + String(rc_inputs[3]);
  s += "}";
  return s;
}

void setup() {
    MAVLINK_UART.begin(MAVLINK_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASS);

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req){
    req->send_P(200, "text/html", index_html);
  });

  server.on("/telemetry", HTTP_GET, [](AsyncWebServerRequest *req){
    req->send(200, "application/json", telemetry_json());
  });

  server.begin();
}

void loop() {
  // Parse incoming MAVLink messages to update RC input telemetry
  parse_mavlink();

  // Continuous MANUAL_CONTROL every 100 ms
  if (millis() - last_override_ms >= 100) {
    last_override_ms = millis();
    send_manual_control(last_ch1, last_ch2, last_ch3, last_ch4);
  }
}
