/*
  Robot con ESP32-CAM: video en vivo + control desde el celular (versión I2C)
  La ESP32-CAM (AI-Thinker) sirve la página web y el video, y envía las órdenes
  por I2C a una ESP32-C3 Super Mini que maneja los motores (MX1508) y los servos
  del brazo. El sketch de la C3 es robot_c3_motores.ino.
  Compatible con ESP32 Arduino core 3.x.

  CONEXIONES ESP32-CAM -> ESP32-C3 Super Mini
    GPIO 13 (SDA) -> GPIO 4 (SDA) de la C3
    GPIO 14 (SCL) -> GPIO 5 (SCL) de la C3
    GND           -> GND (masa común)
    Pull-ups I2C: 4.7 kOhm de SDA a 3.3 V y de SCL a 3.3 V (un solo par en todo el bus)
  El LED flash integrado (GPIO 4 de la ESP32-CAM) funciona y el Monitor Serie está libre.

  ALIMENTACIÓN
    Batería 2x18650 -> regulador 5 V (por ejemplo MP1584) -> ESP32-CAM, ESP32-C3 y servos.
    MX1508 directo a la batería. Todos los GND unidos.

  ARDUINO IDE
    Placa: "AI Thinker ESP32-CAM"
    PSRAM: Enabled
    Partition Scheme: "Huge APP (3MB No OTA/1MB SPIFFS)"

  USO
    1) Sube este sketch (GPIO0 a GND para programar, quítalo y reinicia).
    2) Conecta el celular a la WiFi "RobotCAM" (clave 12345678).
    3) Abre en el navegador: http://192.168.4.1
*/

#include "esp_camera.h"
#include <WiFi.h>
#include <Wire.h>
#include "esp_http_server.h"

// ================== CONFIGURACIÓN ==================
// WiFi: true = el robot crea su propia red (no necesita router)
//       false = se conecta a tu router (la IP sale por el Monitor Serie)
const bool USAR_AP = true;
const char* AP_SSID  = "RobotCAM";
const char* AP_PASS  = "12345678";      // mínimo 8 caracteres
const char* STA_SSID = "TU_WIFI";
const char* STA_PASS = "TU_CLAVE";

// I2C hacia la ESP32-C3 (la ESP32-CAM es la maestra)
const uint8_t  C3_ADDR  = 0x30;         // debe coincidir con I2C_ADDR de la C3
const int      PIN_SDA  = 13;
const int      PIN_SCL  = 14;
const uint32_t I2C_FREQ = 100000;

// LED flash integrado
const int PIN_FLASH = 4;
const int CH_FLASH  = 10;               // canal LEDC del grupo de baja velocidad (el 0 lo usa la cámara)

// Imagen (cambia a true si se ve al revés)
const bool VOLTEAR_VERTICAL = false;
const bool ESPEJO_HORIZONTAL = false;

// Pines de la cámara AI-Thinker
#define PWDN_GPIO_NUM   32
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM    0
#define SIOD_GPIO_NUM   26
#define SIOC_GPIO_NUM   27
#define Y9_GPIO_NUM     35
#define Y8_GPIO_NUM     34
#define Y7_GPIO_NUM     39
#define Y6_GPIO_NUM     36
#define Y5_GPIO_NUM     21
#define Y4_GPIO_NUM     19
#define Y3_GPIO_NUM     18
#define Y2_GPIO_NUM      5
#define VSYNC_GPIO_NUM  25
#define HREF_GPIO_NUM   23
#define PCLK_GPIO_NUM   22

// ================== ESTADO ==================
volatile int velocidad = 200;           // 0-255, se envía a la C3 con cada comando

httpd_handle_t servidorWeb = NULL;
httpd_handle_t servidorStream = NULL;

// ================== PÁGINA WEB ==================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>Robot CAM</title>
<style>
body{margin:0;background:#111;color:#eee;font-family:sans-serif;text-align:center;
     touch-action:none;user-select:none;-webkit-user-select:none}
#cam{width:100%;max-width:640px;max-height:38vh;object-fit:contain;background:#000;display:block;margin:0 auto}
.controles{display:flex;justify-content:center;align-items:center;gap:16px;margin:8px 0}
.pad{display:grid;grid-template-columns:repeat(3,54px);gap:6px}
.pad button{height:54px;font-size:22px;border:0;border-radius:12px;background:#2a7;color:#fff}
.pad button.stop{background:#c33}
.pad button:active{filter:brightness(1.5)}
.row{margin:6px 10px;font-size:14px}
#brazo{position:relative;width:150px;height:150px;background:#222;
       border:2px solid #555;border-radius:12px;touch-action:none}
#punto{position:absolute;left:50%;top:50%;width:24px;height:24px;margin:-12px 0 0 -12px;
       border-radius:50%;background:#39f;pointer-events:none}
#centrar{margin-top:6px;padding:6px 12px;font-size:13px;border:0;border-radius:8px;background:#555;color:#fff}
input[type=range]{width:55%;vertical-align:middle}
</style></head><body>
<img id="cam" alt="camara">
<div class="controles">
  <div class="pad">
    <button data-c="Q">&#8598;</button><button data-c="F">&#8593;</button><button data-c="E">&#8599;</button>
    <button data-c="L">&#8592;</button><button data-c="S" class="stop">&#9632;</button><button data-c="R">&#8594;</button>
    <button data-c="Z">&#8601;</button><button data-c="B">&#8595;</button><button data-c="C">&#8600;</button>
  </div>
  <div>
    <div id="brazo"><div id="punto"></div></div>
    <button id="centrar">Centrar cámara</button>
  </div>
</div>
<div class="row">Velocidad <input id="vel" type="range" min="80" max="255" value="200"></div>
<div class="row">Luz <input id="luz" type="range" min="0" max="255" value="0"></div>
<script>
const cam=document.getElementById('cam');
cam.src='http://'+location.hostname+':81/stream';
const send=u=>fetch(u).catch(()=>{});
let timer=null;
document.querySelectorAll('.pad button').forEach(b=>{
  const c=b.dataset.c;
  b.addEventListener('pointerdown',e=>{
    e.preventDefault();
    send('/cmd?c='+c);
    clearInterval(timer);
    if(c!=='S') timer=setInterval(()=>send('/cmd?c='+c),120);
  });
  const fin=()=>{ if(timer){clearInterval(timer);timer=null;send('/cmd?c=S');} };
  ['pointerup','pointercancel','pointerleave'].forEach(ev=>b.addEventListener(ev,fin));
});
document.getElementById('vel').oninput=e=>send('/speed?v='+e.target.value);
// Panel del brazo: X = horizontal (0-100), Y = vertical (0-100, arriba = 100)
const brazo=document.getElementById('brazo'), punto=document.getElementById('punto');
let ultimo=0, arrastrando=false;
function enviarBrazo(e,forzar){
  const r=brazo.getBoundingClientRect();
  const x=Math.min(1,Math.max(0,(e.clientX-r.left)/r.width));
  const y=Math.min(1,Math.max(0,(e.clientY-r.top)/r.height));
  punto.style.left=(x*100)+'%'; punto.style.top=(y*100)+'%';
  const ahora=Date.now();
  if(forzar||ahora-ultimo>60){
    ultimo=ahora;
    send('/arm?x='+Math.round(x*100)+'&y='+Math.round((1-y)*100));
  }
}
brazo.addEventListener('pointerdown',e=>{arrastrando=true;brazo.setPointerCapture(e.pointerId);enviarBrazo(e,true);});
brazo.addEventListener('pointermove',e=>{if(arrastrando)enviarBrazo(e,false);});
['pointerup','pointercancel'].forEach(ev=>brazo.addEventListener(ev,e=>{
  if(arrastrando){arrastrando=false;enviarBrazo(e,true);}
}));
document.getElementById('centrar').onclick=()=>{
  punto.style.left='50%'; punto.style.top='50%'; send('/arm?x=50&y=50');
};
document.getElementById('luz').oninput=e=>send('/flash?v='+e.target.value);
</script></body></html>
)rawliteral";

// ================== I2C HACIA LA C3 ==================
// Paquete de 3 bytes: [tipo, dato1, dato2]
//   'M' (mover): dato1 = comando (F B L R Q E Z C S), dato2 = velocidad 0-255
//   'A' (brazo): dato1 = X 0-100 (pan), dato2 = Y 0-100 (tilt)
// Solo se llama desde los manejadores HTTP del servidor web (una sola tarea).
bool enviarC3(char tipo, uint8_t a, uint8_t b) {
  Wire.beginTransmission(C3_ADDR);
  Wire.write((uint8_t)tipo);
  Wire.write(a);
  Wire.write(b);
  bool ok = (Wire.endTransmission() == 0);

  static bool ultimoOk = true;          // avisa solo cuando cambia el estado del enlace
  if (ok != ultimoOk) {
    ultimoOk = ok;
    Serial.println(ok ? "I2C: C3 conectada"
                      : "I2C: la C3 no responde (revisa SDA, SCL, GND y pull-ups)");
  }
  return ok;
}

// ================== SERVIDOR HTTP ==================
static bool leerParam(httpd_req_t *req, const char* clave, char* out, size_t n) {
  char q[64];
  if (httpd_req_get_url_query_str(req, q, sizeof(q)) != ESP_OK) return false;
  return httpd_query_key_value(q, clave, out, n) == ESP_OK;
}

static esp_err_t indexHandler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

// /cmd?c=F  (F B L R Q E Z C S)
static esp_err_t cmdHandler(httpd_req_t *req) {
  char val[8];
  if (leerParam(req, "c", val, sizeof(val))) enviarC3('M', (uint8_t)val[0], (uint8_t)velocidad);
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t speedHandler(httpd_req_t *req) {
  char val[8];
  if (leerParam(req, "v", val, sizeof(val))) velocidad = constrain(atoi(val), 0, 255);
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t flashHandler(httpd_req_t *req) {
  char val[8];
  if (leerParam(req, "v", val, sizeof(val))) ledcWrite(PIN_FLASH, constrain(atoi(val), 0, 255));
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
}

// /arm?x=0-100&y=0-100  (x: izquierda-derecha, y: abajo-arriba)
static esp_err_t armHandler(httpd_req_t *req) {
  char vx[8], vy[8];
  if (leerParam(req, "x", vx, sizeof(vx)) && leerParam(req, "y", vy, sizeof(vy))) {
    enviarC3('A', (uint8_t)constrain(atoi(vx), 0, 100), (uint8_t)constrain(atoi(vy), 0, 100));
  }
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
}

// ---- Streaming MJPEG (puerto 81) ----
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t streamHandler(httpd_req_t *req) {
  esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  char partBuf[64];
  while (true) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      res = ESP_FAIL;
    } else {
      size_t hlen = snprintf(partBuf, sizeof(partBuf), STREAM_PART, (unsigned)fb->len);
      res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
      if (res == ESP_OK) res = httpd_resp_send_chunk(req, partBuf, hlen);
      if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
      esp_camera_fb_return(fb);
    }
    if (res != ESP_OK) break;           // el cliente se desconectó
  }
  return res;
}

void iniciarServidores() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.lru_purge_enable = true;

  httpd_uri_t uriIndex = { .uri = "/",      .method = HTTP_GET, .handler = indexHandler, .user_ctx = NULL };
  httpd_uri_t uriCmd   = { .uri = "/cmd",   .method = HTTP_GET, .handler = cmdHandler,   .user_ctx = NULL };
  httpd_uri_t uriSpeed = { .uri = "/speed", .method = HTTP_GET, .handler = speedHandler, .user_ctx = NULL };
  httpd_uri_t uriArm   = { .uri = "/arm",   .method = HTTP_GET, .handler = armHandler,   .user_ctx = NULL };
  httpd_uri_t uriFlash = { .uri = "/flash", .method = HTTP_GET, .handler = flashHandler, .user_ctx = NULL };

  if (httpd_start(&servidorWeb, &config) == ESP_OK) {
    httpd_register_uri_handler(servidorWeb, &uriIndex);
    httpd_register_uri_handler(servidorWeb, &uriCmd);
    httpd_register_uri_handler(servidorWeb, &uriSpeed);
    httpd_register_uri_handler(servidorWeb, &uriArm);
    httpd_register_uri_handler(servidorWeb, &uriFlash);
  }

  // Servidor aparte para el video, así no bloquea los comandos
  httpd_config_t configStream = HTTPD_DEFAULT_CONFIG();
  configStream.server_port = 81;
  configStream.ctrl_port += 1;

  httpd_uri_t uriStream = { .uri = "/stream", .method = HTTP_GET, .handler = streamHandler, .user_ctx = NULL };
  if (httpd_start(&servidorStream, &configStream) == ESP_OK) {
    httpd_register_uri_handler(servidorStream, &uriStream);
  }
}

// ================== CÁMARA ==================
bool iniciarCamara() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;   // siempre el fotograma más reciente (menos retardo)

  if (psramFound()) {
    config.frame_size = FRAMESIZE_VGA;     // 640x480; baja a FRAMESIZE_QVGA si va lento
    config.jpeg_quality = 12;              // 10 = mejor calidad, 63 = peor
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 15;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Error al iniciar la cámara: 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_vflip(s, VOLTEAR_VERTICAL ? 1 : 0);
    s->set_hmirror(s, ESPEJO_HORIZONTAL ? 1 : 0);
  }
  return true;
}

// Destellos del flash para avisar errores (útil sin el Monitor Serie a mano)
void parpadearFlash(int veces) {
  for (int i = 0; i < veces; i++) {
    ledcWrite(PIN_FLASH, 255); delay(150);
    ledcWrite(PIN_FLASH, 0);   delay(150);
  }
}

// ================== SETUP / LOOP ==================
void setup() {
  Serial.begin(115200);
  Serial.println();

  // Descomenta si el ESP32-CAM se reinicia al arrancar (brownout):
  // #include "soc/soc.h" y "soc/rtc_cntl_reg.h" arriba, y luego:
  // WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  // Lo ideal es arreglar la alimentación en vez de desactivar el detector.

  // LED flash: core 3.x -> ledcAttachChannel(pin, freq, bits, canal)
  ledcAttachChannel(PIN_FLASH, 5000, 8, CH_FLASH);
  ledcWrite(PIN_FLASH, 0);

  if (!iniciarCamara()) {
    Serial.println("Cámara no disponible. Reiniciando...");
    parpadearFlash(5);                     // 5 destellos = la cámara no inició
    delay(1000);
    ESP.restart();
  }

  // I2C maestro (después de la cámara, que usa su propio bus en GPIO26/27)
  Wire.begin(PIN_SDA, PIN_SCL, I2C_FREQ);
  Wire.setTimeOut(30);                     // ms: si la C3 no responde, no se cuelga
  enviarC3('M', 'S', 0);                   // orden inicial: stop (también prueba el enlace)

  if (USAR_AP) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);         // IP fija: 192.168.4.1
    Serial.print("Red WiFi: "); Serial.println(AP_SSID);
    Serial.print("Abre: http://"); Serial.println(WiFi.softAPIP());
  } else {
    WiFi.mode(WIFI_STA);
    WiFi.begin(STA_SSID, STA_PASS);
    Serial.print("Conectando");
    bool destello = false;
    while (WiFi.status() != WL_CONNECTED) { // el flash parpadea mientras conecta
      destello = !destello;
      ledcWrite(PIN_FLASH, destello ? 15 : 0);
      Serial.print(".");
      delay(400);
    }
    ledcWrite(PIN_FLASH, 0);
    Serial.print("\nAbre: http://"); Serial.println(WiFi.localIP());
  }
  WiFi.setSleep(false);                    // menor latencia

  iniciarServidores();
}

void loop() {
  // Todo el trabajo lo hacen los servidores HTTP. El failsafe de los motores
  // está en la ESP32-C3: si dejan de llegar comandos, se detienen solos.
  delay(1000);
}
