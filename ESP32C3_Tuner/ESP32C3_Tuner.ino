/*
 * ESP32-C3 <-> STM32F401 (LineFollowerRobot) - ponte de calibracao
 *
 * - Cria uma rede Wi-Fi (AP) com uma pagina de calibracao: PID, velocidade base,
 *   STOP, RUN, teste de motores, leitura/calibracao dos sensores.
 * - Tambem repassa o Serial USB: digite comandos no Serial Monitor (115200, "Newline")
 *   e eles vao direto pro STM32. Tudo que o STM32 manda aparece no Serial Monitor.
 *
 * Ligacao (3.3 V dos dois lados, pode ligar direto):
 *   ESP32-C3 GPIO7 (TX)  ->  STM32 PA10 (USART1_RX)
 *   ESP32-C3 GPIO6 (RX)  <-  STM32 PA9  (USART1_TX)
 *   GND                  --  GND   (obrigatorio!)
 *
 * Arduino IDE: placa "ESP32C3 Dev Module" (ou a da sua placa, ex. "Nologo ESP32C3 Super Mini").
 * Para usar o Serial Monitor pela USB nativa: Tools > USB CDC On Boot > Enabled.
 * Nao precisa de biblioteca extra (WiFi, WebServer e Preferences vem com o core ESP32).
 *
 * Protocolo: veja Core/Src/comm.c no projeto do STM32 ou PINOUT.md.
 */

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

// ---------------- Configuracao ----------------
#define STM_RX_PIN      6         // recebe do PA9 do STM32
#define STM_TX_PIN      7         // envia para o PA10 do STM32
#define STM_BAUD        115200

const char *AP_SSID = "LineFollower";
const char *AP_PASS = "seguidor123";   // minimo 8 caracteres

#define REPLY_TIMEOUT_MS   200
#define AUTO_RESTORE_ON_HELLO  true    // reenviar PID/SPEED salvos quando o STM32 reiniciar

// "Parar se a pagina desconectar": a pagina manda /hb a cada 300 ms. Se parar de chegar
// por WEB_HB_TIMEOUT_MS, o ESP manda STOP. Enquanto isso o ESP manda HB ao STM32, que
// tem o proprio watchdog (WD) e para sozinho se o ESP travar ou o fio soltar.
#define WEB_HB_TIMEOUT_MS  2500
#define STM_HB_PERIOD_MS   300
#define STM_WD_MS          1000
// ----------------------------------------------

HardwareSerial &Stm = Serial1;
WebServer server(80);
Preferences prefs;

// Log circular das ultimas linhas (mostrado na pagina)
#define LOG_LINES 40
String logBuf[LOG_LINES];
int logHead = 0;
uint32_t logSeq = 0;

bool restorePending = false;   // STM32 reiniciou: reenviar config quando o ESP estiver livre
bool hbArmed = false;          // watchdog da pagina ligado
uint32_t lastWebHb = 0;
uint32_t lastStmHb = 0;

String stmLine;   // linha sendo montada vinda do STM32
String usbLine;   // linha sendo montada vinda do Serial USB

void addLog(const String &s) {
  logBuf[logHead] = s;
  logHead = (logHead + 1) % LOG_LINES;
  logSeq++;
}

void restoreSaved();

// Trata uma linha completa vinda do STM32
void onStmLine(const String &line) {
  Serial.print("< ");
  Serial.println(line);
  addLog("< " + line);
  // Nao reenvia aqui: se estivermos dentro de sendCmd(), a resposta "OK PID" seria
  // confundida com a resposta do comando que esta esperando. Faz no loop().
  if (line.startsWith("HELLO")) {
    restorePending = true;
  }
}

// Le o que tiver na UART do STM32. Retorna a primeira linha completa (ou "").
String pollStm() {
  while (Stm.available()) {
    char c = (char)Stm.read();
    if (c == '\r') continue;
    if (c == '\n') {
      String line = stmLine;
      stmLine = "";
      if (line.length() > 0) return line;
      continue;
    }
    if (stmLine.length() < 200) stmLine += c;
  }
  return "";
}

void sendRaw(const String &cmd) {
  Stm.print(cmd);
  Stm.print('\n');
  Serial.print("> ");
  Serial.println(cmd);
  addLog("> " + cmd);
}

// Envia um comando e espera a resposta (ignora eventos assincronos "EVT"/"HELLO").
String sendCmd(const String &cmd, uint32_t timeoutMs = REPLY_TIMEOUT_MS) {
  // descarta/loga o que ja estava pendente
  for (String l = pollStm(); l.length(); l = pollStm()) onStmLine(l);

  sendRaw(cmd);
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    String l = pollStm();
    if (l.length()) {
      onStmLine(l);
      if (!l.startsWith("EVT") && !l.startsWith("HELLO")) return l;
    }
    delay(1);
  }
  addLog("! sem resposta: " + cmd);
  return "TIMEOUT";
}

void restoreSaved() {
  if (!prefs.isKey("kp")) return;
  String pidCmd = "PID " + String(prefs.getFloat("kp"), 5) + " " +
                  String(prefs.getFloat("ki"), 5) + " " +
                  String(prefs.getFloat("kd"), 5);
  sendRaw(pidCmd);
  int speed = prefs.getInt("speed", 595);
  if (speed > 1000) speed = speed * 1000 / 4199;   // valor salvo na escala antiga (duty 0..4199)
  sendRaw("SPEED " + String(speed));
}

// ---------------- Pagina web ----------------
const char PAGE[] PROGMEM = R"HTML(
<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>LineFollower Tuner</title>
<style>
body{font-family:system-ui,sans-serif;margin:0;padding:12px;background:#111;color:#eee;max-width:640px;margin:auto}
h2{font-size:15px;margin:18px 0 6px;color:#9cf}
.row{display:flex;gap:6px;flex-wrap:wrap;align-items:center;margin:4px 0}
input{width:84px;padding:8px;font-size:16px;background:#222;color:#fff;border:1px solid #444;border-radius:6px}
input[type=checkbox]{width:auto;transform:scale(1.4);margin-right:8px}
button{padding:10px 14px;font-size:15px;border:0;border-radius:8px;background:#2a5;color:#fff}
button.gray{background:#444}
#stop{width:100%;padding:22px;font-size:26px;font-weight:bold;background:#d22;margin-bottom:8px}
#run{width:100%;padding:12px;background:#2a5}
#reply{font-family:monospace;background:#000;padding:8px;border-radius:6px;min-height:1.4em;word-break:break-all}
#log{font-family:monospace;font-size:12px;background:#000;padding:8px;height:180px;overflow:auto;white-space:pre;border-radius:6px}
.bars{display:flex;gap:4px;height:90px;align-items:flex-end;background:#000;padding:6px;border-radius:6px}
.bar{flex:1;background:#39f;position:relative}
.bar span{position:absolute;top:-16px;width:100%;text-align:center;font-size:10px}
.rawrow{display:flex;gap:4px;padding:0 6px;font-size:10px;color:#888}
.rawrow div{flex:1;text-align:center}
.track{position:relative;height:22px;background:#000;border-radius:6px;margin-top:6px}
.track .mid{position:absolute;left:50%;top:0;bottom:0;width:1px;background:#555}
#mark{position:absolute;top:3px;width:16px;height:16px;margin-left:-8px;border-radius:50%;background:#2c5}
table{border-collapse:collapse;font-family:monospace;font-size:12px;width:100%}
td,th{border:1px solid #333;padding:3px;text-align:center}
.lost{color:#f55;font-weight:bold}
label{font-size:13px;color:#aaa}
</style></head><body>
<button id="stop" onclick="cmd('STOP')">STOP</button>
<button id="run" onclick="cmd('RUN')">RUN (seguir linha)</button>
<div class="row"><label style="color:#eee"><input type="checkbox" id="hb" checked>Parar o robo se esta pagina desconectar</label></div>
<div id="reply">-</div>

<h2>PID</h2>
<div class="row">
 <label>Kp</label><input id="kp" type="number" step="any">
 <label>Ki</label><input id="ki" type="number" step="any">
 <label>Kd</label><input id="kd" type="number" step="any">
</div>
<div class="row">
 <button onclick="cmd('PID '+v('kp')+' '+v('ki')+' '+v('kd'))">Enviar PID</button>
 <button class="gray" onclick="status()">Ler do robo</button>
 <button class="gray" onclick="save()">Salvar no ESP</button>
</div>

<h2>Velocidade base</h2>
<div class="row">
 <input id="spd" type="number" min="0" max="100" step="1"><label>% (0..100)</label>
 <button onclick="cmd('SPEED '+pct2duty(v('spd')))">Enviar</button>
</div>

<h2>Teste de motores</h2>
<div class="row">
 <label>Esq %</label><input id="ml" type="number" value="40" min="-100" max="100">
 <label>Dir %</label><input id="mr" type="number" value="40" min="-100" max="100">
 <label>ms</label><input id="mt" type="number" value="1000" min="1" max="10000">
</div>
<div class="row">
 <button onclick="motor(v('ml'),v('mr'))">Testar</button>
 <button class="gray" onclick="motor(v('ml'),0)">So esquerdo</button>
 <button class="gray" onclick="motor(0,v('mr'))">So direito</button>
 <button class="gray" onclick="motor(-v('ml'),-v('mr'))">Reverso</button>
</div>

<h2>Sensores (valor calibrado 0..1000)</h2>
<div class="bars" id="bars"></div>
<div class="rawrow" id="rawrow"></div>
<div class="track"><div class="mid"></div><div id="mark"></div></div>
<div class="row"><span id="pos">POS -</span></div>
<div class="row">
 <button class="gray" onclick="sens()">Ler</button>
 <button class="gray" id="live" onclick="toggleLive()">Ao vivo: OFF</button>
 <button onclick="cmd('CAL START')">Calibrar: iniciar</button>
 <button onclick="calStop()">Calibrar: terminar</button>
 <button class="gray" onclick="calRead()">Ver calibracao</button>
</div>
<table id="caltab"></table>
<label>Calibrar: iniciar, arrastar o robo devagar sobre a linha (todos os sensores veem preto e branco), terminar.</label>

<h2>Comando manual</h2>
<div class="row"><input id="raw" style="flex:1" placeholder="ex: PING"><button onclick="cmd(v('raw'))">Enviar</button></div>
<h2>Log</h2>
<div id="log"></div>

<script>
const MAX=1000;   // escala U do STM32: -1000..1000
function v(id){return document.getElementById(id).value.trim()}
function pct2duty(p){return Math.round(Math.max(-100,Math.min(100,+p||0))*MAX/100)}
async function cmd(c){
  if(!c)return '';
  const r=await fetch('/cmd?c='+encodeURIComponent(c),{cache:'no-store'}).then(r=>r.text()).catch(e=>'ERRO '+e);
  document.getElementById('reply').textContent=c+'  ->  '+r;
  return r;
}
function motor(l,r){cmd('MOTOR '+pct2duty(l)+' '+pct2duty(r)+' '+(parseInt(v('mt'))||1000))}
async function status(){
  const r=await cmd('STATUS'); const t=r.split(' ');
  const g=k=>{const i=t.indexOf(k);return i>=0?t[i+1]:''};
  if(t[0]!=='STATUS')return;
  kp.value=+g('KP'); ki.value=+g('KI'); kd.value=+g('KD');
  spd.value=Math.round(+g('SPEED')*100/MAX);
}
async function save(){
  const r=await fetch('/save?kp='+v('kp')+'&ki='+v('ki')+'&kd='+v('kd')+'&speed='+pct2duty(v('spd'))).then(r=>r.text());
  document.getElementById('reply').textContent=r;
}
const bars=document.getElementById('bars'), rawrow=document.getElementById('rawrow');
for(let i=0;i<8;i++){bars.innerHTML+='<div class="bar" id="b'+i+'"><span></span></div>';rawrow.innerHTML+='<div id="r'+i+'">-</div>'}
// resposta: SENS R r0..r7 N n0..n7 POS p LINE 0|1
async function sens(){
  const r=await fetch('/cmd?c=SENS&t='+Date.now(),{cache:'no-store'}).then(r=>r.text()).catch(()=>'');
  const t=r.split(' '); if(t[0]!=='SENS'||t[1]!=='R')return;
  const iN=t.indexOf('N'), iP=t.indexOf('POS'), iL=t.indexOf('LINE');
  for(let i=0;i<8;i++){
    const raw=+t[2+i], n=+t[iN+1+i], b=document.getElementById('b'+i);
    b.style.height=(n/10)+'%'; b.firstChild.textContent=n;
    b.style.background=n>200?'#39f':'#246';        // >200 = sensor ve linha (LINE_DETECT_THRESHOLD)
    document.getElementById('r'+i).textContent=raw;
  }
  const pos=+t[iP+1], seen=t[iL+1]==='1';
  const m=document.getElementById('mark'); m.style.left=(pos/7000*100)+'%'; m.style.background=seen?'#2c5':'#f55';
  const el=document.getElementById('pos');
  el.innerHTML='POS '+pos+'  erro '+(3500-pos)+'  (centro = 3500)'+(seen?'':'  <span class="lost">LINHA PERDIDA - usando ultimo lado</span>');
}
// resposta: CAL MIN m0..m7 MAX M0..M7 [WARN]
function showCal(r){
  const t=r.split(' '); if(t[0]!=='CAL'||t[1]!=='MIN')return;
  const iM=t.indexOf('MAX');
  let h='<tr><th></th>'+[0,1,2,3,4,5,6,7].map(i=>'<th>S'+i+'</th>').join('')+'</tr>';
  const mn=[],mx=[];
  for(let i=0;i<8;i++){mn.push(+t[2+i]);mx.push(+t[iM+1+i])}
  h+='<tr><td>min</td>'+mn.map(x=>'<td>'+x+'</td>').join('')+'</tr>';
  h+='<tr><td>max</td>'+mx.map(x=>'<td>'+x+'</td>').join('')+'</tr>';
  h+='<tr><td>faixa</td>'+mx.map((x,i)=>{const d=x-mn[i];return '<td'+(d<200?' class="lost"':'')+'>'+d+'</td>'}).join('')+'</tr>';
  // REJ = sensores que calibraram mal e ficaram com a calibracao anterior
  const iR=t.indexOf('REJ'), rej=iR>=0?t.slice(iR+1).map(Number):[];
  if(rej.length){
    h+='<tr><td>status</td>'+[0,1,2,3,4,5,6,7].map(i=>rej.includes(i)?'<td class="lost">REJ</td>':'<td>ok</td>').join('')+'</tr>';
    h+='<tr><td colspan="9" class="lost">Sensores '+rej.map(i=>'S'+i).join(', ')+' nao viram linha e fundo: mantida a calibracao anterior. Calibre de novo passando por cima deles.</td></tr>';
  }
  document.getElementById('caltab').innerHTML=h;
}
// heartbeat da pagina (so manda quando ligado, ou uma vez ao desligar)
let hbPrev=null;
function hbTick(){
  const on=document.getElementById('hb').checked;
  if(on||hbPrev!==on) fetch('/hb?on='+(on?1:0),{cache:'no-store'}).catch(()=>{});
  hbPrev=on;
}
setInterval(hbTick,300);
async function calStop(){showCal(await cmd('CAL STOP'))}
async function calRead(){showCal(await cmd('CAL?'))}
let live=false;
async function liveLoop(){ while(live){ await sens(); await new Promise(r=>setTimeout(r,150)); } }
function toggleLive(){
  live=!live; if(live) liveLoop();
  document.getElementById('live').textContent='Ao vivo: '+(live?'ON':'OFF');
}
let seq=-1;
async function pollLog(){
  const r=await fetch('/log?since='+seq).then(r=>r.json()).catch(()=>null);
  if(r&&r.seq!==seq){seq=r.seq;const el=document.getElementById('log');el.textContent=r.lines.join('\n');el.scrollTop=1e9}
}
setInterval(pollLog,700);
document.addEventListener('keydown',e=>{if(e.code==='Space'&&e.target.tagName!=='INPUT'){e.preventDefault();cmd('STOP')}});
status();
calRead();
</script></body></html>
)HTML";

String jsonEscape(const String &s) {
  String o;
  for (char c : s) {
    if (c == '"' || c == '\\') o += '\\';
    if ((uint8_t)c >= 0x20) o += c;
  }
  return o;
}

void handleRoot() { server.send_P(200, "text/html", PAGE); }

void handleCmd() {
  String c = server.arg("c");
  c.trim();
  if (c.length() == 0 || c.length() > 90) {
    server.send(400, "text/plain", "ERR comando vazio/longo");
    return;
  }
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", sendCmd(c));
}

// Le um numero de um argumento; falha se vazio, com lixo, NaN/inf ou fora de [lo, hi]
bool argNumber(const char *name, float lo, float hi, float &out) {
  String s = server.arg(name);
  s.trim();
  if (s.length() == 0) return false;
  char *end;
  out = strtof(s.c_str(), &end);
  if (*end != '\0') return false;
  if (out != out) return false;            // NaN
  return out >= lo && out <= hi;           // tambem pega inf
}

void handleSave() {
  float kp, ki, kd, speed;
  if (!argNumber("kp", -10000, 10000, kp) || !argNumber("ki", -10000, 10000, ki) ||
      !argNumber("kd", -10000, 10000, kd) || !argNumber("speed", 1, 1000, speed)) {
    server.send(400, "text/plain", "ERR nao salvo: preencha Kp, Ki, Kd e velocidade (1..100%) com numeros validos");
    return;
  }
  prefs.putFloat("kp", kp);
  prefs.putFloat("ki", ki);
  prefs.putFloat("kd", kd);
  prefs.putInt("speed", (int)speed);
  server.send(200, "text/plain", "Salvo no ESP (sera reenviado quando o STM32 reiniciar)");
}

void handleHb() {
  bool on = server.arg("on") == "1";
  lastWebHb = millis();
  if (on && !hbArmed) {
    hbArmed = true;
    sendRaw("WD " + String(STM_WD_MS));
  } else if (!on && hbArmed) {
    hbArmed = false;
    sendRaw("WD 0");
  }
  server.send(200, "text/plain", "ok");
}

void handleLog() {
  String out = "{\"seq\":" + String(logSeq) + ",\"lines\":[";
  bool first = true;
  for (int i = 0; i < LOG_LINES; i++) {
    const String &l = logBuf[(logHead + i) % LOG_LINES];
    if (l.length() == 0) continue;
    if (!first) out += ',';
    out += '"';
    out += jsonEscape(l);
    out += '"';
    first = false;
  }
  out += "]}";
  server.send(200, "application/json", out);
}

void setup() {
  Serial.begin(115200);
  Stm.begin(STM_BAUD, SERIAL_8N1, STM_RX_PIN, STM_TX_PIN);
  prefs.begin("lftuner", false);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  server.on("/", handleRoot);
  server.on("/cmd", handleCmd);
  server.on("/save", handleSave);
  server.on("/log", handleLog);
  server.on("/hb", handleHb);
  server.begin();

  delay(300);
  Serial.println();
  Serial.println("LineFollower Tuner pronto.");
  Serial.print("Wi-Fi: ");
  Serial.print(AP_SSID);
  Serial.print("  senha: ");
  Serial.print(AP_PASS);
  Serial.print("  -> abra http://");
  Serial.println(WiFi.softAPIP());
  Serial.println("Ou digite comandos aqui (PING, STATUS, STOP, RUN, PID kp ki kd, MOTOR l r ms, SENS, CAL START/STOP)");

  sendCmd("PING");
  sendCmd("WD 0");   // watchdog do STM32 so liga quando a pagina pedir (checkbox)
}

void loop() {
  server.handleClient();

  // Linhas espontaneas do STM32 (EVT, HELLO, respostas atrasadas)
  for (String l = pollStm(); l.length(); l = pollStm()) onStmLine(l);

  // STM32 reiniciou: reenvia config salva e religa o watchdog dele
  if (restorePending) {
    restorePending = false;
    if (AUTO_RESTORE_ON_HELLO) restoreSaved();
    if (hbArmed) sendRaw("WD " + String(STM_WD_MS));
  }

  // Watchdog da pagina
  if (hbArmed) {
    if (millis() - lastWebHb > WEB_HB_TIMEOUT_MS) {
      hbArmed = false;
      sendRaw("STOP");
      sendRaw("WD 0");
      addLog("! pagina desconectou: robo parado");
    } else if (millis() - lastStmHb >= STM_HB_PERIOD_MS) {
      lastStmHb = millis();
      Stm.print("HB\n");          // silencioso: nao vai para o log
    }
  }

  // Serial USB -> STM32
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      usbLine.trim();
      if (usbLine.length()) sendRaw(usbLine);
      usbLine = "";
    } else if (usbLine.length() < 100) {
      usbLine += c;
    }
  }
}
