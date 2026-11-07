/*
 * ESP32-C3 <-> STM32F401 (LineFollowerRobot) - ponte de calibracao
 *
 * - Cria uma rede Wi-Fi (AP) com uma pagina de calibracao em http://192.168.4.1: PID,
 *   velocidade base, STOP, RUN, teste de motores, leitura/calibracao dos sensores.
 * - Tambem repassa o Serial USB: digite comandos no Serial Monitor (115200, "Newline")
 *   e eles vao direto pro STM32. Tudo que o STM32 manda aparece no Serial Monitor.
 * - Heartbeat: a cada HB_ESP_MS o ESP manda "ESP OK up <s> stm <ok|sem resposta> ..." no
 *   Serial USB, pisca o LED e a pagina mostra o estado no topo.
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

#define HB_ESP_MS          2000    // heartbeat do ESP (Serial USB + LED + pagina)
#define LED_PIN            8       // LED da placa (Super Mini: GPIO8, acende em LOW). -1 = sem LED
#define LED_ON_LEVEL       LOW
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

bool quiet = false;        // heartbeat: nao loga o PING/PONG
uint32_t lastEspHb = 0;
bool stmOk = false;
bool ledState = false;

void addLog(const String &s) {
  logBuf[logHead] = s;
  logHead = (logHead + 1) % LOG_LINES;
  logSeq++;
}

void restoreSaved();

// Trata uma linha completa vinda do STM32
void onStmLine(const String &line) {
  if (quiet) return;
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
  if (quiet) return;
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
  if (!quiet) addLog("! sem resposta: " + cmd);
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
  if (prefs.isKey("dzl")) sendRaw("DZ " + String(prefs.getInt("dzl")) + " " + String(prefs.getInt("dzr")));
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
select{padding:8px;font-size:16px;background:#222;color:#fff;border:1px solid #444;border-radius:6px}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:6px;margin:6px 0}
.ok{color:#2c5;font-weight:bold}
#dzgo.armed{background:#e80;font-weight:bold}
#esp{font-family:monospace;font-size:13px;margin-bottom:8px}
#espdot{display:inline-block;width:10px;height:10px;border-radius:50%;background:#555;margin-right:6px}
</style></head><body>
<div id="esp"><span id="espdot"></span><span id="esptxt">ESP: aguardando...</span></div>
<button id="stop" onclick="cmd('STOP')">STOP</button>
<button id="run" onclick="cmd('RUN')">RUN (seguir linha)</button>
<div class="row"><label style="color:#eee"><input type="checkbox" id="hb" checked>Parar o robo se esta pagina desconectar</label></div>
<div id="reply">-</div>

<h2>Ligacao ESP &lt;-&gt; STM32</h2>
<div class="row">
 <button onclick="testLink()">Testar ligacao</button>
 <span id="link">-</span>
</div>

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
<label>Teste rapido de cada roda (rodas no ar, forca do campo "Esq %"):</label>
<div class="grid">
 <button onclick="motor(Math.abs(v('ml')),0)">Esq frente</button>
 <button onclick="motor(-Math.abs(v('ml')),0)">Esq re</button>
 <button onclick="motor(0,Math.abs(v('ml')))">Dir frente</button>
 <button onclick="motor(0,-Math.abs(v('ml')))">Dir re</button>
</div>
<label>"Frente" tem que girar a roda para a frente do robo. Girou ao contrario: troque MOTOR_ESQ_INVERTIDO / MOTOR_DIR_INVERTIDO
em robot_config.h (ou inverta os fios do motor). Girou a roda do outro lado: troque os motores entre OUT1/OUT2 e OUT3/OUT4.
Nao girou: confira EEP no 3V3, VCC na bateria e GND comum.</label>

<h2>Zona morta (robo no chao)</h2>
<div class="row">
 <select id="dzside"><option value="l">Esquerdo</option><option value="r">Direito</option><option value="b">Os dois</option></select>
 <button onclick="dzStart()">Iniciar rampa</button>
 <button id="dzgo" class="gray" onclick="dzGot()">GIROU!</button>
</div>
<div class="row"><span id="dzres">-</span></div>
<div class="row">
 <label>DZ Esq</label><input id="dzl" type="number" min="0" max="900">
 <label>DZ Dir</label><input id="dzr" type="number" min="0" max="900">
 <button onclick="dzSend()">Enviar DZ</button>
 <button class="gray" onclick="dzRead()">Ler do robo</button>
</div>
<label>A forca sobe sozinha, sem compensacao (MOTORRAW). Aperte GIROU! no instante em que o robo comecar a andar:
o valor vai para o campo DZ daquele lado. Depois "Enviar DZ" (0..900, 1000 = 100%) e "Salvar no ESP" (secao PID) para
nao perder ao reiniciar. Para ficar fixo no firmware: DZ_ESQ / DZ_DIR no robot_config.h.</label>

<h2>Sensores (valor calibrado 0..1000)</h2>
<div class="bars" id="bars"></div>
<div class="rawrow" id="rawrow"></div>
<div class="rawrow" id="pinrow"></div>
<div class="rawrow" id="okrow"></div>
<div class="track"><div class="mid"></div><div id="mark"></div></div>
<div class="row"><span id="pos">POS -</span></div>
<div class="row"><span id="top">-</span></div>
<label>Teste de fiacao: ligue "Ao vivo" e passe a linha devagar embaixo de cada sensor. Cada um tem que ficar com OK
(faixa bruta &gt; 800) e o "mais forte" tem que andar em ordem S0..S7. Fora de ordem = fios OUT trocados.</label>
<div class="row">
 <button class="gray" onclick="sens()">Ler</button>
 <button class="gray" id="live" onclick="toggleLive()">Ao vivo: OFF</button>
 <button onclick="cmd('CAL START')">Calibrar: iniciar</button>
 <button onclick="calStop()">Calibrar: terminar</button>
 <button class="gray" onclick="calRead()">Ver calibracao</button>
 <button class="gray" onclick="resetWire()">Zerar teste de fiacao</button>
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
  if(c.toUpperCase()==='STOP') dzRun=false;     // STOP tambem interrompe a rampa de zona morta
  const r=await fetch('/cmd?c='+encodeURIComponent(c),{cache:'no-store'}).then(r=>r.text()).catch(e=>'ERRO '+e);
  document.getElementById('reply').textContent=c+'  ->  '+r;
  return r;
}
function motor(l,r){cmd('MOTOR '+pct2duty(l)+' '+pct2duty(r)+' '+(parseInt(v('mt'))||1000))}
async function testLink(){
  const el=document.getElementById('link'), t0=performance.now();
  const r=await cmd('PING'), ms=Math.round(performance.now()-t0);
  if(r!=='PONG'){el.innerHTML='<span class="lost">SEM RESPOSTA: confira GPIO7-&gt;PA10, GPIO6&lt;-PA9, GND e se o STM32 esta ligado</span>';return}
  const s=(await cmd('STATUS')).split(' ');
  el.innerHTML='<span class="ok">OK</span> ('+ms+' ms)  estado: '+(s[0]==='STATUS'?s[1]:'?');
}
// Rampa de zona morta: MOTORRAW sobe DZ_STEP a cada DZ_MS ate apertar GIROU! ou STOP
const DZ_STEP=10, DZ_MS=300;
let dzRun=false, dzU=0;
async function dzStart(){
  if(dzRun)return;
  dzRun=true; dzU=0;
  const side=v('dzside'), go=document.getElementById('dzgo'), res=document.getElementById('dzres');
  go.classList.add('armed');
  while(dzRun&&dzU<=MAX){
    const l=side!=='r'?dzU:0, r=side!=='l'?dzU:0;
    await cmd('MOTORRAW '+l+' '+r+' '+(DZ_MS*3));    // dura mais que um passo: sem tranco entre passos
    res.textContent='u = '+dzU+' ('+Math.round(dzU*100/MAX)+'%)';
    await new Promise(r=>setTimeout(r,DZ_MS));
    if(dzRun) dzU+=DZ_STEP;
  }
  go.classList.remove('armed');
  if(dzU>MAX){cmd('STOP');res.innerHTML='<span class="lost">Chegou a 100% sem apertar GIROU! - confira o motor</span>'}
}
function dzGot(){
  if(!dzRun)return;
  dzRun=false; cmd('STOP');
  const side=v('dzside'), name=side==='l'?'DZ Esq':side==='r'?'DZ Dir':'DZ Esq e Dir';
  if(side!=='r') dzl.value=dzU;
  if(side!=='l') dzr.value=dzU;
  document.getElementById('dzres').innerHTML='<span class="ok">'+name+' = '+dzU+'</span>  (aperte "Enviar DZ")';
}
function showDz(r){const t=r.split(' '), i=t.indexOf('DZ'); if(i<0)return; dzl.value=+t[i+1]; dzr.value=+t[i+2]}
async function dzSend(){showDz(await cmd('DZ '+v('dzl')+' '+v('dzr')))}
async function dzRead(){showDz(await cmd('DZ?'))}
async function status(){
  const r=await cmd('STATUS'); const t=r.split(' ');
  const g=k=>{const i=t.indexOf(k);return i>=0?t[i+1]:''};
  if(t[0]!=='STATUS')return;
  kp.value=+g('KP'); ki.value=+g('KI'); kd.value=+g('KD');
  spd.value=Math.round(+g('SPEED')*100/MAX);
}
async function save(){
  const r=await fetch('/save?kp='+v('kp')+'&ki='+v('ki')+'&kd='+v('kd')+'&speed='+pct2duty(v('spd'))+
    '&dzl='+v('dzl')+'&dzr='+v('dzr')).then(r=>r.text());
  document.getElementById('reply').textContent=r;
}
const bars=document.getElementById('bars'), rawrow=document.getElementById('rawrow');
const PINS=['PA1','PA2','PA3','PA4','PA5','PA6','PA7','PB0'];   // QTR-8A OUT1..OUT8
const pinrow=document.getElementById('pinrow'), okrow=document.getElementById('okrow');
for(let i=0;i<8;i++){
  bars.innerHTML+='<div class="bar" id="b'+i+'"><span></span></div>';rawrow.innerHTML+='<div id="r'+i+'">-</div>';
  pinrow.innerHTML+='<div>S'+i+'<br>'+PINS[i]+'</div>';okrow.innerHTML+='<div id="ok'+i+'">-</div>';
}
// Teste de fiacao: min/max bruto visto em cada sensor desde o ultimo "Zerar"
const WIRE_OK=800;
let rmin=[], rmax=[];
function resetWire(){rmin=Array(8).fill(4095);rmax=Array(8).fill(0);for(let i=0;i<8;i++){const o=document.getElementById('ok'+i);o.textContent='-';o.className=''}}
resetWire();
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
    rmin[i]=Math.min(rmin[i],raw); rmax[i]=Math.max(rmax[i],raw);
    const ok=document.getElementById('ok'+i);
    if(rmax[i]-rmin[i]>WIRE_OK){ok.textContent='OK';ok.className='ok'}else{ok.textContent=rmax[i]-rmin[i];ok.className=''}
  }
  let top=0; for(let i=1;i<8;i++) if(+t[iN+1+i]>+t[iN+1+top]) top=i;
  document.getElementById('top').textContent=+t[iN+1+top]>200
    ? 'Mais forte: S'+top+' (QTR OUT'+(top+1)+', '+PINS[top]+')' : 'Nenhum sensor vendo linha';
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
let seq=-1, blink=false;
async function pollLog(){
  const r=await fetch('/log?since='+seq,{cache:'no-store'}).then(r=>r.json()).catch(()=>null);
  const dot=document.getElementById('espdot'), et=document.getElementById('esptxt');
  if(!r){dot.style.background='#f55';et.innerHTML='<span class="lost">ESP SEM CONEXAO</span>';return}
  blink=!blink; dot.style.background=blink?'#2c5':'#164';
  et.innerHTML='ESP: ok, ligado ha '+r.up+' s  |  STM32: '+(r.stm?'<span class="ok">ok</span>':'<span class="lost">sem resposta</span>');
  if(r.seq!==seq){seq=r.seq;const el=document.getElementById('log');el.textContent=r.lines.join('\n');el.scrollTop=1e9}
}
setInterval(pollLog,700);
document.addEventListener('keydown',e=>{if(e.code==='Space'&&e.target.tagName!=='INPUT'){e.preventDefault();cmd('STOP')}});
(async()=>{await status(); await dzRead(); await calRead();})();
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
  float kp, ki, kd, speed, dzl, dzr;
  if (!argNumber("kp", -10000, 10000, kp) || !argNumber("ki", -10000, 10000, ki) ||
      !argNumber("kd", -10000, 10000, kd) || !argNumber("speed", 1, 1000, speed) ||
      !argNumber("dzl", 0, 900, dzl) || !argNumber("dzr", 0, 900, dzr)) {
    server.send(400, "text/plain", "ERR nao salvo: preencha Kp, Ki, Kd, velocidade (1..100%) e DZ Esq/Dir (0..900) com numeros validos");
    return;
  }
  prefs.putFloat("kp", kp);
  prefs.putFloat("ki", ki);
  prefs.putFloat("kd", kd);
  prefs.putInt("speed", (int)speed);
  prefs.putInt("dzl", (int)dzl);
  prefs.putInt("dzr", (int)dzr);
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
  String out = "{\"seq\":" + String(logSeq) + ",\"up\":" + String(millis() / 1000) +
               ",\"stm\":" + (stmOk ? "1" : "0") + ",\"lines\":[";
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
  if (LED_PIN >= 0) pinMode(LED_PIN, OUTPUT);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  WiFi.setSleep(false);              // sem economia de energia: resposta rapida (STOP, heartbeat)

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
  Serial.println("Ou digite comandos aqui (PING, STATUS, STOP, RUN, PID kp ki kd, MOTOR l r ms, MOTORRAW l r ms, DZ esq dir, SENS, CAL START/STOP)");

  stmOk = sendCmd("PING") == "PONG";
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

  // Heartbeat do ESP: PING silencioso no STM32 + linha no Serial + pisca o LED
  if (millis() - lastEspHb >= HB_ESP_MS) {
    lastEspHb = millis();
    quiet = true;
    stmOk = sendCmd("PING", 50) == "PONG";
    quiet = false;
    String hb = "ESP OK up " + String(millis() / 1000) + "s stm " + (stmOk ? "ok" : "sem resposta") +
                " wifi " + String(WiFi.softAPgetStationNum());
    Serial.println(hb);
    if (LED_PIN >= 0) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState ? LED_ON_LEVEL : !LED_ON_LEVEL);
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
