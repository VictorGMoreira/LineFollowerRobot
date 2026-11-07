/*
 * Teste do QTR-8A no Arduino Uno, um sensor por vez, so com o A0.
 *
 * Ligacao:
 *   QTR VCC -> 5V    QTR GND -> GND    LEDON: solto
 *   Fio do A0 -> OUT1, testa, muda para OUT2, testa... ate OUT8.
 *
 * Serial Monitor 115200. A cada 100 ms mostra o valor bruto (0..1023: baixo = branco,
 * alto = preto), o min/max visto e a faixa. Passe a linha preta embaixo do sensor
 * ligado: a faixa tem que passar de FAIXA_OK. Ao mudar o fio de OUT, digite "z" para zerar.
 * Faixa sempre ~0 = sensor morto / fio solto. Valor parado em ~1023 sem nada embaixo =
 * sensor longe do chao ou sem sinal.
 */

#define PINO      A0
#define FAIXA_OK  400    // [0..1023]

int vmin, vmax;

void zerar() { vmin = 1023; vmax = 0; }

void setup() {
  Serial.begin(115200);
  zerar();
  Serial.println(F("Teste QTR-8A no A0. Mude o fio de OUT e digite z para zerar."));
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'z' || c == 'Z') { zerar(); Serial.println(F("--- zerado ---")); }
  }

  int v = analogRead(PINO);
  if (v < vmin) vmin = v;
  if (v > vmax) vmax = v;
  int faixa = vmax - vmin;

  Serial.print(F("valor "));
  Serial.print(v);
  Serial.print(F("   min "));
  Serial.print(vmin);
  Serial.print(F("  max "));
  Serial.print(vmax);
  Serial.print(F("  faixa "));
  Serial.print(faixa);
  Serial.println(faixa > FAIXA_OK ? F("  OK") : F(""));

  delay(100);
}
