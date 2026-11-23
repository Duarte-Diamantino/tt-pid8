// SPDX-License-Identifier: Apache-2.0
//
// Host for the 8-bit PID built from logic gates (Wokwi, Arduino Mega 2560).
// The PID circuit in diagram.json is the Tiny Tapeout design tt_um_pid8
// synthesised into ~1100 gates and 213 flip-flops. The Mega:
//   * generates the circuit clock (Timer1 -> pin 11) and the reset;
//   * writes the configuration through the byte bus (write-only registers);
//   * every TS ms: reads the setpoint (pot) and the plant output (ADC),
//     writes SP and FB (writing FB starts the sample), waits for DRDY and
//     reads the control output U, which the circuit puts as a NUMBER (0..255)
//     on 8 pins. The same 8 pins go to the plant.
//
// Serial commands (115200, end with Enter):
//   kp 2.0   ki 0.4   kd 0.5   gains (0 .. 15.99, UQ4.8)
//   aw 0|1   anti-windup (conditional integration)
//   rev 0|1  reverse action
//   ts 20    sample period in ms (20 .. 1000)
//   clr      clear the integrator
//   ?        current settings

// ---------------------------------------------------------------- pins
// D[7:0] -> PORTA = D22..D29,  U[7:0] <- PORTC = D37..D30
const uint8_t PIN_WR = 2, PIN_AD = 3, PIN_TICK = 5, PIN_RSTN = 6;
const uint8_t PIN_DRDY = 7, PIN_SATHI = 8, PIN_SATLO = 9, PIN_BUSY = 10, PIN_HOLD = 12;
const uint8_t PIN_CLK = 11;                 // OC1A
const uint8_t PIN_SP = A0, PIN_Y = A1;

// ---------------------------------------------------------------- timing
const uint32_t F_CLK     = 10000UL;          // circuit clock (Hz)
const uint16_t T_CLK_US  = 1000000UL / F_CLK;
const uint16_t STROBE_US = 6 * T_CLK_US;    // protocol needs >= 4 clk high / low

// ---------------------------------------------------------------- registers
enum : uint8_t { R_CTRL = 0x0, R_CMD = 0x1, R_SP = 0x2, R_FB = 0x3,
                 R_KP = 0x4, R_KI = 0x6, R_KD = 0x8, R_TS = 0xA };
const uint8_t C_EN = 0x01, C_AW = 0x02, C_REV = 0x04, C_TICK_FB = 0x10;
const uint8_t CMD_CLR = 0x02;

// Plant tau = 200 ms, Ts = 20 ms: Kp = 2, Ti = 0.1 s -> Ki = Kp*Ts/Ti = 0.4
float    kp = 2.0, ki = 0.4, kd = 0.0;
bool     aw = true, rev = false;
uint16_t tsMs = 20;

uint8_t  sp, fb, u, spWritten;
bool     spValid = false;
uint32_t nSample, nTimeout;

// ================================================================ bus
static inline void strobeWr() {
  digitalWrite(PIN_WR, HIGH);
  delayMicroseconds(STROBE_US);
  digitalWrite(PIN_WR, LOW);
  delayMicroseconds(STROBE_US);
}

void busCycle(bool ad, uint8_t d) {
  PORTA = d;
  digitalWrite(PIN_AD, ad ? HIGH : LOW);
  delayMicroseconds(2 * T_CLK_US);          // D/AD set up before WR
  strobeWr();                               // D/AD stay stable while WR = 1
}

void wr8(uint8_t a, uint8_t d)    { busCycle(true, a); busCycle(false, d); }
void wr12(uint8_t a, uint16_t v)  { busCycle(true, a); busCycle(false, v & 0xFF); busCycle(false, (v >> 8) & 0x0F); }

// ================================================================ setup
uint16_t toQ48(float k) {
  long q = lround(k * 256.0);
  if (q < 0) q = 0;
  if (q > 4095) q = 4095;
  return (uint16_t)q;
}

uint8_t ctrlByte() { return C_EN | (aw ? C_AW : 0) | (rev ? C_REV : 0) | C_TICK_FB; }

void writeConfig() {
  wr12(R_KP, toQ48(kp));
  wr12(R_KI, toQ48(ki));
  wr12(R_KD, toQ48(kd));
  wr8(R_CTRL, ctrlByte());
}

void startClock() {
  pinMode(PIN_CLK, OUTPUT);
  TCCR1A = _BV(COM1A0);                     // toggle OC1A on compare match
  TCCR1B = _BV(WGM12) | _BV(CS10);          // CTC, no prescaler
  OCR1A  = F_CPU / (2UL * F_CLK) - 1;
}

void printSettings() {
  Serial.print(F("Kp=")); Serial.print(toQ48(kp) / 256.0, 3);
  Serial.print(F(" Ki=")); Serial.print(toQ48(ki) / 256.0, 3);
  Serial.print(F(" Kd=")); Serial.print(toQ48(kd) / 256.0, 3);
  Serial.print(F(" AW=")); Serial.print(aw);
  Serial.print(F(" REV=")); Serial.print(rev);
  Serial.print(F(" Ts=")); Serial.print(tsMs); Serial.println(F(" ms"));
}

void setup() {
  Serial.begin(115200);
  DDRA = 0xFF; PORTA = 0x00;                // D[7:0] to the circuit
  DDRC = 0x00; PORTC = 0x00;                // U[7:0] from the circuit
  const uint8_t outs[] = { PIN_WR, PIN_AD, PIN_TICK, PIN_RSTN };
  for (uint8_t p : outs) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
  const uint8_t ins[] = { PIN_DRDY, PIN_SATHI, PIN_SATLO, PIN_BUSY, PIN_HOLD };
  for (uint8_t p : ins) pinMode(p, INPUT);

  Serial.println(F("tt_um_pid8 in logic gates: reset..."));
  startClock();
  delay(5);                                 // reset held with the clock running
  digitalWrite(PIN_RSTN, HIGH);
  delay(2);
  Serial.print(F("after reset U=")); Serial.print(PINC);
  Serial.print(F(" BUSY=")); Serial.print(digitalRead(PIN_BUSY));
  Serial.print(F(" DRDY=")); Serial.println(digitalRead(PIN_DRDY));

  writeConfig();
  printSettings();
  Serial.println(F("Commands: kp ki kd aw rev ts clr ?   Output: SP FB U"));
}

// ================================================================ serial
void command(char *s) {
  char *arg = strchr(s, ' ');
  float v = arg ? atof(arg + 1) : 0;
  if      (!strncmp(s, "kp", 2))  { kp = v; writeConfig(); }
  else if (!strncmp(s, "ki", 2))  { ki = v; writeConfig(); }
  else if (!strncmp(s, "kd", 2))  { kd = v; writeConfig(); }
  else if (!strncmp(s, "aw", 2))  { aw = v != 0; writeConfig(); }
  else if (!strncmp(s, "rev", 3)) { rev = v != 0; writeConfig(); }
  else if (!strncmp(s, "ts", 2))  { tsMs = constrain((int)v, 20, 1000); }
  else if (!strncmp(s, "clr", 3)) { wr8(R_CMD, CMD_CLR); }
  else if (s[0] != '?') { Serial.print(F("? unknown: ")); Serial.println(s); return; }
  printSettings();
}

void handleSerial() {
  static char buf[24];
  static uint8_t n = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r' || ch == '\n') {
      if (n) { buf[n] = 0; command(buf); n = 0; }
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = ch;
    }
  }
}

// ================================================================ loop
uint32_t tNext;

void loop() {
  handleSerial();

  uint32_t now = millis();
  if ((int32_t)(now - tNext) < 0) return;
  tNext += tsMs;
  if ((int32_t)(now - tNext) > (int32_t)tsMs) tNext = now + tsMs;   // fell behind: resync

  // 1. measure (10-bit ADC -> 0..255)
  sp = ((uint32_t)analogRead(PIN_SP) * 255UL + 511UL) / 1023UL;
  fb = ((uint32_t)analogRead(PIN_Y)  * 255UL + 511UL) / 1023UL;

  // 2. write SP (if changed) and FB; writing FB starts the sample
  if (!spValid || sp != spWritten) { wr8(R_SP, sp); spWritten = sp; spValid = true; }
  wr8(R_FB, fb);

  // 3. wait for the result (39 clk)
  uint32_t t0 = micros();
  while (!digitalRead(PIN_DRDY)) {
    if (micros() - t0 > 50UL * T_CLK_US) { nTimeout++; break; }
  }

  // 4. the output is a number on 8 pins
  u = PINC;
  nSample++;

  if (nSample % 2 == 0) {
    Serial.print(F("SP:"));  Serial.print(sp);
    Serial.print(F(" FB:")); Serial.print(fb);
    Serial.print(F(" U:"));  Serial.print(u);
    if (digitalRead(PIN_SATHI)) Serial.print(F(" SAT+"));
    if (digitalRead(PIN_SATLO)) Serial.print(F(" SAT-"));
    if (digitalRead(PIN_HOLD))  Serial.print(F(" HOLD"));
    Serial.println();
  }
  if (nTimeout == 1) { Serial.println(F("ERROR: DRDY timeout")); nTimeout++; }
}
