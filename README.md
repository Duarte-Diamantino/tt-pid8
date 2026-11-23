![](../../workflows/gds/badge.svg) ![](../../workflows/docs/badge.svg) ![](../../workflows/test/badge.svg) ![](../../workflows/fpga/badge.svg)

# PID de 8 bits para um tile (e em portas lógicas no Wokwi)

Versão reduzida de um controlador PID de 10 bits (`tt_um_digital_pid`, projeto
irmão que ocupa 1x2 tiles), feita para caber **num só tile** do Tiny Tapeout.
As equações são as mesmas (PID posicional, integrador com limites,
anti-windup por integração condicional, MAC série), mas com 8 bits. A saída U
sai **como número** nos 8 pinos `uo_out`, sem PWM.

No Wokwi, o circuito não é um chip em Verilog: é o próprio circuito sintetizado,
com **1100 portas lógicas e 213 flip-flops** ligados por 3134 fios.

**[Abrir no Wokwi](https://wokwi.com/projects/481476247969142785)**: projeto
público, com os mesmos quatro ficheiros de `wokwi/`.

## Comparação com a versão de 10 bits

| | 10 bits (`tt_um_digital_pid`) | **8 bits (este repositório)** |
|---|---|---|
| SP, FB, U | 0–1000 (10 bits) | **0–255 (8 bits)** |
| Ganhos | UQ4.12 (16 bits) | **UQ4.8 (12 bits)**, 0 a 15,996 em passos de 1/256 |
| Integrador / acumulador | 22 / 29 bits | 16 / 23 bits |
| Saída U | registo (barramento) e U[9:2] em `uo_out` | **número de 8 bits sempre em `uo_out`** |
| PWM | sim | não |
| Leitura dos registos | sim (pino RD) | não (só escrita) |
| Timer de amostragem | 16 bits | 12 bits |
| Ciclos por amostra | 51 | **39** |
| Área após síntese (SKY130 HD) | 15 168 µm² | **10 016 µm²** (medido) |
| Células / flip-flops | 1582 / 311 | **1018 / 213** |
| Tile | 1x2 | **1x1** (55 % antes do place & route) |

**Cabe num tile?** Provavelmente, mas à justa. Depois da síntese ocupa 55 % de
um tile 1x1 (~18 000 µm²). O place & route costuma acrescentar 10 a 25 %, o que
dá 61 a 69 %: acima da densidade alvo do fluxo (60 %) e abaixo dos máximos que
costumam fechar (~80 %). Só o job `gds` do GitHub do Tiny Tapeout confirma. Se
não fechar, retirar o registo Kd (passa a PI) poupa cerca de 500 µm²
(estimativa: 12 flip-flops e a respetiva escrita).

Área por bloco (medida, hierarquia preservada):

| Bloco | µm² | Flip-flops |
|---|---:|---:|
| `pid8_core` (erro, MAC série, integrador, limites) | 6 111 | 110 |
| `pid8_regs` (barramento, SP, FB, Kp, Ki, Kd, TS, CTRL) | 3 125 | 82 |
| `pid8_tick` (timer de 12 bits) | 607 | 12 |
| `pid_sync` ×2 (WR e TICK) | 170 | 6 |
| topo (reset, DRDY) | 86 | 3 |

## Pinos

| Pino TT | Sinal | |
|---|---|---|
| `ui_in[7:0]` | D[7:0] | byte de endereço ou de dados |
| `uo_out[7:0]` | **U[7:0]** | ação de controlo 0–255 (registada, sem glitches) |
| `uio[0]` (in) | WR | escrita no flanco ascendente |
| `uio[1]` (in) | AD | 1 = ciclo de endereço, 0 = ciclo de dados |
| `uio[2]` (in) | TICK | tick externo (`TICK_SRC = 01`) |
| `uio[3]` (out) | HOLD | anti-windup suspendeu a integração na última amostra |
| `uio[4]` (out) | DRDY | há um U novo; desce na escrita seguinte ou na amostra seguinte |
| `uio[5]` (out) | SAT_HI | U cortado em 255 |
| `uio[6]` (out) | SAT_LO | U cortado em 0 |
| `uio[7]` (out) | BUSY | cálculo em curso (38 ciclos) |

## Registos (só escrita)

Protocolo igual ao da versão de 10 bits: um ciclo com AD = 1 escolhe o endereço
(D[3:0]) e cada ciclo com AD = 0 escreve um byte e avança o endereço. Nos
registos de 12 bits escreve-se primeiro o byte baixo e depois o alto (D[3:0]); o
valor só muda quando chega o byte alto.

| Endereço | Registo | Reset | |
|---|---|---|---|
| 0x0 | CTRL | 0x02 | [0] EN · [1] AW_EN · [2] REV · [4:3] TICK_SRC (00 timer, 01 pino, 10 escrita de FB, 11 só software) |
| 0x1 | CMD | — | [0] SW_TICK · [1] CLR (impulsos) |
| 0x2 | SP | 0 | setpoint 0–255 |
| 0x3 | FB | 0 | medida 0–255; inicia a amostra se `TICK_SRC = 10` |
| 0x4 / 0x5 | KP_L / KP_H | 0 | UQ4.8 (Kp = 2,0 → 512) |
| 0x6 / 0x7 | KI_L / KI_H | 0 | UQ4.8 |
| 0x8 / 0x9 | KD_L / KD_H | 0 | UQ4.8 |
| 0xA / 0xB | TS_L / TS_H | 999 | período do timer = TS + 1 ciclos |

Equações (iguais às da versão de 10 bits, com outras larguras):

```
e     = SP − FB                       (REV: FB − SP)          9 bits com sinal
Δe    = e − e_anterior                (0 na 1.ª amostra)
hold  = AW_EN · (SAT_HI·(e > 0) + SAT_LO·(e < 0))
I     = limitar(I + (hold ? 0 : Ki·e), 0, 255)                UQ8.8
U     = limitar(arredondar(Kp·e + I + Kd·Δe), 0, 255)
```

O acumulador de 23 bits nunca transborda: |Kp·e|, |Ki·e| < 2²⁰, |Kd·Δe| < 2²¹ e I < 2¹⁶.

Limitações em relação à versão de 10 bits:
- **Ki mínimo** = 1/256, logo Ti máx. = Kc·Ts·256. Para plantas lentas, aumente Ts.
- **Timer de 12 bits:** no máximo 4096 ciclos, ou seja 410 µs a 10 MHz. Para
  períodos maiores, baixe o relógio do chip ou deixe o anfitrião marcar a
  amostra (escrita de FB ou pino TICK), como no Wokwi.
- Não se lêem os registos, nem o erro e o integrador.

## No Wokwi, em portas lógicas

```
 potenciómetro ─► A0 ┐                         ┌──────────────── circuito (diagram.json) ───────────────┐
                     │  Arduino Mega            │ entradas   sincronização  registos  timer  núcleo PID   │ saídas
 planta (chip C) ─► A1 │  D22-29 ─► D0-D7 ──────► (buffers) ─► 1100 portas lógicas + 213 flip-flops ─────► U0-U7 ─┬─► Mega D37-30
   ▲                 │  2,3,5,6,11 ─► WR AD TICK RST_N CLK                                               │         └─► planta U0-U7
   └── U (número) ◄──┴──────────────────────────────────────────────────────────────────────────────────┘ DRDY SAT HOLD BUSY ─► Mega 7-10, 12
 interruptor ─► DIST (carga de 75)
```

Como foi gerado:
1. [wokwi/gates.ys](wokwi/gates.ys): o Yosys sintetiza o RTL usando **só as peças lógicas que o Wokwi tem**:
   - portas de 2 entradas: AND, OR, XOR, NAND, NOR, XNOR;
   - NOT e MUX de 2 entradas;
   - flip-flop D e flip-flop D com set/reset.
2. [wokwi/gen_wokwi.py](wokwi/gen_wokwi.py) cria o `diagram.json`:
   - uma peça Wokwi por célula;
   - um fio por ligação;
   - as portas agrupadas por bloco e ordenadas da esquerda para a direita pela profundidade lógica;
   - cores dos fios: relógio dourado, reset vermelho, e uma cor por bloco.
3. [wokwi/diagram_to_verilog.py](wokwi/diagram_to_verilog.py) faz o caminho inverso de forma independente. Lê o `diagram.json`, junta os fios em nós como o Wokwi faz e escreve um Verilog com modelos das peças. O mesmo testbench do RTL corre sobre esse Verilog.

Comportamento das peças, medido num teste no Wokwi antes de gerar o circuito:
- **Portas:** tabelas de verdade normais.
- **MUX:** SEL = 0 escolhe A.
- **Flip-flops:** atuam no flanco ascendente.
- **Flip-flop com set/reset:** S e R são assíncronos e ativos a 1, e R ganha a S.
- **Flip-flop D simples:** arranca com Q = 1. Só é usado nos registos do datapath, que são sempre carregados antes de serem usados. O testbench passa com Q inicial a 1 e a 0.

| Peças no `diagram.json` | |
|---|---:|
| NAND / AND / OR / NOR / XNOR / XOR / NOT | 435 / 214 / 115 / 51 / 41 / 21 / 41 |
| MUX de 2 entradas | 182 |
| Flip-flops D com reset/set / sem reset | 153 / 60 |
| Auxiliares (2 NOT do reset, 2 constantes 0/1) | 4 |
| Pinos do circuito (buffers com nome) | 26 |
| Fios | 3134 |
| Profundidade lógica máxima | 46 portas |

O anfitrião ([wokwi/sketch.ino](wokwi/sketch.ino)):
- gera um relógio de 10 kHz e faz o reset;
- escreve Kp = 2, Ki = 0,4 e CTRL;
- a cada 20 ms escreve SP e FB (escrever FB dispara a amostra), espera por DRDY e lê U diretamente nos 8 pinos;
- aceita os comandos série `kp`, `ki`, `kd`, `aw`, `rev`, `ts`, `clr` e `?`.

A planta ([wokwi/plant8.chip.c](wokwi/plant8.chip.c)) recebe U pelos mesmos 8 pinos e segue τ·dy/dt = ganho·U − y − carga. O ecrã mostra SP, y e U.

Resultado no Wokwi (simulação no browser, com as 1100 portas):

| Teste | Resultado |
|---|---|
| Depois do reset | U = 0, BUSY = 0, DRDY = 0 |
| Degrau de SP 0 → 128 | U começa saturado em 255 (`SAT+ HOLD`). FB sobe 29 → 68 → 96 → 114 → pico de **134 (4,7 %)** e estabiliza em **128 com erro 0**, ao fim de cerca de 0,6 s |
| Carga de 75 ligada | FB desce até 111, U sobe de 128 para **203** (= 128 + 75) e FB volta a 128 em cerca de 0,4 s |
| Velocidade | cerca de **3 % do tempo real** com o painel do browser escondido e **34 %** com o painel visível |

Para mudar a carga, a constante de tempo ou o ganho da planta, clique no chip da planta durante a simulação.

### Pôr no Wokwi à mão

O [projeto guardado](https://wokwi.com/projects/481476247969142785) já tem tudo.
Estes passos servem para montar um projeto novo, por exemplo depois de
alterar o RTL e voltar a gerar o `diagram.json`.

1. Abra <https://wokwi.com/projects/new/arduino-mega>.
2. Em *Code* → ▼ → *New file...*, crie `plant8.chip.json` e `plant8.chip.c`. O
   diálogo traz o nome anterior: apague-o antes de escrever.
3. Cole o conteúdo dos quatro ficheiros de `wokwi/`: `sketch.ino`, `diagram.json`,
   `plant8.chip.json` e `plant8.chip.c`. O `diagram.json` é uma única linha de 206 KB.
4. Carregue em ▶. Os avisos "Clock driven by combinatorial logic" são normais:
   o Wokwi dá-os sempre que o relógio de um flip-flop vem de outra peça.

O circuito à direita do Arduino tem cerca de 6 300 × 2 600 px. Use a roda do rato para fazer zoom.

## Verificação

Tudo de uma vez (WSL / Linux): `bash run_all.sh`. Os testes cocotb correm à
parte, como no CI do Tiny Tapeout: `cd test && make` (com `pip install -r test/requirements.txt`).

| O quê | Como | Resultado |
|---|---|---|
| Lint | `iverilog -Wall` | sem avisos |
| cocotb (CI do Tiny Tapeout, RTL e netlist pós-layout) | [test/test.py](test/test.py) | 6 testes: reset, exemplo do datasheet, 400 amostras aleatórias contra o modelo, timer e latência, EN = 0 e reset sem relógio, malha fechada. Um erro de arredondamento introduzido de propósito é detetado |
| RTL pelos pinos | [sim/tb_pid8.v](sim/tb_pid8.v), modelo bit-exato | 20 000 amostras aleatórias sem erros, com ganhos 0 e máximos, AW, REV, CLR e as três fontes de tick. Os casos limite (HOLD, SAT_HI, SAT_LO e os dois limites do integrador) aparecem milhares de vezes. Também testa: timer (100 ciclos com TS = 99), BUSY de 38 ciclos, EN = 0, reset assíncrono sem relógio e malha fechada (overshoot 7,8 %, erro final 0 com carga) |
| Portas do `diagram.json` | o mesmo testbench sobre `wokwi/build/pid8_gates.v` | 3000 amostras sem erros, com Q inicial dos flip-flops a 1 e a 0 (10 000 amostras) |
| Planta | [wokwi/test/plant8_host.c](wokwi/test/plant8_host.c) nativo | τ, y = 153,0 e carga → 78,0 |
| Síntese SKY130 | [synth/synth.sh](synth/synth.sh) | 10 016 µm², 1018 células, 213 FF |
| Wokwi (browser) | ficheiros do projeto comparados por hash com os desta pasta | tabela acima |

Um erro apanhado pelo testbench: o contador de passos do MAC tinha 4 bits e, com
12 passos, não voltava a 0 sozinho entre o termo P e o D (na versão de 16 passos
voltava). O resultado continuava certo, mas cada amostra gastava mais 4 ciclos.
O teste de latência (T5) detetou-o, e o contador agora volta a 0 no último passo.

## Ficheiros

| Ficheiro | Conteúdo |
|---|---|
| `src/tt_um_duarte_diamantino_pid8.v` | topo: pinos, reset, DRDY |
| `src/pid8_core.v` | núcleo PID de 8 bits (MAC série) |
| `src/pid8_regs.v` | barramento e registos (só escrita) |
| `src/pid8_tick.v` | instante de amostragem (timer de 12 bits, pino, escrita de FB, software) |
| `src/pid_sync.v` | sincronizador dos pinos WR e TICK (o mesmo da versão de 10 bits) |
| `src/config.json` | configuração do fluxo de layout (a do template, sem alterações) |
| `info.yaml` | metadados Tiny Tapeout para 1x1 |
| `docs/info.md` | página do datasheet do Tiny Tapeout |
| `test/` | testes cocotb no formato do template (RTL e gate level) |
| `.github/workflows/` | gds, test, docs e fpga do template, na versão `ttsky26e` |
| `sim/tb_pid8.v` | testbench Icarus (RTL e portas) |
| `synth/synth.sh`, `synth/results/` | síntese SKY130 e resultados (`scripts/get_liberty.sh` descarrega a liberty) |
| `wokwi/gates.ys` | síntese para as peças do Wokwi |
| `wokwi/gen_wokwi.py` | netlist → `diagram.json` |
| `wokwi/diagram_to_verilog.py` | `diagram.json` → Verilog, para verificação |
| `wokwi/diagram.json` | **gerado**: circuito + Arduino + planta + potenciómetro + interruptor |
| `wokwi/sketch.ino` | anfitrião (Arduino Mega) |
| `wokwi/plant8.chip.json`, `wokwi/plant8.chip.c` | planta de 1.ª ordem com entrada de 8 bits e gráfico |
| `wokwi/test/plant8_host.c` | teste nativo da planta |
| `run_all.sh` | todos os passos acima |

## Submeter ao Tiny Tapeout

Esta pasta já é a raiz do repositório, no formato do template
`ttsky-verilog-template`, com o módulo de topo `tt_um_duarte_diamantino_pid8`.
No Wokwi, os textos do projeto continuam a dizer `tt_um_pid8`: o circuito é o mesmo.

1. No GitHub, em *Settings → Pages*, escolha *GitHub Actions* como origem. O job `viewer` precisa disto.
2. A cada push correm os workflows `gds` (layout, precheck, teste gate level),
   `test` (cocotb e testbench Icarus) e `docs`. Confirme no resumo do `gds` a
   ocupação real do tile e o timing.
3. Com tudo verde, submeta em <https://app.tinytapeout.com>: entre com a conta
   do GitHub, escolha o shuttle e o repositório, e pague o tile.
