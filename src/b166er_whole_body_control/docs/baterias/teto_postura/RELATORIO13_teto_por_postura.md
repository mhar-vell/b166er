# Teto de velocidade dependente da postura — 2026-09-08

Marco: "segue com o teto de velocidade dependente da postura". Origem: o
robô tombou a 0,3 m/s / 0,5 rad/s com o braço na postura de busca
(2026-09-03, `aproximacao/exp_aproximacao.py`, 5 trocas ALIGN/ADVANCE em
25 s) e o mesmo teste passou com o braço recolhido (travel). A missão
navega em travel, mas o whole-body de longe e a busca usam o braço à
frente — e o teto de 0,3 m/s era o mesmo para qualquer postura.

## O índice: margem de tombamento, não extensão da ponta

A primeira ideia era escalar o teto pela extensão horizontal da ponta.
Não separa as posturas (`extensao_posturas.py`):

| postura | ponta x / y / z (m) | CG do braço x / z (m) |
|---|---|---|
| stow_home | 0,500 / 0,001 / 0,828 | 0,173 / 0,794 |
| travel | 0,433 / −0,250 / 0,828 | 0,149 / 0,794 |
| search | 0,557 / 0,001 / 0,990 | 0,185 / 0,784 |
| deploy | 0,651 / 0,001 / 0,838 | 0,214 / 0,763 |

O que tomba o robô é a frenagem seca com o CG do conjunto perto do eixo
dianteiro. Modelo quase-estático: a resultante (peso + pseudo-força da
aceleração `a`) sai do polígono de apoio quando `a·z_cg > g·(borda −
x_cg)`. A margem `a_max = g·(borda − x_cg)/z_cg` é calculada a cada
ciclo pela postura medida (`margem_tombamento` em
`fuzzy_wb_controller.py`): elos do braço com as massas do URDF no ponto
médio entre quadros de junta consecutivos, base concentrada (13 kg a
0,15 m), rodas a ±0,20 m (x) e ±0,17 m (y) como no `pioneer.urdf`.

| postura | frear (eixo dianteiro) | acelerar (traseiro) | lateral | CG x / z (m) |
|---|---|---|---|---|
| travel | **1,42 m/s²** | 5,66 | 1,79 | 0,120 / 0,554 |
| stow_home | 1,09 | 5,99 | 3,01 | 0,138 / 0,554 |
| search | **0,92** | 6,22 | 3,03 | 0,149 / 0,550 |
| captura (~) | 0,75 | 6,36 | 3,02 | 0,158 / 0,552 |
| deploy | **0,51** | 6,87 | 3,14 | 0,172 / 0,531 |
| deploy com J2 a 0,3 | 0,32 | 7,45 | 3,30 | 0,184 / 0,505 |

A margem de frenagem separa as posturas na ordem em que elas tombam ou
não: travel 1,42 (passou), search 0,92 (tombou). Repare que o valor
absoluto é pequeno em todas — zerar 0,3 m/s num ciclo de 20 Hz é uma
desaceleração de 6 m/s², muito acima de qualquer linha da tabela; o que
salvava o travel era o transiente do ODE e a suspensão implícita das
rodas, não margem de projeto.

## A regra

Três ações, todas a partir da margem `a_min = min(frear, acelerar,
lateral)` da postura medida:

1. **Rampa**: a variação de `/cmd_vel` por ciclo fica limitada a
   `seguranca × a_min` (0,5 × a_min); a troca ADVANCE → ALIGN, que
   zerava `v` de uma vez, passa pela mesma rampa.
2. **Teto escalado**: `v_cap = 0,3 × s`, `ω_cap = 0,5 × s`, com `s` a
   pertinência linear de `a_min` entre `a_lo = 0,5` e `a_hi = 1,2`
   m/s² (mínimo `s_min = 0,3`). Travel anda com o teto cheio; search a
   60 % (0,18 m/s, 0,30 rad/s); deploy no mínimo (0,09 m/s).
3. **Centrípeta**: `|v·ω| ≤ seguranca × a_min`.

Paradas de segurança (inclinação crítica, IK divergente, stand-down)
continuam secas e reiniciam a rampa. Parâmetros em `~teto_postura`
(`b166er_wb.launch`), relidos a cada `wb_enable`; observável em
`/b166er/base_cap` = [v_cap, ω_cap, a_cap, a_frente, a_lat, x_cg, z_cg].

## Bateria

Aproximação de longe (1,76 m, alvo a 0,8 m da parede, girar-avançar-girar),
Fuzzy, uma execução por postura com o código anterior (linha de base) e
A/B no stack novo, ligando e desligando a regra por `rosparam`.

### 1. Linha de base: o cenário de 03 Set não reproduz mais

Código anterior, 0,3 m/s, uma execução por postura
(`varre_posturas.sh`, `linha_base.jsonl`):

| postura | tilt máx (rad) | tombou | melhor erro | trocas ALIGN |
|---|---|---|---|---|
| stow_home | 0,021 | não | 23 mm | 1 |
| travel | 0,019 | não | 29 mm | 1 |
| search | 0,020 | não | 118 mm | 1 |
| deploy | 0,031 | não | 123 mm | 1 |

Em 03 Set a manobra trocava ALIGN/ADVANCE 5 vezes em 25 s e o robô
tombou em search; com o piso de 0,90 m da manobra (RELATORIO8, PR #51)
ela troca uma vez só, parada, no início — e nenhuma postura tombou.
Todas as execuções terminaram por timeout (120 s) caçando o alvo; as
posturas estendidas param a 12 cm porque o alvo 6D da T265 fica fora
do que a postura alcança sem mexer o braço, não é assunto deste
relatório.

O que aconteceu DEPOIS da execução search é: 3,07 s (tempo de
simulação) após o stand-down do controlador, a trava crítica disparou
(pitch −0,46 rad) e o reset seguinte encontrou o robô a 1,475 rad,
caído. O rosout não separa se foi a parada seca do stand-down ou o
teleporte do reset (`reset_sim.py` documenta esse segundo modo). Por
isso o ensaio seguinte ataca o mecanismo diretamente.

### 2. Ensaio de frenagem seca (`exp_frenagem.py`, `frenagem.jsonl`)

O controlador leva a base a 0,3 m/s rumo a um alvo 2,5 m à frente;
quando a odometria passa de 0,27 m/s o alvo salta para 1,5 m ATRÁS
(erro de rumo de 180°): a manobra entra em ALIGN, que zerava `v` de
uma vez e girava a 0,5 rad/s. É o evento que a recuperação da tag ou
um novo alvo do SEARCH produzem na missão. Mede-se a inclinação máxima
nos 5 s seguintes, o maior degrau de `v` entre dois comandos e o tempo
até `v` < 0,05 m/s. Duas repetições por célula, no MESMO stack,
ligando e desligando a regra por `rosparam` (o controlador relê a cada
`wb_enable`; o rosout registra "ligado"/"DESLIGADO" em cada execução).

| postura | regra | v no salto (m/s) | tilt máx depois (rad) | degrau máx de v (m/s) | v→0 (s) | tombou |
|---|---|---|---|---|---|---|
| travel | sem | 0,30 / 0,31 | 0,019 / 0,021 | 0,300 / 0,300 | 0,08 / 0,04 | não |
| travel | com | 0,30 / 0,30 | 0,022 / 0,014 | 0,052 / 0,047 | 0,39 / 0,41 | não |
| stow_home | sem | 0,29 / 0,30 | 0,016 / 0,020 | (n/m) / 0,300 | 0,03 / 0,04 | não |
| stow_home | com | 0,23 / 0,23 | 0,021 / 0,015 | 0,017 / 0,018 | 0,15 / 0,20 | não |
| search | sem | 0,29 / 0,30 | **0,052 / 0,056** | 0,300 / 0,300 | 0,07 / 0,04 | não |
| search | com | 0,17 / 0,17 | **0,017 / 0,021** | 0,014 / 0,020 | 0,17 / 0,20 | não |
| deploy | sem | 0,30 / 0,30 | **0,066 / 0,030** | 0,300 / 0,300 | 0,04 / 0,02 | não |
| deploy | com | 0,10 / 0,11 | **0,016 / 0,018** | 0,014 / 0,015 | 0,20 / 0,27 | não |

(n/m: na primeira execução de stow a janela de medição começava
depois do salto e o degrau não foi contado; corrigido nas seguintes.)

Leitura:

- **Sem a regra a parada é seca em todas as posturas** (0,30 m/s num
  único comando, `v` zerada em 20–80 ms) e a inclinação cresce com a
  extensão: 0,02 rad recolhido, 0,05–0,06 em search, 0,03–0,07 em
  deploy. Nenhuma chegou ao alerta (0,25 rad): o ODE com estas rodas é
  mais rígido do que o evento de 03 Set fazia supor, e a parada seca
  sozinha não tomba o robô a 0,3 m/s neste piso.
- **Com a regra a inclinação fica na faixa da postura recolhida**
  (0,015–0,022 rad) em todas: a rampa (degrau ≤ 0,02 m/s por comando,
  parada em 0,15–0,27 s) e o teto (0,17 m/s em search, 0,10 em deploy)
  tiram o pico de 3–4×.
- **A postura nominal não descreve o risco.** O rosout mostra a margem
  caindo DURANTE a aproximação: em search o controlador começa com
  0,91 m/s² e 5 s depois lê 0,45 (CG x 0,149 → 0,176 m); em stow 1,03
  → 0,50; em deploy 0,51 → 0,38. O laço whole-body estende o braço rumo
  ao alvo enquanto a base anda — é isso que a regra pega ao recalcular
  pela postura MEDIDA a cada ciclo, e é o que uma tabela por nome de
  postura não pegaria.

### 3. A queda depois da execução é do reset, não do stand-down

Quatro de quatro execuções em search (com e sem a regra) terminaram com
o robô caído no reset seguinte ("reset falhou (1)", tilt 1,475 rad),
sempre ~3 s de simulação depois do stand-down. Para separar as duas
hipóteses, `teste_standdown.sh` roda a aproximação por 40 s e observa
`/b166er/tilt` por 12 s SEM resetar (`standdown.jsonl`): tilt máximo
0,017 / 0,016 (sem regra) e 0,015 rad (com), nenhuma queda. A parada
seca do stand-down não derruba o robô; o teleporte do reset com o
braço estendido pelo whole-body derruba — o modo "nasce caindo" que o
`reset_sim.py` já documenta, agora com o braço fora de qualquer
postura nominal. Fica como pendência do reset, não deste trabalho.

### 4. O ensaio de aproximação tinha a ponta dentro da fixture

Na segunda rodada do teste acima, a execução COM a regra tombou
durante a aproximação (tilt 0,568 rad): base a −0,08 m/s, J3 a
0,33 rad/s, e a inclinação saltou de 0,26 para 0,46 rad em 0,12 s — um
impulso, não uma frenagem. A causa é a geometria do ensaio: o alvo da
base em y = 2,2 põe a T265 a 0,8 m da parede COM O BRAÇO EM TRAVEL; em
search a ponta da ferramenta fica 0,74 m à frente da base e cai em
y = 2,94, DENTRO do mecanismo da chave (2,92) e a 6 cm da parede. As
inclinações grandes sem a regra (0,137 e 0,171 rad, `aproximacao_ab.jsonl`
e `standdown.jsonl`) são o mesmo contato. Artefato do ensaio, não da
regra — mas expôs um defeito de desenho real:

**Com a base contida, o DLS jogava o resto do movimento no braço.**
Cortar só a base depois de resolver a pseudo-inversa muda a solução: o
erro que a base deixa de corrigir continua no laço, o ganho Fuzzy sobe
e o braço faz o que a base não fez, mais rápido e mais estendido. A
regra passou a (a) escalar o vetor whole-body INTEIRO (8 DOF) pelo
mesmo fator quando a base excede o teto (`_escala_wb`), mantendo a
direção da solução e deixando a tarefa mais lenta como um todo, e (b)
escalar o teto do braço com base livre por `s` (0,8 × s rad/s, piso
0,25, o teto da manipulação). A rampa e a centrípeta continuam na
saída.

### 5. Aproximação de longe A/B com o alvo fora da fixture (`bateria_teto2.sh`, `aproximacao_ab2.jsonl`)

Alvo da base em y = 1,9 (ponta em 2,64 m), escala conjunta base+braço.

| execução | regra | melhor erro | trocas ALIGN | v máx / ω máx | tilt máx | tombou |
|---|---|---|---|---|---|---|
| search 1 | sem | 115 mm | 1 | 0,30 / 0,50 | 0,020 | não |
| search 1 | com | 108 mm | 2 | 0,17 / 0,29 | 0,021 | não |
| search 2 | sem | 105 mm | 1 | 0,30 / 0,50 | 0,020 | não |
| search 2 | com | 127 mm | 2 | 0,17 / 0,29 | 0,021 | não |
| deploy | com | 126 mm | 3 | 0,09 / 0,15 | 0,021 | não |
| deploy | sem | 108 mm | 1 | 0,30 / 0,50 | 0,020 | não |

Com a ponta fora da fixture nenhuma execução tomba e a inclinação é a
mesma com e sem a regra (0,020–0,021 rad): na aproximação em si, a
0,3 m/s e uma troca de manobra parada, não há evento de frenagem para
a regra atenuar. A regra também não custa convergência (105–127 mm
nos dois modos; o resíduo de ~11 cm é do alvo 6D da T265 e da postura,
igual nos dois). O que a regra muda aqui é o teto (0,17 m/s em search,
0,09 em deploy) e o tempo — e por isso ela só vale quando a margem
está baixa: em travel, que é como a missão navega, o teto fica cheio
(seção 2 e `aproximacao_ab.jsonl`: travel com a regra 21 mm, 0,30 m/s,
tilt 0,019).

Para comparação, a rodada anterior com o alvo em 2,2 m
(`aproximacao_ab.jsonl`) tem search com a regra a 37 e 136 mm e sem a
regra a 128 e 63 mm, tilt 0,022–0,043 com e 0,020–0,137 sem, e é a
que continha o contato com a fixture (seção 4).

## Conclusão

1. A regra faz o que promete no mecanismo certo: a frenagem seca com o
   braço estendido passa de 0,05–0,07 rad de inclinação para
   0,015–0,022 (seção 2), sem tocar na postura recolhida.
2. Ela não previne o tombamento de 03 Set porque esse já não
   reproduz: o piso de 0,90 m da manobra tirou as trocas ALIGN/ADVANCE
   em velocidade. Fica como segunda barreira, calculada a cada ciclo
   pela postura medida — e é a postura MEDIDA que importa, porque o
   whole-body estende o braço enquanto a base anda (seção 2).
3. Dois achados laterais: a queda "depois" da execução é do teleporte
   do reset com o braço estendido (seção 3, pendência do
   `reset_sim.py`), e cortar só a base depois do DLS empurra a tarefa
   para o braço (seção 4, corrigido com a escala conjunta).
4. Regressão da missão completa com a regra ligada: seção 6.

### 6. Regressão da missão completa (`regressao_missao.sh`, `missao/`)

Sete missões completas no mesmo stack, pose de referência, modo
híbrido, alternando a regra por `rosparam` (`missao/resumo.md`):

| regra | resultado | lâmina | duração |
|---|---|---|---|
| com (runs 1, 3, 5, 7) | **4/4 OK** | 28,5–30,4° | 144–160 s (média 151) |
| com (run 2) | ABORT em "atravessa" | — | — |
| sem (runs 4, 6) | 2/2 OK | 29,8–30,1° | 154–161 s |

A regra esteve ativa nas fases de contato (rosout: margem ~0,6 m/s²
com o braço na postura de captura, CG z 0,44 m; teto 0,09 m/s e
0,15 rad/s; rampa 0,3 m/s²) sem alongar as fases nem a missão — as
execuções com a regra foram, se algo, mais curtas. O aborto da run 2
aconteceu na fase "atravessa", que é IK iterativa e não passa pelo
controlador: a profundidade ficou em −6,9 mm contra tolerância de 6 em
cinco iterações, com a base estacionada 2 cm mais longe que nas
outras (x = 0,21 contra 0,18–0,20). É a sensibilidade ao
estacionamento já conhecida (RELATORIO9), e o estacionamento é feito
pela navegação da própria missão, que a regra não toca.

**Limite a registrar:** a missão dirige a base por conta própria
(`ctx.drive`) em SEARCH, APPROACH, RETURN e no ABORT; o teto por
postura só cobre os trechos comandados pelo controlador whole-body
(aproximação de longe, fases de contato). A busca gira em postura
search a 0,35 rad/s no próprio eixo — margem lateral alta (3,0 m/s²),
sem risco — mas a recuperação da tag e o retorno andam sem o teto.
Propagar `/b166er/base_cap` para o `drive` da missão fica como
pendência.
