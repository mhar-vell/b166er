# RELATORIO34 — varredura lateral no recuo por contato da atravessa — 2026-10-21

Sequência do RELATORIO33 (run 4 da bateria `engate_fix`: atravessa presa
duas vezes no mesmo ponto, aborto antes do engate). Marco: "pode seguir
com a varredura lateral na atravessa". Branch `feat/varredura-lateral-atravessa`.

## O que mudou

1. **Varredura no recuo por contato** (`chave_mission.py`,
   `_reach_by_iterative_ik`). O recuo por contato (RELATORIO19/20) saía
   20 mm pelo eixo e descia 5 mm, acumulando, no máximo duas vezes.
   Agora cada recuo mira o alvo **original** deslocado pelo k-ésimo par
   (profundidade, altura) de `~ik_recuo_varredura_mm`, no frame da
   parede: `[0,−5]` (o "desce 5" de sempre), `[+8,−5]`, `[−8,−5]`,
   `[0,+5]`; `~ik_recuos_max` 4. O movimento de recuo já leva a ponta
   para a nova posição no plano do anel, para reentrar alinhada.
   `~ik_recuo_desce_m` deixou de existir (é o primeiro par da lista).
2. **Viés de teste da parede** (`~teste_vies_parede_mm`, arg do
   `chave_mission.launch`): [eixo, profundidade, altura] em mm somados à
   estimativa depois da REFINE, no frame da parede. Só para reproduzir
   uma percepção fora em simulação; zero na missão, não existe no robô.

## Releitura da run 4 de 17 Out pela sonda

A sonda daquela run (`engate_fix/esq/run4/sonda_engate.csv`) grava a
verdade do Gazebo: degrau−anel em eixo/profundidade/altura. Durante as
cinco iterações da atravessa:

| grandeza | run 4 (atravessa, presa) | run 1 (captura boa) |
|---|---|---|
| degrau−anel eixo (sonda) | **7,8 – 8,3 mm** | 9,9 mm |
| degrau−anel profundidade | +5 … +11 mm | −4,9 mm |
| degrau−anel altura | −3 … −11 mm | −9,0 mm |
| eixo no frame da parede (missão) | **+9,7 mm** (alvo −10) | −0,9 mm |

Ou seja: **o dedo estava inserido na profundidade de um engate bom** (o
arame no canto do degrau, como na run 1), e a missão acreditava que
faltavam 20 mm. A diferença bate com a parede da REFINE 13 mm fora ao
longo do eixo (x da parede −0,011 contra +0,002 nas outras runs) mais o
yaw de 2,2°. O "não avança com J1 aquém" era o dedo já engatado
empurrando o anel contra o canto do degrau. A varredura lateral **não
teria resolvido a run 4**: a componente que a derrubou é ao longo do
eixo, e a varredura trata profundidade e altura. O que a resolveria é
tratar esse bloqueio como informação — "travou a menos de X mm do alvo
no eixo, duas vezes no mesmo ponto = engatou; zera o eixo da captura no
ponto do contato" —, que é decisão do Marco (muda o critério da
atravessa/captura; as fases seguintes já usam o eixo relativo à captura).

A varredura continua valendo para o que ela trata: erro lateral da
percepção (profundidade pelo yaw da parede, altura pela tag), que na
bancada é o caso comum — o viés da T265 é no eixo óptico, e a REFINE
pelo laser corrige profundidade e yaw mas não o lateral.

## Bateria `varredura` (esq, viés sintético depois da REFINE)

Sete missões da pose esq (−0,5, 1,0, 0), garra fechada, sonda ligada,
cada uma com um viés somado à parede depois da REFINE. A sonda grava a
verdade do Gazebo (degrau−anel em eixo/profundidade/altura, lingueta).

| run | viés (eixo, prof, alt) mm | atravessa | depois | resultado |
|---|---|---|---|---|
| 1 ctrl | 0, 0, 0 | fechou direto | captura −10,1; lâmina 34,4° | **OK** |
| 2 prof +12 | 0, +12, 0 | **travou na borda** (erro 16,6 mm, J1 aquém); recuos 1 e 2 não mudaram; **recuo 3 (prof −8) entrou** | captura −9,1; lâmina 34,4° | **OK — resgate da varredura** |
| 3 prof −12 | 0, −12, 0 | fechou (o dedo entrou: verdade prof −4 mm) | captura não fecha: erro de prof −9…−11 mm nas 10 iterações (anel segura o dedo); reassentou, idem | ABORT |
| 4 alt −10 | 0, 0, −10 | fechou | dedo 10 mm mais baixo já comprimiu a lingueta a 14 mm na captura; destrava desceu 5,5 mm até os 20 de fim de curso (gatilho destravado de fato) e o critério "curso ≥ 10 mm desde a captura" reprovou, 2× | ABORT |
| 5 alt +10 | 0, 0, +10 | dedo 10 mm acima roça no arame de cima: eixo oscilou +17,6 → −18,7 → −8,9 e só na 5ª iteração deu "travou"; 1 recuo (alt −5) e o orçamento de 5 iterações acabou | — | ABORT |
| 6 eixo +13 | +13, 0, 0 | **= run 4 de 17 Out**: dedo engatado na verdade (degrau−anel 8,4 mm), missão vê +12 mm faltando; 4 recuos com varredura, sempre o mesmo ponto | — | ABORT |
| 7 eixo −13 | −13, 0, 0 | fechou com o dedo raso (degrau−anel 27 mm: arame na quina) | captura −14,4; no destrava o anel escapou (lingueta 11,7 → 0, anel sobe 35 mm) | ABORT (perda do RELATORIO32) |

**2/7.** A varredura faz exatamente o que foi desenhada para fazer — a
run 2 é a borda do furo por erro de profundidade, e o terceiro recuo
entrou — e não faz nada pelos outros cinco, que são de outra natureza:

- **Viés no eixo (runs 6 e 7)** é o caso da run 4 de 17 Out e a
  varredura não o alcança, nos dois sentidos: para um lado o dedo
  engata certo e a missão insiste (run 6); para o outro engata raso e
  perde na libera (run 7).
- **Erro de altura (runs 4 e 5)**: para baixo o dedo entra e comprime a
  lingueta antes da hora, e o critério do destrava, que conta a descida
  desde a captura, não enxerga os 14 mm já descidos; para cima o dedo
  roça no arame superior e gasta as 5 iterações oscilando antes de o
  detector de contato (que exige "não avança" entre duas iterações
  consecutivas) disparar.
- **Profundidade para o outro lado (run 3)**: o dedo entra (folga de
  −12 existe) e a captura, que exige ±6 mm de profundidade em relação a
  um alvo 12 mm fora, não fecha nunca — o anel segura o dedo.

Nas runs 3, 4 e 6 **o dedo estava engatado** e a missão abortou por
acreditar na própria estimativa contra o contato.

## Conclusão

1. **Varredura lateral: entra**, resolve a borda do furo por erro de
   profundidade (run 2) e não regrediu o caso nominal (run 1). Custo
   ~15 s de simulação por recuo.
2. **O que a bateria mostrou é maior que a varredura.** Cinco dos seis
   vieses sintéticos derrubam a missão com o dedo dentro do furo, e em
   três deles (prof −12, alt −10, eixo +13) o engate real era bom. A
   missão confia na estimativa da parede contra a evidência do contato.
   Propostas, para decisão do Marco:
   - **Engate por contato na atravessa/captura**: quando "não avança com
     J1 aquém" se repete no mesmo ponto a menos de ~15 mm do alvo no
     eixo, tratar como engatado e zerar o eixo da captura no ponto do
     contato (as fases seguintes já são relativas à captura). Cobre as
     runs 6 e 4 de 17 Out.
   - **Captura fecha por contato em profundidade**: o mesmo sinal
     (progresso nulo em prof com o J1 aquém) vale como "o anel já
     segura o dedo". Cobre a run 3.
   - **Curso do destrava contado desde a entrada**, não desde a captura:
     a altura da ponta no fim da aproxima_lateral é a referência de
     "lingueta solta"; o que desceu desde lá é o que a lingueta cedeu.
     Cobre a run 4. Na bancada é a mesma medida (T265).
   - **Orçamento de iterações separado dos recuos** (run 5): hoje as
     oscilações consomem as 5 iterações antes de a varredura chegar ao
     passo útil.
3. **O viés no eixo é o que a REFINE não corrige** (o laser dá
   profundidade e yaw; o lateral vem só da tag). Na bancada a T265 tem
   viés no eixo óptico, e o eixo do furo é perpendicular a ele na pose
   de manipulação — ou seja, o caso da run 6/7 é menos provável lá que o
   da run 2. Mas a run 4 de 17 Out mostra que acontece em simulação com
   a tag a 0,68 m.

Pastas: `docs/baterias/varredura/<rotulo>/run1` com `sonda_engate.csv`
(contatos e bags fora do git). Scripts da bateria no scratchpad da sessão.
