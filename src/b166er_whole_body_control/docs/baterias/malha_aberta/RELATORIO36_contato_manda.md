# RELATORIO36 — "o contato manda": o que passou e o que voltou — 2026-10-22/23

Sequência do RELATORIO35. Marco: "SEGUE EM FRENTE" (pendências em
simulação: contato caótico, saída pelo eixo divergindo, itens 2–3 do
RELATORIO34). Branch `feat/contato-caotico`, empilhada sobre
`feat/varredura-lateral-atravessa` (PR #100).

**Resumo.** Três baterias com viés sintético da parede. Das sete
mudanças tentadas, ficam quatro; três foram revertidas porque a sonda
mostrou que pioravam. O estado final está medido na bateria
`contato_manda3` (seção final): **2/8, sem ganho mensurável sobre a PR #100**.

## Mudanças, por destino

| mudança | commit | destino | evidência |
|---|---|---|---|
| recuo por contato **caótico** (`~ik_recuo_caotico_its` 3) | 65be96b | **fica** | contato_manda run 3 (prof +12): disparou e o passo −8 entrou |
| libera: **lâmina abrindo ≠ escorregão** (`~libera_subida_prof_max_m` 15) | 765da4e | **fica** | bag + sonda da run 5 de 21 Out: o "subiu" era a lâmina girando 3 → 32° com o dedo engatado |
| destrava: **descida total desde a entrada** (`ctx.alt_entrada`, `~curso_total_min_m` 25) | 765da4e | **fica** | contato_manda run 7 (alt −10): estagnou curto em 8,3 mm com 29,9 desde a entrada → OK |
| **teto da mira além** (`~ik_aim_max_m` 30) | 43f556d | **fica** | sem ele a IK chicoteou o braço com a ponta presa (saida_sobe 173/226 mm) |
| **captura por contato** em profundidade | 765da4e | **desligada** (`~captura_contato_its` 0) | contato_manda run 6 (prof −12): aceitou eixo −5,8 / alt +8 e o destrava estagnou curto |
| **preso por trás do degrau** | a64677e | **revertido** | contato_manda2 run 4 (prof +12): disparou com o dedo FORA do anel (sonda: prof 17–25, lingueta 0) |
| **engate por contato como hipótese** (a captura confirma, senão restaura a parede) | a64677e | **revertido** | contato_manda2 run 3: a captura se move com a parede deslocada e fecha de qualquer jeito — não confirma nada |
| **destrava com alvo de 30 mm** (fecha por fim de curso) | a64677e | **revertido** | contato_manda2 runs 5/6 (eixo −13): empurrou o dedo 40 mm para baixo fora do anel |
| teto do engate por contato 15 → **13,5 mm** | 43f556d | **fica** | borda do furo medida a 14,6–15,3 (2 runs), canto do degrau a 10,6–12,4 |

## A lição dos revertidos

As três mudanças revertidas tentavam decidir, pela estimativa, se o
dedo estava no anel. Nenhuma conseguiu, pelo mesmo motivo: o J1 sem
encoder erra ~1° ≈ 10 mm na ponta, e essa é a escala das diferenças que
elas tentavam separar (borda × canto, dedo dentro × fora). Os sinais que
**funcionaram** são os que não dependem dessa precisão: o ponto repetido
da trava (engate por contato, RELATORIO35), a falta de progresso
(caótico), a ponta correndo dezenas de mm (lâmina abrindo) e a descida
acumulada (curso desde a entrada). Na bancada o sinal que falta é o mais
direto: força ou corrente no J2/J3 durante o destrava diria se o anel
resiste. Fica como sugestão para o hardware.

## Bateria `contato_manda` (765da4e + 65be96b): 3/7

| run | viés | resultado | causa |
|---|---|---|---|
| ctrl | 0 | ABORT | REFINE ficou pela tag (laser discordou 7°), olhal 13 mm antes do real; dedo passou e enganchou atrás do degrau; os recuos laterais afundaram a ponta |
| prof +12a | 0,+12,0 | ABORT | a borda bloqueou a 14,7–14,9 (dentro do teto de 15): engate por contato com o dedo fora |
| prof +12b | 0,+12,0 | **OK** 34,4° | caótico disparou, passo −8 entrou |
| alt +10a | 0,0,+10 | **OK** 31,0° | |
| alt +10b | 0,0,+10 | ABORT | destrava fechou por tolerância com a lingueta em 9 mm; libera perdeu o olhal |
| prof −12 | 0,−12,0 | ABORT | captura por contato aceitou estado ruim; reassentar divergiu |
| alt −10 | 0,0,−10 | **OK** | **curso desde a entrada** fechou o destrava (29,9 mm) |

## Bateria `contato_manda2` (+ a64677e): 3/8

| run | viés | resultado | causa (sonda) |
|---|---|---|---|
| ctrl a, b | 0 | **OK** ×2 | |
| prof +12a | 0,+12,0 | ABORT | engate por contato na borda (14,6–15,3); captura "fechou" com o dedo a 25 mm do degrau; destrava desceu 37 mm fora do anel |
| prof +12b | 0,+12,0 | ABORT | "preso por trás" falso positivo (dedo fora, prof 17–25) |
| eixo −13a, b | −13,0,0 | ABORT ×2 | dedo raso (27–34 mm do degrau, na quina); destrava de 30 mm empurrou 40 mm para baixo; libera perdeu |
| alt +10a | 0,0,+10 | ABORT | |
| alt +10b | 0,0,+10 | **OK** | |

## Bateria `contato_manda3` (estado final da branch: 43f556d)

| run | viés | resultado | causa |
|---|---|---|---|
| ctrl a | 0 | ABORT | **percepção**: tag perdida na remedida do APPROACH, 30 s de giro sem reencontrar (não é manipulação) |
| ctrl b | 0 | **OK** 31,4° | |
| prof +12a | 0,+12,0 | ABORT | caótico disparou 2×, 4 recuos, 11 iterações sem fechar |
| prof +12b | 0,+12,0 | **OK** 34,4° | |
| eixo −13a | −13,0,0 | ABORT | dedo raso (caso conhecido desde o RELATORIO34) |
| eixo −13b | −13,0,0 | ABORT | libera SUBIU 9,5 e depois perdeu o olhal (+39,7 mm) |
| alt +10a | 0,0,+10 | ABORT | destrava fechou com a lingueta em 10,6 mm, gatilho não soltou, lâmina 0,5° — o caso que o alvo de 30 mm atacava (revertido) |
| alt +10b | 0,0,+10 | ABORT | caótico disparou 4× com o erro em 7–8 mm (logo acima da tolerância de 6) e consumiu a atravessa; saída divergiu |

**2/8.** Somando as três baterias de hoje, nenhuma configuração se
destacou: 3/7, 3/8, 2/8, com o mesmo caso passando numa run e falhando
na seguinte (prof +12: OK/ABORT nas três; alt +10: 1/2, 1/2, 0/2). Com
uma ou duas runs por viés a variação entre execuções é maior que o
efeito das mudanças, e **não dá para afirmar que alguma delas melhora o
resultado**. O que dá para afirmar é qualitativo, pela sonda:

- o contato caótico dispara no caso para que foi feito (prof +12, borda),
  mas também quando o erro só está um pouco acima da tolerância (alt +10b:
  7–8 mm contra 6) e aí gasta a fase em recuos — o limiar de 3 iterações
  é curto demais para esse caso;
- a descida desde a entrada resolveu a run em que disparou (alt −10);
- o "lâmina abrindo" não disparou nas runs de hoje;
- o viés de 13 mm no eixo para o lado negativo continua sem solução
  (0/6 nas três baterias): o dedo engata raso, na quina do degrau.

## Pendências

- **Não recomendo mergear `feat/contato-caotico` agora.** As mudanças são
  defensáveis uma a uma pela sonda, mas o conjunto não mostrou ganho
  mensurável; a PR fica aberta como registro e base de discussão.
- Para medir efeito de verdade: 5 runs por viés (40 por bateria, ~5 h),
  ou isolar uma mudança por bateria.
- Contato caótico: limiar de iterações maior, ou exigir erro > 2× a
  tolerância.
- Viés no eixo negativo (dedo raso): sem proposta ainda.
- Hardware: corrente/força no J2/J3 durante o destrava diria se o anel
  resiste — o sinal que a estimativa não consegue dar.
- PR #100 (varredura + engate por contato) continua sendo a parte com
  evidência mais forte (3/3 nos vieses do eixo positivo, RELATORIO35).

## Medição com cinco execuções por viés (23–24 Out)

Marco: "continue com os pontos da simulação" (medir a #101, viés −13,
contato caótico). Antes da medição entraram 550aeb0 (o caótico passa a
exigir a ponta saltando ≥ 10 mm) e a sonda de inserção, desligada em
f08192e depois do teste rápido (`sonda_insercao_smoke`): no viés −13 a
verdade do Gazebo mostrou o degrau a 7,6 mm do anel ao fim da
atravessa, a posição de uma captura boa, e a sonda empurrou o dedo
20 mm para fora. **A hipótese "dedo raso na atravessa" estava errada**:
no viés −13 a perda acontece depois, na captura ou no destrava.

### Bateria `medida_101`: 8 vieses × 5, intercalados — 18/40

| viés | acertos |
|---|---|
| controle | **5/5** |
| altura −10 | **5/5** |
| profundidade +12 | 2/5 |
| eixo +13 | 2/5 |
| eixo +8 | 2/5 |
| eixo −13 | 1/5 |
| altura +10 | 1/5 |
| profundidade −12 | 0/5 |

Num eixo +13 o engate por contato nunca valeu com quatro travas
idênticas em 9,8 mm, porque a primeira trava (13,0, antes de o dedo
assentar) mantinha o espalhamento em 3,2 > 3. Corrigido em f93d139: o
critério olha só as duas últimas travas.

### Confirmação `medida_101_eixo`: eixo +13 × 5, com f93d139 — 1/5

| run | atravessa | depois | resultado |
|---|---|---|---|
| 1 | travas 11,5/11,3 recusadas (prof/alt fora da tolerância) | libera perdeu o olhal | ABORT |
| 2 | **engate por contato** 10,0/10,1 | chave 31,3° | **OK** |
| 3 | travas 8,4/8,5 recusadas, depois caótico; 10 iterações | — | ABORT |
| 4 | **engate por contato** 10,5/10,6 | libera SUBIU 10,2, recuperação falhou | ABORT |
| 5 | **engate por contato** em −7,0/−6,6 | libera SUBIU 12,2, chave não abriu (0,6°) | ABORT |

A correção fez o engate por contato disparar em 3 de 5 (antes 2 de 5),
mas as perdas foram para a libera. No total de hoje, eixo +13 fica em
**3 de 10**. Na PR #100 eram 3 de 3, com n pequeno; com n = 10 o caso
está longe de resolvido.

### Conclusão da medição

- **Funciona de forma reprodutível:** o caminho sem viés (5/5) e o
  altura −10 (5/5, coberto pela descida desde a entrada).
- **Não funciona:** os outros seis vieses ficam entre 0 e 2 de 5. A
  maioria das perdas acontece depois do engate (libera SUBIU, perdeu o
  olhal, chave não abriu), não na atravessa.
- **Recomendação:** mergear da #101 só o que tem efeito medido — a
  descida do destrava desde a entrada — se o Marco quiser; o resto
  continua sem evidência de ganho. O próximo trabalho útil é a libera
  depois de um engate fora do centro, e o viés −13 precisa de um
  desenho da geometria do degrau antes de qualquer código
  (`feedback_desenhar_para_esclarecer`).
