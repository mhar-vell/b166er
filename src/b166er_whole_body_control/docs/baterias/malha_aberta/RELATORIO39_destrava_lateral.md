# RELATORIO39 — destrava pela lateral (procedimento do Marco) — 2026-10-27

Procedimento descrito pelo Marco para o braço sem encoder (Figura 6 em `docs/geometria`):
1. o degrau no centro do furo do olhal;
2. o dedo anda em +Y até a face F5 encostar no arame;
3. com F5 encostada, desce em −Z e depois puxa em +Y, mantendo embaixo.

Branch `feat/destrava-lateral-f5`, a partir da main. Commits aa5fde2b (procedimento) e 71a383f7 (o arco começa
pelo alvo deslizante; critério de contato da F5).

## O que mudou

- **Captura** no centro do furo: altura +5 mm (antes −10, apoiada no fundo do olhal), tolerância de altura ±6.
- **Encostar F5** (`_encosta_f5`): anda até 16 mm em +Y; contato = a ponta parar antes do alvo.
- **Destrava** mantendo a profundidade da encostada; o curso conta os ~15 mm livres até F2 chegar ao fundo do olhal
  (curso mín 30, estagnação 25, margem 8, limite 42).
- **Libera** puxa na altura em que o destrava terminou, sem subir (antes ia a −25 mm e subia 2 a 9 mm).
- **Arcos** começam pelo alvo deslizante, não pelo ponto final (defeito antigo, achado no teste rápido).

## O que a sonda mostrou no teste rápido (2 controles)

Nas duas execuções o dedo andou em +Y e parou no arame a 11,4 mm de profundidade; o destrava levou a lingueta a 20 mm;
na libera a lingueta ficou entre 15 e 20 mm enquanto a lâmina abria — antes ela caía para 6. Uma execução abriu a chave
(34,0°); a outra perdeu o olhal na passagem para o segundo arco, pelo defeito do alvo inicial, corrigido.

## Bateria `destrava_lateral` (5 vieses × 5, intercalados): 8/25

| viés | acertos | comparação |
|---|---|---|
| controle | 3/5 | 5/5 na `medida_101` (PR #101) |
| eixo −13 | 0/5 | 1/5 |
| eixo +8 | 2/5 | 2/5 |
| profundidade +12 | 0/5 | 2/5 |
| altura +10 | 3/5 | 1/5 |

Onde falhou:

| fase | execuções |
|---|---|
| atravessa sem fechar | 4 (eixo +8, prof +12, alt +10) — falha anterior ao procedimento novo |
| **captura sem fechar** | 3 (eixo +8 ×2, prof +12) — **nova**: a captura agora pede o centro do furo com tolerância de ±6 mm em altura |
| perdeu o olhal (libera ou arcos) | 6 |
| chave não abriu | 4 (eixo −13 ×3, prof +12) |

## Conclusão

- O procedimento faz o que foi desenhado: a F5 encosta, o destrava chega ao fim de curso e a libera mantém a lingueta
  embaixo. O ganho mais claro é na **altura +10** (3/5 contra 1/5), o caso em que a libera subia e deixava o gatilho
  voltar.
- No total **não há ganho mensurável** (8/25). O controle caiu para 3/5, e apareceu uma falha nova na captura, que
  agora mira o centro do furo.
- Eixo −13 continua 0/5.

**Próximo passo combinado com o Marco:** testar 2 mm a mais de inserção em X (atravessa e captura de −10 para −8 mm).

**Recomendação:** não mergear ainda.

## Teste com 2 mm a mais de inserção (`insercao_mais2`, f974e2b7): 6/10

Marco, vendo a simulação: "o degrau poderia entrar mais um pouco no olhal". Atravessa e captura de −10 para −8 mm em X.
Cinco controles e cinco com viés −13, intercalados.

| viés | com −8 mm | com −10 mm (bateria acima) |
|---|---|---|
| controle | **5/5** (lâmina 30,8–34,4°) | 3/5 |
| eixo −13 | 1/5 | 0/5 |

- **Controle 5/5**: a F5 encostou nas cinco (14,1–14,9 mm em +Y) e a chave abriu em todas. É o melhor resultado de
  controle com o destrava pela lateral.
- **Eixo −13**: duas execuções não abriram a chave, duas atingiram o limite de descida e perderam o olhal na libera. É o
  caso em que o degrau fica curto em X, e 2 mm não compensam 13.

O ganho no controle é consistente com o que o Marco viu, mas são só cinco execuções. A comparação justa com os outros
vieses fica para uma bateria completa com os −8 mm.

## Bateria completa com −8 mm (`destrava_lateral_mais2`, 5 vieses × 5): 6/25

| viés | −8 mm | −10 mm |
|---|---|---|
| controle | 3/5 | 3/5 |
| eixo −13 | 0/5 | 0/5 |
| eixo +8 | 0/5 | 2/5 |
| profundidade +12 | 0/5 | 0/5 |
| altura +10 | 3/5 | 3/5 |
| **total** | **6/25** | **8/25** |

O 5/5 do controle no teste curto não se repetiu (3/5 aqui). Somando as duas medições com −8 mm, o controle fica em 8/10.
No conjunto, **os 2 mm a mais não mostraram ganho**.

Onde falhou com −8 mm:

| falha | execuções |
|---|---|
| chave não abriu (prof +12 ×3, eixo −13 ×3) | 6 |
| atravessa ou captura sem fechar (eixo +8 ×5, prof +12, controle) | 7 |
| perdeu o olhal (libera ou arcos) | 5 |

Duas observações do Marco durante a bateria, a investigar a seguir:

- **O dedo foi parar do outro lado do olhal** (controle, execução 11). Na atravessa a IK pediu ao J1 uma correção de
  0,6° e o braço girou 4 a 5°: pela T265 a ponta andou 48 mm em X numa iteração, com o estimador dizendo "postura
  alcançada". O dedo ficou preso atrás do olhal e a missão abortou em segurança. É o J1 em malha aberta saltando bem mais
  que o erro de ~1° habitual.
- **O braço treme**: medido na execução 12, as juntas invertem o sentido 1 a 1,8 vezes por segundo em quase todas as
  fases (J1 nas fases de IK, J2 e J3 na saída do furo), com a libera parada. É o servo das juntas caçando o alvo.
