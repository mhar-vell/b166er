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
