# run8 e o indicador de soltura — 2026-09-04

Marco: "segue com a run8, o indicador de soltura na fase libera".

## O que distinguia a run8 (não abriu, lâmina 8,9°)

Comparação fase a fase de 27 sucessos × 1 falha (bateria de poses +
revalidação da busca), eixo/prof/alt no frame da parede ao FECHAR cada fase:

| | captura eixo | libera eixo | arco1 eixo | arco2 eixo | lâmina |
|---|---|---|---|---|---|
| 27 sucessos | −1,1 … +10,3 | mesmo ±2,4 mm | mesmo ±2,4 | mesmo ±2,4 | 27–33° |
| run8 | +7,2 | **+27,2** | **+29,6** | +19,8 (+35,9 no meio) | 8,9° |

- A lingueta no fim do destrava (run8: 8,1 mm) NÃO discrimina: nos
  sucessos vai de 0,4 a 20 mm (run23 abriu com 8,4; perto/run6 com 0,4).
- O olhal não se move ao longo do eixo da chave (preso à dobradiça). A
  ponta da run8 deslizou 20 mm nesse eixo entre a captura e o fecho do
  libera: **saiu do anel**. A tolerância de eixo das fases (40 mm) não vê.
- Por que o libera nunca acusou: com o gatilho preso o olhal recua ~31 mm
  até a aba encostar no laço (lâmina ~9°; 15° ⇔ 51,8 mm ⇒ ~3,45 mm/°). O
  alvo do libera é 30 mm. **A fase fecha com ou sem soltura.**
- Por que a run8 escorregou: não fechado. Único sinal diferente: captura
  2,9 mm alta (dentro dos 10 de tolerância). Amostra de 1.

## Implementação (branch `fix/indicador-de-soltura`, commits aa6e182 + ba8251f)

- `captura` guarda a pose da ponta (eixo, prof, alt) no frame da parede
  (`ctx.pose_captura`; `_pose_na_parede`).
- `deriva_eixo_max_m` em libera/arco1/arco2: a fase FALHA ("ferramenta
  PERDEU O OLHAL") se a ponta derivar mais que isso no eixo desde a
  captura, por 5 amostras seguidas. Medido pela T265 → vale na bancada.
- `soltura_recuo_min_m` (40 mm) no arco1: indicador de soltura = recuo
  desde a captura além do curso com gatilho preso, com o eixo estável
  (log "SOLTURA observável … → SOLTA/NÃO SOLTA"; status `soltura`,
  `recuo_mm`, `deriva_eixo_mm`).

## Validação — pose "longe" (onde a run8 aconteceu)

| régua | abriu | SOLTA no arco1 | deriva no arco1 | guarda disparou |
|---|---|---|---|---|
| 10 mm (estatística) | 4/5 | 4/4 (recuo 49–60 mm) | −0,9 … +0,9 mm | 1× no libera, −10,7 mm (run2) |
| 15 mm + 5 amostras (geométrica) | **5/5** | 5/5 (recuo 47–57 mm) | −0,5 … +0,7 mm | 0 |

- O disparo a 10 mm veio depois de um destrava atípico (35 s, ponta
  desceu 27 mm, lingueta no fim de curso de 20 mm, punho cedendo, eixo
  saltou +8,5 → +15,1 no último segundo) — a ponta deslizou 10,7 mm
  **dentro da folga do furo**. Não dá para saber se abriria.
- A régua certa é geométrica: furo oval 40 × 30 mm, dedo ~10 mm ⇒ ±15 mm
  de deslizamento sem sair do anel. run8 (20 mm) continua fora.

## O que fica

- O indicador de soltura da bancada é: **arco1 fechado com recuo ≥ 40 mm
  e |deriva de eixo| ≤ 15 mm**. A lingueta (só sim) não entra.
- Por que a run8 escorregou segue aberto; a guarda agora transforma esse
  caso em falha declarada em vez de "trajetória fechou".
- Convenção de sinal: `_pose_na_parede` dá eixo = −8,6 onde o erro por
  eixo loga +8,6 (posição × erro); a deriva compara pose com pose.
