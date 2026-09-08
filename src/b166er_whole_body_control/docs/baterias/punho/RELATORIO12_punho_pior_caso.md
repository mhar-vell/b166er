# Pior caso do punho: J4 a 2,9 N·m — 2026-09-08

Marco: "merges realizados, roda o pior caso de 2,9 N·m". Limite = peso
próprio do punho (1,7 N·m) + 1 × 1,2 N·m da spec (com 4,2 era 2 ×).
Branch local `exp/punho-pior-caso` (e06538c); o URDF da main fica em 4,2.
Pose ref × 5, híbrido, sonda do J4 por fase em cada execução.

## Resultado: 5/5

| run | destrava (desc./lingueta) | soltura (recuo/deriva) | lâmina |
|---|---|---|---|
| 1 | 13,9 / 9,0 mm | 55,5 / −0,4 mm | 30,1° |
| 2 | 13,7 / 7,2 | 56,1 / −0,8 | 28,6° |
| 3 | 12,1 / 2,9 | 49,0 / −0,5 | 30,3° |
| 4–5 | 13,7–13,9 / 7–9 | 55–56 / −0,4..−0,8 | 28,6–30,1° |

## Esforço do J4 por fase (`tabela.txt`; sat = amostras ≥ 95 % de 2,9)

| run | destrava | libera | arco1 | arco2 |
|---|---|---|---|---|
| 1 | 1,72 N·m, 0 %, +3° | 2,90, **59 %**, **+7°** | 2,90, 23 %, +4° | 2,90, 50 %, +5° |
| 2 | 1,59, 0 %, +1° | 2,90, **65 %**, **+16°** | 2,90, 27 %, +3° | 2,90, 6 %, +5° |
| 3 | 1,77, 0 %, +1° | 2,90, **57 %**, **+22°** | 2,90, 12 %, +2° | 2,82, 8 %, +2° |
| 4 | 2,54, 0 %, +6° | 2,90, **60 %**, **+15°** | 2,90, 31 %, +4° | 2,90, 46 %, +5° |
| 5 | 2,49, 0 %, +2° | 2,90, **65 %**, **+21°** | 2,90, 23 %, +3° | 2,90, 31 %, +3° |

- **Destrava**: 1,5–2,5 N·m, nunca satura, punho cede ≤ 6°. O empurrão cabe
  mesmo sem margem.
- **Libera**: satura 57–65 % do tempo e o punho cede **7 a 22°** (com 4,2
  eram 48 % e 12°). O anel não escapa em nenhuma (deriva de eixo ≤ 0,8 mm)
  porque a base faz o puxão e a ponta acompanha o anel enquanto o punho
  dobra. É a fase crítica, confirmada no pior caso.
- **Arcos**: saturam 6–50 % com cedência ≤ 5°: o anel já está solto e a
  ferramenta só acompanha.

## Leitura

1. Com um punho do tamanho da spec **sem margem**, a missão ainda abre a
   chave 5/5: o híbrido não depende do punho para o empurrão, e no puxão o
   punho pode ceder até ~20° sem perder o anel.
2. O que a bancada deve medir no libera é a **cedência do J4** (a T265 vê a
   ponta; a deriva de eixo é a guarda) — 22° é o maior valor simulado sem
   perda; acima disso o modelo não garante nada.
3. Recomendação: manter 4,2 N·m como modelo (derivação documentada) e
   citar o pior caso no artigo como faixa: "libera satura 48–65 % e o
   punho cede 12–22° entre 4,2 e 2,9 N·m, sem perder o anel".
