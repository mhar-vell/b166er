# RELATORIO35 — engate por contato na atravessa — 2026-10-21

Item 1 do RELATORIO34 (Marco: "pode seguir com o item 1"). Branch
`feat/varredura-lateral-atravessa` (PR #100), commits 74b733b e d7230e2.

## O que mudou

1. **Engate por contato** (`chave_mission.py`, `_engate_por_contato` +
   `_reach_by_iterative_ik`). Na fase `atravessa` (`~engate_contato_fases`),
   a partir do segundo travamento por contato (`~engate_contato_min_travas`
   2), se todos os travamentos ficaram no mesmo ponto do eixo
   (espalhamento ≤ `~engate_contato_tol_mm` 3), a menos de
   `~engate_contato_max_mm` (15) do alvo e com profundidade/altura dentro
   da tolerância da fase, o bloqueio é o arame no canto do degrau: a fase
   fecha e `ctx.wall_pos` é deslocada pelo eixo do furo até o ponto do
   contato. A captura e as fases seguintes (já relativas à captura) partem
   do engate real. O contato manda sobre a estimativa.
2. **Orçamento de iterações recomeça a cada recuo** (`it_desde_recuo`).
   Com quatro recuos as cinco iterações acabavam antes de um novo
   travamento poder ser detectado (runs 5 e 6 da bateria de vieses).

### Por que decidir na segunda trava, com teto de 15 mm

A primeira versão esperava a varredura esgotar (quatro recuos). Na
bateria v1 (eixo +13) o passo de +8 mm em profundidade deixou o dedo
**escorregar 10 mm além do canto do degrau**: a captura fechou a −5,2
mm estimados (−18 reais, arame no fim do degrau) e a libera perdeu o
anel (deriva −29 mm). A geometria separa os dois casos sem esperar:

| bloqueio | eixo no frame da parede | depende do viés lateral? |
|---|---|---|
| borda do furo (erro lateral) | ≈ 16,6 mm (10 de inserção + raio do arame) | não — geometria fixa |
| canto do degrau (erro no eixo) | ≈ viés − 2 mm (9,7 em 17 Out; 10,6–12,4 hoje) | sim |

Até ~15 mm de viés no eixo os dois não se confundem; acima disso é
problema da REFINE. A borda também não satisfaz "mesmo ponto": muda de
lugar com a posição lateral e com o escorregão (hoje: 46,9 e −8,3).

## Bateria `engate_contato_v1` (decisão só com a varredura esgotada): interrompida

| run | viés | resultado |
|---|---|---|
| 1 ctrl | 0 | **OK** (31,0°) — captura rasa (−7,5), libera SUBIU 8,6 mm, **primeira recuperação completa pela saída do furo**: saída pelo eixo, reentrada lateral, atravessa travou 2× (varredura), recaptura, destrava 16 mm, libera, arcos |
| 2 eixo +13 | +13, 0, 0 | ABORT: travou 2× no canto (14,2/12,8), o 3º passo da varredura (+8 prof) afundou o dedo 10 mm além do canto, captura −5,2, libera PERDEU (−29 mm) |

Interrompida na run 3 para corrigir a decisão (commit d7230e2).

## Bateria `engate_contato` (decisão na 2ª trava, teto 15): 3/5

| run | viés | atravessa | resultado |
|---|---|---|---|
| 1 eixo +13 | +13, 0, 0 | travou 11,3 → recuo (0,−5) → travou 11,1: **ENGATE POR CONTATO**, parede −11,2 mm pelo eixo | **OK** 31,2° (captura −11,1; destrava 17,3) |
| 2 eixo +13 | +13, 0, 0 | travou 11,5 → recuo → travou 10,6: **ENGATE POR CONTATO** | **OK** 32,3° (captura −11,1) |
| 3 eixo +8 | +8, 0, 0 | fechou direto (8 mm cabe na tolerância ±6 do canto) | **OK** 33,1° (captura −12,0) |
| 4 prof +12 | 0, +12, 0 | escorregou na borda (eixo +46,9, erro 55 mm), travou em −8,3: **recusado** (espalhamento 55); recuo 2; as 5 iterações seguintes oscilaram (+46 → −34 → −9 → −9 → +12) sem a assinatura "não avança", e o 3º recuo (prof −8, o que entra) nunca veio | ABORT (8 it, 2 recuos) |
| 5 alt +10 | 0, 0, +10 | fechou | captura alt −15; libera SUBIU 11,3; a saída pelo eixo divergiu (`saida_sobe` 69 → 173 mm, J2 45 → 11 → 61°, rolagem 39°) | ABORT |

**Item 1 fez o que devia: 3/3 nos vieses do eixo**, dois deles pelo
engate por contato na segunda trava, com o deslocamento aplicado (11,2
mm) igual ao viés menos a folga do canto. O critério recusou
corretamente a borda do furo (run 4).

**O que a bateria mostrou além do item 1:**

- **Run 4 (prof +12) é variabilidade do contato na borda**, não do
  engate por contato: na bateria de vieses a mesma run travou três vezes
  limpas e o 3º recuo entrou; hoje o dedo escorregou pela borda
  (sonda: 30 mm fora em profundidade, lingueta tocada por fora) e as
  iterações seguintes não repetiram "não avança". O detector de contato
  exige progresso nulo entre duas iterações consecutivas; um contato
  caótico escapa dele. Candidato: contar "iterações sem fechar com o
  eixo parado" como travamento também.
- **Run 5 (alt +10): a recuperação do `subiu` pela saída do furo
  divergiu** (1/2 hoje — na run 1 da v1 funcionou de ponta a ponta). Com
  o arame por cima do degrau segurando o dedo, o `saida_sobe` (30 mm
  para cima a partir da ponta) mandou a IK para posturas erradas.
  Separado do item 1; fica como pendência.

## Conclusão

Engate por contato: entra. Resolve o caso da run 4 de 17 Out (reproduzido
duas vezes com viés de 13 mm no eixo) sem roubar o caso da borda da
varredura. Pendências novas: detector de contato para contato caótico
(run 4) e a saída pelo eixo com o dedo preso (run 5).

Pastas: `docs/baterias/engate_contato_v1/` (2 runs) e
`docs/baterias/engate_contato/` (5 runs), `sonda_engate.csv` por run.
