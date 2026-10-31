# RELATORIO45 — atravessa: teto da mira, amortecimento e sentido do recuo — 2026-10-30

Marco: "pode seguir com a atravessa que não fecha". Branch `fix/atravessa-oscila` (commit 7100968b), a partir da main de
30 Out (dedo v4, libera sem subir, olhal do lab, servo sem tremor).

## O diagnóstico de partida

Na bateria do degrau de 30 mm (RELATORIO44, 17/25), 6 das 8 falhas foram a atravessa que não fecha. Nas seis a ponta
saltava dezenas de mm entre iterações (eixo +6 → −42 → +39 → −62 mm) ou alternava de lado (+11, −11, +13, −12).

## As mudanças

1. **Teto da mira** (`~ik_aim_max_m` 30 mm): a correção somada ao alvo a cada iteração acumulava sem limite.
2. **Amortecimento**: quando o erro troca de sinal entre iterações, o ganho naquele eixo cai pela metade, até 0,2.
3. **Recuo contra a inserção** (o mesmo da #106): com a ponta além do alvo, o recuo avançava 20 mm.

## Bateria `atravessa_oscila` (5 vieses × 5): 11/25 — pior

| viés | com as mudanças | degrau 30 mm (main) |
|---|---|---|
| controle | 3/5 | 5/5 |
| eixo −13 | 3/5 | 3/5 |
| eixo +8 | 3/5 | 4/5 |
| profundidade +12 | 1/5 | 2/5 |
| altura +10 | 1/5 | 3/5 |
| **total** | **11/25** | **17/25** |

| falha | execuções |
|---|---|
| atravessa sem fechar | 5 (profundidade +12 ×4, altura +10) |
| perdeu o olhal ou a libera subiu | 5 |
| destrava ou whole-body não fechou | 2 |
| captura sem fechar | 2 (eixo +8) |
| chave não abriu | 2 (eixo −13) |

## Por que não funcionou

- **O salto acontece na primeira iteração**, antes de haver mira acumulada para limitar ou troca de sinal para
  amortecer: na profundidade +12, execução 1, a ponta foi de +31 mm a −57 mm em X no primeiro passo. A atravessa leva a
  ponta da aproximação lateral (−90 mm) ao furo (−10 mm) num comando só, 80 mm de J1 em malha aberta, e o J1 passa do
  pedido. O teto e o amortecimento atuam depois, quando o dano já aconteceu.
- O amortecimento também vale na captura, e ela passou a não fechar em duas execuções: com o ganho menor, as 5 iterações
  não bastam.
- Com cinco execuções por viés, parte da queda é ruído, mas nada aqui mostra ganho.

## Conclusão

**Não recomendo mergear.** O próximo passo é atacar a primeira iteração: dividir a entrada em passos menores em X (por
exemplo, ir da aproximação a −40 mm e só então ao furo), para o J1 nunca receber 80 mm de uma vez perto do olhal.
