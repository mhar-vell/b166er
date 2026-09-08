# Bateria whole-body com a MESMA RÉGUA — 2026-09-03 (runs 26–30)

Pedido do Marco: "merges realizados, vamos realizar os testes com a mesma
régua". `_reach_by_wholebody` passou a fechar a fase por `_fase_fechou`
(tolerância por eixo no frame da parede, 4–15 mm conforme a fase, sustentada
por 5 amostras), o controlador Fuzzy passou a servir até 2 mm (era 5) e o
timeout do modo subiu para 40 s. Stack reiniciado; 5 execuções seguidas.

| run | resultado | lâmina | destrava | libera | arco1 | arco2 | desengata | lingueta máx |
|---|---|---|---|---|---|---|---|---|
| 26 | ABORTED (chave NÃO abriu: 23,2° < 25°) | 29,3° no arco2 → 23,2° após desengata | 1,7 s (lingueta 4,9 mm) | 4,8 s | 0,4 s | 0,4 s | 6,5 s | 18,8 mm |
| 27 | ABORTED (destrava não fechou em 40 s) | – | timeout | – | – | – | – | 20,1 mm (26,6 s no fim de curso) |
| 28 | **OK** | 29,6° | 34,7 s — "lingueta em 12,3 mm — gatilho solto" | 10,9 s | 0,4 s | 0,4 s | 3,7 s | 20,2 mm |
| 29 | ABORTED (destrava não fechou) | – | timeout | – | – | – | – | 20,1 mm (9,6 s no fim) |
| 30 | ABORTED (destrava não fechou) | – | timeout | – | – | – | – | 20,1 mm (4,4 s no fim) |

Comparação: mesma régua, modo postura (runs 19/20): 2/2 OK; whole-body com
esfera de 20 mm (runs 21–25): 5/5 OK; **whole-body com a régua por eixo: 1/5**.

## Onde emperra: profundidade, não altura

Resíduo por eixo (mm, target − atual) ao longo do `destrava` nas runs com timeout:

```
run27  12:44:33  eixo +7.6  prof  -3.2  alt -17.8   (início)
       12:44:52  eixo +7.0  prof -11.0  alt  +0.3
       12:45:11  eixo +7.1  prof -11.2  alt  +0.1   (timeout)
run29  12:52:08  eixo +9.1  prof  -0.6  alt -19.2
       12:52:46  eixo +8.6  prof  -8.8  alt  -0.4
run30  12:56:05  eixo +7.3  prof  -5.2  alt -20.1
       12:56:43  eixo +6.4  prof -10.6  alt  -1.9
```

A altura (tolerância 4 mm) FECHA: a ferramenta desce os 15–20 mm e leva a
lingueta ao fim de curso (20 mm). O que não fecha é a PROFUNDIDADE
(tolerância 6 mm): enquanto desce, a ponta anda 9–11 mm em profundidade e o
servo whole-body não recupera. Hipótese a verificar: corrigir profundidade
nessa postura exige mover a BASE em direção à parede, e o plano de exclusão
(`_apply_keepout`) proíbe justamente isso; o braço, com o EE quase horizontal,
tem pouca autoridade nessa direção (nota no controlador: "a direção x do EE é
singular para o braço e só a base corrige"). No modo postura a IK recompõe a
profundidade com as juntas e a fase fecha.

Na run28 a fase fechou (35 s) e, pela primeira vez em qualquer modo, o
`destrava` sozinho soltou o gatilho (12,3 mm). Na run26 o `desengata` em
whole-body (base livre, orientação não controlada) empurrou a lâmina de volta
de 29,3° para 23,2°.

## Duas correções ao que eu tinha dito

1. **Não há viés de +8 mm no REFINE.** O olhal estimado nas cinco runs foi
   (0,20, 2,87, 0,805) contra o real (0,200, 2,870, 0,805). E a cadeia
   L5→tool_tip da cinemática é idêntica à do URDF (diferença 0,0 mm nos três
   eixos, conferido hoje). A conclusão de ontem ("REFINE estima o olhal ~8 mm
   acima do real") estava errada.
2. O que continua sem explicação é por que, no modo postura, o `destrava`
   fecha por eixo com a lingueta em só ~5 mm. Com a estimativa e a cinemática
   corretas, a hipótese que sobra é mecânica: o degrau desliza sobre o arame
   curvo do anel em vez de empurrá-lo (profundidade fora do centro), ou a
   ferramenta mede uma coisa e o contato faz outra. Precisa de medição, não
   de dedução.

## O que fica

- Commits: 5743a82 (régua por eixo no whole-body), aca81b4 (tol_pos 2 mm;
  parâmetro morto removido).
- Próximo: entender a profundidade no whole-body (keepout × base; cap de
  0,06 m/s; peso 12 da base) antes de repetir; e medir o engate degrau–arame
  no modo postura.
