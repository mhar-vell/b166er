# RELATORIO33 — engate: inserção −10 ± 4, subida na libera e o raio efetivo do arco — 2026-10-17

Sequência do RELATORIO32 (sonda do engate). Marco: "manda ver" nas
propostas 1 e 2. Branch `feat/engate-subida`.

## O que mudou

1. **Atravessa e captura a −10 ± 6 mm no eixo** (era −8 ± 8; ±4 foi
   tentado e não fecha — ver a bateria v2): o alvo no meio dos 20 mm do
   degrau, em vez de −8 (`chave_seccionadora_task.yaml`).
2. **Subida súbita na libera** (`chave_mission.py`): se a ponta (T265)
   sobe mais de `~libera_subida_m` (8 mm) em `~libera_subida_janela_s`
   (0,3 s), a fase falha com `subiu` e o reassentar existente entra
   (sobe, atravessa, recaptura, destrava, puxa de novo; `~reassenta_subida`
   True). É a assinatura do escorregão da sonda e vale na bancada.
3. **Fim do arco2 em 30° nominal** (y 100 mm), não 33. Achado da primeira
   bateria desta branch: com os dois fixes acima a libera e o arco1
   ficaram limpos e a perda mudou de lugar — fim do arco2, lâmina no
   batente (34,4°) com o controlador ainda empurrando, e o dedo saltou
   (eixo 25 → 59 mm, alt −25 → −65 mm num ciclo da sonda). O anel vai
   pressionado ~20 mm para baixo na lingueta durante o arco, então a
   circunferência que ele descreve tem raio efetivo ~180 mm: o ponto a
   33° nominal (y 109 mm) exige a lâmina a asin(109/180) = 37°, além do
   batente. A 30° nominal (y 100) a lâmina fecha em ~34° no limite, e
   como o desengata em whole-body não a deixa recuar, termina acima do
   limiar de 25°.

## Bateria `engate_fix_v1` (antes do item 3): esq × 2

| run | captura (eixo) | libera/arco1 | resultado |
|---|---|---|---|
| 1 | −9,8 mm | limpos (deriva +1,4) | **OK** |
| 2 | −7,5 mm | limpos (deriva +0,7) | ABORT no fim do arco2, lâmina no batente (ver item 3) |

## Bateria `engate_fix_v2` (os três fixes, tolerância ±4): esq × 3

| run | captura (eixo) | resultado |
|---|---|---|
| 1 | −9,4 mm | **OK** (arco2 fechou com a lâmina em 31,6°, final 32,8°) |
| 2 | −10,8 mm | **OK** (final 34,4°) |
| 3 | não fechou | ABORT: a captura oscilou em eixo entre −7 e +4…+12 mm em 5 iterações, reassentou, oscilou de novo |

**±4 mm está abaixo do que o estimador dá.** O eixo é o J1; o servo
reportava "postura alcançada" (erro estimado ≤ 0,5°) enquanto a ponta
medida no frame da parede alternava −7 ↔ +4…+12 mm: a estimativa do J1
sem encoder erra ~1°, que a 0,6 m de alcance são ~10 mm, e a correção
seguinte passa para o outro lado. Com ±8 essas oscilações fechavam na
primeira ou segunda; com ±4, nunca. Tolerância → **±6** (alvo −10
mantido: o arame fica entre −4 e −16 mm) e o detector de subida na
libera cobre o que escapar.

## Bateria `engate_fix` (os três fixes, tolerância ±6): esq × 6

## Bateria `engate_fix_v3` (tolerância ±6, reassentar antigo): esq × 2

| run | captura (eixo) | libera | resultado |
|---|---|---|---|
| 1 | −9,0 mm | limpa | **OK** (final 30,2°) |
| 2 | −9,1 mm | **SUBIU 10,4 mm em 0,3 s** a 8,6° de lâmina — detector funcionou | ABORT: o reassentar "sobe 25 mm" não recuperou |

A run 2 é a primeira vez que o detector do item 2 dispara, e mostra que a
recuperação herdada não serve para esse caso. A sonda (`sonda_engate.csv`):

| t (sim) | fase | lâmina | degrau−anel eixo / prof / alt (mm) | lingueta | contatos |
|---|---|---|---|---|---|
| 2610,0 | libera | 8,6° | 11,5 / 2,5 / −30,3 | 20,0 | olhal×t_tip 53 N |
| 2611,5 | libera | 8,3° | 14,8 / 0,8 / **−7,7** | **0,0** | olhal×t_tip 8 N |
| 2613,0 | libera | 8,1° | 13,3 / 1,7 / **+12,2** | 0,0 | — |
| 2617,5 | reassenta_sobe | 5,9° | 10,7 / −4,3 / +5,7 | 0,6 | olhal×t_tip 88 N |
| 2623,5 | atravessa | 6,1° | 9,8 / −6,1 / −10,4 | 6,4 | olhal×t_tip 54 N |
| 2625,0 | atravessa | 5,9° | 9,1 / −4,5 / **+4,1** | 0,0 | olhal×t_tip 84, olhal×t_rod 31 N |
| 2635,5 | atravessa | 6,3° | 9,2 / −6,4 / −13,1 | 10,7 | olhal×t_tip 98, olhal×t_rod 70 N |
| 2640,0 | atravessa | 5,9° | 9,0 / −4,0 / +7,2 | 0,0 | olhal×t_tip 46, olhal×t_rod 21 N |

O escorregão é o mesmo do RELATORIO32: a lingueta devolve 20 → 0 mm num
ciclo e o anel sobe 40 mm (alt −30 → +12). Depois disso o arame está
**em cima** do degrau, e "subir 25 mm e atravessar de novo" só empurra o
degrau contra o arame por cima (alt oscila entre −16 e +7 com a lingueta
comprimida a 10–13 mm pelo próprio anel, contato olhal×tool_rod de 70 N):
o dedo não volta a entrar por baixo do arame sem sair do furo. Daí o
commit seguinte: `subiu` sai pelo eixo do furo (`_sai_pelo_eixo`, os
mesmos degraus do ABORT) e reentra pela `aproxima_lateral`, como na
primeira vez.

## Bateria `engate_fix` (os três fixes, ±6, saída pelo eixo no `subiu`): esq × 6

| run | captura (eixo) | libera / arcos (sonda: degrau−anel eixo, lingueta) | lâmina final | resultado |
|---|---|---|---|---|
| 1 | −9,1 mm | limpos: eixo 9 → 15 mm, lingueta 19–20 mm, só olhal×t_tip | 33,9° | **OK** |
| 2 | −12,0 mm | limpos: eixo 11 → 19 mm, lingueta 17–20 mm, só olhal×t_tip | 31,3° | **OK** |
| 3 | −7,7 mm | limpos: eixo 8 → 13 mm, lingueta 12–20 mm, t_rod ≤ 22 N | 30,9° | **OK** |
| 4 | — | **percepção**: tag da REFINE com +42 mm de profundidade e −13 mm lateral em relação às outras runs (parede (−0,011, 3,042) vs (0,00x, 3,00x)); o laser corrigiu profundidade e yaw mas não o lateral; a atravessa ficou presa a +9,7 mm no eixo (borda do furo), dois recuos de 20 mm não mudaram nada, 5 iterações → ABORT limpo | — | ABORT (antes do engate) |
| 5 | −8,0 mm | limpos: eixo 8 → 11 mm, lingueta 12 → 20 mm, t_rod 35 N só no início da libera | 31,2° | **OK** |
| 6 | −9,4 mm | limpos: eixo 9 → 17 mm, lingueta 15 → 20 mm, t_rod 34 N na libera/arco1, depois só t_tip | 32,3° | **OK** |

**Engate**: nas três runs que engataram, a sonda não mostra o que o
RELATORIO32 mostrava — nenhum salto de eixo na libera, lingueta mantida
em ~20 mm até o desengata, contato olhal×tool_rod residual (≤ 22 N na
run 3, zero nas outras). O detector de subida não precisou disparar.
A recuperação nova (saída pelo eixo + reentrada) **ainda não foi
exercitada** nesta bateria; vale só a análise da v3.

**Run 4 não é do engate.** A missão desta run já vinha de um SEARCH ruim
(parede a 28° de yaw por 18 amostras, laser com 11 pontos) recuperado no
APPROACH; a REFINE remediu a tag com 9 amostras e saiu 13 mm deslocada no
lateral. Como o laser só corrige profundidade e yaw, o eixo do furo ficou
13 mm fora e o dedo bateu na borda (furo oval 40 × 30, folga lateral ≈
10 mm). O comportamento da missão foi o correto — contato detectado,
recuo, aborto em degraus sem tocar na chave. Candidato para depois: na
atravessa, quando "não avança" se repete no mesmo ponto, varrer ±8 mm no
lateral antes de abortar (a run 4 teria fechado).

**Resultado: 5/6, sendo 5/5 nas runs que chegaram ao engate.** Lâmina
final 30,9–33,9° (limiar 25°; batente 34,4°). Nenhuma run perdeu o
anel na libera ou nos arcos — a perda que o RELATORIO32 reproduzia em
2/4 não apareceu em 5 engates com a inserção a −10 ± 6. A captura ficou
entre −7,7 e −12,0 mm (o arame entre 8 e 12 mm da quina do degrau,
contra 2–3 mm nas runs perdidas de ontem).

## Conclusão

1. Inserção −10 ± 6 mm: o fix que resolve. A sonda mostra o arame no
   meio do degrau durante a libera inteira e a lingueta mantida.
2. Detector de subida: validado na v3 run2 (disparou no escorregão real
   em 0,3 s). A recuperação pela saída do furo está implementada mas
   **sem run que a exercite** — fica como seguro para a bancada.
3. Fim do arco2 a 30° nominal: 5/5 fecham o arco com a lâmina entre
   29,8 e 34,0° no waypoint, sem a lâmina no batente com o controlador
   empurrando.
4. Fora do engate: run 4 (percepção lateral da tag) é o próximo ponto
   fraco visível — varredura lateral na atravessa quando "não avança".

Pastas: `engate_fix_v1` (2), `engate_fix_v2` (3), `engate_fix_v3` (2),
`engate_fix` (6), com `sonda_engate.csv` por run (contatos.txt fora do git).
