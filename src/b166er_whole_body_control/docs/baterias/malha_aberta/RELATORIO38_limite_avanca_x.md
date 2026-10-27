# RELATORIO38 — limite de descida: recuperar avançando em +X — 2026-10-27

Marco: "pode seguir com a implementação". Branch `fix/limite-descida-avanca-x`, empilhada sobre a PR #102.
Geometria de referência: Figura 6 em `docs/geometria/`.

## A mudança

Quando o destrava atinge o **limite de descida** (a ponta desceu 22 mm desde a captura sem o mecanismo parar),
a sonda mostrava o degrau curto em X, antes do plano do olhal, sem entrar no furo. A recuperação da PR #102
repetia a entrada com a mesma estimativa e recuperava 1 de 9. Esta (`_avanca_em_x`, `~limite_avanco_x_m` 15 mm):

1. sobe até a altura da atravessa;
2. avança em +X até a haste encostar no olhal ou até 15 mm;
3. corrige a estimativa da parede para que ali seja o ponto da atravessa;
4. recaptura e destrava de novo.

## Bateria `limite_avanca_x` (5 vieses × 5): interrompida na execução 9

A bateria foi parada na execução 9 porque a parede foi movida por engano no Gazebo; essa execução foi descartada.
Logo depois o Marco descreveu um procedimento de destrava diferente (seção final), que substitui o que esta
recuperação tentava consertar, e a bateria não foi retomada.

| execução | viés | resultado | o que aconteceu |
|---|---|---|---|
| 1 | controle | **OK** 33,9° | destrava normal |
| 2 | eixo −13 | ABORT | limite atingido; a subida de 49 mm falhou com o J1 travado (faltava 2,5° → 8,9° a cada iteração): algo encostava no dedo |
| 3 | eixo +8 | **OK** 32,1° | |
| 4 | profundidade +12 | sem resultado | a chave **abriu** (33,9°), mas o robô **tombou na volta** (inclinação 88° depois de ir para a postura de transporte) — anomalia fora da manipulação |
| 5 | altura +10 | não rodou | o reset falhou por causa do tombamento da execução 4 |
| 6 | controle | **OK** 34,4° | |
| 7 | eixo −13 | ABORT | libera "subiu" 8 mm e depois o limite de descida |
| 8 | eixo +8 | **OK** 31,3° | |
| 9 | profundidade +12 | descartada | parede movida; antes disso a recuperação subiu e avançou 19 mm em X, sem contato |

**A recuperação em +X disparou duas vezes com resultado útil para análise e não resolveu nenhuma.** Na execução 2 a
subida travou; na 9 ela funcionou, mas a execução foi descartada. Com isso não há evidência de que a recuperação
ajude, e o tombamento da execução 4 é um problema novo, que fica registrado.

## O que vem depois: o destrava descrito pelo Marco

Sem encoder, o destrava deve ser:

1. o degrau no centro do furo do olhal;
2. a partir do centro, o dedo anda em **+Y** até a face lateral do lado do robô (**F5**) encostar no arame;
3. com F5 encostada, o robô **desce em −Z** e **depois puxa em +Y**, mantendo embaixo.

A libera de hoje também sobe a ponta 2 a 9 mm enquanto puxa (altura fixa de −25 mm), e isso deixa a aba voltar para
o laço. As duas coisas entram juntas na próxima branch.

**Recomendação:** não mergear esta branch. Ela fica como registro da tentativa.
