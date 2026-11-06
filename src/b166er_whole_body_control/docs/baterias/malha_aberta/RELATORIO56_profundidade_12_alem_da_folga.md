# RELATORIO56 — profundidade +12: além da folga geométrica do dedo — 06 Nov 2026

Marco: "pode seguir com a profundidade +12". Análise das baterias laser_angulo, busca_haste, parada_no_eixo e
refine_sem_garra, sem mudança de código.

## O modo de falha

Em 6 missões dessas quatro baterias, a lâmina abriu mais de 5° **antes do primeiro destrava**: 5 com profundidade +12 e 1
com eixo +8. Todas abortaram. Pela sonda, a sequência é a mesma. O degrau entra 8 a 13 mm deslocado em profundidade,
bate no arame lateral, escorrega para dentro do furo, cai e empurra a lingueta até 15 a 18 mm. O gatilho solta, e os
passos seguintes da atravessa ou da busca da haste, ainda comandados com a profundidade errada, puxam o olhal e abrem a
lâmina de 5 a 26°. Quando chegam a captura e o destrava, a chave já está aberta e o olhal fora do lugar. Na execução 1 de
profundidade +12 da laser_angulo, isso começou com um salto do ombro durante o passo intermediário, que levou o degrau à
região da haste antes da hora.

## Por que não há correção pela atravessa

- **Geometria:** o degrau tem 10 × 10 mm, e o furo é uma elipse com semieixos internos de cerca de 14 mm em profundidade e 20 mm em altura. A folga em profundidade é de no máximo ±9 mm, na altura do centro, e de cerca de ±7 mm na altura de entrada. Um erro de 12 mm põe o degrau contra o arame lateral em qualquer altura. O resultado depende de como o contato escorrega no simulador.
- **Sem sinal observável a tempo:** a queda da ponta durante os passos intermediários, visível pela T265, aconteceu antes de 2 das 6 aberturas antecipadas e em 3 missões sem abertura. A lingueta empurrada durante a atravessa acontece também em missões que deram certo, e só existe na simulação.
- **O viés anula o laser:** o teste soma 12 mm à parede DEPOIS do refinamento, por cima da correção de profundidade pelo laser, que errou no máximo 2 mm desde o RELATORIO24. É um erro residual que o sistema atual não deveria ter, e maior que a tolerância geométrica do dedo.

## Opções (decisão do Marco)

1. **Trocar o viés de teste de profundidade por um valor dentro da folga, como +6 mm.** Isso testaria a robustez a um erro residual plausível, sem medir o escorregamento do contato no simulador.
2. **Manter +12 como teste de estresse,** sabendo que ele mede principalmente como o contato escorrega no simulador e que não há correção pela atravessa.
3. **Criar uma manobra de centragem em profundidade por contato,** tocando os dois arames laterais antes de entrar. Esbarra no mesmo problema: com 12 mm de erro, o degrau não entra sem bater, e o anel cede.

Nenhuma mudança de código acompanha este relatório.
