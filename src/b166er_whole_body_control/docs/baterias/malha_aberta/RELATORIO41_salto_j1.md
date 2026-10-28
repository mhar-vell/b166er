# RELATORIO41 — salto do J1 na atravessa e o sentido do recuo — 2026-10-28

Marco, vendo a simulação: "o sistema robótico está posicionando o dedo no lado oposto da chave". Branch
`fix/atravessa-salto-j1` (commit aec8152b), empilhada sobre a PR #105 (servo sem caçar o alvo), que está sobre a #104.

## O caso que motivou

Execução 11 da bateria `destrava_lateral_mais2` (controle): na atravessa a IK pediu ao J1 uma correção de 0,6° e o braço,
sem encoder, girou 4 a 5°. Pela T265 a ponta andou 48 mm em X numa iteração e passou para o outro lado do olhal, onde
enganchou; o estimador dizia "postura alcançada" com 0,4° de erro.

## As mudanças

1. **Salto** (`~salto_eixo_m` 12 mm, `~salto_max` 2, fases atravessa e captura): se a ponta aparece além do alvo em X por
   mais de 12 mm, a missão volta na hora ao X do alvo, sem gastar iteração.
2. **Sentido do recuo por contato** (defeito achado ao reler o código): o sentido vinha do sinal do erro de eixo; com a
   ponta além do alvo, o "recuo" avançava mais 20 mm. Foi isso que levou o dedo ainda mais para o outro lado na execução
   11. Agora o recuo é sempre contra a inserção.

O passo máximo do J1 perto do olhal, proposto antes, não entrou: o salto aconteceu com um pedido de 0,6°, então limitar o
passo não o evitaria. O que ataca a causa é a PR #105.

## Bateria `salto_j1` (5 vieses × 5): 6/25

| viés | acertos | destrava_lateral_mais2 (sem #105 e sem esta) |
|---|---|---|
| controle | 2/5 | 3/5 |
| eixo −13 | 0/5 | 0/5 |
| eixo +8 | 2/5 | 0/5 |
| profundidade +12 | 0/5 | 0/5 |
| altura +10 | 2/5 | 3/5 |
| **total** | **6/25** | **6/25** |

**Saltos:** a detecção disparou 9 vezes, em 4 execuções.

- **Altura +10, execução 4:** um salto de 32 mm, a missão voltou e **abriu a chave** (34,4°). É o caso para que foi feita.
- **Profundidade +12, execuções 2, 3 e 4:** saltos repetidos e crescentes (12 → 28 → 54 mm; 36 → 38 → 47; 20 → 48). Aqui o
  "salto" provavelmente não é o J1 passando do pedido: com a parede 12 mm fora em profundidade o dedo passa **ao lado** do
  olhal, nada o segura em X, e a ponta vai além a cada correção. A volta ao X do alvo não resolve o erro de profundidade.

**Onde falhou (19 abortos):**

| falha | execuções |
|---|---|
| perdeu o olhal no arco2 | 5 |
| chave não abriu (eixo −13 ×3, prof +12, alt +10) | 5 |
| atravessa, captura ou aproximação sem fechar | 7 |
| perdeu o olhal na libera | 2 |

## Conclusão

- A volta no salto funcionou no único caso de salto verdadeiro (altura +10) e é inofensiva no resto. O sentido do recuo
  estava errado e fica corrigido.
- **No total não há ganho**: 6/25, igual à bateria anterior. As perdas no **arco2** (5 de 19) são agora a falha mais
  frequente junto com a chave que não abre, e são o próximo alvo.
- **Recomendação:** mergear junto com a #105 se o Marco quiser as correções de comportamento (tremor, sentido do recuo);
  nenhuma das três PRs empilhadas melhora a taxa de sucesso de forma mensurável.
