# Linha de base do escalonador Fuzzy — ganhos fixos × Mamdani — 2026-09-03 (runs 73–87)

Marco: "segue com a linha de base". `~fixed_gains = [k_pos, k_orient, λ]`
substitui a saída do Mamdani nas fases whole-body (híbrido por fase, mesma
régua por eixo, destrava com estagnação curta). Três regimes × 5 missões,
comparados com a bateria 63–67 (Fuzzy), tudo no mesmo fixture e sem
reiniciar o stack (o controlador relê o parâmetro ao assumir o laço).

| regime | k_pos / k_orient / λ | OK | destrava | libera | arco1 | arco2 | soma das fases wb |
|---|---|---|---|---|---|---|---|
| Fuzzy (Mamdani) | escalonado | 5/5 | 3,0 ± 0,3 s | 6,6 ± 1,4 s | 0,6 | 0,4 | 10,6 s |
| fixo conservador | 0,3 / 0,3 / 0,05 | 5/5 | 3,9 ± 1,1 | 6,8 ± 2,8 | 0,4 | 1,0 | 12,1 s |
| fixo médio | 0,8 / 0,8 / 0,08 | 5/5 | 2,6 ± 0,2 | 4,1 ± 1,1 | 0,4 | 0,7 | 7,7 s |
| fixo agressivo | 1,4 / 1,4 / 0,03 | 5/5 | 2,4 ± 0,2 | 2,4 ± 0,3 | 0,4 | 0,4 | 5,6 s |

Resíduos por eixo ao fechar e ângulos da lâmina (28,6–30,9°) sem diferença
entre regimes. Nenhum timeout, nenhum colapso do punho (estagnação curta).

## Leitura honesta

1. **Nesta tarefa, o escalonador Fuzzy não supera ganhos fixos.** O regime
   agressivo é o mais rápido (5,6 s contra 10,6 s do Fuzzy) com o mesmo
   sucesso e os mesmos resíduos; o conservador é o mais lento. O Fuzzy fica
   entre o conservador e o médio — perto do alvo ele está na banda NEAR
   (k_pos 0,1–0,35), que é o que o deixa lento.
2. **Por que o agressivo não paga o preço esperado**: na variante de ponta a
   velocidade cartesiana é limitada a 0,06 m/s (`MAX_CART_VEL_LOCKED`) e o
   braço a 0,25 rad/s. O teto nivela os regimes longe do alvo; a diferença
   fica só na rampa final, onde o ganho alto chega mais rápido e a
   tolerância de 4–15 mm por eixo absorve qualquer sobre-passagem. O que o
   escalonador foi feito para evitar (sobre-passagem e caça ao alvo,
   observados em 13 Ago com 0,3 m de erro inicial) não ocorre aqui porque
   as fases de contato começam a 1–3 cm do alvo.
3. **O que este teste NÃO diz**: nada sobre o rastreamento base+braço a
   partir de longe (aproximação whole-body, manobra girar-avançar-girar),
   que é onde o λ adaptativo e a banda FAR importam — essas fases hoje são
   da navegação da missão, não do Fuzzy. Uma linha de base honesta para o
   escalonador precisa também dessa bateria (Fase 3).
4. Amostra: 5 por regime, uma pose. Diferenças de 2–5 s são consistentes
   (desvios pequenos), mas não há estatística de robustez.

## Para o artigo
Seção V-A ganha a tabela e a frase: com o híbrido e a régua por eixo, o
escalonador não melhora as fases de contato em relação a ganhos fixos; o
teto de velocidade domina e a banda NEAR o torna conservador. A
contribuição do Fuzzy tem de ser sustentada (ou não) pelo rastreamento de
longe, não pelas fases de contato.
