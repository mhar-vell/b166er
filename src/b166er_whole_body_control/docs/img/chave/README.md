# Chave seccionadora da bancada — gatilho (2026-09-03)

Material do Marco sobre o mecanismo que trava a chave, entregue no dia
em que o gatilho entrou na simulação (PR #42). O olhal de latão está
numa LINGUETA com mola, presa à lâmina por um pino; a aba da lingueta
fica dentro do laço em U do contato fixo, e é isso que impede a lâmina
de girar. Puxar o olhal ~15 mm para baixo solta a aba; puxar ~20 mm
para fora segurando embaixo leva a aba por baixo do laço; daí a lâmina
está livre.

| arquivo | o que é |
|---|---|
| `2026-09-03_desenho_pecas.png` | desenho do Marco: lingueta tipo gatilho, olhal, lâmina |
| `2026-09-03_desenho_sequencia_abertura.png` | desenho do Marco, 4 quadros: puxa −Z, lingueta solta, gira, gira mais |
| `2026-09-03_foto_gatilho_IMG_0524.jpeg` | close do olhal de latão e das molas de arame, visto de lado (fotos reduzidas a 2000 px para o git; originais de 12 MP ficaram em ~/Downloads) |
| `2026-09-03_foto_gatilho_IMG_0527/0528/0529.jpeg` | lingueta, aba, laço em U do contato fixo, de três ângulos |
| `2026-09-03_video_abertura_IMG_0525.mov` | 34 s: puxa o olhal para baixo, gira até ~45°, fecha de volta (a aba entra sozinha pela rampa) |
| `2026-09-03_video_gatilho_lateral_IMG_0526.mov` | 1,8 s: vista lateral do gatilho |
| `2026-09-03_interpretacao_gatilho_v2.png` | leitura do mecanismo feita a partir desse material, com a sequência e as fases `destrava`/`libera` propostas |

O vídeo de 34 s (47 MB) NÃO está no git de propósito (.gitignore) — fica só na cópia
local desta pasta; se for para versionar, é caso de git-lfs.

A foto da bancada inteira, mais antiga, é `../chave_bancada_teste_sem_isolador.jpg`.
Como isso virou modelo: `b166er_robot/urdf/fixtures/chave_seccionadora_lf.urdf.xacro`
(seção LINGUETA) e `config/chave_seccionadora_task.yaml` (fases 3b/3c).
