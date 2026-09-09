#!/bin/bash
# A/B do reset do zero: 3 ciclos recolhendo, 3 sem. uso: teste_reset_ab.sh
/home/marco/.claude/jobs/89ade7b6/tmp/gatilho/teste_reset.sh 3 2>&1 | sed 's/^\[ciclo /[com-recolher ciclo /'
/home/marco/.claude/jobs/89ade7b6/tmp/gatilho/teste_reset.sh 3 --sem-recolher 2>&1 | sed 's/^\[ciclo /[sem-recolher ciclo /'
echo "[ab] fim"
