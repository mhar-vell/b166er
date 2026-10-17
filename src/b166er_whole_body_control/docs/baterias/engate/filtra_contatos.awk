# Filtra `gz topic -e /gazebo/default/physics/contacts`: imprime t, colisão1, colisão2, |F| (N), posição, para contatos
# em que uma das colisões é da FERRAMENTA (tool_, Grip, CameraSupport). Uma linha por contato.
/^contact \{/ { c1=""; c2=""; fx=""; fy=""; fz=""; px=""; py=""; pz=""; inw=0; sec=""; nsec="" }
/collision1:/ { c1=$2 }
/collision2:/ { c2=$2 }
/body_1_wrench \{/ { inw=1 }
inw==1 && /^ *x:/ && fx=="" { fx=$2 } inw==1 && /^ *y:/ && fy=="" { fy=$2 } inw==1 && /^ *z:/ && fz=="" { fz=$2; inw=2 }
/^ *position \{/ { inp=1; next } inp==1 && /x:/ && px=="" { px=$2 } inp==1 && /y:/ && py=="" { py=$2 } inp==1 && /z:/ && pz=="" { pz=$2; inp=0 }
/^ *time \{/ { intime=1 } intime==1 && /sec:/ && !/nsec/ { sec=$2 } intime==1 && /nsec:/ { nsec=$2; intime=0 }
/^\}/ && (c1 ~ /tool_|Grip|CameraSupport/ || c2 ~ /tool_|Grip|CameraSupport/) {
  f=sqrt(fx*fx+fy*fy+fz*fz); printf "%s.%03d %s %s F=%.1f p=(%s %s %s)\n", sec, nsec/1e6, c1, c2, f, px, py, pz; fflush()
}
