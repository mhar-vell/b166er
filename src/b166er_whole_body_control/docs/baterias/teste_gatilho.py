#!/usr/bin/env python3
"""Bancada do gatilho (2026-09-03): a lâmina só abre depois que o olhal desce?

A) puxa o olhal para fora (+Y da parede) com 10 N por 3 s SEM descer -> a
   lâmina deve ficar ~0°.
B) empurra o olhal para baixo (-Z) com 8 N e para fora com 10 N por 4 s ->
   a lingueta desce >= 12 mm e a lâmina abre.
Lê as duas juntas pelo Gazebo. Reseta a chave no fim.
"""
import math, time, sys, rospy
from geometry_msgs.msg import Wrench, Point
from gazebo_msgs.srv import ApplyBodyWrench, GetJointProperties, SetModelConfiguration, GetLinkState

MODEL = "chave_seccionadora_fixture"
BODY = MODEL + "::chave_lingueta"          # o olhal está fundido na lingueta (junta fixa)
FORA = 20.0                                 # N para fora: acima dos 7,5 N que o atrito da junta segura

def juntas():
    gj = rospy.ServiceProxy("/gazebo/get_joint_properties", GetJointProperties)
    b = gj("chave_blade_joint"); l = gj("chave_lingueta_joint")
    return math.degrees(b.position[0]), 1000.0 * l.position[0]

FORA_W = None   # versor "para fora da parede" no mundo (preenchido no main)

def wrench(fx, fy, fz, dur):
    """Força NO OLHAL (0,0,0.2 acima do pivô, eixos do mundo), componentes no
    frame da chave: +Y = para fora da parede, -Z = para baixo.

    Frame WORLD de propósito: o gazebo_ros_api_plugin calcula o torque com
    reference_point x força, mas aplica força/torque como se estivessem em
    coordenadas do MUNDO mesmo quando reference_frame é um link — com o
    fixture girado 180° isso mandava o "para fora" para DENTRO da parede,
    e foi por isso que as duas primeiras versões deste teste não abriam a
    chave (2026-09-03). E a origem do link é o PIVÔ: aplicar em (0,0,0)
    dá torque zero."""
    aw = rospy.ServiceProxy("/gazebo/apply_body_wrench", ApplyBodyWrench)
    w = Wrench()
    w.force.x, w.force.y, w.force.z = fy * FORA_W[0], fy * FORA_W[1], fz
    r = aw(body_name=BODY, reference_frame="world", reference_point=Point(0, 0, 0.200),
           wrench=w, start_time=rospy.Time(0), duration=rospy.Duration(dur))
    return r.success

def reset():
    sc = rospy.ServiceProxy("/gazebo/set_model_configuration", SetModelConfiguration)
    sc(MODEL, "chave_fixture_description", ["chave_lingueta_joint", "chave_blade_joint"], [0.0, 0.0])

if __name__ == "__main__":
    rospy.init_node("teste_gatilho", anonymous=True)
    for s in ("/gazebo/get_joint_properties", "/gazebo/apply_body_wrench", "/gazebo/get_link_state"):
        rospy.wait_for_service(s, timeout=10)
    # direção "para fora da parede" no mundo: normal do olhal_link (eixo Y do frame da chave)
    gl = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
    st = gl(BODY, "world").link_state.pose
    import tf.transformations as T
    q = [st.orientation.x, st.orientation.y, st.orientation.z, st.orientation.w]
    R = T.quaternion_matrix(q)[:3, :3]
    fora = R[:, 1]                     # +Y do frame da lâmina = para fora da parede
    FORA_W = fora
    print("olhal/lingueta em (%.3f, %.3f, %.3f); 'para fora' = (%.2f, %.2f, %.2f)" % (st.position.x, st.position.y, st.position.z, *fora))
    reset(); time.sleep(1.0)
    print("repouso: lâmina %.2f°, lingueta %.1f mm" % juntas())

    print("\nA) %.0f N para fora, 3 s, SEM descer (só o atrito seguraria 7,5 N)" % FORA)
    wrench(0, FORA, 0, 3.0)
    for k in range(6):
        time.sleep(0.5); print("   t=%.1fs lâmina %.2f°, lingueta %.1f mm" % ((k + 1) * 0.5, *juntas()))
    a_final = juntas()
    time.sleep(1.5)

    print("\nB) 8 N para baixo + %.0f N para fora, 4 s" % FORA)
    wrench(0, FORA, -8.0, 4.0)
    l_max = 0.0
    for k in range(10):
        time.sleep(0.5); j = juntas(); l_max = max(l_max, j[1]); print("   t=%.1fs lâmina %.2f°, lingueta %.1f mm" % ((k + 1) * 0.5, *j))
    b_final = (juntas()[0], l_max)   # a lingueta volta pela mola depois que a lâmina gira: vale o MÁXIMO
    time.sleep(2.0); print("   solto: lâmina %.2f°, lingueta %.1f mm" % juntas())

    ok = a_final[0] < 2.0 and b_final[1] >= 10.0 and b_final[0] > 5.0
    print("\nRESULTADO: %s  (A: lâmina %.2f° sem descer | B: lingueta máx %.1f mm, lâmina %.2f°)" % ("GATILHO OK" if ok else "GATILHO FALHOU", a_final[0], b_final[1], b_final[0]))
    reset()
    sys.exit(0 if ok else 1)
