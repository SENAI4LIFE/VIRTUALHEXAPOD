import pybullet as p
import pybullet_data
import time
import math
import random
import numpy as np

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.setGravity(0, 0, -9.81)

random.seed(10)
heightfieldData = np.zeros(shape=[100, 100], dtype=np.float64)
for i in range(50):
    for j in range(100):
        n1 = heightfieldData[i, j-1] if j > 0 else 0
        n2 = heightfieldData[i-1, j] if i > 0 else n1
        heightfieldData[i, j] = (n1+n2)/2 + random.uniform(-0.1, 0.1)

heightfieldData_2 = np.concatenate((heightfieldData[::-1,:], heightfieldData))
col, row = heightfieldData_2.shape
terrainShape = p.createCollisionShape(shapeType=p.GEOM_HEIGHTFIELD, heightfieldData=heightfieldData_2.reshape(-1), meshScale=[0.5, 0.5, 1], numHeightfieldRows=row, numHeightfieldColumns=col)
terrainId = p.createMultiBody(0, terrainShape)
p.changeDynamics(terrainId, -1, lateralFriction=1.5)

robot = p.loadURDF("robot.urdf", [0, 0, 0.8])
for joint in range(p.getNumJoints(robot)):
    p.changeDynamics(robot, joint, lateralFriction=2.0)

def deg_to_rad(deg): return deg * (math.pi / 180.0)

L_X = 0.22  
L_Y = 0.15  

ang_coxa_fd, ang_femur_fd, ang_tibia_fd = -30, -26, 100
ang_coxa_md, ang_femur_md, ang_tibia_md = 0, -26, 100
ang_coxa_td, ang_femur_td, ang_tibia_td = 30, -26, 100
ang_coxa_fe, ang_femur_fe, ang_tibia_fe = -30, -26, 100
ang_coxa_me, ang_femur_me, ang_tibia_me = 0, -26, 100
ang_coxa_te, ang_femur_te, ang_tibia_te = 30, -26, 100

OFF_FEMUR, OFF_TIBIA = -80, 140
EXAG_FEMUR, EXAG_TIBIA_LIGAR = -60, 120

estado_atual = "MUNDO"
prog_perna, trans_final, prog_geral = 0.0, 0.0, 0.0
tecla_ativa, fase_marcha = None, 0
auto_mundo = False 

femur_offsets = [0.0] * 6
blockage_lifts = [0.0] * 6 
auto_pitch = 0.0 
avg_roll = 0.0 

frozen_femur_offsets = [0.0] * 6
frozen_auto_pitch = 0.0
reset_progress = 0.0
last_mc = [0.0] * 6

current_gait_id = 1
gait_offsets = [0, math.pi, 0, math.pi, 0, math.pi] 
current_gait_offsets = list(gait_offsets)
smooth_dv, smooth_dg = 0.0, 0.0

ordem_pernas = [
    (0, 1, 2, ang_coxa_fd, ang_femur_fd, ang_tibia_fd, "dir"),    
    (3, 4, 5, ang_coxa_md, ang_femur_md, ang_tibia_md, "dir"),    
    (6, 7, 8, ang_coxa_td, ang_femur_td, ang_tibia_td, "dir"),    
    (9, 10, 11, ang_coxa_te, ang_femur_te, ang_tibia_te, "esq"),  
    (12, 13, 14, ang_coxa_me, ang_femur_me, ang_tibia_me, "esq"), 
    (15, 16, 17, ang_coxa_fe, ang_femur_fe, ang_tibia_fe, "esq")   
]

while True:
    keys = p.getKeyboardEvents()
    if tecla_ativa and not (tecla_ativa in keys and keys[tecla_ativa] & p.KEY_IS_DOWN): tecla_ativa = None
    if not tecla_ativa:
        for k in keys:
            if keys[k] & p.KEY_IS_DOWN: tecla_ativa = k; break

    if ord('1') in keys and keys[ord('1')] & p.KEY_IS_DOWN:
        current_gait_id = 1
        gait_offsets = [0, math.pi, 0, math.pi, 0, math.pi]
    elif ord('2') in keys and keys[ord('2')] & p.KEY_IS_DOWN:
        current_gait_id = 2
        step = 2 * math.pi / 6
        gait_offsets = [0, 2*step, 4*step, 3*step, 5*step, 1*step]
    elif ord('3') in keys and keys[ord('3')] & p.KEY_IS_DOWN:
        current_gait_id = 3
        step = 2 * math.pi / 6
        gait_offsets = [0, 1*step, 2*step, 3*step, 4*step, 5*step]

    if estado_atual not in ["LIGANDO_PARA_MUNDO", "LIGANDO_SEQUENCIAL", "DESLIGANDO", "MUNDO_PARA_DESLIGADO", "DESLIGADO_PARA_LIGADO"]:
        if tecla_ativa == ord('q') and estado_atual != "MUNDO":
            if estado_atual == "LIGADO": 
                frozen_femur_offsets = list(femur_offsets); frozen_auto_pitch = auto_pitch
                estado_atual, prog_geral, auto_mundo = "DESLIGANDO", 0.0, True
            else: estado_atual, prog_geral = "LIGANDO_PARA_MUNDO", 0.0
        elif tecla_ativa == ord('e'):
            if estado_atual == "MUNDO": estado_atual, prog_perna, trans_final = "LIGANDO_SEQUENCIAL", 0.0, 0.0
            elif estado_atual == "DESLIGADO": estado_atual, prog_geral = "DESLIGADO_PARA_LIGADO", 0.0
        elif tecla_ativa == ord('r'):
            if estado_atual == "LIGADO": 
                frozen_femur_offsets = list(femur_offsets); frozen_auto_pitch = auto_pitch
                estado_atual, prog_geral = "DESLIGANDO", 0.0
            elif estado_atual == "MUNDO": estado_atual, prog_geral = "MUNDO_PARA_DESLIGADO", 0.0

    direcionais = [k for k in [p.B3G_UP_ARROW, p.B3G_DOWN_ARROW, p.B3G_LEFT_ARROW, p.B3G_RIGHT_ARROW] if k in keys and keys[k] & p.KEY_IS_DOWN]
    v_trans = 0.01
    
    _, baseOrn = p.getBasePositionAndOrientation(robot)
    euler = p.getEulerFromQuaternion(baseOrn)
    avg_roll += (euler[0] - avg_roll) * 0.1
    auto_pitch += (euler[1] - auto_pitch) * 0.1

    if estado_atual == "LIGADO":
        for i in range(6):
            diff = (gait_offsets[i] - current_gait_offsets[i] + math.pi) % (2 * math.pi) - math.pi
            current_gait_offsets[i] += diff * 0.05 

        if len(direcionais) >= 1: 
            speed_mult = 3.0
            fase_marcha += 120.0**-1 * speed_mult
            reset_progress = 0.0
            k = direcionais[0]
            dv = 1 if k == p.B3G_UP_ARROW else -1 if k == p.B3G_DOWN_ARROW else 0
            dg = 1 if k == p.B3G_RIGHT_ARROW else -1 if k == p.B3G_LEFT_ARROW else 0
        else:
            dv = dg = 0
            reset_progress = min(1.0, reset_progress + 0.01)
        
        smooth_dv += (dv - smooth_dv) * 0.1
        smooth_dg += (dg - smooth_dg) * 0.1
    else:
        dv = dg = 0; smooth_dv = smooth_dg = 0; fase_marcha = 0

    if estado_atual == "LIGANDO_PARA_MUNDO":
        prog_geral = min(2.0, prog_geral + v_trans)
        if prog_geral >= 2.0: estado_atual = "MUNDO"
    elif estado_atual == "MUNDO_PARA_DESLIGADO":
        prog_geral = min(2.0, prog_geral + v_trans)
        if prog_geral >= 2.0: 
            estado_atual = "DESLIGADO"
            femur_offsets = [0.0] * 6; auto_pitch = 0.0; last_mc = [0.0] * 6
    elif estado_atual == "DESLIGANDO":
        prog_geral = min(4.0, prog_geral + v_trans)
        if prog_geral >= 4.0:
            if auto_mundo: estado_atual, prog_geral, auto_mundo = "LIGANDO_PARA_MUNDO", 0.0, False
            else: estado_atual = "DESLIGADO"
            femur_offsets = [0.0] * 6; auto_pitch = 0.0; last_mc = [0.0] * 6
    elif estado_atual == "DESLIGADO_PARA_LIGADO":
        prog_geral = min(4.0, prog_geral + v_trans)
        if prog_geral >= 4.0: estado_atual = "LIGADO"
    elif estado_atual == "LIGANDO_SEQUENCIAL":
        prog_perna = min(6.0, prog_perna + 0.03)
        if prog_perna >= 6.0:
            trans_final = min(1.0, trans_final + 0.005)
            if trans_final >= 1.0: estado_atual = "LIGADO"

    for i, (id_c, id_f, id_t, b_c, b_f, b_t, lado) in enumerate(ordem_pernas):
        if estado_atual == "MUNDO":
            for j in [id_c, id_f, id_t]: p.setJointMotorControl2(robot, j, p.POSITION_CONTROL, 0)
        elif estado_atual == "DESLIGADO":
            p.setJointMotorControl2(robot, id_c, p.POSITION_CONTROL, 0); p.setJointMotorControl2(robot, id_f, p.POSITION_CONTROL, deg_to_rad(OFF_FEMUR)); p.setJointMotorControl2(robot, id_t, p.POSITION_CONTROL, deg_to_rad(OFF_TIBIA))
        elif estado_atual == "LIGANDO_SEQUENCIAL":
            ativ = max(0.0, min(1.0, prog_perna - i))
            f_tar = EXAG_FEMUR * (1.0-trans_final) + b_f * trans_final
            t_tar = (EXAG_TIBIA_LIGAR - 20) * (1.0-trans_final) + b_t * trans_final
            p.setJointMotorControl2(robot, id_c, p.POSITION_CONTROL, deg_to_rad(b_c * ativ))
            p.setJointMotorControl2(robot, id_f, p.POSITION_CONTROL, deg_to_rad(f_tar * ativ))
            p.setJointMotorControl2(robot, id_t, p.POSITION_CONTROL, deg_to_rad(t_tar * ativ))
        elif estado_atual == "DESLIGADO_PARA_LIGADO":
            p_g = prog_geral
            if p_g <= 1.0: c_n, f_n, t_n = 0, OFF_FEMUR, OFF_TIBIA * (1.0-p_g) + b_t * p_g
            elif p_g <= 2.0: p_a = p_g - 1.0; c_n, f_n, t_n = 0, OFF_FEMUR * (1.0-p_a) + EXAG_FEMUR * p_a, b_t
            elif p_g <= 3.0: p_a = p_g - 2.0; c_n, f_n, t_n = b_c * p_a, EXAG_FEMUR, b_t
            else: p_a = p_g - 3.0; c_n, f_n, t_n = b_c, EXAG_FEMUR * (1.0-p_a) + b_f * p_a, b_t
            p.setJointMotorControl2(robot, id_c, p.POSITION_CONTROL, deg_to_rad(c_n)); p.setJointMotorControl2(robot, id_f, p.POSITION_CONTROL, deg_to_rad(f_n)); p.setJointMotorControl2(robot, id_t, p.POSITION_CONTROL, deg_to_rad(t_n))
        elif estado_atual == "DESLIGANDO":
            p_g = prog_geral
            balance_adj = (-frozen_auto_pitch if i in [0, 5] else frozen_auto_pitch if i in [2, 3] else 0)
            f_start = b_f + frozen_femur_offsets[i] + balance_adj
            if p_g <= 1.0: 
                p_a = p_g; c_n, f_n = b_c, f_start * (1.0 - p_a) + EXAG_FEMUR * p_a; t_n = 74.0 - f_n 
            elif p_g <= 2.0: 
                p_a = p_g - 1.0; c_n, f_n = b_c, EXAG_FEMUR; t_n = (74.0 - EXAG_FEMUR) * (1.0 - p_a) + 60.0 * p_a 
            elif p_g <= 3.0: 
                p_a = p_g - 2.0; c_n, f_n, t_n = b_c * (1.0 - p_a), EXAG_FEMUR, 60.0
            else: 
                p_a = p_g - 3.0; c_n, f_n = 0, EXAG_FEMUR * (1.0 - p_a) + OFF_FEMUR * p_a; t_n = 60.0 * (1.0 - p_a) + OFF_TIBIA * p_a
            p.setJointMotorControl2(robot, id_c, p.POSITION_CONTROL, deg_to_rad(c_n)); p.setJointMotorControl2(robot, id_f, p.POSITION_CONTROL, deg_to_rad(f_n)); p.setJointMotorControl2(robot, id_t, p.POSITION_CONTROL, deg_to_rad(t_n))
        elif estado_atual == "LIGANDO_PARA_MUNDO":
            p_g = prog_geral
            if p_g <= 1.0: f_n, t_n = (OFF_FEMUR*(1-p_g))+(EXAG_FEMUR*p_g), OFF_TIBIA*(1-p_g)
            else: f_n, t_n = EXAG_FEMUR*(1-(p_g-1)), 0
            p.setJointMotorControl2(robot, id_c, p.POSITION_CONTROL, 0); p.setJointMotorControl2(robot, id_f, p.POSITION_CONTROL, deg_to_rad(f_n)); p.setJointMotorControl2(robot, id_t, p.POSITION_CONTROL, deg_to_rad(t_n))
        elif estado_atual == "MUNDO_PARA_DESLIGADO":
            p_g = prog_geral
            if p_g <= 1.0: f_n, t_n = EXAG_FEMUR*p_g, 0
            else: f_n, t_n = EXAG_FEMUR*(1-(p_g-1)) + OFF_FEMUR*(p_g-1), OFF_TIBIA*(p_g-1)
            p.setJointMotorControl2(robot, id_c, p.POSITION_CONTROL, 0); p.setJointMotorControl2(robot, id_f, p.POSITION_CONTROL, deg_to_rad(f_n)); p.setJointMotorControl2(robot, id_t, p.POSITION_CONTROL, deg_to_rad(t_n))
        
        elif estado_atual == "LIGADO":
            is_moving = (dv != 0 or dg != 0)
            phi = fase_marcha + current_gait_offsets[i]
            
            osc = math.sin(phi)
            sub = max(0, math.cos(phi) - 0.4) * 20.0
            
            if is_moving or abs(smooth_dv) > 0.01 or abs(smooth_dg) > 0.01:
                target_mc = (osc * 15.0 * smooth_dv) + (osc * 15.0 * (smooth_dg if lado == "dir" else -smooth_dg))
                last_mc[i] += (target_mc - last_mc[i]) * 0.2 
                c_n = b_c + (last_mc[i] if lado == "esq" else -last_mc[i])
                f_base = b_f - sub
            else:
                if sub > 0:
                    p_lift = max(0, math.sin(reset_progress * math.pi)) * 15.0
                    f_base = b_f - p_lift
                    c_n = b_c + ((last_mc[i] * (1.0 - reset_progress)) if lado == "esq" else (-last_mc[i] * (1.0 - reset_progress)))
                else:
                    f_base = b_f - sub
                    c_n = b_c + (last_mc[i] if lado == "esq" else -last_mc[i])

            lx = L_X if i in [0, 5] else -L_X if i in [2, 3] else 0
            ly = L_Y if lado == "dir" else -L_Y
            z_level = (lx * math.sin(auto_pitch) - ly * math.sin(avg_roll)) * 50.0

            com_shift = math.sin(auto_pitch) * 20.0
            c_n += (com_shift if lado == "dir" else -com_shift)

            contacts = p.getContactPoints(bodyA=robot, linkIndexA=id_t)
            is_touching = len(contacts) > 0
            actual_f = math.degrees(p.getJointState(robot, id_f)[0])
            
            if sub < 2.0: 
                if not is_touching: femur_offsets[i] = min(femur_offsets[i] + 0.8, 40.0)
                else:
                    target_angle = f_base + femur_offsets[i] + z_level
                    diff = actual_f - target_angle
                    femur_offsets[i] += diff * 0.1
                femur_offsets[i] *= 0.99 
            else: femur_offsets[i] *= 0.7 

            f_n = f_base + femur_offsets[i] + z_level
            f_n = max(f_n, -50.0) 
            t_n = 74.0 - f_n 
            
            p.setJointMotorControl2(robot, id_c, p.POSITION_CONTROL, deg_to_rad(c_n))
            p.setJointMotorControl2(robot, id_f, p.POSITION_CONTROL, deg_to_rad(f_n))
            p.setJointMotorControl2(robot, id_t, p.POSITION_CONTROL, deg_to_rad(t_n))

    p.stepSimulation()
    time.sleep(1./120.)
