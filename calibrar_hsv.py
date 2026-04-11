"""
Calibrador HSV en vivo para las figuras de la competencia.
Muestra la cámara con sliders para ajustar los rangos HSV
de dos colores (verde = recoger, rojo = evadir).

Uso:
  python3 calibrar_hsv.py

Controles:
  Tab        — alternar entre color VERDE y ROJO
  S          — guardar los valores actuales en hsv_config.txt
  Q          — salir

Al terminar copia los valores al barco_launch.py
"""

import cv2
import numpy as np

def nothing(x):
    pass

def crear_ventana(nombre):
    cv2.namedWindow(nombre, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(nombre, 400, 300)
    cv2.createTrackbar('H min', nombre, 0,   179, nothing)
    cv2.createTrackbar('H max', nombre, 179, 179, nothing)
    cv2.createTrackbar('S min', nombre, 0,   255, nothing)
    cv2.createTrackbar('S max', nombre, 255, 255, nothing)
    cv2.createTrackbar('V min', nombre, 0,   255, nothing)
    cv2.createTrackbar('V max', nombre, 255, 255, nothing)

def get_valores(nombre):
    h_min = cv2.getTrackbarPos('H min', nombre)
    h_max = cv2.getTrackbarPos('H max', nombre)
    s_min = cv2.getTrackbarPos('S min', nombre)
    s_max = cv2.getTrackbarPos('S max', nombre)
    v_min = cv2.getTrackbarPos('V min', nombre)
    v_max = cv2.getTrackbarPos('V max', nombre)
    return h_min, h_max, s_min, s_max, v_min, v_max

def set_valores(nombre, h_min, h_max, s_min, s_max, v_min, v_max):
    cv2.setTrackbarPos('H min', nombre, h_min)
    cv2.setTrackbarPos('H max', nombre, h_max)
    cv2.setTrackbarPos('S min', nombre, s_min)
    cv2.setTrackbarPos('S max', nombre, s_max)
    cv2.setTrackbarPos('V min', nombre, v_min)
    cv2.setTrackbarPos('V max', nombre, v_max)

def guardar_config(verde_vals, rojo_vals):
    with open('hsv_config.txt', 'w') as f:
        f.write("# Configuracion HSV para barco_launch.py\n\n")
        f.write("# VERDE (sargaso verde = RECOGER)\n")
        f.write(f"'hsv_lower_verde': [{verde_vals[0]}, {verde_vals[2]}, {verde_vals[4]}],\n")
        f.write(f"'hsv_upper_verde': [{verde_vals[1]}, {verde_vals[3]}, {verde_vals[5]}],\n\n")
        f.write("# ROJO (sargaso rojo = EVADIR)\n")
        f.write(f"'hsv_lower_rojo': [{rojo_vals[0]}, {rojo_vals[2]}, {rojo_vals[4]}],\n")
        f.write(f"'hsv_upper_rojo': [{rojo_vals[1]}, {rojo_vals[3]}, {rojo_vals[5]}],\n")
    print("\nGuardado en hsv_config.txt")
    print(f"Verde  lower=[{verde_vals[0]}, {verde_vals[2]}, {verde_vals[4]}] upper=[{verde_vals[1]}, {verde_vals[3]}, {verde_vals[5]}]")
    print(f"Rojo   lower=[{rojo_vals[0]}, {rojo_vals[2]}, {rojo_vals[4]}] upper=[{rojo_vals[1]}, {rojo_vals[3]}, {rojo_vals[5]}]")

def main():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not cap.isOpened():
        print("Error: no se pudo abrir la camara")
        return

    # Ventanas de sliders
    crear_ventana('Sliders')

    # Valores iniciales para sargaso verde
    # H: 35-85 cubre verde natural incluyendo algas
    set_valores('Sliders', 35, 85, 40, 255, 40, 255)

    # Guardar valores de cada color
    vals_verde = [35, 85, 40, 255, 40, 255]
    vals_rojo  = [0, 10, 100, 255, 100, 255]

    color_activo = 'VERDE'  # alternar con Tab

    print("=" * 50)
    print("Calibrador HSV — Sargaso")
    print("Tab = alternar verde/rojo")
    print("S   = guardar valores")
    print("Q   = salir")
    print("=" * 50)

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Leer sliders
        h_min, h_max, s_min, s_max, v_min, v_max = get_valores('Sliders')
        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])

        # Guardar en el color activo
        if color_activo == 'VERDE':
            vals_verde = [h_min, h_max, s_min, s_max, v_min, v_max]
        else:
            vals_rojo = [h_min, h_max, s_min, s_max, v_min, v_max]

        # Máscara
        mask  = cv2.inRange(hsv, lower, upper)

        # Limpiar ruido
        kernel = np.ones((5, 5), np.uint8)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Encontrar contornos
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        resultado = frame.copy()
        for c in contours:
            area = cv2.contourArea(c)
            if area > 300:
                cv2.drawContours(resultado, [c], -1,
                    (0, 255, 0) if color_activo == 'VERDE' else (0, 0, 255), 2)
                M = cv2.moments(c)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    cv2.circle(resultado, (cx, cy), 6,
                        (0, 255, 0) if color_activo == 'VERDE' else (0, 0, 255), -1)
                    cv2.putText(resultado, f"area={int(area)}",
                        (cx + 8, cy), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (255, 255, 255), 1)

        # Info en pantalla
        color_bgr = (0, 200, 0) if color_activo == 'VERDE' else (0, 0, 220)
        cv2.putText(resultado,
            f"Color: {color_activo}  (Tab para cambiar)",
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_bgr, 2)
        cv2.putText(resultado,
            f"H:[{h_min}-{h_max}] S:[{s_min}-{s_max}] V:[{v_min}-{v_max}]",
            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(resultado,
            f"Objetos detectados: {len([c for c in contours if cv2.contourArea(c)>300])}",
            (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(resultado,
            "S=guardar  Q=salir",
            (10, 450), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

        # Mostrar
        mask_rgb = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        combined = np.hstack([resultado, mask_rgb])
        cv2.imshow('Camara | Mascara', combined)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q') or key == ord('Q'):
            break

        elif key == ord('\t') or key == 9:   # Tab
            if color_activo == 'VERDE':
                # Guardar verde y cargar rojo
                color_activo = 'ROJO'
                set_valores('Sliders', *vals_rojo)
                print("Calibrando: ROJO (evadir)")
            else:
                # Guardar rojo y cargar verde
                color_activo = 'VERDE'
                set_valores('Sliders', *vals_verde)
                print("Calibrando: VERDE (recoger)")

        elif key == ord('s') or key == ord('S'):
            guardar_config(vals_verde, vals_rojo)

    cap.release()
    cv2.destroyAllWindows()

    # Mostrar resumen final
    print("\n" + "=" * 50)
    print("VALORES FINALES para barco_launch.py:")
    print(f"VERDE lower=[{vals_verde[0]}, {vals_verde[2]}, {vals_verde[4]}]  upper=[{vals_verde[1]}, {vals_verde[3]}, {vals_verde[5]}]")
    print(f"ROJO  lower=[{vals_rojo[0]},  {vals_rojo[2]}, {vals_rojo[4]}]   upper=[{vals_rojo[1]},  {vals_rojo[3]}, {vals_rojo[5]}]")
    print("=" * 50)

if __name__ == '__main__':
    main()
