#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Тестовый скрипт для проверки process_letters_dinam в SITL.
Использует видеофайл вместо камеры, телеметрию из SITL.
ВСЯ ЛОГИКА process_letters_dinam сохранена без изменений.
"""

import cv2
import numpy as np
import time
import torch
import math
import logging
import sys
import os
from datetime import datetime
from torchvision import transforms
from PIL import Image
from pymavlink import mavutil
from collections import defaultdict

# === КОНФИГУРАЦИЯ ===
VIDEO_PATH = "flight_test.mp4"           # Путь к видео для теста
MAVLINK_PORT = "udpin:127.0.0.1:14551"   # SITL: udpin для приёма телеметрии
MAVLINK_BAUD = 57600

MODEL_PATH = "ALM_best (1).pth"
LABELS_PATH = "labels.txt"
CSV_OUTPUT_PATH = "/home/skatwsl/test/objects-coordinates.csv"

# Параметры из оригинального кода
CONFIDENCE_THRESHOLD = 0.7
PROCESSING_FPS = 60
PIXELS_PER_METER = 44.482
MIN_AREA = 11000
MAX_AREA = 400000
MAX_ALTITUDE_M = 150.0
ANOMALOUS_HEIGHT = 14.5
LETTER_TIMEOUT = 2.0
MIN_DETECTIONS = 3
SAVE_INTERVAL = 0.3
LOOP_DELAY = 0.02

# Пути для сохранения
LETTERS_DIR = "/home/skatwsl/test/letters_images"
LOG_DIR = "/home/skatwsl/test"
os.makedirs(LETTERS_DIR, exist_ok=True)
os.makedirs(LOG_DIR, exist_ok=True)

# Логирование
logging.basicConfig(
   # filename=os.path.join(LOG_DIR, "test_letters_dinam.log"),
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
    handlers = [
        logging.FileHandler(os.path.join(LOG_DIR, "test_letters_dinam.log"), encoding = 'utf-8'),
        logging.StreamHandler(sys.stdout)
        ]
    )
# === ГЛОБАЛЬНЫЕ ФЛАГИ (эмуляция состояния контроллера) ===
class TestState:
    current_mode = 1  # MODE_LETTERS
    model_loaded = False
    processing_active = False
    last_detection_time = 0
    letter_stats = defaultdict(lambda: {'confidences': [], 'coords': [], 'timestamps': []})
    processing_sessions = []
    last_processing_time = 0
    last_save_time = 0
    processing_interval = 1.0 / PROCESSING_FPS
    
    # Телеметрия (кэшируется)
    last_lat, last_lon, last_alt_agl, last_alt_amsl, last_yaw = 55.754066, 37.617498, 30.0, 30.0, 0.0

state = TestState()

# === ЗАГЛУШКИ ДЛЯ ОРИГИНАЛЬНЫХ МЕТОДОВ ===
def send_ssh_message(msg):
    """Эмуляция вывода в консоль/SSH"""
    print(f"[{datetime.now().strftime('%H:%M:%S')}] {msg}")
    sys.stdout.flush()

def load_model():
    """Загрузка модели (оригинальная логика)"""
    try:
        from modelold import ArmenianLetterNet
        model = ArmenianLetterNet()
        model.load_state_dict(torch.load(MODEL_PATH, map_location=torch.device('cpu'), weights_only=False))
        model.eval()
        
        with open(LABELS_PATH, "r", encoding="utf-8") as f:
            labels = [line.strip() for line in f.readlines()]
        
        transform = transforms.Compose([
            transforms.Grayscale(num_output_channels=1),
            transforms.Resize((32, 32)),
            transforms.ToTensor(),
            transforms.Normalize((0.5,), (0.5,))
        ])
        logging.info("Модель загружена")
        return model, labels, transform
    except Exception as e:
        logging.error(f"Ошибка загрузки модели: {e}")
        raise

def get_frame(cap):
    """Получение кадра из видеофайла"""
    ret, frame = cap.read()
    if not ret:
        return None
    # Приводим к 720x720 как в оригинале
    if frame.shape[0] != 720 or frame.shape[1] != 720:
        frame = cv2.resize(frame, (720, 720))
    return frame

def check_mode():
    """Проверка режима (в тесте всегда LETTERS, если не нажали 'q')"""
    return state.current_mode

def get_coordinates(master):
    """Получение телеметрии из SITL с кэшированием"""
    try:
        if master:
            msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False, timeout=0.05)
            if msg:
                state.last_lat = msg.lat / 1e7
                state.last_lon = msg.lon / 1e7
                state.last_alt_agl = msg.relative_alt / 1e3
                state.last_alt_amsl = msg.alt / 1e3
            
            msg_att = master.recv_match(type='ATTITUDE', blocking=False, timeout=0.05)
            if msg_att:
                state.last_yaw = math.degrees(msg_att.yaw)
    except Exception as e:
        logging.warning(f"Телеметрия: {e}")
    
    return state.last_lat, state.last_lon, state.last_alt_agl, state.last_alt_amsl, state.last_yaw

def geo_to_3d_xyz(lat_deg, lon_deg, h_ellipsoid):
    a = 6378137.0
    f = 1/298.257223563
    e2 = 2*f - f**2
    B = math.radians(lat_deg)
    L = math.radians(lon_deg)
    N = a / math.sqrt(1 - e2 * math.sin(B)**2)
    X = (N + h_ellipsoid) * math.cos(B) * math.cos(L)
    Y = (N + h_ellipsoid) * math.cos(B) * math.sin(L)
    Z = ((1 - e2) * N + h_ellipsoid) * math.sin(B)
    return X, Y, Z

def xyz_to_geo_bowring(X, Y, Z):
    a = 6378137.0
    f = 1/298.257223563
    e2 = 2*f - f**2
    b = a * math.sqrt(1 - e2)
    ep2 = e2 / (1 - e2)
    Q = math.sqrt(X**2 + Y**2)
    if Q == 0:
        return (90.0 if Z > 0 else -90.0), 0.0, (abs(Z) - b)
    r = math.sqrt(Z**2 + Q**2 * (1 - e2))
    num = r**3 + b * ep2 * Z**2
    den = r**3 - b * e2 * (1 - e2) * Q**2
    B_rad = math.atan((Z / Q) * (num / den))
    L_rad = math.atan2(Y, X)
    N = a / math.sqrt(1 - e2 * math.sin(B_rad)**2)
    H = Q / math.cos(B_rad) - N
    return math.degrees(B_rad), math.degrees(L_rad), H

def add_meters_to_coords(lat, lon, dx_m, dy_m, yaw_deg=0, alt_amsl=0):
    yaw_rad = math.radians(yaw_deg)
    north_m = dy_m * math.cos(yaw_rad) - dx_m * math.sin(yaw_rad)
    east_m = dy_m * math.sin(yaw_rad) + dx_m * math.cos(yaw_rad)
    h_ellipsoid = alt_amsl + ANOMALOUS_HEIGHT
    X, Y, Z = geo_to_3d_xyz(lat, lon, h_ellipsoid)
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    dX = -math.sin(lat_rad)*math.cos(lon_rad)*north_m - math.sin(lon_rad)*east_m
    dY = -math.sin(lat_rad)*math.sin(lon_rad)*north_m + math.cos(lon_rad)*east_m
    dZ = math.cos(lat_rad)*north_m
    new_lat, new_lon, _ = xyz_to_geo_bowring(X + dX, Y + dY, Z + dZ)
    return new_lat, new_lon

def pixels_to_meters(pixel_x, pixel_y, ppm):
    return pixel_x / ppm, pixel_y / ppm

def process_frame(frame, lat, lon, alt_agl, alt_amsl, yaw):
    """ОРИГИНАЛЬНАЯ функция process_frame_dinam (без изменений логики)"""
    if alt_agl > MAX_ALTITUDE_M:
        return [], None

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)
    thresh_color = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    alt_safe = max(alt_agl, 1.0)
    _scale_sq = (30.0 / alt_safe) ** 2
    dynamic_min_area = max(MIN_AREA * _scale_sq, 50)
    dynamic_max_area = MAX_AREA * _scale_sq

    results = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if not (dynamic_min_area < area < dynamic_max_area):
            continue

        perimeter = cv2.arcLength(cnt, True)
        circularity = 4 * np.pi * area / (perimeter ** 2) if perimeter > 0 else 0
        if circularity < 0.4:
            continue

        mask = np.zeros_like(gray)
        cv2.drawContours(mask, [cnt], -1, 255, -1)
        mean_color = cv2.mean(gray, mask=mask)[0]
        if mean_color < 160:
            continue

        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect)
        box = np.intp(box)
        center_x, center_y = int(rect[0][0]), int(rect[0][1])
        
        frame_center_x, frame_center_y = 720/2, 720/2
        dx_px = center_x - frame_center_x
        dy_px = -center_y + frame_center_y

        dynamic_ppm = PIXELS_PER_METER * (30.0 / alt_safe)
        raw_dx_m, raw_dy_m = pixels_to_meters(dx_px, dy_px, dynamic_ppm)
        
        dx_m = raw_dx_m
        dy_m = raw_dy_m + alt_agl
        
        new_lat, new_lon = add_meters_to_coords(lat, lon, dx_m, dy_m, yaw_deg=yaw, alt_amsl=alt_amsl)
        
        results.append({
            'center_px': (center_x, center_y),
            'center_m': (dx_m, dy_m),
            'raw_dx_m': raw_dx_m,
            'raw_dy_m': raw_dy_m,
            'coords': (new_lat, new_lon),
            'alt': alt_agl,
            'box': box
        })
    
    return results, thresh_color

def _save_to_csv(letter_label, avg_lat, avg_lon):
    """ОРИГИНАЛЬНАЯ функция записи в CSV (ТЗ п.5.2.3.5.2)"""
    try:
        parts = letter_label.strip().split()
        letter_index = int(parts[0]) if len(parts) >= 1 else -1
        lat_e7 = int(round(avg_lat * 1e7))
        lon_e7 = int(round(avg_lon * 1e7))

        with open(CSV_OUTPUT_PATH, 'a', newline='', encoding='utf-8') as f:
            f.write(f"{letter_index},{lat_e7},{lon_e7}\n")

        letter_char = parts[1] if len(parts) >= 2 else "?"
        log_msg = f"CSV записано: {letter_index},{lat_e7},{lon_e7} (буква: {letter_char})"
        send_ssh_message(log_msg)
        logging.info(log_msg)
    except Exception as e:
        logging.error(f"Ошибка записи CSV: {e}")


# =============================================================================
# === ВАША ОРИГИНАЛЬНАЯ ФУНКЦИЯ (ВСТАВЛЕНА БЕЗ ИЗМЕНЕНИЙ) ===
# =============================================================================
def process_letters_dinam(cap, master, model, labels, transform):
    if not state.model_loaded:
        state.model_loaded = True

    send_ssh_message("=== Активирован режим распознавания букв ===")
    state.last_processing_time = time.time()
    state.last_save_time = time.time()
    consecutive_empty_frames = 0
    MAX_EMPTY_FRAMES = 30
    try:
        while state.current_mode == 1:  # MODE_LETTERS
            new_mode = check_mode()
            if new_mode != state.current_mode:
                state.current_mode = new_mode
                return

            current_time = time.time()
            if current_time - state.last_processing_time < state.processing_interval:
                time.sleep(0.01)
                continue
            state.last_processing_time = current_time
            
            frame = get_frame(cap)
            if frame is None:
                time.sleep(0.1)
                consecutive_empty_frames +=1
                send_ssh_message("Кадр не получен")
                continue
            if consecutive_empty_frames >= MAX_EMPTY_FRAMES:
                print ('Final')
                break
            
            results, thresh_color = process_frame(frame, *get_coordinates(master))
            letter_detected = False

            for result in results:
                cv2.drawContours(frame, [result['box']], 0, (0, 255, 0), 2)
                cv2.circle(frame, result['center_px'], 5, (0, 0, 255), -1)
                
                width, height = map(int, cv2.minAreaRect(result['box'])[1])
                if width > 40 and height > 40:
                    try:
                        letter_crop = cv2.warpPerspective(
                            thresh_color, 
                            cv2.getPerspectiveTransform(
                                result['box'].astype("float32"),
                                np.array([[0, height-1], [0, 0], [width-1, 0], [width-1, height-1]], dtype="float32")
                            ),
                            (width, height)
                        )
                        
                        if np.mean(letter_crop) < 250:
                            img_tensor = transform(Image.fromarray(letter_crop)).unsqueeze(0)
                            
                            with torch.no_grad():
                                output = model(img_tensor)
                                conf, pred = torch.max(torch.nn.functional.softmax(output, dim=1), 1)
                                
                                if conf.item() > CONFIDENCE_THRESHOLD:
                                    letter_detected = True
                                    state.last_detection_time = current_time
                                    label = labels[pred.item()]
                                    label_parts = label.strip().split()
                                    letter_char  = label_parts[1] if len(label_parts) >= 2 else label
                                    letter_idx   = label_parts[0] if len(label_parts) >= 1 else "?"

                                    state.letter_stats[label]['confidences'].append(conf.item())
                                    state.letter_stats[label]['coords'].append(result['coords'])
                                    state.letter_stats[label]['timestamps'].append(current_time)
                                
                                    state.processing_active = True
                                    text = f"{letter_char} ({conf.item():.2f})"
                                    cv2.putText(frame, text, result['center_px'], 
                                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                                    
                                    timestamp_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                                    log_msg = (f"[{timestamp_str}] LETTER: {letter_char} | "
                                               f"CONF: {conf.item():.2f} | "
                                               f"LAT: {result['coords'][0]:.6f} | "
                                               f"LON: {result['coords'][1]:.6f} | "
                                               f"ALT: {result['alt']:.2f} | "
                                               f"OFFSET_X: {result['raw_dx_m']:.2f} | "
                                               f"OFFSET_Y: {result['raw_dy_m']:.2f}")
                                    send_ssh_message(log_msg)
                                    logging.info(log_msg)

                                    if current_time - state.last_save_time > SAVE_INTERVAL:
                                        coords_str = f"{result['coords'][0]:.6f}_{result['coords'][1]:.6f}"
                                        filename = os.path.join(
                                            LETTERS_DIR,
                                            f"armenian_letter_{coords_str}_{letter_idx}.jpg"
                                        )
                                        cv2.imwrite(filename, frame)
                                        state.last_save_time = current_time
                                        
                    except Exception as e:
                        logging.error(f"Ошибка обработки буквы: {e}")

            if state.processing_active and (current_time - state.last_detection_time) > LETTER_TIMEOUT:
                if state.letter_stats:
                    best_letter, best_data = max(state.letter_stats.items(),
                                            key=lambda x: len(x[1]['confidences']))
                    
                    if len(best_data['confidences']) >= MIN_DETECTIONS:
                        avg_conf = sum(best_data['confidences'])/len(best_data['confidences'])
                        avg_lat = sum(c[0] for c in best_data['coords'])/len(best_data['coords'])
                        avg_lon = sum(c[1] for c in best_data['coords'])/len(best_data['coords'])
                        last_time = datetime.fromtimestamp(max(best_data['timestamps'])).strftime('%H:%M:%S')
                        
                        log_msg = "\n=== Результаты обработки ==="
                        log_msg += f"\nНаиболее вероятная буква: {best_letter}"
                        log_msg += f"\nКоличество обнаружений: {len(best_data['confidences'])}"
                        log_msg += f"\nСредняя уверенность: {avg_conf:.2f}"
                        log_msg += f"\nСредние координаты: ({avg_lat:.6f}, {avg_lon:.6f})"
                        log_msg += f"\nВремя фиксации: {last_time}"
                        log_msg += "\n==========================="
                        
                        logging.info(log_msg)
                        _save_to_csv(best_letter, avg_lat, avg_lon)
                        
                        state.processing_sessions.append({
                            'letter': best_letter,
                            'count': len(best_data['confidences']),
                            'avg_confidence': avg_conf,
                            'avg_coords': (avg_lat, avg_lon),
                            'timestamp': last_time
                        })
                
                state.letter_stats = defaultdict(lambda: {'confidences': [], 'coords': [], 'timestamps': []})
                state.processing_active = False
            
            time.sleep(LOOP_DELAY)
            
    except Exception as e:
        logging.error(f"Критическая ошибка в режиме LETTERS: {e}")
        send_ssh_message(f"ОШИБКА: {str(e)}")
    
    # Финализация
    if state.processing_active and state.letter_stats:
        best_letter, best_data = max(state.letter_stats.items(),
                                key=lambda x: len(x[1]['confidences']))
        
        if len(best_data['confidences']) >= MIN_DETECTIONS:
            avg_conf = sum(best_data['confidences'])/len(best_data['confidences'])
            avg_lat = sum(c[0] for c in best_data['coords'])/len(best_data['coords'])
            avg_lon = sum(c[1] for c in best_data['coords'])/len(best_data['coords'])
            last_time = datetime.fromtimestamp(max(best_data['timestamps'])).strftime('%H:%M:%S')
            
            log_msg = "\n=== Финальные результаты обработки ==="
            log_msg += f"\nНаиболее вероятная буква: {best_letter}"
            log_msg += f"\nКоличество обнаружений: {len(best_data['confidences'])}"
            log_msg += f"\nСредняя уверенность: {avg_conf:.2f}"
            log_msg += f"\nСредние координаты: ({avg_lat:.6f}, {avg_lon:.6f})"
            log_msg += f"\nВремя фиксации: {last_time}"
            log_msg += "\n====================================="
            
            logging.info(log_msg)
            _save_to_csv(best_letter, avg_lat, avg_lon)
            state.processing_sessions.append({
                'letter': best_letter,
                'count': len(best_data['confidences']),
                'avg_confidence': avg_conf,
                'avg_coords': (avg_lat, avg_lon),
                'timestamp': last_time
            })
    
    # Сводка
    if state.processing_sessions:
        log_msg = "\n=== Сводка всех сеансов обработки ==="
        for i, session in enumerate(state.processing_sessions, 1):
            log_msg += f"\n{i}. Буква: {session['letter']}"
            log_msg += f"\n   Обнаружений: {session['count']}"
            log_msg += f"\n   Уверенность: {session['avg_confidence']:.2f}"
            log_msg += f"\n   Координаты: {session['avg_coords']}"
            log_msg += f"\n   Время: {session['timestamp']}"
            log_msg += "\n-----------------------------"
        logging.info(log_msg)


# =============================================================================
# === MAIN ===
# =============================================================================
def main():
    # Ограничение потоков для CPU
    #torch.set_num_threads(2)
    #cv2.setNumThreads(2)
    
    # Подключение к SITL
    master = None
    try:
        master = mavutil.mavlink_connection(MAVLINK_PORT, baud=MAVLINK_BAUD)
        master.wait_heartbeat(timeout=5)
        logging.info("Подключение к SITL успешно")
        send_ssh_message("SITL подключен")
    except Exception as e:
        logging.warning(f"SITL не подключен: {e}. Работа с дефолтной телеметрией.")
        send_ssh_message("ВНИМАНИЕ: Телеметрия не доступна, используются координаты по умолчанию")
    
    # Загрузка модели
    model, labels, transform = load_model()
    
    # Открытие видео
    cap = cv2.VideoCapture(VIDEO_PATH)
    if not cap.isOpened():
        logging.error(f"Не удалось открыть видео: {VIDEO_PATH}")
        return
    
    fps = cap.get(cv2.CAP_PROP_FPS)
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    logging.info(f"Видео: {total} кадров @ {fps:.1f} FPS")
    send_ssh_message(f"Старт теста: {os.path.basename(VIDEO_PATH)}")
    
    try:
        # === ЗАПУСК ВАШЕЙ ФУНКЦИИ ===
        process_letters_dinam(cap, master, model, labels, transform)
    finally:
        cap.release()
        if master:
            master.close()
        logging.info("Тест завершён")
        send_ssh_message("=== ТЕСТ ЗАВЕРШЁН ===")


if __name__ == "__main__":
    main()
