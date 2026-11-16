# power_budget.py — розрахунок маси, потужності та батареї для дрона.
# Запуск:  python tools/power_budget.py
# Змінюйте числа в блоці «НАЛАШТУВАННЯ» і дивіться, як змінюється час польоту.
import math

# ===================== НАЛАШТУВАННЯ =====================
FLIGHT_MIN = 30          # потрібний час польоту, хв
PROP_INCH = 15           # діаметр пропелера, дюйми
MOTORS = 4
PAYLOAD_KG = 0.0         # вантаж, що підвішується (крім камер і комп'ютера)

DRY_PARTS_KG = {         # все, крім батареї
    "Рама 700 мм карбон + шасі": 0.60,
    "Мотори 4x (клас 4114, 340-400 KV)": 0.62,
    "Пропелери 15\" карбон 4x": 0.10,
    "Регулятори 4x45 A": 0.14,
    "Політний контролер + GPS/компас + приймач + телеметрія": 0.20,
    "Проводка, PDB, кріплення, BEC": 0.20,
    "Оптичний потік + далекомір": 0.06,
    "Комп'ютер (Jetson-клас, з радіатором)": 0.50,
    "Камери (ширококутна + зум) з кріпленням": 0.35,
}

BATTERY_WH_PER_KG = 230  # Li-ion 21700 (Molicel P45B/P50B) ~230; LiPo ~150
USABLE = 0.80            # не розряджати нижче 20%
ACTIVE_FACTOR = 1.25     # «активний політ» (маневри, вітер, набір висоти) = висіння x 1.25
AVIONICS_W = 30          # комп'ютер ~15-20 Вт + камери + польотний контролер
FIGURE_OF_MERIT = 0.70   # якість гвинта (0.6-0.75)
MOTOR_EFF = 0.85
ESC_EFF = 0.95
CELLS = 6                # 6S
# ========================================================

G, RHO = 9.81, 1.225


def hover_power(mass_kg):
    """Потужність висіння з теорії імпульсу (на вході батареї), Вт."""
    area = math.pi * (PROP_INCH * 0.0254 / 2) ** 2
    thrust = mass_kg * G / MOTORS
    p_ideal = thrust ** 1.5 / math.sqrt(2 * RHO * area)
    return MOTORS * p_ideal / (FIGURE_OF_MERIT * MOTOR_EFF * ESC_EFF)


def flight_minutes(batt_kg, dry_kg):
    m = dry_kg + PAYLOAD_KG + batt_kg
    p = hover_power(m) * ACTIVE_FACTOR + AVIONICS_W
    return batt_kg * BATTERY_WH_PER_KG * USABLE / p * 60, m, p


def main():
    dry = sum(DRY_PARTS_KG.values())
    print("Маса без батареї:")
    for k, v in DRY_PARTS_KG.items():
        print(f"  {k:58s}{v * 1000:6.0f} г")
    print(f"  {'РАЗОМ':58s}{dry * 1000:6.0f} г   + вантаж {PAYLOAD_KG * 1000:.0f} г\n")

    print("Батарея -> час польоту:")
    for b in [1.0, 1.5, 2.0, 2.5, 3.0, 4.0]:
        t, m, p = flight_minutes(b, dry)
        print(f"  {b:.1f} кг: {t:5.1f} хв  (злітна маса {m:.2f} кг, середня потужність {p:.0f} Вт)")

    need = next((b / 100 for b in range(20, 1000) if flight_minutes(b / 100, dry)[0] >= FLIGHT_MIN), None)
    if need is None:
        best = max((flight_minutes(b / 100, dry)[0], b / 100) for b in range(20, 1000))
        print(f"\n{FLIGHT_MIN} хв недосяжно: максимум {best[0]:.1f} хв при батареї {best[1]:.1f} кг. Потрібні більші пропелери.")
        return
    t, m, p = flight_minutes(need, dry)
    ph = hover_power(m)
    wh = need * BATTERY_WH_PER_KG
    volts = CELLS * 3.6
    per_motor_hover = m / MOTORS
    print(f"\nДля {FLIGHT_MIN} хв:")
    print(f"  батарея {need:.2f} кг = {wh:.0f} Вт·год ≈ {wh / volts:.0f} А·год при {CELLS}S")
    print(f"  злітна маса {m:.2f} кг")
    print(f"  висіння: {ph:.0f} Вт, струм ≈ {ph / volts:.0f} А; середня {p:.0f} Вт")
    print(f"  тяга мотора при висінні: {per_motor_hover * 1000:.0f} г")
    print(f"  потрібна макс. тяга мотора (тяга/вага 2.2): ≥ {per_motor_hover * 2.2 * 1000:.0f} г")
    print(f"  макс. потужність мотора (≈ при T/W 2.2): ≈ {ph / MOTORS * 2.2 ** 1.5:.0f} Вт")


if __name__ == "__main__":
    main()
