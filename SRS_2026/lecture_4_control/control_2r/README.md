# 2R: три закона управления в MuJoCo

Вертикальный 2R, гравитация включена. Примеры независимы от соседних лекций. Python 3.10+, зависимости из requirements.txt; проверено на Python 3.13 / MuJoCo 3.10.0.

Из корня репозитория:

```bash
python -m pip install -r SRS_2026/lecture_4_control/control_2r/requirements.txt
# Восьмёрка: три регулятора, без внешней силы
python SRS_2026/lecture_4_control/control_2r/simulate.py --out /tmp/2r-tracking
# Удержание положения с одинаковым внешним толчком
python SRS_2026/lecture_4_control/control_2r/simulate.py --task interaction --force --out /tmp/2r-interaction
# Интерактивное окно MuJoCo
python SRS_2026/lecture_4_control/control_2r/simulate.py --method admittance --task interaction --force --viewer
# Настройка податливости
python SRS_2026/lecture_4_control/control_2r/simulate.py --method impedance --task interaction --force --stiffness 60 --damping 15 --out /tmp/2r-soft
# Проверка уравнений и численного эксперимента
python SRS_2026/lecture_4_control/control_2r/verify.py
# Модульный запуск также поддерживается
python -m SRS_2026.lecture_4_control.control_2r.simulate --method computed-torque
```

При необходимости вместо `python` используйте `/home/ib/miniconda3/envs/idt/bin/python`. Прямой запуск по абсолютному пути работает из любой папки; XML ищется относительно скрипта. Для headless-экспериментов дисплей и OpenGL не нужны; viewer требует графической сессии. На macOS для viewer используйте `mjpython` из пакета MuJoCo.

Результаты: по CSV на метод, `metrics.json` с настройками и метриками, `comparison.png` с траекториями, отклонением x, моментом и силой. `--duration` — длительность, `--step` — шаг MuJoCo и частота обновления регулятора; по умолчанию 10 с и 2 мс. `--method` выбирает `computed-torque`, `impedance`, `admittance` или `all`. Для viewer выбирайте один метод.

`controllers.py` содержит законы, аналитические производные задания и RK4 внешнего admittance-фильтра. `robot_math.py` содержит физическую модель, совпадающую с `robot.xml` и моделью лекции 2. Impedance — декартовы пружина/демпфер с компенсацией тяжести, без задания искусственной инерции. Admittance — внешний фильтр силы плюс внутренний computed torque. Без силы admittance совпадает с computed torque.

Сила среды действует **на робот** по +x, гладкий импульс 4 Н между 3–5 с. Мы прикладываем её через `mj_applyFT` и передаём как идеальное измерение в admittance. Это воспроизводимый силовой опыт; контактов со стенкой и физического датчика в XML нет. Красная сфера показывает команду p_c, а не обязательно номинальное p_d. При перетаскивании робота мышью дополнительное воздействие viewer не входит в идеальное измерение внешнего фильтра; для сравнимого опыта используйте `--force`.

Kx/Ka: Н/м; Dx/Da: Н·с/м; Ma: кг. `--virtual-mass` влияет только на admittance. `--stiffness`, `--damping` настраивают impedance и admittance; computed torque использует фиксированные 1225 с⁻² и 70 с⁻¹. При экстремальных параметрах цель может стать недостижимой: IK выдаст явную ошибку. CLI проверяет положительность параметров, но не гарантирует устойчивость любой их комбинации.

RMS относительно номинального p_d не является универсальной оценкой качества податливости. Смотрите также слежение за p_c, насыщение и возврат после снятия силы. [Подробный вывод и задания](../lecture_notes.md).

Проверенные графики для настроек по умолчанию: [слeжение по восьмёрке](reference_results/tracking.png) и [силовой опыт](reference_results/interaction.png). Они получены MuJoCo 3.10.0; для обновления выполните команды выше. Метрики и кривые презентации встроены в её HTML-файл.
