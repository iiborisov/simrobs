# Пространственная динамика: Python и MuJoCo

Тот же 3R, что в лекции 5: home-оси z,x,x, длины 0,4/0,35/0,25 м. Теперь добавлены массы 2/1,5/1 кг, цилиндрические инерции и реальная интеграция динамики. Нотация конспекта: H — конфигурация, T=(w;v) — твист, W — строка сил, I — пространственная инерция, τₐ — строка моментов. В коде `tau` представляет столбец τₐᵀ.

Из корня репозитория:

```bash
python -m pip install -r SRS_2026/lecture_6_spatial_dynamics/spatial_dynamics/requirements.txt
# Один численный пример, включая M/C/N и Newton–Euler для тела
python SRS_2026/lecture_6_spatial_dynamics/spatial_dynamics/inspect_model.py
# Четыре режима на физическом роботе без внешнего толчка
python SRS_2026/lecture_6_spatial_dynamics/spatial_dynamics/simulate.py --out /tmp/dynamics-default
# Пространственная виртуальная пружина + сила среды на tip
python SRS_2026/lecture_6_spatial_dynamics/spatial_dynamics/simulate.py --method impedance --force --out /tmp/dynamics-impedance
# Ошибка модели: физический XML прежний, модель регулятора 80%
python SRS_2026/lecture_6_spatial_dynamics/spatial_dynamics/simulate.py --method computed-torque --model-mass-scale .8 --out /tmp/dynamics-mismatch
# Графическое окно MuJoCo
python SRS_2026/lecture_6_spatial_dynamics/spatial_dynamics/simulate.py --method computed-torque --viewer
# Свободное вращение, эффект промежуточной оси
python SRS_2026/lecture_6_spatial_dynamics/spatial_dynamics/rigid_body.py --axis middle --out /tmp/dynamics-rigid
# Также: --axis min и --axis max
# Проверки всех основных уравнений и динамических прогонов
python SRS_2026/lecture_6_spatial_dynamics/spatial_dynamics/verify.py
# Модульный запуск
python -m SRS_2026.lecture_6_spatial_dynamics.spatial_dynamics.verify
```

Проверено: Python 3.13, NumPy 2.4.2, MuJoCo 3.10.0. В имеющемся окружении можно использовать `/home/ib/miniconda3/envs/idt/bin/python`. Прямой запуск по абсолютному пути работает из любой рабочей папки. XML ищется относительно скрипта. Для headless-прогонов дисплей не нужен; viewer требует графической сессии. На macOS используйте `mjpython` для viewer.

`simulate.py`: режимы `free`, `gravity`, `computed-torque`, `impedance`, `all`. По умолчанию 8 с, шаг и период управления 2 мс; параметры `--duration`, `--step`, `--out`. Viewer требует один выбранный метод. Приводы ±20 Н·м, суставное трение 0,06 Н·м·с/рад, контакты отключены. Визуальная плоскость не препятствует прохождению звеньев. Контроллер вычисляет момент, а MuJoCo интегрирует q и q̇ через `mj_step`.

`--force` прикладывает к tip известную силу по +x: 3sin²(π(t−3)/2) Н на интервале 3–5 с, вне его 0. Сила направлена **от среды на робот**. Это воспроизводимый внешний толчок через `mj_applyFT`, без стенки, датчика и его шума. При перетаскивании мышью viewer может добавить другое возмущение; для повторяемого сравнения используйте программный толчок.

`--model-mass-scale` меняет все номинальные массы и соответствующие инерции модели контроллера. Физический XML и фактическая энергия не меняются. Это не добавление полезной нагрузки к физическому роботу. При экстремальных параметрах возможны насыщение или потеря устойчивости; в CSV записан признак насыщения.

`model.py` вычисляет M и её производные аналитически, C — по Кристоффелю, N=∇P. `mass_from_energy` независимо собирает M из CoM-якобианов и пространственных инерций. `verify.py` сравнивает обе M с MuJoCo; проверяет bias, inverse/forward dynamics, Ṁ−2C, градиенты гравитационного и виртуального потенциалов, перенос инерции и нулевую мощность gyro.

Результаты: CSV по методу, `metrics.json`, `comparison.png`. CSV содержит q,q̇, фактическую/заданную позиции tip, ограниченные моменты, силу, механическую энергию K+P, виртуальный потенциал, мощность входа и суставные потери. У разных методов разные задачи: их RMS не следует трактовать как общий рейтинг. Для impedance ненулевое отклонение во время толчка ожидаемо.

`rigid_body.py` — отдельная аналитическая модель свободного вращения, не MuJoCo. RK4 для w и экспоненциальный midpoint для R; CSV включает w,R,K,L⁰. Проверяйте отдельно энергию, угловой импульс и ортогональность. J=(0,02;0,04;0,055) кг·м², основной компонент скорости 5 рад/с.

Эталонные графики: [общие режимы](reference_results/default.png), [impedance и толчок](reference_results/impedance.png), [неточная модель](reference_results/mismatch.png), [промежуточная ось](reference_results/rigid.png). [Полный вывод](../lecture_notes.md).
