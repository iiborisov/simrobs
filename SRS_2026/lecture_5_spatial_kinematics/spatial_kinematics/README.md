# Пространственная кинематика: Python и MuJoCo

Обозначения: Hᵢʲ переводит координаты из Ψᵢ в Ψⱼ, T=(w;v) — твист-столбец, W=(τ F) — wrench-строка. В коде `H_i_j` сохраняет порядок индексов. Функция `exp_se3` принимает интегрированный твист, например TΔt, а не скорость без множителя времени.

Из корня репозитория:

```bash
python -m pip install -r SRS_2026/lecture_5_spatial_kinematics/spatial_kinematics/requirements.txt
# Координаты, мощность, винтовая траектория, дрейф SO(3), якобианы
python SRS_2026/lecture_5_spatial_kinematics/spatial_kinematics/examples.py --out /tmp/spatial-results
# Независимая проверка аналитики по MuJoCo, без окна
python SRS_2026/lecture_5_spatial_kinematics/spatial_kinematics/mujoco_kinematics.py --out /tmp/spatial-mujoco
# Окно с кинематически заданным движением 3R
python SRS_2026/lecture_5_spatial_kinematics/spatial_kinematics/mujoco_kinematics.py --viewer
# Проверка определений и модели
python SRS_2026/lecture_5_spatial_kinematics/spatial_kinematics/verify.py
# Модульный запуск
python -m SRS_2026.lecture_5_spatial_kinematics.spatial_kinematics.verify
```

Проверено на Python 3.13, NumPy 2.4.2 и MuJoCo 3.10.0. При необходимости используйте `/home/ib/miniconda3/envs/idt/bin/python`. Скрипты работают напрямую по абсолютному пути из любой папки. Для примеров без окна дисплей не нужен; viewer требует графической сессии. На macOS для viewer используйте `mjpython`.

Файлы:

- `geometry.py`: skew/hat/vee, SO(3)/SE(3) exp, inverse, Adjoint, преобразование строки сил; PoE, независимая локальная цепь, Js,Jb,Jp.
- `examples.py`: печатает отчёт и сохраняет `report.json`, `screw.csv`, `rotation_drift.csv`, `geometry.png`.
- `robot3r.xml`: пространственная геометрия с осями z,x,x в home-позе, длины 0,4/0,35/0,25 м.
- `mujoco_kinematics.py`: задаёт q(t),q̇(t), вызывает **mj_forward**, сверяет pose и сохраняет `3r_kinematics.csv` (t, суставные координаты/скорости, позиция, скорость tip, пространственный твист).
- `verify.py`: 50 случайных состояний, инвариантность мощности, PoE против локальной цепи, конечные разности Ḣ, Js/Jb/Jp, ориентация и позиция MuJoCo, `mj_jacSite`, чистый перенос и малый угол.

В MuJoCo-примере нет динамического интегрирования и регулятора: q задаётся кинематически, `mj_step` не вызывается. Физическая реакция на гравитацию, приводы и контакты здесь не исследуются. Это позволяет независимо проверить пространственную кинематику до добавления динамики.

Js имеет порядок строк (w;v). `mj_jacSite` возвращает угловую скорость и **скорость точки site**, поэтому для позиции сравниваем с Jp, а не нижними тремя строками Js. Эта разница проверяется численно.

[Эталонный график винтового движения и дрейфа](reference_results/geometry.png) · [Конспект и исправления опечаток источника](../lecture_notes.md).
