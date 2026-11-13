# Лекции / Lecture materials

[Открыть HTML-каталог курса](index.html). Конспекты доступны для чтения в браузере, презентации — для интерактивных занятий.

Русские и английские версии содержат одинаковые модели, формулы и интерактивные эксперименты. / Both language versions share the same models, equations and interactive experiments.

| Lecture | Конспект RU | Notes EN | Слайды RU | Slides EN |
|---|---|---|---|---|
| Modelling and simulation | [RU](lecture_1_modelling_n_simulation/lecture_notes.md) | [EN](lecture_1_modelling_n_simulation/lecture_notes.en.md) | [RU](lecture_1_modelling_n_simulation/presentation/index.html) | [EN](lecture_1_modelling_n_simulation/presentation/index.en.html) |
| Dynamics of a planar 2R robot | [RU](lecture_2_2R_robot/lecture_notes.md) | [EN](lecture_2_2R_robot/lecture_notes.en.md) | [RU](lecture_2_2R_robot/presentation/index.html) | [EN](lecture_2_2R_robot/presentation/index.en.html) |
| Bond graphs | [RU](lecture_3_bond_graphs/lecture_notes.md) | [EN](lecture_3_bond_graphs/lecture_notes.en.md) | [RU](lecture_3_bond_graphs/presentation/index.html) | [EN](lecture_3_bond_graphs/presentation/index.en.html) |
| Control of a 2R robot | [RU](lecture_4_control/lecture_notes.md) | [EN](lecture_4_control/lecture_notes.en.md) | [RU](lecture_4_control/presentation/index.html) | [EN](lecture_4_control/presentation/index.en.html) |
| Spatial kinematics | [RU](lecture_5_spatial_kinematics/lecture_notes.md) | [EN](lecture_5_spatial_kinematics/lecture_notes.en.md) | [RU](lecture_5_spatial_kinematics/presentation/index.html) | [EN](lecture_5_spatial_kinematics/presentation/index.en.html) |
| Spatial dynamics | [RU](lecture_6_spatial_dynamics/lecture_notes.md) | [EN](lecture_6_spatial_dynamics/lecture_notes.en.md) | [RU](lecture_6_spatial_dynamics/presentation/index.html) | [EN](lecture_6_spatial_dynamics/presentation/index.en.html) |

## Публикация через GitHub Pages

Готовые HTML лежат рядом с Markdown-конспектами: `lecture_notes.html` и `lecture_notes.en.html`. Сборщик не требуется. Общий каталог — `index.html`; шрифты и стили конспектов — `web_assets/`.

1. Отправьте изменения в ветку `srs/new_lectures` или `main` репозитория `iiborisov/simrobs`.
2. В **Settings → Pages → Build and deployment → Source** выберите **GitHub Actions**.
3. В **Actions → Publish SRS lecture materials → Run workflow** выберите ветку с HTML и запустите публикацию. Следующие изменения материалов в этих ветках запускают её автоматически.

После успешного завершения workflow адрес каталога: `https://iiborisov.github.io/simrobs/SRS_2026/`. Фактический адрес публикации также отображается в результате задания Deploy Pages. До первого успешного развёртывания ссылка может быть недоступна.

Workflow [.github/workflows/lecture-pages.yml](../.github/workflows/lecture-pages.yml) публикует материалы курса и только те исходные PDF/ноутбук из `SRS`, на которые ссылаются лекции. При правках конспекта обновляйте также соответствующий HTML — автоматической конвертации Markdown в этом workflow нет.

[Настройка публикации GitHub Pages через Actions](https://docs.github.com/en/pages/getting-started-with-github-pages/using-custom-workflows-with-github-pages).
