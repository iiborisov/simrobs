# Общие ресурсы презентаций

Все конечные презентации автономны: CSS, JavaScript, CMU и математические шрифты встроены в `index.html`. Этот каталог нужен для воспроизводимого обновления; рядом с HTML его переносить не требуется.

## Набор и лицензии

- **CMU Serif**: `cmunrm`, `cmunbx`, `cmunti`, `cmunbi`; **CMU Typewriter Text**: `cmuntt`. Файлы получены из установленного TeX Live, конвертированы в WOFF без изменения глифов. Лицензия SIL OFL находится в [cmu/OFL.txt](cmu/OFL.txt). Исходный проект: [Computer Modern Unicode / CTAN](https://ctan.org/tex-archive/fonts/cm-unicode/fonts/otf).
- **KaTeX 0.16.22**: локальный JavaScript, CSS и WOFF2-файлы из официального npm-пакета `https://registry.npmjs.org/katex/-/katex-0.16.22.tgz`. Лицензия MIT: [LICENSE](katex-0.16.22/LICENSE). [Проект KaTeX](https://github.com/KaTeX/KaTeX).

Текст набран CMU, математический набор использует специализированные Computer Modern-подобные шрифты KaTeX: CMU Serif сам по себе не заменяет математический движок для дробей, матриц и растягиваемых скобок.

## Обновление

[lecture_expansions.py](lecture_expansions.py) содержит дополнительные разборы и задания в HTML и LaTeX. [upgrade_presentations.py](upgrade_presentations.py) добавляет ресурсы, пересобирает дополнительные страницы и планирует по 90 минут на каждую лекцию. Исходные интерактивные страницы сохраняются через `data-original-index`; новые маркируются `data-extra`.

Нужен Python с `lxml`:

```bash
python SRS_2026/presentation_assets/upgrade_presentations.py
```

После изменения формул проверьте `.tex-error`, загрузку CMU и интерактивы в браузере. После изменения содержания сверяйте автоматически созданные `TEACHING_PLAN.md`; 90 минут — рекомендованный сценарий с обсуждением, а не гарантия одинакового темпа для любой аудитории.
