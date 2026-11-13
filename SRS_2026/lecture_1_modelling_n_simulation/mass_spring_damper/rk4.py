"""Учебный запуск метода rk4; реализация находится в model.py."""
if __package__:
    from .experiments import main
else:  # Прямой запуск файла, в том числе из IDE.
    from experiments import main

if __name__ == '__main__':
    main('rk4')
