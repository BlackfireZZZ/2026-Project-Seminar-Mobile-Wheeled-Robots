# Практическая работа 1 — число в turtlesim

Пакет `practice01_digits` находится в `solution/`. Launch-файл запускает
`turtlesim`, удаляет стандартную черепаху, создаёт две новые и запускает два
экземпляра одной программы. Каждый экземпляр управляет своей черепахой через
`pose` и `cmd_vel`; перо выключается при переходе между отрезками. После
завершения оба узла остаются запущенными и публикуют нулевую скорость.

По умолчанию рисуется вариант **02**. При необходимости цифры можно изменить
аргументами запуска.

В контейнере курса:

```bash
cd ~/practices_ws
colcon build --symlink-install --packages-select practice01_digits
source install/local_setup.zsh
ros2 launch practice01_digits draw_number.launch.py
```

В другом терминале после завершения рисования:

```bash
ros2 node list
ros2 topic list
ros2 service list
rqt_graph
```

На графе должны быть видны `draw_left` и `draw_right`, связанные с `turtlesim`
через топики `/digit_left/pose`, `/digit_left/cmd_vel`, `/digit_right/pose` и
`/digit_right/cmd_vel`.
