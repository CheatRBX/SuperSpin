# BioBuzz FTC — код робота

Скопируйте `TeamCode/` в проект [FtcRobotController](https://github.com/FIRST-Tech-Challenge/FtcRobotController).

**Конфигурация Driver Hub:** моторы `fl fr bl br lift intake`, серво `claw`, IMU `imu`.

**Управление**
- Gamepad1: левый стик — езда, правый — поворот, RT — медленно, BACK — field/robot-centric, START — сброс IMU.
- Gamepad2: RT/LT — интейк, A — клешня, D-pad вниз/влево/вверх — пресеты лифта, левый стик — лифт вручную.

Подстройте константы в `Robot.java` (позиции серво, лимиты лифта, TICKS_PER_INCH) под вашего робота.
