# Example-for-the-Summer-School
Пример кода для Летней школы от Яндекс.

Файлы AcelerometerV2.h и AcelerometerV2.cpp – изменения для библиотеки компании "Образование Будущего", в которой я работаю. Данные фалйы позволяют подключить датчик LSM6DS3 (измеряющий линейные ускорения и угловые скорости по трём осям) к микроконтроллеру STM32F103C8 по шине I2C, настроить его и считать данные. Для программирования в данном случае используется среда разработки Arduino.

Библиотеку целиком (автор не я) можно посмотреть по ссылке:
https://github.com/Obu-IntroSat/IntroSatLib

Файл main.c – код для работы с "Имитатором солнечных батарей" – платой с микроконтроллером, считывающей информацию с нескольких типов датчиков (датчик-источник информации выбирается пользователем при подключении платы к компьютеру по UART) и обеспечивающей определённое значение напряжения и тока на выходе. Код написан в среде разработки STM32CubeIDE. 
