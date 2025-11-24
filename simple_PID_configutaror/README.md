# Что это такое и с чем его едят

 

# Как развернуть программу
1. В терминале выполните (без поключенного esp32):
```bash
lsusb > without_device.txt
```
2. Подключите устройство (esp32) к компьютеру
3. В терминале выполните:
```bash
lsusb > with_device.txt
```
4. Сравните два файла и найдите разницу:
```bash
sudo bash determine_id.bash
```
5. Удалите временные файлы:
```bash
rm without_device.txt
rm with_device.txt
```
6. Физически переподключите устройство (esp32) к компьютеру
7. Проверьте, что оно успешно определилось системой:
```bash
ls /dev/esp32
```
8. Установите все python-зависимости:
```bash
sudo apt isntall python3-pip python3-tk
pip install -r pip_requirements.txt
```
9. В терминале выполните:
```bash 
python3 fast_gui.py
```
