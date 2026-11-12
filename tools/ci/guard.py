#!/usr/bin/env python3
"""Проверки согласованности протокола и гигиены репозитория (для CI).

Запуск:
    python3 tools/ci/guard.py            # все проверки
    python3 tools/ci/guard.py ids        # только выбранные проверки

Проверки:
  ids       набор ID, который принимает прошивка (decodeCommand), совпадает с
            таблицей «Jetson -> ESP32» в esp_firmware/PROTOCOL.md; каждый ID
            кадра в рабочем коде входит в контракт; $6 (отдельный канал
            Serial1) встречается только в своём модуле.
  comments  wire-ID в комментариях command_parser.h совпадают с decodeCommand.
  version   версия протокола в кадре $501 (protocol_wrapper.cpp) совпадает с
            комментарием на той же строке, со строкой «Protocol version» в
            таблицах «Versioning» (PROTOCOL.md, esp_firmware/README.md), с
            EXPECTED_PROTOCOL_VERSION в стенде и с моком стенда.
  hygiene   в индексе git нет артефактов сборки и результатов прогонов.
  paths     нет ссылок на пути, удалённые при чистке репозитория (Этап 5).

Скрипт опирается только на стандартную библиотеку Python и на файлы,
зафиксированные в git (git ls-files), поэтому результат одинаков на рабочей
машине и в CI.
"""

import argparse
import os
import re
import subprocess
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    '..', '..'))

PROTOCOL_MD = 'esp_firmware/PROTOCOL.md'
PARSER_CPP = 'esp_firmware/esp32_firmware/command_parser.cpp'
PARSER_H = 'esp_firmware/esp32_firmware/command_parser.h'
FIRMWARE_DIR = 'esp_firmware/esp32_firmware'

# Версия протокола ($501) дублируется в нескольких местах — проверяем,
# что копии не разъезжаются.
PROTOCOL_WRAPPER = 'esp_firmware/esp32_firmware/protocol_wrapper.cpp'
FIRMWARE_README = 'esp_firmware/README.md'
HARDWARE_SUITE = 'esp_firmware/tools/uart_bench/hardware_suite.py'
HARDWARE_SUITE_TEST = 'esp_firmware/tools/uart_bench/test_hardware_suite.py'

TABLE_TO_ESP = 'Jetson -> ESP32'
TABLE_FROM_ESP = 'ESP32 -> Jetson'

# $6 — не протокол v2, а отдельный канал Serial1 (дисплей и сеть).
SERIAL1_ID = 6
SERIAL1_FILES = ('init_scripts/jetbot_stats/jetbot_stats.py',)

# Каталоги и файлы, которые не являются рабочим кодом протокола.
SKIP_DIRS = ('docs/', 'esp_firmware/archive/', 'tools/ci/')
SKIP_SUFFIXES = ('.md', '.rst', '.png', '.jpg', '.jpeg', '.pdf', '.ico')

# Стендовые утилиты, которые намеренно шлют заведомо неверные кадры.
NEGATIVE_FRAME_FILES = (
    'esp_firmware/tools/uart_bench/check_malformed.py',
    'esp_firmware/tools/uart_bench/hardware_suite.py',
)

# Пути, удалённые при чистке репозитория (Этап 5).
REMOVED_PATHS = ('esp32_firmware_old', 'Speed_for_PID_test_speed')

FRAME_ID_RE = re.compile(r'\$(\d{1,3})(?=[;#])')

VERSION_FRAME_RE = re.compile(r'"\$501;(\d+);(\d+);#')
VERSION_COMMENT_RE = re.compile(r'//\s*major\s*=\s*(\d+)\s*,\s*minor\s*=\s*(\d+)')
VERSION_ROW_RE = re.compile(r'^\|\s*Protocol version\s*\|[^|]*\|\s*`(\d+);(\d+)`')
VERSION_CONST_RE = re.compile(
    r'EXPECTED_PROTOCOL_VERSION\s*=\s*\(\s*(\d+)(?:\.0+)?\s*,\s*(\d+)(?:\.0+)?\s*\)')
VERSION_MOCK_RE = re.compile(r'501\s*:\s*(\([^)]*\)|[A-Za-z_][A-Za-z_0-9]*)')


def git(*args):
    return subprocess.run(('git',) + args, cwd=ROOT, check=True,
                          capture_output=True, text=True).stdout


def tracked_files():
    return [f for f in git('ls-files').splitlines() if f]


def read_text(rel):
    """Текст файла или None, если это не текст (бинарник, не UTF-8)."""
    path = os.path.join(ROOT, rel)
    if not os.path.isfile(path):
        return None
    try:
        with open(path, encoding='utf-8') as handle:
            return handle.read()
    except (UnicodeDecodeError, OSError):
        return None


def protocol_tables():
    """ID из таблиц PROTOCOL.md: заголовок раздела -> множество ID."""
    text = read_text(PROTOCOL_MD) or ''
    sections = {}
    current = None
    for line in text.splitlines():
        if line.startswith('## '):
            current = line[3:].strip()
            sections.setdefault(current, set())
            continue
        if current is None or not line.startswith('|'):
            continue
        cells = [c.strip() for c in line.strip().strip('|').split('|')]
        if cells and cells[0].isdigit():
            sections[current].add(int(cells[0]))
    return sections


def firmware_accepted_ids():
    """ID, которые принимает прошивка: decodeCommand() в command_parser.cpp."""
    text = read_text(PARSER_CPP) or ''
    return {int(m) for m in re.findall(r'case\s+(\d+):\s*return\s+Command::', text)}


def enum_comment_ids():
    """Wire-ID из комментариев enum Command в command_parser.h.

    Берётся первый $N в комментарии: он соответствует самой команде.
    Дальше в комментарии могут упоминаться ID ответов ($202, $301, $401).
    """
    text = read_text(PARSER_H) or ''
    ids = set()
    for line in text.splitlines():
        match = re.match(r'\s*[A-Z_0-9]+\s*=\s*\d+,?\s*//(.*)$', line)
        if not match:
            continue
        found = re.findall(r'\$(\d{1,3})', match.group(1))
        if found:
            ids.add(int(found[0]))
    return ids


def is_production_code(path):
    """True для файлов рабочего кода (не документация и не тесты)."""
    if path.startswith(SKIP_DIRS):
        return False
    if path.endswith(SKIP_SUFFIXES):
        return False
    base = os.path.basename(path)
    if base.startswith('test_') or base.endswith('_test.py'):
        return False
    if '/test/' in path or path.startswith('test/'):
        return False
    if path in NEGATIVE_FRAME_FILES:
        return False
    return True


def check_ids():
    errors = []
    tables = protocol_tables()
    to_esp = tables.get(TABLE_TO_ESP, set())
    from_esp = tables.get(TABLE_FROM_ESP, set())
    accepted = firmware_accepted_ids()

    if not to_esp:
        errors.append('%s: не найдена таблица «%s»' % (PROTOCOL_MD, TABLE_TO_ESP))
    if to_esp != accepted:
        only_doc = sorted(to_esp - accepted)
        only_code = sorted(accepted - to_esp)
        if only_doc:
            errors.append('%s: описаны, но не принимаются прошивкой: %s'
                          % (PROTOCOL_MD, ', '.join('$%d' % i for i in only_doc)))
        if only_code:
            errors.append('%s: принимаются прошивкой, но не описаны: %s'
                          % (PARSER_CPP, ', '.join('$%d' % i for i in only_code)))

    allowed = to_esp | from_esp | {SERIAL1_ID}
    for path in tracked_files():
        if not is_production_code(path):
            continue
        text = read_text(path)
        if text is None:
            continue
        for match in FRAME_ID_RE.finditer(text):
            ident = int(match.group(1))
            line = text[:match.start()].count('\n') + 1
            if ident not in allowed:
                errors.append('%s:%d: ID $%d не описан в %s'
                              % (path, line, ident, PROTOCOL_MD))
            elif ident == SERIAL1_ID and path not in SERIAL1_FILES:
                errors.append('%s:%d: $%d (канал Serial1) используется вне %s'
                              % (path, line, SERIAL1_ID, ', '.join(SERIAL1_FILES)))

    # Ответы прошивки должны где-то реализовываться: ищем ID в её коде.
    firmware_text = ''
    for path in tracked_files():
        if path.startswith(FIRMWARE_DIR) and path.endswith('.cpp'):
            firmware_text += read_text(path) or ''
    for ident in sorted(from_esp):
        if not re.search(r'(?<!\d)%d(?!\d)' % ident, firmware_text):
            errors.append('%s: ID $%d описан как ответ прошивки, '
                          'но не встречается в коде прошивки' % (PROTOCOL_MD, ident))
    return errors


def check_comments():
    errors = []
    accepted = firmware_accepted_ids()
    comments = enum_comment_ids()
    only_doc = sorted(comments - accepted)
    only_code = sorted(accepted - comments)
    if only_doc:
        errors.append('%s: в комментариях есть ID, которых нет в decodeCommand: %s'
                      % (PARSER_H, ', '.join('$%d' % i for i in only_doc)))
    if only_code:
        errors.append('%s: в decodeCommand есть ID без комментария с wire-ID: %s'
                      % (PARSER_H, ', '.join('$%d' % i for i in only_code)))
    return errors


def check_version():
    """Версия протокола в $501 должна совпадать во всех копиях.

    Авторитетный источник — кадр в ``protocol_wrapper.cpp``. Проверяются:
      * комментарий ``// major=X, minor=Y`` на той же строке;
      * строка «Protocol version» в таблице «Versioning» (PROTOCOL.md и
        esp_firmware/README.md);
      * константа ``EXPECTED_PROTOCOL_VERSION`` в стенде;
      * мок ответа ``$501`` в тесте стенда — только через эту константу.

    Отсутствие любого из источников — ошибка: иначе переименование строки или
    реформат таблицы отключили бы проверку молча.
    """
    errors = []

    wrapper = read_text(PROTOCOL_WRAPPER)
    if wrapper is None:
        return ['%s: файл не найден' % PROTOCOL_WRAPPER]

    frames = []
    for number, line in enumerate(wrapper.splitlines(), start=1):
        if '"$501;' not in line:
            continue
        frame = VERSION_FRAME_RE.search(line)
        if not frame:
            errors.append('%s:%d: кадр $501 не распознан' % (PROTOCOL_WRAPPER, number))
            continue
        comment = VERSION_COMMENT_RE.search(line)
        if not comment:
            errors.append('%s:%d: на строке с кадром $501 нет комментария '
                          '«// major=X, minor=Y»' % (PROTOCOL_WRAPPER, number))
            comment = None
        else:
            comment = (int(comment.group(1)), int(comment.group(2)))
        frames.append((number, (int(frame.group(1)), int(frame.group(2))), comment))
    if not frames:
        errors.append('%s: не найден кадр $501' % PROTOCOL_WRAPPER)
        return errors
    if len(frames) > 1:
        errors.append('%s: найдено несколько кадров $501 (строки %s)'
                      % (PROTOCOL_WRAPPER, ', '.join(str(f[0]) for f in frames)))
        return errors

    line_no, frame, comment = frames[0]
    if comment is not None and frame != comment:
        errors.append('%s:%d: комментарий «major=%d, minor=%d» не совпадает с '
                      'кадром «$501;%d;%d;#»'
                      % (PROTOCOL_WRAPPER, line_no, comment[0], comment[1],
                         frame[0], frame[1]))

    for path in (PROTOCOL_MD, FIRMWARE_README):
        text = read_text(path)
        if text is None:
            errors.append('%s: файл не найден' % path)
            continue
        rows = [m for m in (VERSION_ROW_RE.match(line) for line in text.splitlines())
                if m]
        if not rows:
            errors.append('%s: не найдена строка «| Protocol version | ... | `N;M` |»'
                          % path)
            continue
        if len(rows) > 1:
            errors.append('%s: в таблице версий несколько строк «Protocol version»'
                          % path)
            continue
        doc = (int(rows[0].group(1)), int(rows[0].group(2)))
        if doc != frame:
            errors.append('%s: версия протокола «%d;%d» не совпадает с кадром '
                          '«$501;%d;%d;#» в %s'
                          % (path, doc[0], doc[1], frame[0], frame[1],
                             PROTOCOL_WRAPPER))

    suite = read_text(HARDWARE_SUITE)
    if suite is None:
        errors.append('%s: файл не найден' % HARDWARE_SUITE)
    else:
        const = VERSION_CONST_RE.search(suite)
        if not const:
            errors.append('%s: не найдена константа EXPECTED_PROTOCOL_VERSION'
                          % HARDWARE_SUITE)
        else:
            bench = (int(const.group(1)), int(const.group(2)))
            if bench != frame:
                errors.append('%s: EXPECTED_PROTOCOL_VERSION = %d;%d не совпадает '
                              'с кадром «$501;%d;%d;#» в %s'
                              % (HARDWARE_SUITE, bench[0], bench[1],
                                 frame[0], frame[1], PROTOCOL_WRAPPER))

    mock = read_text(HARDWARE_SUITE_TEST)
    if mock is None:
        errors.append('%s: файл не найден' % HARDWARE_SUITE_TEST)
    else:
        imports = [line for line in mock.splitlines()
                   if 'from hardware_suite import' in line]
        if not imports:
            errors.append('%s: нет импорта из hardware_suite' % HARDWARE_SUITE_TEST)
        elif 'EXPECTED_PROTOCOL_VERSION' not in imports[0]:
            errors.append('%s: ожидается импорт EXPECTED_PROTOCOL_VERSION из '
                          'hardware_suite' % HARDWARE_SUITE_TEST)
        payload = VERSION_MOCK_RE.search(mock)
        if not payload:
            errors.append('%s: не найден мок ответа $501' % HARDWARE_SUITE_TEST)
        elif payload.group(1) != 'EXPECTED_PROTOCOL_VERSION':
            errors.append('%s: мок ответа $501 использует литерал «%s» — версия '
                          'должна браться из EXPECTED_PROTOCOL_VERSION'
                          % (HARDWARE_SUITE_TEST, payload.group(1)))
    return errors


ARTIFACT_PATTERNS = (
    (re.compile(r'\.o$'), 'объектный файл сборки'),
    (re.compile(r'\.diff$'), 'diff-файл'),
    (re.compile(r'_log.*\.csv$'), 'лог прогона'),
    (re.compile(r'_plot.*\.png$'), 'график прогона'),
    (re.compile(r'^esp_firmware/reports/'), 'отчёт стенда'),
)


def check_hygiene():
    errors = []
    files = tracked_files()
    for path in files:
        for pattern, description in ARTIFACT_PATTERNS:
            if pattern.search(path):
                errors.append('%s: %s не должен попадать в индекс git'
                              % (path, description))
    if 'esp_firmware/archive/README.md' not in files:
        errors.append('нет esp_firmware/archive/README.md — архив не описан')
    return errors


def check_paths():
    errors = []
    for path in tracked_files():
        # Сам скрипт содержит имена удалённых путей (список REMOVED_PATHS),
        # поэтому его и документацию из проверки исключаем.
        if (path.startswith('docs/ai/') or path.startswith('esp_firmware/archive/')
                or path.startswith('tools/ci/')):
            continue
        if path.endswith(SKIP_SUFFIXES):
            continue
        text = read_text(path)
        if text is None:
            continue
        for token in REMOVED_PATHS:
            if token in text:
                errors.append('%s: ссылка на удалённый путь «%s»' % (path, token))
    return errors


CHECKS = {
    'ids': check_ids,
    'comments': check_comments,
    'version': check_version,
    'hygiene': check_hygiene,
    'paths': check_paths,
}


def main(argv):
    parser = argparse.ArgumentParser(description='Проверки CI проекта jetbot')
    parser.add_argument('checks', nargs='*', default=[],
                        help='подмножество проверок: ' + ', '.join(sorted(CHECKS)))
    args = parser.parse_args(argv)

    selected = args.checks or sorted(CHECKS)
    unknown = [name for name in selected if name not in CHECKS]
    if unknown:
        parser.error('неизвестные проверки: %s' % ', '.join(unknown))

    failed = False
    for name in selected:
        errors = CHECKS[name]()
        if errors:
            failed = True
            print('[%s] FAIL (%d)' % (name, len(errors)))
            for error in errors:
                print('  - %s' % error)
        else:
            print('[%s] OK' % name)
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
