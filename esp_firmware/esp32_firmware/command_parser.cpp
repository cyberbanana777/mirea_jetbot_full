// command_parser.cpp
#include "command_parser.h"

ParsedCommand CommandParser::parse(const String& input) {
    ParsedCommand cmd;
    if (input.length() < 3) return cmd;   // минимальная длина "$1;#"
    if (input[0] != '$')    return cmd;

    const char* str = input.c_str();
    // Пропускаем '$'
    const char* p = str + 1;

    // 1. Извлекаем тип команды (до первой ';')
    char typeStr[4] = {0};
    int i = 0;
    while (*p && *p != ';' && i < 3) {
        typeStr[i++] = *p++;
    }
    if (*p != ';') return cmd;  // нет разделителя
    int rawType = atoi(typeStr);
    if (rawType < 1 || rawType > 5) return cmd;
    cmd.type = static_cast<Command>(rawType);

    // 2. Извлекаем аргументы до '#'
    p++;  // перешагнули ';'
    uint8_t argIdx = 0;
    while (*p && *p != '#' && argIdx < 6) {
        // Собираем очередное число до ';' или '#'
        char numBuf[16] = {0};
        int j = 0;
        while (*p && *p != ';' && *p != '#' && j < 15) {
            numBuf[j++] = *p++;
        }
        if (j > 0) {
            cmd.args[argIdx++] = atof(numBuf);
        }
        if (*p == ';') p++;   // пропускаем разделитель
    }
    cmd.count = argIdx;
    return cmd;
}