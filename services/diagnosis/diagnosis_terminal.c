#include "diagnosis_terminal.h"
#include "diagnosis.h"
#include "diagnosis_rtt.h"
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static char input[256], reply[1024];
static size_t used, reply_size;
static bool discard, awaiting;
static uint32_t pending_token, cursor, dropped;

static bool number(const char *text, uint32_t *value)
{
    if (!text || *text == '-' || *text == '+') return false;
    errno = 0;
    char *end;
    unsigned long raw = strtoul(text, &end, 0);
    if (errno || end == text || *end || raw > UINT32_MAX) return false;
    *value = (uint32_t)raw;
    return true;
}
static void result(uint32_t token, uint32_t status)
{
    reply_size = (size_t)snprintf(reply, sizeof(reply),
        "{\"type\":\"reply\",\"token\":%lu,\"result\":%lu}\n", (unsigned long)token, (unsigned long)status);
}
static void status(uint32_t token)
{
    size_t n = (size_t)snprintf(reply, sizeof(reply),
        "{\"type\":\"status\",\"protocol\":2,\"token\":%lu,\"uptime_ms\":%lu,\"loops\":%lu,\"event_head\":%lu,\"rtt_dropped\":%lu,\"bus\":[",
        (unsigned long)token, (unsigned long)g_diagnosis.uptime_ms, (unsigned long)g_diagnosis.loops,
        (unsigned long)g_diagnosis.event_head, (unsigned long)dropped);
    for (unsigned i = 0; i < 2; ++i) {
        DiagnosisBus bus = g_diagnosis.bus[i];
        uint32_t words[16];
        memcpy(words, &bus, sizeof(words));
        n += (size_t)snprintf(reply+n, sizeof(reply)-n, "%s[", i ? "," : "");
        for (unsigned j = 0; j < 16; ++j)
            n += (size_t)snprintf(reply+n, sizeof(reply)-n, "%s%lu", j ? "," : "", (unsigned long)words[j]);
        n += (size_t)snprintf(reply+n, sizeof(reply)-n, "]");
    }
    n += (size_t)snprintf(reply+n, sizeof(reply)-n, "]}\n");
    reply_size = n;
}
static void command(void)
{
    char *args[8];
    unsigned count = 0;
    char *word = strtok(input, " \t\r");
    while (word && count < 8) { args[count++] = word; word = strtok(NULL, " \t\r"); }
    uint32_t token = 0, values[5] = {0};
    if (count < 2 || word || !number(args[1], &token) || !token) { result(0, DIAG_INVALID); return; }
    if (!strcmp(args[0], "status") && count == 2) { status(token); return; }
    if (!strcmp(args[0], "hello") && count == 2) {
        reply_size = (size_t)snprintf(reply, sizeof(reply),
            "{\"type\":\"hello\",\"protocol\":2,\"token\":%lu}\n", (unsigned long)token);
        return;
    }
    bool is_send = !strcmp(args[0], "send");
    bool is_config = !strcmp(args[0], "config");
    bool is_read = !strcmp(args[0], "read");
    if ((!is_send && !is_config && !is_read) || (is_send ? count != 6 : count != 5)) {
        result(token, DIAG_INVALID); return;
    }
    unsigned numeric_end = is_send ? 5 : count;
    for (unsigned i = 2; i < numeric_end; ++i)
        if (!number(args[i], &values[i-2])) { result(token, DIAG_INVALID); return; }
    DiagnosisRequest request = {.seq = token, .channel = values[0]};
    if (!strcmp(args[0], "config") && count == 5) {
        request.op = DIAG_CONFIGURE; request.arg0 = values[1]; request.arg1 = values[2];
    } else if (!strcmp(args[0], "read") && count == 5 && values[1] <= 0x7ff && values[2] <= 255) {
        request.op = DIAG_SEND; request.id = 0x7ff; request.length = 8;
        request.data[0] = values[1] | (0x33U << 16) | (values[2] << 24);
    } else if (!strcmp(args[0], "send") && count == 6) {
        request.op = DIAG_SEND; request.id = values[1]; request.flags = values[2];
        const char *hex = args[5];
        size_t size = !strcmp(hex, "-") ? 0 : strlen(hex);
        if (size > 16 || size % 2) { result(token, DIAG_INVALID); return; }
        uint8_t bytes[8] = {0};
        for (size_t i = 0; i < size / 2; ++i) {
            char pair[3] = {hex[i*2], hex[i*2+1], 0};
            char *end;
            if (strspn(pair, "0123456789abcdefABCDEF") != 2) { result(token, DIAG_INVALID); return; }
            bytes[i] = (uint8_t)strtoul(pair, &end, 16);
        }
        request.length = (uint32_t)(size / 2);
        memcpy(request.data, bytes, sizeof(bytes));
    } else { result(token, DIAG_INVALID); return; }
    if (token == g_diagnosis.ack) { result(token, DIAG_INVALID); return; }
    /* This foreground terminal exclusively owns the engine's request slot.
     * Tokens correlate terminal replies with CAN completion events.
     */
    g_diagnosis.request = request;
    pending_token = token;
    awaiting = true;
}

void DiagnosisTerminal_Init(void)
{
    used = reply_size = 0; discard = awaiting = false;
    pending_token = cursor = dropped = 0;
    DiagnosisRtt_Init();
}

void DiagnosisTerminal_Step(uint32_t now_ms)
{
    if (reply_size && DiagnosisRtt_Write(reply, reply_size)) reply_size = 0;
    if (!reply_size && !awaiting) {
        char ch;
        for (unsigned i = 0; i < 256 && DiagnosisRtt_Read(&ch, 1) == 1; ++i) {
            if (ch == '\n') {
                input[used] = 0;
                if (discard) result(0, DIAG_INVALID);
                else if (used) command();
                used = 0; discard = false;
                break;
            }
            if (ch == 0) discard = true;
            if (!discard && used + 1 < sizeof(input)) input[used++] = ch;
            else discard = true;
        }
    }
    Diagnosis_Step(now_ms);
    if (awaiting && g_diagnosis.ack == pending_token) {
        result(pending_token, g_diagnosis.result);
        awaiting = false;
    }
    if (reply_size && DiagnosisRtt_Write(reply, reply_size)) reply_size = 0;

    /* Reserve drained RTT capacity for a pending reply before emitting more
     * events; a busy CAN stream must not starve a larger status response.
     */
    if (reply_size) return;

    uint32_t head = g_diagnosis.event_head;
    if (head < cursor) cursor = 0;
    if (head - cursor > DIAG_EVENT_COUNT) { dropped += head - cursor - DIAG_EVENT_COUNT; cursor = head - DIAG_EVENT_COUNT; }
    for (unsigned i = 0; i < 16 && cursor < head; ++i) {
        DiagnosisEvent event = g_diagnosis.events[cursor % DIAG_EVENT_COUNT];
        cursor++;
        uint32_t words[10];
        memcpy(words, &event, sizeof(words));
        char line[256];
        size_t n = (size_t)snprintf(line, sizeof(line), "{\"type\":\"event\",\"words\":[");
        for (unsigned j = 0; j < 10; ++j)
            n += (size_t)snprintf(line+n, sizeof(line)-n, "%s%lu", j ? "," : "", (unsigned long)words[j]);
        n += (size_t)snprintf(line+n, sizeof(line)-n, "]}\n");
        if (!DiagnosisRtt_Write(line, n)) dropped++;
    }
}
