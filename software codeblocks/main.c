#include <gtk/gtk.h>
#include <windows.h>
#include <locale.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>  // Для M_PI и математических функций

/* ---------- Конфигурация ---------- */
#define GET_STATUS_POLL_MS 300  /* опрашивать телеметрию каждые 300 ms */
#define MAX_LOG_LINES 1000

/* ---------- список точек (time_sec, temp_C) ---------- */
typedef struct {
    double t; /* seconds */
    double temp; /* °C */
} ProfilePoint;

typedef struct {
    ProfilePoint *pts;
    size_t count;
    size_t cap;
} Profile;

/* ---------- Real-time data logging ---------- */
typedef struct {
    double time;    // время в секундах
    double temp;    // температура
    double setpoint; // заданная температура
} RealtimePoint;

typedef struct {
    RealtimePoint *points;
    size_t count;
    size_t capacity;
    guint start_time; // время начала записи в мс
    gboolean recording;
} RealtimeData;

static RealtimeData realtime_data;
static gboolean profile_draw(GtkWidget *widget, cairo_t *cr, gpointer data);
static void draw_axes(cairo_t *cr, int w, int h, double tmin, double tmax, double tempmin, double tempmax);
static void process_received_data(const char *data);
static void on_get_profile(GtkButton *b, gpointer user_data);
static gboolean update_profile_time_cb(gpointer data);
static gboolean disable_serial_logging(gpointer data);
static void stop_periodic_telemetry(void);
static void start_periodic_telemetry(void);
static void realtime_data_add(RealtimeData *rd, double temp, double setpoint);
static void realtime_data_start(RealtimeData *rd);
static void realtime_data_stop(RealtimeData *rd);
static void realtime_data_clear(RealtimeData *rd);
static void realtime_data_init(RealtimeData *rd);
static void realtime_data_free(RealtimeData *rd);
static void fill_table_from_profile(void);
static void draw_legend(cairo_t *cr, int w, int h, gboolean draw_profile, gboolean draw_realtime);

static void profile_init(Profile *p) {
    p->pts = NULL; p->count = 0; p->cap = 0;
}
static void profile_free(Profile *p) { free(p->pts); p->pts = NULL; p->count = p->cap = 0; }
static void profile_add(Profile *p, double t, double temp) {
    if (p->count+1 > p->cap) {
        p->cap = p->cap ? p->cap*2 : 8;
        p->pts = realloc(p->pts, p->cap * sizeof(ProfilePoint));
    }
    p->pts[p->count].t = t;
    p->pts[p->count].temp = temp;
    p->count++;
}
static void profile_remove(Profile *p, size_t idx) {
    if (idx >= p->count) return;
    memmove(&p->pts[idx], &p->pts[idx+1], (p->count - idx - 1) * sizeof(ProfilePoint));
    p->count--;
}

/* ---------- Serial (Windows) ---------- */
static HANDLE serial_handle = INVALID_HANDLE_VALUE;

int serial_open(const char *dev, int baud) {
    if (serial_handle != INVALID_HANDLE_VALUE) CloseHandle(serial_handle);

    char port[20];
    if (strncmp(dev, "COM", 6) != 0) {
        snprintf(port, sizeof(port), "\\\\.\\%s", dev);
    } else {
        snprintf(port, sizeof(port), "\\\\.\\%s", dev);
    }

    serial_handle = CreateFileA(port, GENERIC_READ | GENERIC_WRITE, 0, NULL,
                               OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

    if (serial_handle == INVALID_HANDLE_VALUE) {
        return -1;
    }

    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

    if (!GetCommState(serial_handle, &dcbSerialParams)) {
        CloseHandle(serial_handle);
        serial_handle = INVALID_HANDLE_VALUE;
        return -1;
    }

    dcbSerialParams.BaudRate = baud;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;

    if (!SetCommState(serial_handle, &dcbSerialParams)) {
        CloseHandle(serial_handle);
        serial_handle = INVALID_HANDLE_VALUE;
        return -1;
    }

    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = 10;
    timeouts.ReadTotalTimeoutConstant = 10;
    timeouts.ReadTotalTimeoutMultiplier = 1;
    timeouts.WriteTotalTimeoutConstant = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;

    if (!SetCommTimeouts(serial_handle, &timeouts)) {
        CloseHandle(serial_handle);
        serial_handle = INVALID_HANDLE_VALUE;
        return -1;
    }

    return 0;
}

void serial_close_h() {
    if (serial_handle != INVALID_HANDLE_VALUE) {
        CloseHandle(serial_handle);
        serial_handle = INVALID_HANDLE_VALUE;
    }
}

ssize_t serial_write_all(const void *buf, size_t len) {
    if (serial_handle == INVALID_HANDLE_VALUE) return -1;

    DWORD bytes_written;
    if (WriteFile(serial_handle, buf, (DWORD)len, &bytes_written, NULL)) {
        return (ssize_t)bytes_written;
    }
    return -1;
}

ssize_t serial_read_some(char *buf, size_t maxlen) {
    if (serial_handle == INVALID_HANDLE_VALUE) return -1;

    DWORD bytes_read;
    if (ReadFile(serial_handle, buf, (DWORD)maxlen, &bytes_read, NULL)) {
        return (ssize_t)bytes_read;
    }
    return -1;
}

/* ---------- GUI state ---------- */
typedef struct {
    GtkWidget *window;
    GtkWidget *connect_btn;
    GtkWidget *get_status_btn;
    GtkWidget *port_entry;
    GtkWidget *log_textview;
    GtkTextBuffer *log_buf;

    GtkWidget *pid_freq_entry, *window_time_entry, *kp_entry, *ki_entry, *kd_entry;
    GtkWidget *send_pid_btn, *save_pid_btn, *get_pid_btn;

    GtkWidget *temp_label, *setpoint_label, *state_label, *time_label;

    GtkWidget *profile_canvas;
    Profile profile;

    GtkWidget *start_btn, *stop_btn;
    gboolean profile_active;
    guint profile_start_time;
    guint profile_timeout_id;
    gboolean log_serial_data;
    guint telemetry_timeout_id;
    GtkWidget *setpoint_entry;
    GtkWidget *set_setpoint_btn;
    GtkWidget *start_setpoint_btn;
    GtkWidget *stop_setpoint_btn;
    GtkWidget *time_entries[5];
    GtkWidget *temp_entries[5];
    gboolean fixed_scale;      // фиксированный масштаб
    double fixed_tmax;         // фиксированное максимальное время
    double fixed_tempmin;      // фиксированная минимальная температура
    double fixed_tempmax;      // фиксированная максимальная температура
} AppWidgets;

static AppWidgets W;

static void scroll_log_to_top(void) {
    GtkTextIter start;
    gtk_text_buffer_get_start_iter(W.log_buf, &start);
    gtk_text_view_scroll_to_iter(GTK_TEXT_VIEW(W.log_textview), &start, 0.0, FALSE, 0.0, 0.0);
}
/* ---------- Logging ---------- */
static void append_log(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    char buf[1024];
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);

    GtkTextIter start;
    gtk_text_buffer_get_start_iter(W.log_buf, &start);
    gtk_text_buffer_insert(W.log_buf, &start, buf, -1);
    gtk_text_buffer_insert(W.log_buf, &start, "\n", -1);

    // ОГРАНИЧИВАЕМ КОЛИЧЕСТВО СТРОК
    GtkTextIter end;
    gtk_text_buffer_get_end_iter(W.log_buf, &end);
    int line_count = gtk_text_buffer_get_line_count(W.log_buf);

    if (line_count > MAX_LOG_LINES) {
        GtkTextIter line_end;
        gtk_text_buffer_get_iter_at_line(W.log_buf, &line_end, line_count - MAX_LOG_LINES);
        gtk_text_buffer_delete(W.log_buf, &line_end, &end);
    }
    scroll_log_to_top();
}

// "Тихая" отправка команды (без лога в TX)
static void send_command_quiet(const char *cmd) {
    if (serial_handle == INVALID_HANDLE_VALUE) return;

    char out[1024];
    snprintf(out, sizeof out, "%s\n", cmd);
    serial_write_all(out, strlen(out));
}
// Обычная отправка команды (с логом)
static void send_command(const char *cmd) {
    if (serial_handle == INVALID_HANDLE_VALUE) { append_log("Not connected"); return; }
    char out[1024];
    snprintf(out, sizeof out, "%s\n", cmd);
    ssize_t w = serial_write_all(out, strlen(out));
    if (w < 0) {
        DWORD error = GetLastError();
        append_log("Error writing to serial: error code %lu", error);
    }
    else append_log("TX: %s", cmd);
}
// Обработчик для установки температуры
static void on_set_setpoint(GtkWidget *widget, gpointer data) {
    stop_periodic_telemetry();
    const char *text = gtk_entry_get_text(GTK_ENTRY(W.setpoint_entry));

    char command[64];
    snprintf(command, sizeof(command), "TEMP_POINT_SET:%s", text);
    //  ЗАМЕНЯЕМ ЗАПЯТУЮ НА ТОЧКУ
    for (char *p = command; *p; p++) {
        if (*p == ',') *p = '.';
    }
    send_command(command);
    g_timeout_add(2000, (GSourceFunc)start_periodic_telemetry, NULL);
}

// Обработчик для запуска режима Holding Temp Point
static void on_start_setpoint(GtkWidget *widget, gpointer data) {
if (serial_handle == INVALID_HANDLE_VALUE) {
        append_log("Not connected");
        return;
    }
    realtime_data_start(&realtime_data);
    // ОСТАНАВЛИВАЕМ АВТОМАТИЧЕСКИЕ ЗАПРОСЫ
    stop_periodic_telemetry();
    W.log_serial_data = TRUE;

    send_command("START_SETPOINT");
    append_log("Started holding temperature point");

    // ЗАПУСКАЕМ АВТОМАТИЧЕСКИЕ ЗАПРОСЫ ЧЕРЕЗ 1 СЕКУНДЫ
    g_timeout_add(1000, (GSourceFunc)disable_serial_logging, NULL);
    g_timeout_add(1000, (GSourceFunc)start_periodic_telemetry, NULL);
}

// Обработчик для остановки режима Holding Temp Point
static void on_stop_setpoint(GtkWidget *widget, gpointer data) {
    if (serial_handle == INVALID_HANDLE_VALUE) {
        append_log("Not connected");
        return;
    }
    realtime_data_stop(&realtime_data);
    // ОСТАНАВЛИВАЕМ АВТОМАТИЧЕСКИЕ ЗАПРОСЫ
    stop_periodic_telemetry();
    W.log_serial_data = TRUE;

    send_command("STOP_SETPOINT");
    append_log("Stopped holding temperature point");

    // ЗАПУСКАЕМ АВТОМАТИЧЕСКИЕ ЗАПРОСЫ ЧЕРЕЗ 1 СЕКУНДУ
    g_timeout_add(1000, (GSourceFunc)disable_serial_logging, NULL);
    g_timeout_add(1000, (GSourceFunc)start_periodic_telemetry, NULL);
}
/* ---------- PID handlers ---------- */
static void on_send_pid(GtkButton *b, gpointer user_data) {
    const char *freq = gtk_entry_get_text(GTK_ENTRY(W.pid_freq_entry));
    const char *window = gtk_entry_get_text(GTK_ENTRY(W.window_time_entry));
    const char *kp = gtk_entry_get_text(GTK_ENTRY(W.kp_entry));
    const char *ki = gtk_entry_get_text(GTK_ENTRY(W.ki_entry));
    const char *kd = gtk_entry_get_text(GTK_ENTRY(W.kd_entry));
    char cmd[128];
    snprintf(cmd, sizeof cmd, "SET_PID:%s,%s,%s,%s,%s", freq, window, kp, ki, kd);
    stop_periodic_telemetry();
    W.log_serial_data = TRUE;
    send_command(cmd);
    g_timeout_add(1000, (GSourceFunc)disable_serial_logging, NULL);
    // ЗАПУСКАЕМ АВТОМАТИЧЕСКИЕ ЗАПРОСЫ ЧЕРЕЗ 1 СЕКУНДУ
    g_timeout_add(1000, (GSourceFunc)start_periodic_telemetry, NULL);
}

static void on_save_pid(GtkButton *b, gpointer ud) {
    stop_periodic_telemetry();
    W.log_serial_data = TRUE;
    send_command("SAVE_PID");
    g_timeout_add(1000, (GSourceFunc)disable_serial_logging, NULL);
    // ЗАПУСКАЕМ АВТОМАТИЧЕСКИЕ ЗАПРОСЫ ЧЕРЕЗ 1 СЕКУНДЫ
    g_timeout_add(1000, (GSourceFunc)start_periodic_telemetry, NULL);
}

static void on_get_pid(GtkButton *b, gpointer ud) {
    stop_periodic_telemetry();
    W.log_serial_data = TRUE;
    send_command("GET_PID");
    g_timeout_add(1000, (GSourceFunc)disable_serial_logging, NULL);
    g_timeout_add(1000, (GSourceFunc)start_periodic_telemetry, NULL);
}

static void on_start(GtkButton *b, gpointer ud) {
    stop_periodic_telemetry();
    realtime_data_start(&realtime_data);
    // Включаем логирование для этой команды
    W.log_serial_data = TRUE;
    send_command("START_PROFILE");

    // ЗАПУСКАЕМ ТАЙМЕР ПРОФИЛЯ
    W.profile_active = TRUE;
    W.profile_start_time = g_get_monotonic_time() / 1000;

    // Если таймер еще не запущен, запускаем
    if (W.profile_timeout_id == 0) {
        W.profile_timeout_id = g_timeout_add(1000, update_profile_time_cb, NULL);
    }
    // Выключаем логирование через 1 секунду
    g_timeout_add(1000, (GSourceFunc)disable_serial_logging, NULL);
}
static void on_stop(GtkButton *b, gpointer ud) {
    stop_periodic_telemetry();
    realtime_data_stop(&realtime_data);
    // Включаем логирование для этой команды
    W.log_serial_data = TRUE;
    send_command("STOP_PROFILE");

    //ОСТАНАВЛИВАЕМ ТАЙМЕР ПРОФИЛЯ
    W.profile_active = FALSE;
    // Выключаем логирование через 1 секунду
    g_timeout_add(1000, (GSourceFunc)disable_serial_logging, NULL);
    g_timeout_add(1000, (GSourceFunc)start_periodic_telemetry, NULL);
}

static gboolean profile_draw(GtkWidget *widget, cairo_t *cr, gpointer data)
{
    //  ОБЪЯВЛЯЕМ ПЕРЕМЕННЫЕ МАСШТАБА В НАЧАЛЕ ФУНКЦИИ
    double tmin = 0;
    double tmax = 100;
    double tempmin = 0;
    double tempmax = 200;
    int w, h;
    // Если идет запись реальных данных И НЕ активен профиль - значит это Holding режим
    gboolean is_holding_mode = realtime_data.recording && !W.profile_active;

    if (W.fixed_scale) {
        // Используем фиксированный масштаб
        tmax = W.fixed_tmax;
        tempmin = W.fixed_tempmin;
        tempmax = W.fixed_tempmax;

        // Получаем размеры виджета
        GtkAllocation alloc;
        gtk_widget_get_allocation(widget, &alloc);
        w = alloc.width;
        h = alloc.height;
    } else {
        // Автоматический масштаб
        GtkAllocation alloc;
        gtk_widget_get_allocation(widget, &alloc);
        w = alloc.width;
        h = alloc.height;

        /* white background */
        cairo_set_source_rgb(cr, 1,1,1);
        cairo_paint(cr);

        /* grid lines */
        cairo_set_source_rgb(cr, 0.9,0.9,0.9);
        for (int x=0;x<w;x+=50) {
            cairo_move_to(cr,x,0);
            cairo_line_to(cr,x,h);
        }
        for (int y=0;y<h;y+=40) {
            cairo_move_to(cr,0,y);
            cairo_line_to(cr,w,y);
        }
        cairo_stroke(cr);

        // ОПРЕДЕЛЯЕМ, ЧТО РИСОВАТЬ
        gboolean draw_realtime = realtime_data.recording && realtime_data.count > 0;
        gboolean draw_profile = W.profile.count > 0;

        if (!draw_realtime && !draw_profile) {
            // Если нет данных, рисуем только оси
            draw_axes(cr, w, h, 0, 100, 0, 200);
            return FALSE;
        }

        // ИНИЦИАЛИЗИРУЕМ МИНИМАЛЬНЫЕ И МАКСИМАЛЬНЫЕ ЗНАЧЕНИЯ
        tempmin = 1e9;
        tempmax = -1e9;
        double current_max_time = 0;
        double current_min_time = 1e9;

        // СНАЧАЛА ОПРЕДЕЛЯЕМ МАСШТАБ ПО РЕАЛЬНЫМ ДАННЫМ (ЕСЛИ ОНИ ЕСТЬ)
        if (draw_realtime && realtime_data.count > 0) {
            gboolean found_valid_data = FALSE;

            for (size_t i = 0; i < realtime_data.count; i++) {
                // пропускаем все начальные точки с setpoint=0 до первой валидной
                if (realtime_data.points[i].setpoint == 0 && !found_valid_data) {
                    continue;
                }
                found_valid_data = TRUE;
                if (realtime_data.points[i].temp < tempmin) tempmin = realtime_data.points[i].temp;
                if (realtime_data.points[i].temp > tempmax) tempmax = realtime_data.points[i].temp;
                if (realtime_data.points[i].setpoint < tempmin) tempmin = realtime_data.points[i].setpoint;
                if (realtime_data.points[i].setpoint > tempmax) tempmax = realtime_data.points[i].setpoint;

                if (realtime_data.points[i].time < current_min_time) current_min_time = realtime_data.points[i].time;
                if (realtime_data.points[i].time > current_max_time) current_max_time = realtime_data.points[i].time;
            }
            // В РЕЖИМЕ HOLDING TEMP POINT - ПОКАЗЫВАЕМ ТОЛЬКО ПОСЛЕДНИЕ 1000 СЕКУНД
            if (is_holding_mode) {
                if (current_max_time > 1000) {
                    tmin = current_max_time - 1000;
                    tmax = current_max_time;
                } else {
                    // Если данных меньше 1000 секунд - показываем с нуля
                    tmin = 0;
                    tmax = current_max_time + 10;
                }
            } else {
                // В обычном режиме Profile - показываем все данные от начала
                tmin = current_min_time;
                tmax = current_max_time + 10;
            }
        } else {
            // Если нет реальных данных, используем значения по умолчанию для времени
            tmax = 100;
        }
        // ЗАТЕМ РАСШИРЯЕМ МАСШТАБ ПОД ПРОФИЛЬ (ЕСЛИ ОН ЕСТЬ)
        // В РЕЖИМЕ HOLDING ПРОФИЛЬ НЕ УЧИТЫВАЕМ ДЛЯ МАСШТАБА
        if (draw_profile && !is_holding_mode) {
            double profile_tmax = W.profile.pts[W.profile.count-1].t;
            double profile_tempmin = 1e9;
            double profile_tempmax = -1e9;

            for (size_t i=0;i<W.profile.count;i++) {
                if (W.profile.pts[i].temp < profile_tempmin) profile_tempmin = W.profile.pts[i].temp;
                if (W.profile.pts[i].temp > profile_tempmax) profile_tempmax = W.profile.pts[i].temp;
            }
            // ОБНОВЛЯЕМ ТЕМПЕРАТУРНЫЙ ДИАПАЗОН ДЛЯ ПРОФИЛЯ
            if (profile_tempmin < tempmin) tempmin = profile_tempmin;
            if (profile_tempmax > tempmax) tempmax = profile_tempmax;
            if (profile_tmax > tmax) tmax = profile_tmax;

            // ДЛЯ ПРОФИЛЯ ВРЕМЯ НАЧИНАЕТСЯ С 0
            tmin = 0;
        }

        // ЕСЛИ НЕТ ДАННЫХ ДЛЯ ТЕМПЕРАТУР, ИСПОЛЬЗУЕМ ЗНАЧЕНИЯ ПО УМОЛЧАНИЮ
        if (tempmin > 1e8 || tempmax < -1e8) {
            tempmin = 0;
            tempmax = 100;
        }
        // ДОБАВЛЯЕМ ЗАПАС 10% СВЕРХУ И СНИЗУ ДЛЯ КОМФОРТНОГО ПРОСМОТРА
        double temp_range = tempmax - tempmin;
        if (temp_range > 0) {
            tempmin -= temp_range * 0.1;
            tempmax += temp_range * 0.1;
        } else {
            // Если все данные одинаковые
            tempmin -= 10;
            tempmax += 10;
        }

        // ГАРАНТИРУЕМ МИНИМАЛЬНЫЙ ДИАПАЗОН ТЕМПЕРАТУР (20 градусов)
        if (tempmax - tempmin < 20 && tempmax < 1500) {
            double center = (tempmin + tempmax) / 2;
            tempmin = center - 10;
            tempmax = center + 10;
        }

        double temp_range_rounded = tempmax - tempmin;
        double temp_step;
        if (temp_range_rounded > 500) {
            temp_step = 100;
        } else if (temp_range_rounded > 200) {
            temp_step = 50;  // Для больших диапазонов - шаг 50
        } else if (temp_range_rounded > 100) {
            temp_step = 20;  // Для средних диапазонов - шаг 20
        } else if (temp_range_rounded > 50) {
            temp_step = 10;  // Для обычных диапазонов - шаг 10
        } else {
            temp_step = 5;   // Для маленьких диапазонов - шаг 5
        }

        tempmin = floor(tempmin / temp_step) * temp_step;
        tempmax = ceil(tempmax / temp_step) * temp_step;

        // ДЛЯ РЕЖИМА HOLDING - ОКРУГЛЯЕМ ВРЕМЯ
        if (is_holding_mode && tmax - tmin > 1000) {
            tmin = floor(tmin / 10.0) * 10.0;
            tmax = ceil(tmax / 10.0) * 10.0;
        } else {
            tmax = ceil(tmax / 10.0) * 10.0;
        }
        // ГАРАНТИРУЕМ МИНИМАЛЬНЫЕ ЗНАЧЕНИЯ ДИАПАЗОНОВ
        if (tempmax - tempmin < 20 && tempmax < 1500) tempmax = tempmin + 20;
        if (tmax - tmin < 20) tmax = tmin + 20;
     }
     // Рисуем оси с вычисленными пределами
     draw_axes(cr, w, h, tmin, tmax, tempmin, tempmax);
     // Параметры области графика
     int axis_margin_left = 50;
     int axis_margin_bottom = 120;
     int plot_width = w - axis_margin_left - 10;
     int plot_height = h - axis_margin_bottom - 10;
     // ОПРЕДЕЛЯЕМ, ЧТО РИСОВАТЬ
     gboolean draw_realtime = realtime_data.recording && realtime_data.count > 0;
     gboolean draw_profile = W.profile.count > 0 && !is_holding_mode; //  В РЕЖИМЕ HOLDING НЕ РИСУЕМ ПРОФИЛЬ
     // СНАЧАЛА РИСУЕМ ПРОФИЛЬ (ПОД РЕАЛЬНЫМИ ДАННЫМИ)
     if (draw_profile) {
        cairo_set_line_width(cr, 3.0);
        cairo_set_source_rgb(cr, 0.8,0,0); // Красный для профиля

        for (size_t i=0;i<W.profile.count;i++) {
            double x = axis_margin_left + ((W.profile.pts[i].t - tmin) / (tmax - tmin)) * plot_width;
            double y = (h - axis_margin_bottom) - ((W.profile.pts[i].temp - tempmin) / (tempmax - tempmin)) * plot_height;
            if (i==0) cairo_move_to(cr, x, y);
            else cairo_line_to(cr, x, y);
        }
        cairo_stroke(cr);
        /* points */
        cairo_set_source_rgb(cr, 0.8,0,0);
        for (size_t i=0;i<W.profile.count;i++) {
            double x = axis_margin_left + ((W.profile.pts[i].t - tmin) / (tmax - tmin)) * plot_width;
            double y = (h - axis_margin_bottom) - ((W.profile.pts[i].temp - tempmin) / (tempmax - tempmin)) * plot_height;
            cairo_arc(cr, x, y, 6, 0, 2*M_PI);
            cairo_fill(cr);
        }
    }
    // ПОТОМ РИСУЕМ ДАННЫЕ РЕАЛЬНОГО ВРЕМЕНИ (СВЕРХУ)
    if (draw_realtime && realtime_data.count > 1) {
        // Линия температуры
        cairo_set_line_width(cr, 2.0);
        cairo_set_source_rgb(cr, 0,0,0.8); // Синий для реальной температуры
        // РИСУЕМ ВСЕ ТОЧКИ, НО МАСШТАБ САМ ОГРАНИЧИТ ВИДИМУЮ ОБЛАСТЬ
        gboolean move_to_next = TRUE;
        for (size_t i = 0; i < realtime_data.count; i++) {
            // В режиме Holding пропускаем точки, которые выходят за временные пределы
            if (is_holding_mode && realtime_data.points[i].time < tmin) {
                move_to_next = TRUE;
                continue;
            }
            double x = axis_margin_left + ((realtime_data.points[i].time - tmin) / (tmax - tmin)) * plot_width;
            double y = (h - axis_margin_bottom) - ((realtime_data.points[i].temp - tempmin) / (tempmax - tempmin)) * plot_height;
            // Если точка за пределами видимой области по X, пропускаем ее
            if (x < axis_margin_left || x > w - 10) {
                move_to_next = TRUE;
                continue;
            }
            if (move_to_next) {
                cairo_move_to(cr, x, y);
                move_to_next = FALSE;
            } else cairo_line_to(cr, x, y);
        }
        cairo_stroke(cr);
        // Линия заданной температуры
        cairo_set_source_rgb(cr, 0,0.6,0); // Зеленый для setpoint
        cairo_set_dash(cr, (double[]){5, 5}, 2, 0); // Пунктирная линия
        move_to_next = TRUE;
        gboolean found_non_zero_setpoint = FALSE; // ФЛАГ ДЛЯ ОТСЛЕЖИВАНИЯ ПЕРВОГО НЕНУЛЕВОГО SETPOINT
        for (size_t i = 0; i < realtime_data.count; i++) {
        // В режиме Holding пропускаем точки, которые выходят за временные пределы
            if (is_holding_mode && realtime_data.points[i].time < tmin)
            {
                move_to_next = TRUE;
                continue;
            }
            // ПРОПУСКАЕМ ВСЕ НАЧАЛЬНЫЕ ТОЧКИ С SETPOINT = 0 ДО ПЕРВОЙ НЕНУЛЕВОЙ
            if (!found_non_zero_setpoint)
            {
                if (realtime_data.points[i].setpoint == 0)
                {
                    move_to_next = TRUE;
                    continue;
                } else found_non_zero_setpoint = TRUE; //НАШЛИ ПЕРВЫЙ НЕНУЛЕВОЙ SETPOINT
            }
            double x = axis_margin_left + ((realtime_data.points[i].time - tmin) / (tmax - tmin)) * plot_width;
            double y = (h - axis_margin_bottom) - ((realtime_data.points[i].setpoint - tempmin) / (tempmax - tempmin)) * plot_height;
            // Если точка за пределами видимой области по X, пропускаем ее
            if (x < axis_margin_left || x > w - 10)
            {
                move_to_next = TRUE;
                continue;
            }
            if (move_to_next)
            {
                cairo_move_to(cr, x, y);
                move_to_next = FALSE;
            } else cairo_line_to(cr, x, y);
        }
        cairo_stroke(cr);
        cairo_set_dash(cr, NULL, 0, 0); // Сбрасываем пунктир
   }
   //ДОБАВЛЯЕМ ИНФОРМАЦИЮ О РЕЖИМЕ В ЛЕГЕНДУ
   draw_legend(cr, w, h, draw_profile, draw_realtime);
   //ЕСЛИ РЕЖИМ HOLDING - ДОБАВЛЯЕМ ПОДПИСЬ О "СКОЛЬЗЯЩЕМ ОКНЕ"
   if (is_holding_mode)
   {
       cairo_select_font_face(cr, "Sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
       cairo_set_font_size(cr, 12);
       cairo_set_source_rgb(cr, 0.3, 0.3, 0.3); // Темно-серый цвет
       char mode_text[64];
       snprintf(mode_text, sizeof(mode_text), "Holding Mode - Last 1000s");
       cairo_move_to(cr, w - 200, 30);
       cairo_show_text(cr, mode_text);
    }
    return FALSE;
}

static void draw_legend(cairo_t *cr, int w, int h, gboolean draw_profile, gboolean draw_realtime) {
    cairo_select_font_face(cr, "Sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
    cairo_set_font_size(cr, 10);

    int legend_x = w - 200; // Правый верхний угол
    int legend_y = h - 80;
    int line_height = 18;
    int symbol_size = 10;
    // Фон легенды
    cairo_set_source_rgba(cr, 1, 1, 1, 0.8); // Полупрозрачный белый
    cairo_rectangle(cr, legend_x - 10, legend_y - 5, 120, 60);
    cairo_fill(cr);
    cairo_set_source_rgb(cr, 0, 0, 0);
    cairo_move_to(cr, legend_x, legend_y);
    cairo_show_text(cr, "Legend:");
    int current_y = legend_y + line_height;
    // ПРОФИЛЬ (красный)
    if (draw_profile) {
        cairo_set_source_rgb(cr, 0.8, 0, 0);
        cairo_set_line_width(cr, 3);
        cairo_move_to(cr, legend_x, current_y);
        cairo_line_to(cr, legend_x + symbol_size, current_y);
        cairo_stroke(cr);

        cairo_set_source_rgb(cr, 0.8, 0, 0);
        cairo_arc(cr, legend_x + symbol_size/2, current_y, 4, 0, 2*M_PI);
        cairo_fill(cr);

        cairo_set_source_rgb(cr, 0, 0, 0);
        cairo_move_to(cr, legend_x + symbol_size + 10, current_y + 4);
        cairo_show_text(cr, "Profile");

        current_y += line_height;
    }
    //  РЕАЛЬНАЯ ТЕМПЕРАТУРА (синий)
    if (draw_realtime) {
        cairo_set_source_rgb(cr, 0, 0, 0.8);
        cairo_set_line_width(cr, 2);
        cairo_move_to(cr, legend_x, current_y);
        cairo_line_to(cr, legend_x + symbol_size, current_y);
        cairo_stroke(cr);
        cairo_set_source_rgb(cr, 0, 0, 0);
        cairo_move_to(cr, legend_x + symbol_size + 10, current_y + 4);
        cairo_show_text(cr, "Actual Temp");
        current_y += line_height;
    }
    //  ЗАДАННАЯ ТЕМПЕРАТУРА (зеленый пунктир)
    if (draw_realtime) {
        cairo_set_source_rgb(cr, 0, 0.6, 0);
        cairo_set_line_width(cr, 2);
        cairo_set_dash(cr, (double[]){5, 5}, 2, 0);
        cairo_move_to(cr, legend_x, current_y);
        cairo_line_to(cr, legend_x + symbol_size, current_y);
        cairo_stroke(cr);
        cairo_set_dash(cr, NULL, 0, 0);
        cairo_set_source_rgb(cr, 0, 0, 0);
        cairo_move_to(cr, legend_x + symbol_size + 10, current_y + 4);
        cairo_show_text(cr, "Setpoint");
        current_y += line_height;
    }
    //  ТОЧКИ ПРОФИЛЯ
    if (draw_profile) {
        cairo_set_source_rgb(cr, 0.8, 0, 0);
        cairo_arc(cr, legend_x + symbol_size/2, current_y, 6, 0, 2*M_PI);
        cairo_fill(cr);

        cairo_set_source_rgb(cr, 0, 0, 0);
        cairo_move_to(cr, legend_x + symbol_size + 10, current_y + 4);
        cairo_show_text(cr, "Profile Points");
    }
}
/* ---------- Log clear ---------- */
static void on_clear_log(GtkButton *b, gpointer user_data) {
    gtk_text_buffer_set_text(W.log_buf, "", -1);
    append_log("Log cleared");
}
static void draw_axes(cairo_t *cr, int w, int h, double tmin, double tmax, double tempmin, double tempmax) {
    // Настройки
    cairo_set_source_rgb(cr, 0,0,0);
    cairo_set_line_width(cr, 2.0);
    cairo_select_font_face(cr, "Sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
    cairo_set_font_size(cr, 12);

    int axis_margin_left = 50;   // Отступ для оси Y
    int axis_margin_bottom = 120; // Отступ для оси X
    int plot_width = w - axis_margin_left - 10;
    int plot_height = h - axis_margin_bottom - 10;
    // Рисуем оси
    cairo_move_to(cr, axis_margin_left, 10); // Y ось (верх)
    cairo_line_to(cr, axis_margin_left, h - axis_margin_bottom); // Y ось
    cairo_line_to(cr, w - 10, h - axis_margin_bottom); // X ось
    // Стрелки для осей
    cairo_move_to(cr, axis_margin_left, 10);
    cairo_line_to(cr, axis_margin_left - 5, 20);
    cairo_move_to(cr, axis_margin_left, 10);
    cairo_line_to(cr, axis_margin_left + 5, 20);

    cairo_move_to(cr, w - 10, h - axis_margin_bottom);
    cairo_line_to(cr, w - 20, h - axis_margin_bottom - 5);
    cairo_move_to(cr, w - 10, h - axis_margin_bottom);
    cairo_line_to(cr, w - 20, h - axis_margin_bottom + 5);

    cairo_stroke(cr);
    // Подписи осей
    cairo_move_to(cr, 15, h/2);
    cairo_save(cr);
    cairo_rotate(cr, -M_PI/2);
    cairo_show_text(cr, "Temperature (C)");
    cairo_restore(cr);

    cairo_move_to(cr, w/2 - 30, h - axis_margin_bottom + 40);
    cairo_show_text(cr, "Time (seconds)");
    //  АДАПТИВНЫЙ ШАГ ТЕМПЕРАТУРЫ В ЗАВИСИМОСТИ ОТ ДИАПАЗОНА
    double temp_range = tempmax - tempmin;
    double temp_step;

    if (temp_range > 500) temp_step = 100;
    else if (temp_range > 200) temp_step = 50;
    else if (temp_range > 100) temp_step = 20;
    else if (temp_range > 50) temp_step = 10;
    else if (temp_range > 20) temp_step = 5;
    else if (temp_range > 10) temp_step = 2;
    else if (temp_range > 5) temp_step = 1;
    else temp_step = 0.5;
    //  ИСПОЛЬЗУЕМ АДАПТИВНЫЙ ШАГ ДЛЯ РАСЧЕТА НАЧАЛА И КОНЦА
    double start_temp = ceil(tempmin / temp_step) * temp_step;
    double end_temp = floor(tempmax / temp_step) * temp_step;

    int temp_step_count = (end_temp - start_temp) / temp_step + 1;

    for (int i = 0; i < temp_step_count; i++) {
        double temp = start_temp + i * temp_step;
        if (temp > tempmax) break;

        double y = h - axis_margin_bottom - ((temp - tempmin) / (tempmax - tempmin)) * plot_height;
        // Деление
        cairo_move_to(cr, axis_margin_left - 5, y);
        cairo_line_to(cr, axis_margin_left + 5, y);
        // Подпись
        char label[20];
        if (temp_step >= 100) snprintf(label, sizeof(label), "%.0f", temp);
        else if (temp_step >= 10) snprintf(label, sizeof(label), "%.0f", temp);
        else if (temp_step >= 1) snprintf(label, sizeof(label), "%.0f", temp); // Для шага 5
        else snprintf(label, sizeof(label), "%.1f", temp); // Для дробных шагов

        cairo_move_to(cr, axis_margin_left - 30, y + 4);
        cairo_show_text(cr, label);
    }

    double time_step = 10.0; // Всегда шаг 10
    // Находим ближайшее кратное 10 для начала с учетом tmin
    double start_time = ceil(tmin / 10.0) * 10.0;
    double end_time = floor(tmax / 10.0) * 10.0;

    int time_step_count = (end_time - start_time) / time_step + 1;

    for (int i = 0; i < time_step_count; i++) {
        double time = start_time + i * time_step;
        if (time > tmax) break;

        double x = axis_margin_left + ((time - tmin) / (tmax - tmin)) * plot_width;
        // Деление
        cairo_move_to(cr, x, h - axis_margin_bottom - 5);
        cairo_line_to(cr, x, h - axis_margin_bottom + 5);
        // Подпись
        char label[20];
        snprintf(label, sizeof(label), "%.0f", time);
        cairo_move_to(cr, x - 10, h - axis_margin_bottom + 20);
        cairo_show_text(cr, label);
    }
    cairo_stroke(cr);
}
//Save / Load profile
static void on_save_profile(GtkButton *b, gpointer ud) {
    GtkWidget *dialog = gtk_file_chooser_dialog_new("Save profile", GTK_WINDOW(W.window),
        GTK_FILE_CHOOSER_ACTION_SAVE, "_Cancel", GTK_RESPONSE_CANCEL, "_Save", GTK_RESPONSE_ACCEPT, NULL);
    gtk_file_chooser_set_do_overwrite_confirmation(GTK_FILE_CHOOSER(dialog), TRUE);
    if (gtk_dialog_run(GTK_DIALOG(dialog)) == GTK_RESPONSE_ACCEPT) {
        char *fname = gtk_file_chooser_get_filename(GTK_FILE_CHOOSER(dialog));
        FILE *f = fopen(fname, "w");
        if (!f)
            append_log("Can't open %s for write", fname);
        else {
            fprintf(f, "{\n  \"points\": [\n");
            for (size_t i=0;i<W.profile.count;i++) {
                fprintf(f, "    { \"t\": %.3f, \"temp\": %.3f }%s\n",
                    W.profile.pts[i].t, W.profile.pts[i].temp,
                    (i+1==W.profile.count)?"":" ,");
            }
            fprintf(f, "  ]\n}\n");
            fclose(f);
            append_log("Profile saved to %s", fname);
        }
        g_free(fname);
    }
    gtk_widget_destroy(dialog);
}

static void on_load_profile(GtkButton *b, gpointer ud) {
    GtkWidget *dialog = gtk_file_chooser_dialog_new("Open profile", GTK_WINDOW(W.window),
        GTK_FILE_CHOOSER_ACTION_OPEN, "_Cancel", GTK_RESPONSE_CANCEL, "_Open", GTK_RESPONSE_ACCEPT, NULL);
    if (gtk_dialog_run(GTK_DIALOG(dialog)) == GTK_RESPONSE_ACCEPT) {
        char *fname = gtk_file_chooser_get_filename(GTK_FILE_CHOOSER(dialog));
        FILE *f = fopen(fname, "r");
        if (!f)
            append_log("Can't open %s for read", fname);
        else {
            profile_free(&W.profile);
            profile_init(&W.profile);
            char line[256];
            while (fgets(line, sizeof line, f)) {
                double t, temp;
                if (sscanf(line, " { \"t\": %lf, \"temp\": %lf", &t, &temp)==2) {
                    profile_add(&W.profile, t, temp);
                } else {
                    if (sscanf(line, "    { \"t\": %lf, \"temp\": %lf }", &t, &temp)==2)
                        profile_add(&W.profile, t, temp);
                }
            }
            fclose(f);
            append_log("Loaded profile %s with %zu points", fname, W.profile.count);
            gtk_widget_queue_draw(W.profile_canvas);
        }
        g_free(fname);
    }
    gtk_widget_destroy(dialog);
}

static gboolean update_profile_time_cb(gpointer data) {
    if (W.profile_active) {
        guint current_time = g_get_monotonic_time() / 1000; // миллисекунды
        guint elapsed = (current_time - W.profile_start_time) / 1000; // секунды
        char lab[64];
        snprintf(lab, sizeof lab, "Profile time: %d s", elapsed);
        gtk_label_set_text(GTK_LABEL(W.time_label), lab);
    }
    return G_SOURCE_CONTINUE;
}

static void process_received_data(const char *data) {
    static double last_temp = 0;
    static double last_setpoint = 0;
    if (data == NULL || strlen(data) == 0) {
        return;
    }
    if (strncmp(data, "TEMP:", 5) == 0)
    {
        char* temp_str = data + 5;
        double temp = 0.0;
        // Сохраняем и устанавливаем локаль C для корректного парсинга
        char *old_locale = setlocale(LC_NUMERIC, NULL);
        setlocale(LC_NUMERIC, "C");
        // Подготавливаем буфер и заменяем запятые на точки
        char temp_buffer[32];
        strncpy(temp_buffer, temp_str, sizeof(temp_buffer) - 1);
        temp_buffer[sizeof(temp_buffer) - 1] = '\0';
        for (char *p = temp_buffer; *p; p++)
        {
            if (*p == ',') *p = '.';
        }
        // Простое преобразование с atof
        temp = atof(temp_buffer);
        // Восстанавливаем локаль
        setlocale(LC_NUMERIC, old_locale);
        last_temp = temp;
        // Обновляем данные в реальном времени
        if (realtime_data.recording)
        {
            realtime_data_add(&realtime_data, temp, last_setpoint);
            gtk_widget_queue_draw(W.profile_canvas);
        }
        // Форматируем и отображаем температуру
        char lab[64];
        int integer_part = (int)temp;
        int fractional_part = (int)((temp - integer_part) * 1000);
        fractional_part = fractional_part < 0 ? -fractional_part : fractional_part;
        snprintf(lab, sizeof(lab), "Temp: %d.%03d C", integer_part, fractional_part);
        gtk_label_set_text(GTK_LABEL(W.temp_label), lab);
        // Логируем при необходимости
        if (W.log_serial_data)
        {
            append_log("RX: %s", data);
        }
}
    else if (strncmp(data, "SETPOINT:", 9) == 0) {
        char *old_locale = setlocale(LC_NUMERIC, NULL);
        setlocale(LC_NUMERIC, "C");
        double sp = atof(data + 9);
        setlocale(LC_NUMERIC, old_locale);
        last_setpoint = sp;
        char lab[64];
        snprintf(lab, sizeof lab, "Setpoint: %.3f C", sp);
        gtk_label_set_text(GTK_LABEL(W.setpoint_label), lab);
        if (W.log_serial_data) {
            append_log("RX: %s", data);
        }
    }
    else if (strncmp(data, "SETPOINT_TIME:", 14) == 0) {
        int setpoint_time = atoi(data + 14);
        char lab[64];
        snprintf(lab, sizeof lab, "Setpoint time: %d s", setpoint_time);
        gtk_label_set_text(GTK_LABEL(W.time_label), lab);
        if (W.log_serial_data) {
            append_log("RX: %s", data);
        }
    }
    else if (strncmp(data, "PROFILE_TIME:", 13) == 0) {
        int profile_time = atoi(data + 13);
        char lab[64];
        snprintf(lab, sizeof lab, "Profile time: %d s", profile_time);
        gtk_label_set_text(GTK_LABEL(W.time_label), lab);
        if (W.log_serial_data) {
            append_log("RX: %s", data);
        }
    }
    else if (strncmp(data, "STATE:", 6) == 0) {
        gtk_label_set_text(GTK_LABEL(W.state_label), data + 6);
        if (W.log_serial_data) {
            append_log("RX: %s", data);
        }
    }
    else if (strncmp(data, "PID:", 4) == 0) {
    float kp, ki, kd, integral, prev_error, output_min, output_max;
    int freq, window;
    if (sscanf(data + 4, "%d,%d,%f,%f,%f,%f,%f,%f,%f", &freq, &window, &kp, &ki, &kd, &integral, &prev_error, &output_min, &output_max) == 9) {
        char tmp[64];
        // Обновляем только основные 5 полей ввода
        snprintf(tmp, sizeof tmp, "%d", freq);
        gtk_entry_set_text(GTK_ENTRY(W.pid_freq_entry), tmp);
        snprintf(tmp, sizeof tmp, "%d", window);
        gtk_entry_set_text(GTK_ENTRY(W.window_time_entry), tmp);
        snprintf(tmp, sizeof tmp, "%.3f", kp);
        gtk_entry_set_text(GTK_ENTRY(W.kp_entry), tmp);
        snprintf(tmp, sizeof tmp, "%.3f", ki);
        gtk_entry_set_text(GTK_ENTRY(W.ki_entry), tmp);
        snprintf(tmp, sizeof tmp, "%.3f", kd);
        gtk_entry_set_text(GTK_ENTRY(W.kd_entry), tmp);
        //  ВЫВОДИМ ВСЕ ПАРАМЕТРЫ В ЛОГ
        append_log("PID parameters: Freq=%dHz, Window=%dms", freq, window);
        append_log("  Kp=%.3f, Ki=%.3f, Kd=%.3f", kp, ki, kd);
        append_log("  Integral=%.3f, PrevError=%.3f", integral, prev_error);
        append_log("  OutputMin=%.3f, OutputMax=%.3f", output_min, output_max);
    }
        if (W.log_serial_data) {
            append_log("RX: %s", data);
        }
    }
    else if (strncmp(data, "POINT:", 6) == 0) {
        int index;
        double time_sec, temperature;
        char *ptr = (char *)data + 6; // указатель на начало данных после "POINT:"
        // Парсим индекс
        index = atoi(ptr);
        // Ищем первую запятую
        ptr = strchr(ptr, ',');
        if (!ptr) {
            append_log("ERROR: No comma after index in: %s", data);
            return;
        }
        ptr++; // переходим к времени
        // Парсим время
        time_sec = atof(ptr);
        // Ищем вторую запятую
        ptr = strchr(ptr, ',');
        if (!ptr) {
            append_log("ERROR: No comma after time in: %s", data);
            return;
        }
        ptr++; // переходим к температуре
        // Парсим температуру
        temperature = atof(ptr);
        append_log("DEBUG: Parsed point %d: time=%.2f, temp=%.2f", index, time_sec, temperature);
        if (index == 0) {
            profile_free(&W.profile);
            profile_init(&W.profile);
            append_log("Started receiving profile from MCU");
            // ОЧИЩАЕМ ТАБЛИЦУ ПРИ НАЧАЛЕ ПОЛУЧЕНИЯ НОВОГО ПРОФИЛЯ
            for (int i = 0; i < 5; i++) {
                gtk_entry_set_text(GTK_ENTRY(W.time_entries[i]), "");
                gtk_entry_set_text(GTK_ENTRY(W.temp_entries[i]), "");
            }
        }
        profile_add(&W.profile, time_sec, temperature);
        // ЗАПОЛНЯЕМ ТАБЛИЦУ
        if (index < 5) {
            char time_str[16], temp_str[16];
            snprintf(time_str, sizeof(time_str), "%.1f", time_sec);
            snprintf(temp_str, sizeof(temp_str), "%.1f", temperature);

            gtk_entry_set_text(GTK_ENTRY(W.time_entries[index]), time_str);
            gtk_entry_set_text(GTK_ENTRY(W.temp_entries[index]), temp_str);
            append_log("DEBUG: Filled table row %d with time=%s, temp=%s", index, time_str, temp_str);
        }
        gtk_widget_queue_draw(W.profile_canvas);
        if (W.log_serial_data) {
            append_log("RX: %s", data);
        }
    }
    // Логируем другие команды которые всегда должны быть в логе
    else if (strncmp(data, "PROFILE_RECEIVED", 16) == 0) append_log("Profile received by MCU");
    else if (strncmp(data, "PROFILE_SENT", 12) == 0) {
        append_log("Profile sent by MCU");
        fill_table_from_profile();
    }
    else if (strncmp(data, "PROFILE_FLASH_SAVED", 19) == 0) append_log("Profile saved to MCU flash");
    else if (strncmp(data, "PID_SET", 7) == 0) append_log("PID coefficients set on MCU");
    else if (strncmp(data, "PID_SAVED", 9) == 0) append_log("PID coefficients saved to MCU flash");
    else if (strncmp(data, "SETPOINT_SET:", 13) == 0) append_log("Setpoint set on MCU: %s", data + 13);
    // Для неизвестных команд всегда логируем
    else if (strlen(data) > 1) append_log("RX: %s", data);

}

// Чтение COM порта
static gboolean poll_serial_cb(gpointer data) {
    if (serial_handle == INVALID_HANDLE_VALUE) return G_SOURCE_CONTINUE;
    char buf[512];
    DWORD bytes_available = 0;
    COMSTAT comstat;
    if (!ClearCommError(serial_handle, NULL, &comstat)) return G_SOURCE_CONTINUE;
    bytes_available = comstat.cbInQue;
    if (bytes_available == 0) return G_SOURCE_CONTINUE;
    DWORD bytes_to_read = (bytes_available < sizeof(buf)-1) ? bytes_available : sizeof(buf)-1;
    DWORD bytes_read = 0;
    if (ReadFile(serial_handle, buf, bytes_to_read, &bytes_read, NULL) && bytes_read > 0) {
        buf[bytes_read] = 0;
        //  УЛУЧШЕННАЯ ОБРАБОТКА: разбиваем по строкам и обрабатываем каждую отдельно
        char *s = buf;
        char *line_start = s;
        while (*s) {
            if (*s == '\n' || *s == '\r') {
                // Нашли конец строки
                if (s > line_start) {
                    // Временно заменяем символ конца строки на нуль-терминатор
                    char temp = *s;
                    *s = '\0';
                    // Обрабатываем строку
                    if (strlen(line_start) > 0) process_received_data(line_start);
                    // Восстанавливаем символ
                    *s = temp;
                }
                // Пропускаем все символы конца строки
                while (*s == '\n' || *s == '\r') {
                    s++;
                }
                line_start = s;
            } else {
                s++;
            }
        }
        // Обрабатываем остаток (если нет завершающего \n)
        if (s > line_start && strlen(line_start) > 0) {
            process_received_data(line_start);
        }
    }
    return G_SOURCE_CONTINUE;
}
// Функция для ПЕРИОДИЧЕСКОГО запроса телеметрии
static gboolean periodic_telemetry_request(gpointer user_data) {
    //  ПРОВЕРЯЕМ, ЧТО СОЕДИНЕНИЕ ВСЕ ЕЩЕ АКТИВНО
    if (serial_handle == INVALID_HANDLE_VALUE || W.log_serial_data) {
        W.telemetry_timeout_id = 0;  // Сбрасываем ID при выходе
        return G_SOURCE_REMOVE;
    }
    send_command_quiet("GET_TELEMETRY");
    return G_SOURCE_CONTINUE;
}
// Функция для остановки автоматических запросов
static void stop_periodic_telemetry(void) {
    if (W.telemetry_timeout_id != 0) {
        g_source_remove(W.telemetry_timeout_id);
        W.telemetry_timeout_id = 0;
    }
}
// Функция для запуска автоматических запросов
static void start_periodic_telemetry(void) {
    if (serial_handle != INVALID_HANDLE_VALUE && W.telemetry_timeout_id == 0) {
        W.telemetry_timeout_id = g_timeout_add(800, periodic_telemetry_request, NULL);
    }
}
//Обработка кнопки соединения
static void on_connect_clicked(GtkButton *b, gpointer ud) {
    const char *port = gtk_entry_get_text(GTK_ENTRY(W.port_entry));

    if (serial_handle != INVALID_HANDLE_VALUE) {
        /* disconnect */
        stop_periodic_telemetry();
        serial_close_h();
        gtk_button_set_label(GTK_BUTTON(W.connect_btn), "Connect");
        append_log("Disconnected");
        return;
    }
    if (serial_open(port, 115200) == 0) {
        gtk_button_set_label(GTK_BUTTON(W.connect_btn), "Disconnect");
        append_log("Connected to %s", port);
        // Инициализируем флаг логирования
        W.log_serial_data = FALSE;
        g_timeout_add(GET_STATUS_POLL_MS, poll_serial_cb, NULL);
        // Включаем логирование для начального запроса
        W.log_serial_data = TRUE;
        send_command("GET_STATUS");
        start_periodic_telemetry();
        // Выключаем логирование после начального запроса
        g_timeout_add(1000, (GSourceFunc)disable_serial_logging, NULL);
    } else {
        DWORD error = GetLastError();
        append_log("Failed to open %s: error code %lu", port, error);
    }
}
static void send_profile_to_mcu(GtkButton *b, gpointer user_data) {
    if (serial_handle == INVALID_HANDLE_VALUE) {
        append_log("Not connected");
        return;
    }
    char cmd[512];
    snprintf(cmd, sizeof(cmd), "SET_PROFILE:");
    // Формируем строку профиля
    for (size_t i = 0; i < W.profile.count; i++) {
        char point[32];
        // используем %.0f для целых чисел без десятичной части
        if (W.profile.pts[i].t == (int)W.profile.pts[i].t &&
            W.profile.pts[i].temp == (int)W.profile.pts[i].temp) {
            // Оба числа целые - выводим без десятичной части
            snprintf(point, sizeof(point), "%.0f,%.0f;", W.profile.pts[i].t, W.profile.pts[i].temp);
        } else if (W.profile.pts[i].t == (int)W.profile.pts[i].t) {
            // Только время целое
            snprintf(point, sizeof(point), "%.0f,%.1f;", W.profile.pts[i].t, W.profile.pts[i].temp);
        } else if (W.profile.pts[i].temp == (int)W.profile.pts[i].temp) {
            // Только температура целая
            snprintf(point, sizeof(point), "%.1f,%.0f;", W.profile.pts[i].t, W.profile.pts[i].temp);
        } else {
            // Оба числа дробные
            snprintf(point, sizeof(point), "%.1f,%.1f;", W.profile.pts[i].t, W.profile.pts[i].temp);
        }
        if (strlen(cmd) + strlen(point) < sizeof(cmd) - 10) {
            strcat(cmd, point);
        } else {
            append_log("Profile too large to send");
            break;
        }
    }
    // Убираем последнюю точку с запятой
    if (strlen(cmd) > 12) cmd[strlen(cmd) - 1] = '\0';
    //  ОСТАНАВЛИВАЕМ АВТОМАТИЧЕСКИЕ ЗАПРОСЫ
    stop_periodic_telemetry();
    W.log_serial_data = TRUE;
    send_command(cmd);
    append_log("Profile sent to MCU (%zu points)", W.profile.count);
    g_timeout_add(1000, (GSourceFunc)disable_serial_logging, NULL);
    //  ЗАПУСКАЕМ АВТОМАТИЧЕСКИЕ ЗАПРОСЫ ЧЕРЕЗ 1 СЕКУНДУ
    g_timeout_add(1000, (GSourceFunc)start_periodic_telemetry, NULL);
}
// Функция для сохранения профиля во Flash МК
static void save_profile_to_flash(GtkButton *b, gpointer user_data) {
    stop_periodic_telemetry();
    if (serial_handle == INVALID_HANDLE_VALUE) {
        append_log("Not connected");
        return;
    }
    send_command("SAVE_PROFILE");
    append_log("Saving profile to MCU flash...");
    g_timeout_add(1000, (GSourceFunc)disable_serial_logging, NULL);
    g_timeout_add(1000, (GSourceFunc)start_periodic_telemetry, NULL);
}
// Функция для выключения логирования
static gboolean disable_serial_logging(gpointer data) {
    W.log_serial_data = FALSE;
    return G_SOURCE_REMOVE;
}
static void on_get_status(GtkButton *b, gpointer user_data) {
    if (serial_handle == INVALID_HANDLE_VALUE) {
        append_log("Not connected");
        return;
    }
     stop_periodic_telemetry();
    // Включаем логирование для этого запроса
    W.log_serial_data = TRUE;
    send_command("GET_STATUS");
    append_log("Getting status...");
    // Выключаем логирование через короткое время (после получения ответа)
    g_timeout_add(1000, (GSourceFunc)disable_serial_logging, NULL);
    g_timeout_add(1000, (GSourceFunc)start_periodic_telemetry, NULL);
}
// Функция загрузки профиля из таблицы с кумулятивным временем
static void on_load_from_table(GtkButton *b, gpointer user_data) {
    profile_free(&W.profile);
    profile_init(&W.profile);
    int valid_points = 0;
    float cumulative_time = 0.0f;
    for (int i = 0; i < 5; i++) {
        const char *time_text = gtk_entry_get_text(GTK_ENTRY(W.time_entries[i]));
        const char *temp_text = gtk_entry_get_text(GTK_ENTRY(W.temp_entries[i]));
        // Пропускаем пустые строки
        if (strlen(time_text) == 0 || strlen(temp_text) == 0) continue;
        float time_abs = atof(time_text);  // Время сегмента
        float temp = atof(temp_text);
        if (time_abs >= 0 && temp >= 0) {  // Проверяем валидность
            profile_add(&W.profile, time_abs, temp);
            valid_points++;
            append_log("Point %d: %.1fs, %.1f°C", i+1, time_abs, temp);
        }
    }
    if (valid_points > 0) {
        gtk_widget_queue_draw(W.profile_canvas);
        append_log("Profile loaded from table: %d points, total time: %.1fs",
                  valid_points, cumulative_time);
    } else  append_log("No valid points in table");
}
// Функция для заполнения таблицы из профиля
static void fill_table_from_profile(void) {
    // Очищаем таблицу
    for (int i = 0; i < 5; i++) {
        gtk_entry_set_text(GTK_ENTRY(W.time_entries[i]), "");
        gtk_entry_set_text(GTK_ENTRY(W.temp_entries[i]), "");
    }
    // Заполняем таблицу данными из профиля
    for (size_t i = 0; i < W.profile.count && i < 5; i++) {
        char time_str[16], temp_str[16];
        // Форматируем числа: если целые - без десятичной части, иначе с одной цифрой
        if (W.profile.pts[i].t == (int)W.profile.pts[i].t) {
            snprintf(time_str, sizeof(time_str), "%.0f", W.profile.pts[i].t);
        } else {
            snprintf(time_str, sizeof(time_str), "%.1f", W.profile.pts[i].t);
        }
        if (W.profile.pts[i].temp == (int)W.profile.pts[i].temp) {
            snprintf(temp_str, sizeof(temp_str), "%.0f", W.profile.pts[i].temp);
        } else {
            snprintf(temp_str, sizeof(temp_str), "%.1f", W.profile.pts[i].temp);
        }
        gtk_entry_set_text(GTK_ENTRY(W.time_entries[i]), time_str);
        gtk_entry_set_text(GTK_ENTRY(W.temp_entries[i]), temp_str);
    }
    append_log("Table filled from profile (%zu points)", W.profile.count);
}
static void on_get_profile(GtkButton *b, gpointer user_data) {
    stop_periodic_telemetry();
    W.log_serial_data = TRUE;
    send_command("GET_PROFILE");
    append_log("Getting profile from MCU...");
    g_timeout_add(1000, (GSourceFunc)disable_serial_logging, NULL);
    g_timeout_add(1000, (GSourceFunc)start_periodic_telemetry, NULL);
}
// Функция очистки таблицы
static void on_clear_table(GtkButton *b, gpointer user_data) {
    for (int i = 0; i < 5; i++) {
        gtk_entry_set_text(GTK_ENTRY(W.time_entries[i]), "");
        gtk_entry_set_text(GTK_ENTRY(W.temp_entries[i]), "");
    }
    // ОЧИЩАЕМ ПРОФИЛЬ ДАННЫХ
    W.profile.count = 0;
    realtime_data_clear(&realtime_data);
    // ПЕРЕРИСОВЫВАЕМ ГРАФИК
    gtk_widget_queue_draw(W.profile_canvas);
    append_log("Table and graph cleared");
}
static void realtime_data_init(RealtimeData *rd) {
    rd->points = NULL;
    rd->count = 0;
    rd->capacity = 0;
    rd->start_time = 0;
    rd->recording = FALSE;
}
static void realtime_data_free(RealtimeData *rd) {
    free(rd->points);
    rd->points = NULL;
    rd->count = rd->capacity = 0;
}
static void realtime_data_add(RealtimeData *rd, double temp, double setpoint) {
    if (!rd->recording) return;
    double current_time = (g_get_monotonic_time() / 1000 - rd->start_time) / 1000.0; // секунды
    if (rd->count + 1 > rd->capacity) {
        rd->capacity = rd->capacity ? rd->capacity * 2 : 1000; // начальная емкость 1000 точек
        rd->points = realloc(rd->points, rd->capacity * sizeof(RealtimePoint));
    }
    rd->points[rd->count].time = current_time;
    rd->points[rd->count].temp = temp;
    rd->points[rd->count].setpoint = setpoint;
    rd->count++;
}
static void realtime_data_start(RealtimeData *rd) {
    realtime_data_free(rd);
    rd->start_time = g_get_monotonic_time() / 1000;
    rd->recording = TRUE;
}
static void realtime_data_stop(RealtimeData *rd) {
    rd->recording = FALSE;
}
static void realtime_data_clear(RealtimeData *rd) {
    realtime_data_free(rd);
    realtime_data_init(rd);
}
//Рисуем
static void build_ui(void) {
    W.window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_default_size(GTK_WINDOW(W.window), 1000, 600);
    gtk_window_set_title(GTK_WINDOW(W.window), "Reflow Controller - PC");

    g_signal_connect(W.window, "destroy", G_CALLBACK(gtk_main_quit), NULL);

    GtkWidget *main_box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 6);
    gtk_container_add(GTK_CONTAINER(W.window), main_box);
    // left column:
    GtkWidget *left = gtk_box_new(GTK_ORIENTATION_VERTICAL, 6);
    gtk_box_pack_start(GTK_BOX(main_box), left, FALSE, FALSE, 6);
    gtk_widget_set_size_request(left, 300, -1); //  ФИКСИРОВАННАЯ УЗКАЯ ШИРИНА
    // КОМПАКТНЫЙ БЛОК */
    GtkWidget *h = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 4);
    W.port_entry = gtk_entry_new();
    gtk_entry_set_text(GTK_ENTRY(W.port_entry), "COM6");
    gtk_entry_set_width_chars(GTK_ENTRY(W.port_entry), 10);
    W.connect_btn = gtk_button_new_with_label("Connect");
    GtkWidget *get_status_btn = gtk_button_new_with_label("Status");

    g_signal_connect(W.connect_btn, "clicked", G_CALLBACK(on_connect_clicked), NULL);
    g_signal_connect(get_status_btn, "clicked", G_CALLBACK(on_get_status), NULL);

    gtk_box_pack_start(GTK_BOX(h), W.port_entry, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(h), W.connect_btn, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(h), get_status_btn, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(left), h, FALSE, FALSE, 0);
    // PID controls
    GtkWidget *frame_pid = gtk_frame_new("PID");
    gtk_box_pack_start(GTK_BOX(left), frame_pid, FALSE, FALSE, 0);
    // СОЗДАЕМ ВЕРТИКАЛЬНЫЙ КОНТЕЙНЕР ДЛЯ ВСЕЙ РАМКИ PID
    GtkWidget *pid_vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 4);
    gtk_container_add(GTK_CONTAINER(frame_pid), pid_vbox);
    //ВЫРАВНИВАЕМ ПО ЦЕНТРУ ПО ГОРИЗОНТАЛИ
    gtk_widget_set_halign(pid_vbox, GTK_ALIGN_CENTER);

    GtkWidget *pid_grid = gtk_grid_new();
    gtk_box_pack_start(GTK_BOX(pid_vbox), pid_grid, FALSE, FALSE, 0);
    gtk_grid_set_row_spacing(GTK_GRID(pid_grid), 2);
    gtk_grid_set_column_spacing(GTK_GRID(pid_grid), 2);

    gtk_grid_attach(GTK_GRID(pid_grid), gtk_label_new("Kp:"), 0,0,1,1);
    W.kp_entry = gtk_entry_new();
    gtk_entry_set_width_chars(GTK_ENTRY(W.kp_entry), 6);
    gtk_grid_attach(GTK_GRID(pid_grid), W.kp_entry, 1,0,1,1);

    gtk_grid_attach(GTK_GRID(pid_grid), gtk_label_new("Ki:"), 0,1,1,1);
    W.ki_entry = gtk_entry_new();
    gtk_entry_set_width_chars(GTK_ENTRY(W.ki_entry), 6);
    gtk_grid_attach(GTK_GRID(pid_grid), W.ki_entry, 1,1,1,1);

    gtk_grid_attach(GTK_GRID(pid_grid), gtk_label_new("Kd:"), 0,2,1,1);
    W.kd_entry = gtk_entry_new();
    gtk_entry_set_width_chars(GTK_ENTRY(W.kd_entry), 6);
    gtk_grid_attach(GTK_GRID(pid_grid), W.kd_entry, 1,2,1,1);
    // ДОБАВЛЯЕМ ПОЛЯ ДЛЯ ЧАСТОТЫ И ВРЕМЕНИ ОКНА ВО 2-ю КОЛОНКУ
    gtk_grid_attach(GTK_GRID(pid_grid), gtk_label_new("Freq:"), 2,0,1,1);
    W.pid_freq_entry = gtk_entry_new();
    gtk_entry_set_width_chars(GTK_ENTRY(W.pid_freq_entry), 4);
    gtk_entry_set_text(GTK_ENTRY(W.pid_freq_entry), "30");
    gtk_grid_attach(GTK_GRID(pid_grid), W.pid_freq_entry, 3,0,1,1);
    gtk_grid_attach(GTK_GRID(pid_grid), gtk_label_new("Hz"), 4,0,1,1);

    gtk_grid_attach(GTK_GRID(pid_grid), gtk_label_new("Window:"), 2,1,1,1);
    W.window_time_entry = gtk_entry_new();
    gtk_entry_set_width_chars(GTK_ENTRY(W.window_time_entry), 4);
    gtk_entry_set_text(GTK_ENTRY(W.window_time_entry), "5000");
    gtk_grid_attach(GTK_GRID(pid_grid), W.window_time_entry, 3,1,1,1);
    gtk_grid_attach(GTK_GRID(pid_grid), gtk_label_new("ms"), 4,1,1,1);
    // ОТДЕЛЬНЫЙ БЛОК ДЛЯ КНОПОК (НЕ ЗАВИСИТ ОТ СЕТКИ)
    GtkWidget *pid_buttons_box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 6);
    gtk_box_pack_start(GTK_BOX(pid_vbox), pid_buttons_box, FALSE, FALSE, 0);
    // Кнопки занимают всю ширину
    W.send_pid_btn = gtk_button_new_with_label("Set");
    g_signal_connect(W.send_pid_btn, "clicked", G_CALLBACK(on_send_pid), NULL);
    W.save_pid_btn = gtk_button_new_with_label("Save to MCU flash");
    g_signal_connect(W.save_pid_btn, "clicked", G_CALLBACK(on_save_pid), NULL);
    W.get_pid_btn = gtk_button_new_with_label("Get");
    g_signal_connect(W.get_pid_btn, "clicked", G_CALLBACK(on_get_pid), NULL);
    //  УПАКОВЫВАЕМ КНОПКИ В ОТДЕЛЬНЫЙ КОНТЕЙНЕР
    gtk_box_pack_start(GTK_BOX(pid_buttons_box), W.send_pid_btn, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(pid_buttons_box), W.save_pid_btn, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(pid_buttons_box), W.get_pid_btn, TRUE, TRUE, 0);
    //НОВЫЙ БЛОК: Holding Temp Point
    GtkWidget *frame_setpoint = gtk_frame_new("Holding Temp Point");
    gtk_box_pack_start(GTK_BOX(left), frame_setpoint, FALSE, FALSE, 0);
    GtkWidget *setpoint_v = gtk_box_new(GTK_ORIENTATION_VERTICAL, 4);
    gtk_container_add(GTK_CONTAINER(frame_setpoint), setpoint_v);
    // Поле для установки температуры
    GtkWidget *setpoint_h = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 4);
    gtk_box_pack_start(GTK_BOX(setpoint_v), setpoint_h, FALSE, FALSE, 0);

    gtk_box_pack_start(GTK_BOX(setpoint_h), gtk_label_new("Set Point:"), FALSE, FALSE, 0);
    W.setpoint_entry = gtk_entry_new();
    gtk_entry_set_text(GTK_ENTRY(W.setpoint_entry), "30.0");
    gtk_entry_set_width_chars(GTK_ENTRY(W.setpoint_entry), 6);
    gtk_widget_set_size_request(W.setpoint_entry, 50, -1); // Ширина 80px, высота по умолчанию
    gtk_box_pack_start(GTK_BOX(setpoint_h), W.setpoint_entry, FALSE, FALSE, 0);

    W.set_setpoint_btn = gtk_button_new_with_label("Set");
    g_signal_connect(W.set_setpoint_btn, "clicked", G_CALLBACK(on_set_setpoint), NULL);
    gtk_box_pack_start(GTK_BOX(setpoint_h), W.set_setpoint_btn, FALSE, FALSE, 0);
    // Кнопки Start/Stop для режима Holding Temp Point
    GtkWidget *setpoint_control_box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 2);
    gtk_box_pack_start(GTK_BOX(setpoint_v), setpoint_control_box, FALSE, FALSE, 0);

    W.start_setpoint_btn = gtk_button_new_with_label("Start Hold");
    W.stop_setpoint_btn = gtk_button_new_with_label("Stop Hold");
    g_signal_connect(W.start_setpoint_btn, "clicked", G_CALLBACK(on_start_setpoint), NULL);
    g_signal_connect(W.stop_setpoint_btn, "clicked", G_CALLBACK(on_stop_setpoint), NULL);

    gtk_box_pack_start(GTK_BOX(setpoint_control_box), W.start_setpoint_btn, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(setpoint_control_box), W.stop_setpoint_btn, TRUE, TRUE, 0);
    // Profile section
    GtkWidget *frame_profile = gtk_frame_new("Profile");
    gtk_box_pack_start(GTK_BOX(left), frame_profile, FALSE, FALSE, 0);
    GtkWidget *profile_v = gtk_box_new(GTK_ORIENTATION_VERTICAL, 4);
    gtk_container_add(GTK_CONTAINER(frame_profile), profile_v);
    // Control buttons
    GtkWidget *control_box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 2);
    gtk_box_pack_start(GTK_BOX(profile_v), control_box, FALSE, FALSE, 0);

    W.start_btn = gtk_button_new_with_label("Start");
    W.stop_btn = gtk_button_new_with_label("Stop");
    g_signal_connect(W.start_btn, "clicked", G_CALLBACK(on_start), NULL);
    g_signal_connect(W.stop_btn, "clicked", G_CALLBACK(on_stop), NULL);

    gtk_box_pack_start(GTK_BOX(control_box), W.start_btn, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(control_box), W.stop_btn, TRUE, TRUE, 0);

    GtkWidget *saveloadget_buttons_box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 2);
    gtk_box_pack_start(GTK_BOX(profile_v), saveloadget_buttons_box, FALSE, FALSE, 0);

    GtkWidget *btn_save_prof = gtk_button_new_with_label("Save...");
    GtkWidget *btn_load_prof = gtk_button_new_with_label("Load...");
    GtkWidget *btn_get_profile = gtk_button_new_with_label("Get");

    g_signal_connect(btn_save_prof, "clicked", G_CALLBACK(on_save_profile), NULL);
    g_signal_connect(btn_load_prof, "clicked", G_CALLBACK(on_load_profile), NULL);
    g_signal_connect(btn_get_profile, "clicked", G_CALLBACK(on_get_profile), NULL);

    gtk_box_pack_start(GTK_BOX(saveloadget_buttons_box), btn_save_prof, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(saveloadget_buttons_box), btn_load_prof, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(saveloadget_buttons_box), btn_get_profile, TRUE, TRUE, 0);
    // Profile points table
    GtkWidget *table_frame = gtk_frame_new("Points");
    gtk_box_pack_start(GTK_BOX(profile_v), table_frame, TRUE, TRUE, 0);
    GtkWidget *table_v = gtk_box_new(GTK_ORIENTATION_VERTICAL, 2);
    gtk_container_add(GTK_CONTAINER(table_frame), table_v);
    // Создаем grid для таблицы
    GtkWidget *table_grid = gtk_grid_new();
    gtk_widget_set_halign(table_grid, GTK_ALIGN_CENTER);
    gtk_grid_set_row_spacing(GTK_GRID(table_grid), 2);
    gtk_grid_set_column_spacing(GTK_GRID(table_grid), 2);
    gtk_box_pack_start(GTK_BOX(table_v), table_grid, FALSE, FALSE, 0);
    // Заголовки таблицы
    gtk_grid_attach(GTK_GRID(table_grid), gtk_label_new(""), 0, 0, 1, 1);
    gtk_grid_attach(GTK_GRID(table_grid), gtk_label_new("Time (s)"), 1, 0, 1, 1);
    gtk_grid_attach(GTK_GRID(table_grid), gtk_label_new("Temp (C)"), 2, 0, 1, 1);
    // Создаем поля для 5 точек профиля
for (int i = 0; i < 5; i++) {
    char label[10];
    snprintf(label, sizeof(label), "%d", i+1);
    gtk_grid_attach(GTK_GRID(table_grid), gtk_label_new(label), 0, i+1, 1, 1);

    W.time_entries[i] = gtk_entry_new();
    gtk_entry_set_max_length(GTK_ENTRY(W.time_entries[i]), 8);
    gtk_entry_set_width_chars(GTK_ENTRY(W.time_entries[i]), 8);
    gtk_widget_set_size_request(W.time_entries[i], 115, 30);
    gtk_grid_attach(GTK_GRID(table_grid), W.time_entries[i], 1, i+1, 1, 1);

    W.temp_entries[i] = gtk_entry_new();
    gtk_entry_set_max_length(GTK_ENTRY(W.temp_entries[i]), 8);
    gtk_entry_set_width_chars(GTK_ENTRY(W.temp_entries[i]), 8);
    gtk_widget_set_size_request(W.temp_entries[i], 115, 30);
    gtk_grid_attach(GTK_GRID(table_grid), W.temp_entries[i], 2, i+1, 1, 1);;
}
    // Кнопки управления таблицей
    GtkWidget *table_buttons_box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 2);
    gtk_box_pack_start(GTK_BOX(table_v), table_buttons_box, FALSE, FALSE, 0);

    GtkWidget *btn_load_table = gtk_button_new_with_label("Draw");
    GtkWidget *btn_clear_table = gtk_button_new_with_label("Clear");

    g_signal_connect(btn_load_table, "clicked", G_CALLBACK(on_load_from_table), NULL);
    g_signal_connect(btn_clear_table, "clicked", G_CALLBACK(on_clear_table), NULL);

    gtk_box_pack_start(GTK_BOX(table_buttons_box), btn_load_table, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(table_buttons_box), btn_clear_table, TRUE, TRUE, 0);

    GtkWidget *profile_move = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 2);
    gtk_box_pack_end(GTK_BOX(profile_v), profile_move, FALSE, FALSE, 0);

    GtkWidget *btn_send_profile = gtk_button_new_with_label("Set");
    GtkWidget *btn_save_flash = gtk_button_new_with_label("Save to MCU flash");

    g_signal_connect(btn_send_profile, "clicked", G_CALLBACK(send_profile_to_mcu), NULL);
    g_signal_connect(btn_save_flash, "clicked", G_CALLBACK(save_profile_to_flash), NULL);

    gtk_box_pack_start(GTK_BOX(profile_move), btn_send_profile, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(profile_move), btn_save_flash, TRUE, TRUE, 0);
    //telemetry labels
    GtkWidget *frame_tel = gtk_frame_new("Telemetry");
    gtk_box_pack_start(GTK_BOX(left), frame_tel, FALSE, FALSE, 0);
    GtkWidget *tel_v = gtk_box_new(GTK_ORIENTATION_VERTICAL, 2);
    gtk_container_add(GTK_CONTAINER(frame_tel), tel_v);
    W.temp_label = gtk_label_new("Temp: --");
    W.setpoint_label = gtk_label_new("Setpoint: --");
    W.time_label = gtk_label_new("Time: --");
    W.state_label = gtk_label_new("State: --");
    gtk_box_pack_start(GTK_BOX(tel_v), W.temp_label, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(tel_v), W.setpoint_label, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(tel_v), W.time_label, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(tel_v), W.state_label, FALSE, FALSE, 0);
    // right: canvas + log
    GtkWidget *right = gtk_box_new(GTK_ORIENTATION_VERTICAL, 6);
    gtk_box_pack_start(GTK_BOX(main_box), right, TRUE, TRUE, 6);

    W.profile_canvas = gtk_drawing_area_new();
    gtk_widget_set_size_request(W.profile_canvas, 800, 600);
    g_signal_connect(W.profile_canvas, "draw", G_CALLBACK(profile_draw), NULL);
    gtk_box_pack_start(GTK_BOX(right), W.profile_canvas, TRUE, TRUE, 0);
    // log
    GtkWidget *log_frame = gtk_frame_new("Log");
    gtk_box_pack_start(GTK_BOX(right), log_frame, TRUE, TRUE, 0);

    GtkWidget *log_vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 4);
    gtk_container_add(GTK_CONTAINER(log_frame), log_vbox);

    GtkWidget *log_header = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 4);
    gtk_box_pack_start(GTK_BOX(log_vbox), log_header, FALSE, FALSE, 0);

    GtkWidget *log_label = gtk_label_new("Log (newest on top)");
    GtkWidget *clear_log_btn = gtk_button_new_with_label("Clear");
    g_signal_connect(clear_log_btn, "clicked", G_CALLBACK(on_clear_log), NULL);

    gtk_box_pack_start(GTK_BOX(log_header), log_label, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(log_header), clear_log_btn, FALSE, FALSE, 0);

    W.log_textview = gtk_text_view_new();
    gtk_text_view_set_editable(GTK_TEXT_VIEW(W.log_textview), FALSE);
    W.log_buf = gtk_text_view_get_buffer(GTK_TEXT_VIEW(W.log_textview));
    GtkWidget *scrolled = gtk_scrolled_window_new(NULL, NULL);
    gtk_container_add(GTK_CONTAINER(scrolled), W.log_textview);
    gtk_box_pack_start(GTK_BOX(log_vbox), scrolled, TRUE, TRUE, 0);

    gtk_widget_show_all(W.window);
}

int main(int argc, char *argv[]) {
    gtk_init(&argc, &argv);
    profile_init(&W.profile);
    realtime_data_init(&realtime_data);
    W.profile_active = FALSE;
    W.profile_start_time = 0;
    W.profile_timeout_id = 0;
    W.log_serial_data = FALSE;
    W.telemetry_timeout_id = 0;  // ИНИЦИАЛИЗИРУЕМ ТАЙМЕР

    build_ui();

    gtk_entry_set_text(GTK_ENTRY(W.kp_entry), "6.0");
    gtk_entry_set_text(GTK_ENTRY(W.ki_entry), "0.12");
    gtk_entry_set_text(GTK_ENTRY(W.kd_entry), "8.1");

    gtk_main();
    // ОЧИЩАЕМ ВСЕ ТАЙМЕРЫ ПРИ ВЫХОДЕ
    if (W.profile_timeout_id != 0) {
        g_source_remove(W.profile_timeout_id);
    }
    if (W.telemetry_timeout_id != 0) {
        g_source_remove(W.telemetry_timeout_id);
    }

    profile_free(&W.profile);
    serial_close_h();

    return 0;
}
