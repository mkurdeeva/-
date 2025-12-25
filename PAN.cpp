// ============================================================================
// МОДЕЛИРОВАНИЕ ТРАЕКТОРИИ ПОЛЕТА САМОЛЕТА ТУ-154
// ============================================================================
#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <string>
#include <algorithm>
#include <memory>
#include <cstdio>
#include <stdio.h>
#include <stdlib.h>
#include <locale.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Массовые и геометрические параметры
const double BASELINE_MASS = 98000.0;   // Стартовая масса, кг
const double WING_AREA = 201.1;     // Площадь крыла, м²

// Двигательная установка
const double ENGINE_THRUST = 3 * 88000.0; // Суммарная тяга, Н
const double THROTTLE_POSITION = 0.9;       // Положение РУД

// Заданные параметры полета
const double START_ALTITUDE = 400.0;     // Начальная высота, м
const double TARGET_ALTITUDE = 6500.0;    // Крейсерская высота, м

// Скоростные режимы (км/ч -> м/с)
const double START_SPEED_KPH = 320.0;
const double TARGET_SPEED_KPH = 900.0;
const double START_SPEED_MS = START_SPEED_KPH / 3.6;
const double TARGET_SPEED_MS = TARGET_SPEED_KPH / 3.6;

// Физические константы
const double GRAVITY = 9.81;      // Ускорение свободного падения, м/с²
const double AIR_GAS_CONSTANT = 287.05;    // Удельная газовая постоянная, Дж/(кг·К)

class CSVWriter {
private:
    std::ofstream file;

public:

    CSVWriter(const std::string& filename) {
        file.open(filename);
        if (!file.is_open()) {
            throw std::runtime_error("Ошибка создания файла: " + filename);
        }
    }

        ~CSVWriter() {
        if (file.is_open()) file.close();
    }

    void writeHeader(const std::vector<std::string>& column_names) {
        for (size_t i = 0; i < column_names.size(); ++i) {
            file << column_names[i];
            if (i < column_names.size() - 1) file << ',';
        }
        file << std::endl;
    }


    void writeDataRow(const std::vector<double>& values) {
        for (size_t i = 0; i < values.size(); ++i) {
            file << std::fixed << std::setprecision(6) << values[i];
            if (i < values.size() - 1) file << ',';
        }
        file << std::endl;
    }
};

class AtmosphericModel {
private:

    std::vector<double> altitude_data;      // м
    std::vector<double> temperature_data;   // К
    std::vector<double> pressure_data;      // Па
    std::vector<double> density_data;       // кг/м³
    std::vector<double> sound_speed_data;   // м/с

public:
    // Инициализация табличными значениями
    AtmosphericModel() {
        initializeAtmosphereTable();
    }

private:
    void initializeAtmosphereTable() {
        // Высотные отметки
        altitude_data = { 0, 500, 1000, 1500, 2000, 2500, 3000,
                         3500, 4000, 4500, 5000, 5500, 6000, 6500, 7000 };

        // Температура по высоте
        temperature_data = { 288.15, 284.90, 281.65, 278.40, 275.15,
                            271.90, 268.65, 265.40, 262.15, 258.90,
                            255.65, 252.40, 249.15, 245.95, 242.70 };

        // Атмосферное давление
        pressure_data = { 101325.0, 95463.0, 89976.0, 84808.0, 79916.0,
                         75260.0, 70801.0, 66604.0, 62634.0, 58860.0,
                         55252.0, 51881.0, 48720.0, 44075.0, 41105.0 };

        // Плотность воздуха
        density_data = { 1.2250, 1.1673, 1.1117, 1.0581, 1.0066,
                        0.9570, 0.9093, 0.8635, 0.8196, 0.7775,
                        0.7369, 0.6979, 0.6605, 0.6243, 0.5900 };

        // Скорость звука
        sound_speed_data = { 340.29, 338.37, 336.44, 334.50, 332.54,
                            330.57, 328.58, 326.58, 324.56, 322.52,
                            320.47, 318.40, 316.31, 314.38, 312.30 };
    }

    double interpolateLinear(double x,
        const std::vector<double>& x_points,
        const std::vector<double>& y_points) const {
        // Проверка граничных условий
        if (x <= x_points.front()) return y_points.front();
        if (x >= x_points.back()) return y_points.back();

        for (size_t i = 0; i < x_points.size() - 1; ++i) {
            if (x >= x_points[i] && x <= x_points[i + 1]) {
                double t = (x - x_points[i]) / (x_points[i + 1] - x_points[i]);
                return y_points[i] + t * (y_points[i + 1] - y_points[i]);
            }
        }
        return y_points.back();
    }

public:
    // Получение параметров атмосферы на заданной высоте
    double getTemperature(double altitude) const {
        return interpolateLinear(altitude, altitude_data, temperature_data);
    }

    double getPressure(double altitude) const {
        return interpolateLinear(altitude, altitude_data, pressure_data);
    }

    double getDensity(double altitude) const {
        return interpolateLinear(altitude, altitude_data, density_data);
    }

    double getSoundSpeed(double altitude) const {
        return interpolateLinear(altitude, altitude_data, sound_speed_data);
    }

    // Расчет числа Маха
    double calculateMachNumber(double velocity, double altitude) const {
        return velocity / getSoundSpeed(altitude);
    }
};

struct AircraftState {
    // Временные параметры
    double time_seconds;        // Время с начала полета, с

    // Пространственные координаты
    double position_x;          // Горизонтальная координата, м
    double altitude;            // Вертикальная координата (высота), м

    // Кинематические параметры
    double velocity_total;      // Полная скорость, м/с
    double velocity_horizontal; // Горизонтальная составляющая, м/с
    double velocity_vertical;   // Вертикальная составляющая, м/с

    // Угловые параметры
    double flight_path_angle;   // Угол траектории, рад
    double angle_of_attack;     // Угол атаки, рад

    // Массовые параметры
    double fuel_consumed;       // Израсходованное топливо, кг
    double current_mass;        // Текущая масса, кг

    // Динамические параметры
    double acceleration;        // Тангенциальное ускорение, м/с²
    double mach_number;         // Число Маха

    // Конструктор с параметрами по умолчанию
    AircraftState(double t = 0, double x = 0, double h = 0,
        double V = 0, double Vx = 0, double Vy = 0,
        double theta = 0, double alpha = 0, double fuel = 0,
        double mass = BASELINE_MASS, double acc = 0,
        double mach = 0)
        : time_seconds(t), position_x(x), altitude(h),
        velocity_total(V), velocity_horizontal(Vx), velocity_vertical(Vy),
        flight_path_angle(theta), angle_of_attack(alpha),
        fuel_consumed(fuel), current_mass(mass),
        acceleration(acc), mach_number(mach) {}

    // Вывод состояния в консоль
    void printStatus() const {
        std::cout << std::fixed << std::setprecision(1);
        std::cout << "Время: " << time_seconds << " с | ";
        std::cout << "Высота: " << altitude << " м | ";
        std::cout << "Скорость: " << velocity_total * 3.6 << " км/ч | ";
        std::cout << "Вертикальная скорость: " << velocity_vertical << " м/с | ";
        std::cout << "Угол траектории: " << flight_path_angle * 180 / M_PI << "° | ";
        std::cout << "Масса: " << current_mass << " кг\n";
    }
};

class FlightTrajectory {
private:
    std::vector<AircraftState> trajectory_points;

public:
    FlightTrajectory() = default;

    // Добавление новой точки траектории
    void addStatePoint(const AircraftState& state) {
        trajectory_points.push_back(state);
    }

    // Получение всех точек траектории
    const std::vector<AircraftState>& getAllStates() const {
        return trajectory_points;
    }

    // Общее время полета
    double getTotalFlightTime() const {
        return trajectory_points.empty() ? 0.0 : trajectory_points.back().time_seconds;
    }

    // Общий расход топлива
    double getTotalFuelConsumption() const {
        return trajectory_points.empty() ? 0.0 : trajectory_points.back().fuel_consumed;
    }

    // Экспорт траектории в CSV-файл
    void exportToCSV(const std::string& filename) {
        CSVWriter csv_writer(filename);

        // Заголовки столбцов
        csv_writer.writeHeader({
            "time_s", "altitude_m", "velocity_ms", "velocity_kmh",
            "vertical_velocity_ms", "theta_deg", "alpha_deg",
            "fuel_kg", "mass_kg", "acceleration_ms2", "mach_number"
            });

        // Запись данных
        for (const auto& state : trajectory_points) {
            csv_writer.writeDataRow({
                state.time_seconds,
                state.altitude,
                state.velocity_total,
                state.velocity_total * 3.6,
                state.velocity_vertical,
                state.flight_path_angle * 180 / M_PI,
                state.angle_of_attack * 180 / M_PI,
                state.fuel_consumed,
                state.current_mass,
                state.acceleration,
                state.mach_number
                });
        }

        std::cout << "Траектория сохранена в файл: " << filename << std::endl;
    }

    // Визуализация траектории с помощью Gnuplot
    void visualize() const {
        // Проверка доступности Gnuplot
        FILE* gnuplot = _popen("gnuplot -persist", "w");
        if (!gnuplot) {
            std::cerr << "✗ Ошибка: Gnuplot не найден!" << std::endl;
            std::cerr << "  Установите Gnuplot и добавьте в PATH" << std::endl;
            return;
        }

        // Настройка графиков
        fprintf(gnuplot, "set terminal wxt size 1200,800 enhanced font 'Arial,12'\n");
        fprintf(gnuplot, "set multiplot layout 2,2 title 'Динамика полета ТУ-134'\n");
        fprintf(gnuplot, "set grid\n");

        // График 1: Высота от времени
        fprintf(gnuplot, "set title 'Высота полета'\n");
        fprintf(gnuplot, "set xlabel 'Время, с'\n");
        fprintf(gnuplot, "set ylabel 'Высота, м'\n");
        fprintf(gnuplot, "plot '-' with lines lw 2 lc rgb 'blue' title 'Высота'\n");
        for (const auto& state : trajectory_points) {
            fprintf(gnuplot, "%f %f\n", state.time_seconds, state.altitude);
        }
        fprintf(gnuplot, "e\n");

        // График 2: Скорость от времени
        fprintf(gnuplot, "set title 'Скорость полета'\n");
        fprintf(gnuplot, "set xlabel 'Время, с'\n");
        fprintf(gnuplot, "set ylabel 'Скорость, км/ч'\n");
        fprintf(gnuplot, "plot '-' with lines lw 2 lc rgb 'red' title 'Скорость'\n");
        for (const auto& state : trajectory_points) {
            fprintf(gnuplot, "%f %f\n", state.time_seconds, state.velocity_total * 3.6);
        }
        fprintf(gnuplot, "e\n");

        // График 3: Масса от времени
        fprintf(gnuplot, "set title 'Изменение массы'\n");
        fprintf(gnuplot, "set xlabel 'Время, с'\n");
        fprintf(gnuplot, "set ylabel 'Масса, кг'\n");
        fprintf(gnuplot, "plot '-' with lines lw 2 lc rgb 'green' title 'Масса'\n");
        for (const auto& state : trajectory_points) {
            fprintf(gnuplot, "%f %f\n", state.time_seconds, state.current_mass);
        }
        fprintf(gnuplot, "e\n");

        fprintf(gnuplot, "unset multiplot\n");
        fflush(gnuplot);

        std::cout << "\n Графики построены в Gnuplot" << std::endl;
        std::cout << "   Закройте окно Gnuplot для продолжения..." << std::endl;
        std::cin.get();

        _pclose(gnuplot);
    }
};

// КЛАСС МОДЕЛИ

class Aircraft {
private:
    AtmosphericModel atmosphere;    // Модель атмосферы
    double wing_area;              // Площадь крыла
    double initial_mass;           // Начальная масса

public:
    double current_mass;           // Текущая масса
    double engine_thrust;          // Текущая тяга
    double fuel_consumption_rate;  // Расход топлива
    double zero_lift_drag;         // Коэффициент сопротивления при нулевой подъемной силе
    double induced_drag_coeff;     // Коэффициент индуктивного сопротивления
    double max_lift_coefficient;   // Максимальный коэффициент подъемной силы

    Aircraft(double mass = BASELINE_MASS, double area = WING_AREA)
        : wing_area(area), initial_mass(mass), current_mass(mass) {

        engine_thrust = ENGINE_THRUST * THROTTLE_POSITION;
        fuel_consumption_rate = 2.5;           // кг/с
        zero_lift_drag = 0.02;                 // Cx0
        induced_drag_coeff = 0.05;             // K
        max_lift_coefficient = 1.2;            // Cy_max
    }

    // Расчет коэффициента подъемной силы
    double calculateLiftCoefficient(double angle_of_attack) const {
        const double LIFT_SLOPE = 5.0;  // Производная коэффициента подъемной силы
        return std::min(LIFT_SLOPE * angle_of_attack, max_lift_coefficient);
    }

    // Расчет коэффициента лобового сопротивления
    double calculateDragCoefficient(double lift_coefficient) const {
        // Параболическая поляра: Cx = Cx0 + K*Cy²
        return zero_lift_drag + induced_drag_coeff * lift_coefficient * lift_coefficient;
    }

    // Расчет подъемной силы
    double computeLiftForce(double velocity, double altitude, double aoa) const {
        double air_density = atmosphere.getDensity(altitude);
        double dynamic_pressure = 0.5 * air_density * velocity * velocity;
        double lift_coeff = calculateLiftCoefficient(aoa);
        return lift_coeff * wing_area * dynamic_pressure;
    }

    // Расчет силы сопротивления
    double computeDragForce(double velocity, double altitude, double aoa) const {
        double air_density = atmosphere.getDensity(altitude);
        double dynamic_pressure = 0.5 * air_density * velocity * velocity;
        double lift_coeff = calculateLiftCoefficient(aoa);
        double drag_coeff = calculateDragCoefficient(lift_coeff);
        return drag_coeff * wing_area * dynamic_pressure;
    }

    AircraftState predictNextState(const AircraftState& current_state,
        double time_step,
        double commanded_aoa) {
        AircraftState next_state = current_state;

        // Ограничение угла атаки для безопасности
        commanded_aoa = std::max(-0.1, std::min(0.2, commanded_aoa));

        // Расчет аэродинамических сил
        double lift = computeLiftForce(current_state.velocity_total,
            current_state.altitude, commanded_aoa);
        double drag = computeDragForce(current_state.velocity_total,
            current_state.altitude, commanded_aoa);

        // Силы по осям
        double force_horizontal = engine_thrust - drag -
            current_mass * GRAVITY * sin(current_state.flight_path_angle);
        double force_vertical = lift -
            current_mass * GRAVITY * cos(current_state.flight_path_angle);

        // Ускорения
        double acceleration_horizontal = force_horizontal / current_mass;
        double acceleration_vertical = force_vertical / current_mass;

        // Обновление скорости
        next_state.velocity_total = current_state.velocity_total +
            acceleration_horizontal * time_step;
        next_state.velocity_total = std::max(100.0, next_state.velocity_total);

        // Обновление угла траектории
        if (next_state.velocity_total > 0) {
            next_state.flight_path_angle = atan2(acceleration_vertical,
                acceleration_horizontal + GRAVITY * sin(current_state.flight_path_angle));
        }

        // Ограничение угла траектории
        next_state.flight_path_angle = std::max(-0.3, std::min(0.3, next_state.flight_path_angle));

        // Составляющие скорости
        next_state.velocity_horizontal = next_state.velocity_total * cos(next_state.flight_path_angle);
        next_state.velocity_vertical = next_state.velocity_total * sin(next_state.flight_path_angle);

        // Обновление координат
        next_state.position_x = current_state.position_x +
            next_state.velocity_horizontal * time_step;
        next_state.altitude = current_state.altitude +
            next_state.velocity_vertical * time_step;

        // Защита от отрицательной высоты
        if (next_state.altitude < 0) {
            next_state.altitude = 10.0;
            next_state.velocity_vertical = std::max(0.0, next_state.velocity_vertical);
        }

        // Обновление аэродинамических параметров
        next_state.angle_of_attack = commanded_aoa;
        next_state.mach_number = atmosphere.calculateMachNumber(next_state.velocity_total,
            next_state.altitude);
        next_state.acceleration = acceleration_horizontal;

        // Расход топлива и массы
        double fuel_burned = fuel_consumption_rate * time_step;
        next_state.fuel_consumed = current_state.fuel_consumed + fuel_burned;
        next_state.current_mass = current_state.current_mass - fuel_burned;

        // Коррекция тяги (упрощенная модель)
        engine_thrust = ENGINE_THRUST * THROTTLE_POSITION *
            (next_state.current_mass / initial_mass);

        next_state.time_seconds = current_state.time_seconds + time_step;

        return next_state;
    }

    // Получение начальной массы
    double getInitialMass() const {
        return initial_mass;
    }
};

// КЛАСС ОПТИМИЗАЦИИ

class TrajectoryPlanner {
public:
    TrajectoryPlanner() = default;

    // Основная функция расчета траектории
    FlightTrajectory calculateFlightPath(Aircraft& aircraft, double max_flight_time = 600.0) {
        FlightTrajectory flight_path;
        const double TIME_INCREMENT = 1.0;  // Шаг интегрирования, с

        // --------------------------------------------------------------------
        // НАЧАЛЬНЫЕ УСЛОВИЯ
        // --------------------------------------------------------------------
        AircraftState initial_condition;
        initial_condition.time_seconds = 0;
        initial_condition.position_x = 0;
        initial_condition.altitude = START_ALTITUDE;
        initial_condition.velocity_total = START_SPEED_MS;
        initial_condition.velocity_horizontal = START_SPEED_MS;
        initial_condition.velocity_vertical = 5.0;      // Начальный вертикальный разгон
        initial_condition.flight_path_angle = 0.05;     // Начальный угол подъема (~3°)
        initial_condition.angle_of_attack = 0.03;       // Начальный угол атаки
        initial_condition.fuel_consumed = 0;
        initial_condition.current_mass = aircraft.getInitialMass();
        initial_condition.acceleration = 0;

        // Расчет начального числа Маха
        AtmosphericModel atmosphere_calc;
        initial_condition.mach_number = atmosphere_calc.calculateMachNumber(
            initial_condition.velocity_total, initial_condition.altitude);

        AircraftState current_state = initial_condition;
        flight_path.addStatePoint(current_state);

        // --------------------------------------------------------------------
        // ИНФОРМАЦИЯ О МОДЕЛИРОВАНИИ
        // --------------------------------------------------------------------
        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "МОДЕЛИРОВАНИЕ ПОЛЕТА ТУ-134" << std::endl;
        std::cout << std::string(60, '=') << std::endl;

        std::cout << "\n ПАРАМЕТРЫ ПОЛЕТА:" << std::endl;
        std::cout << "   Стартовая высота: " << START_ALTITUDE << " м" << std::endl;
        std::cout << "   Крейсерская высота: " << TARGET_ALTITUDE << " м" << std::endl;
        std::cout << "   Стартовая скорость: " << START_SPEED_KPH << " км/ч" << std::endl;
        std::cout << "   Крейсерская скорость: " << TARGET_SPEED_KPH << " км/ч" << std::endl;
        std::cout << "   Максимальное время: " << max_flight_time << " с" << std::endl;

        // --------------------------------------------------------------------
        // ИНТЕГРАЦИЯ УРАВНЕНИЙ ДВИЖЕНИЯ
        // --------------------------------------------------------------------
        int iteration_count = 0;
        bool destination_reached = false;

        std::cout << "\n ВЫПОЛНЕНИЕ РАСЧЕТОВ..." << std::endl;

        while (current_state.time_seconds < max_flight_time && !destination_reached) {
            iteration_count++;

            // Простой алгоритм управления углом атаки
            double altitude_progress = current_state.altitude / TARGET_ALTITUDE;
            double target_aoa;

            if (altitude_progress < 0.3) {
                target_aoa = 0.06;      // Интенсивный набор высоты
            }
            else if (altitude_progress < 0.7) {
                target_aoa = 0.04;      // Плавный набор высоты
            }
            else {
                target_aoa = 0.02;      // Стабилизация на крейсерской высоте
            }

            // Коррекция для набора скорости
            if (current_state.velocity_total < TARGET_SPEED_MS * 0.9) {
                target_aoa -= 0.01;
            }

            // Интегрирование уравнений движения
            current_state = aircraft.predictNextState(current_state, TIME_INCREMENT, target_aoa);
            flight_path.addStatePoint(current_state);

            // Периодический вывод информации
            if (iteration_count % 30 == 0) {
                std::cout << "   Шаг " << iteration_count
                    << " | Время: " << current_state.time_seconds << " с"
                    << " | Высота: " << current_state.altitude << " м"
                    << " (" << (altitude_progress * 100) << "%)" << std::endl;
            }

            // Проверка достижения целевой высоты
            if (current_state.altitude >= TARGET_ALTITUDE) {
                destination_reached = true;
                std::cout << "\n ЦЕЛЬ ДОСТИГНУТА!" << std::endl;
            }

            // Проверка на физическую корректность
            if (current_state.altitude > 20000 || current_state.velocity_total > 1000) {
                std::cout << "\n ВНИМАНИЕ: Выход за физические пределы!" << std::endl;
                break;
            }
        }

        // --------------------------------------------------------------------
        // РЕЗУЛЬТАТЫ РАСЧЕТОВ
        // --------------------------------------------------------------------
        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << " РЕЗУЛЬТАТЫ" << std::endl;
        std::cout << std::string(60, '=') << std::endl;

        std::cout << "\n ФИНАЛЬНЫЕ ПАРАМЕТРЫ:" << std::endl;
        std::cout << "   Высота: " << current_state.altitude << " м ("
            << (current_state.altitude / TARGET_ALTITUDE * 100) << "% от цели)" << std::endl;
        std::cout << "   Скорость: " << std::fixed << std::setprecision(1)
            << current_state.velocity_total * 3.6 << " км/ч" << std::endl;
        std::cout << "   Время полета: " << current_state.time_seconds << " с" << std::endl;
        std::cout << "   Расход топлива: " << current_state.fuel_consumed << " кг" << std::endl;
        std::cout << "   Число Маха: " << std::fixed << std::setprecision(3)
            << current_state.mach_number << std::endl;
        std::cout << "   Остаток массы: " << current_state.current_mass << " кг" << std::endl;

        std::cout << "\n СТАТИСТИКА:" << std::endl;
        std::cout << "   Средняя вертикальная скорость: "
            << (current_state.altitude - START_ALTITUDE) / current_state.time_seconds
            << " м/с" << std::endl;
        std::cout << "   Средний расход топлива: "
            << current_state.fuel_consumed / current_state.time_seconds
            << " кг/с" << std::endl;

        return flight_path;
    }
};

int main() {
    // Настройка локали для корректного вывода кириллицы
    setlocale(LC_ALL, "ru");

    try {
        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << " СИМУЛЯТОР ПОЛЕТА ТУ-154" << std::endl;
        std::cout << "   Версия 2.0 | Аэродинамическое моделирование" << std::endl;
        std::cout << std::string(60, '=') << std::endl;

        // Создание модели самолета
        Aircraft tu154_model;

        // Создание планировщика траектории
        TrajectoryPlanner flight_planner;

        // Расчет траектории полета
        FlightTrajectory optimal_path = flight_planner.calculateFlightPath(tu154_model, 300.0);

        // Сохранение результатов
        optimal_path.exportToCSV("tu154_flight_profile.csv");

        // Визуализация результатов
        optimal_path.visualize();

        std::cout << "\nМОДЕЛИРОВАНИЕ УСПЕШНО ЗАВЕРШЕНО!" << std::endl;

    }
    catch (const std::exception& error) {
        std::cerr << "\nОШИБКА: " << error.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

