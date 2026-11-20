#include "definition.h"

extern "C" void app_main()
{
    esp_task_wdt_deinit();
    // encoder
    encoder.setup(pins, degrees_per_edge);

    // H_bridge
    motor.setup(in, ch, timer, threshold);
    motor.setDuty(0);

    // PID
    // controler.setup(gains, dt_us * 1e-6f);

    float angulo = encoder.getAngle();
    float angulo_anterior = angulo;
    prev_time = esp_timer_get_time();
    prev_send_time = prev_time;

    while (1)
    {
        int capacity = sizeof(message);

        for (int i = 0; i < capacity && uart.available() > 0; i++)
        {
            char dato[2];
            if (uart.read(dato, sizeof(dato)) != 1)
                break;

            if (dato[0] == '\n')
            {
                if (pos < capacity)
                {
                    message[pos] = '\0';

                    float new_ref;
                    int end = -1;

                    if (message[0] == 'T')
                    {
                        float new_dt_ms;

                        if (sscanf(message, "T,%f %n", &new_dt_ms, &end) == 1 &&
                            end >= 0 && message[end] == '\0' &&
                            std::isfinite(new_dt_ms) &&
                            new_dt_ms >= 20.0f && new_dt_ms <= 1000.0f)
                        {
                            send_dt_us = static_cast<int64_t>(new_dt_ms * 1000.0f);
                        }
                    }
                    else if (message[0] == 'P')
                    {
                        if (sscanf(message, "P,%f %n", &new_ref, &end) == 1 &&
                            end >= 0 && message[end] == '\0' &&
                            new_ref >= -100.0f && new_ref <= 100.0f)
                        {
                            mode = 'P';
                            reference = new_ref;
                        }
                    }
                    else
                    {

                        char new_mode;
                        float kp, ki, kd;

                        if (sscanf(message, "%c,%f,%f,%f,%f %n",
                                   &new_mode, &new_ref, &kp, &ki, &kd, &end) == 5 &&
                            end >= 0 && message[end] == '\0' &&
                            (new_mode == 'S' || new_mode == 'A'))
                        {

                            bool position = new_mode == 'A';
                            float limit = position ? 360.0f : 130.0f;

                            if (new_ref >= -limit && new_ref <= limit &&
                                std::isfinite(kp) && kp >= 0.0f &&
                                std::isfinite(ki) && ki >= 0.0f &&
                                std::isfinite(kd) && kd >= 0.0f)
                            {
                                bool change_in_gains = kp != gains[0] || ki != gains[1] || kd != gains[2];
                                bool stop_or_reverse = new_mode == 'S' && (new_ref == 0.0f || new_ref * reference < 0.0f);

                                if (new_mode != mode || change_in_gains || stop_or_reverse)
                                {
                                    gains[0] = kp;
                                    gains[1] = ki;
                                    gains[2] = kd;

                                    controler.setup(gains,
                                                    (position ? position_dt_us : dt_us) * 1e-6f,
                                                    position ? -20.0f : 0.0f,
                                                    position ? 20.0f : 100.0f);
                                }

                                mode = new_mode;
                                reference = new_ref;
                            }
                        }
                    }
                }
                pos = 0;
            }
            else if (pos < capacity - 1)
            {
                message[pos++] = dato[0];
            }
            else
            {
                pos = capacity;
            }
        }
        // Actualizacion del control.
        current_time = esp_timer_get_time();
        int64_t periodo_us = (mode == 'A') ? position_dt_us : dt_us;

        if (current_time - prev_time >= periodo_us)
        {
            float dt = (current_time - prev_time) * 1e-6f;

            angulo = encoder.getAngle();

            // Velocidad medida sin filtrar.
            float rpm_raw = (angulo - angulo_anterior) / (6.0f * dt);

            // Filtro digital pasa bajas de primer orden.
            float alpha = dt / (tau_filtro + dt);
            rpm_filtrada += alpha * (rpm_raw - rpm_filtrada);

            // El controlador y LabVIEW utilizaran la velocidad filtrada.
            rpm = rpm_filtrada;

            error = 0.0f;
            duty = 0.0f;

            switch (mode)
            {
            case 'P':
            {
                duty = reference;
                break;
            }
            case 'S':
            {
                error = reference - rpm;
                const float sentido = reference < 0.0f ? -1.0f : 1.0f;

                if (reference != 0.0f && sentido * rpm >= -2.0f)
                {
                    duty = sentido * controler.calculate(sentido * error);
                }
                break;
            }
            case 'A':
            {
                error = reference - angulo;

                if (std::fabs(error) > 2.0f)
                    duty = controler.calculate(error);

                break;
            }
            }

            motor.setDuty(duty);

            angulo_anterior = angulo;
            prev_time = current_time;
        }

        // Envio de las ultimas mediciones a LabVIEW.
        current_time = esp_timer_get_time();

        if (current_time - prev_send_time >= send_dt_us)
        {
            printf("Ref: %.1f | RPM: %.1f | Error: %.1f | Comando: %.1f | Angle: %.1f\n",
                   reference, rpm, error, duty, angulo);

            prev_send_time = current_time;
        }
    }
}