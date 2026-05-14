try:
    import lgpio
except ImportError:
    lgpio = None


PWMA_PIN = 18
AIN1_PIN = 22
AIN2_PIN = 27
PWMB_PIN = 23
BIN1_PIN = 25
BIN2_PIN = 24
GPIOCHIP = 4
PWM_FREQ = 1000


def main(args=None):
    if lgpio is None:
        raise RuntimeError('lgpio is required to force motor GPIO pins low.')

    pins = [PWMA_PIN, AIN1_PIN, AIN2_PIN, PWMB_PIN, BIN1_PIN, BIN2_PIN]
    handle = lgpio.gpiochip_open(GPIOCHIP)
    try:
        for pin in pins:
            lgpio.gpio_claim_output(handle, pin)

        lgpio.tx_pwm(handle, PWMA_PIN, PWM_FREQ, 0)
        lgpio.tx_pwm(handle, PWMB_PIN, PWM_FREQ, 0)

        for pin in pins:
            lgpio.gpio_write(handle, pin, 0)

        print('Motor GPIO/PWM pins forced to 0.')
    finally:
        lgpio.gpiochip_close(handle)


if __name__ == '__main__':
    main()
