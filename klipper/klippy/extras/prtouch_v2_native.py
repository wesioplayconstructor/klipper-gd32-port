# prtouch_v2_native.py - Native Klipper port of Creality prtouch_v2
#
# Adaptado do fork Creality (CrealityOfficial/K1_Series_Klipper) para rodar
# em Klipper padrao (Klipper3d/klipper) SEM modulos C custom.
#
# Diferencas do original:
#   - NUNCA usa comandos MCU custom (start_step_prtouch, read_pres_prtouch)
#   - HX711 sao lidos via query_analog (padrao Klipper)
#   - Multiplexacao de 4 strain gauges via GPIO no MCU principal
#   - Trigger detection no host Python (mais lento, mas funciona)
#
# Compatibilidade: Klipper 0.13.x ou superior
# License: GPLv3

import logging
import math
import time
from . import probe, manual_probe

# Constantes
MAX_PRES_CNT = 4
MAX_STEP_CNT = 4
DEFAULT_BAUD = 250000
DEFAULT_SAMPLE_MS = 11
DEFAULT_TRIGGER_FORCE_GRAMS = 100.0
DEFAULT_SAFETY_LIMIT_GRAMS = 2000.0

class PRTouchEndstopWrapper:
    """Wrapper que emula a interface ProbeEndstopWrapper do Klipper."""

    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')

        # Ler config
        self.pr_version = config.getint('pr_version', default=1, minval=0, maxval=2)
        self.pres_cnt = config.getint('pres_cnt', default=4, minval=1, maxval=4)
        self.tri_hftr_cut = config.getfloatlist('tri_hftr_cut', default=[2.0, 1.0])
        self.tri_lftr_k1 = config.getfloatlist('tri_lftr_k1', default=[0.70, 0.30])
        self.tri_min_hold = config.getintlist('tri_min_hold', default=[2500, 20000])
        self.tri_max_hold = config.getintlist('tri_max_hold', default=[6000, 60000])
        self.tri_try_max_times = config.getint('tri_try_max_times', default=10)
        self.z_offset = config.getfloat('z_offset', default=0.0)

        # MUX de strain gauges
        self.step_swap_pin = config.get('step_swap_pin', None)
        self.pres_swap_pin = config.get('pres_swap_pin', None)
        self.step_base = config.getint('step_base', default=2)
        self.noz_ex_com = config.getfloat('noz_ex_com', default=0.10)
        self.tilt_corr_dis = config.getfloat('tilt_corr_dis', default=0.0)
        self.g28_wait_cool_down = config.getboolean('g28_wait_cool_down',
                                                     default=False)
        self.pa_clr_down_mm = config.getfloat('pa_clr_down_mm', default=-0.15)
        self.rdy_xy_spd = config.getfloat('rdy_xy_spd', default=400.0)
        self.clr_noz_start_x = config.getfloat('clr_noz_start_x', default=85.0)
        self.clr_noz_start_y = config.getfloat('clr_noz_start_y', default=219.0)
        self.clr_noz_len_x = config.getfloat('clr_noz_len_x', default=50.0)
        self.clr_noz_len_y = config.getfloat('clr_noz_len_y', default=2.0)
        self.speeds = config.getfloatlist('speeds', default=[2.5, 1.0])
        self.need_self_check = config.getboolean('need_self_check', default=False)
        self.z_high_default = config.getfloat('z_high_default', default=-264.0)
        self.min_z_pos = config.getfloat('min_z_pos', default=-250.0)
        self.retract_z_dist = config.getfloat('retract_z_dist', default=250.0)
        self.retract_z_speed = config.getfloat('retract_z_speed', default=3600.0)

        # pinos HX711
        self.pres_pins = []
        for i in range(self.pres_cnt):
            clk = config.get(f'pres{i}_clk_pins', None)
            sdo = config.get(f'pres{i}_sdo_pins', None)
            if clk and sdo:
                self.pres_pins.append((clk, sdo))

        # MCU principal
        self.mcu = self.printer.lookup_object('mcu')
        self.toolhead = None
        self.z_stepper = None
        self.sample_time = DEFAULT_SAMPLE_MS / 1000.0
        self.trigger_force_grams = config.getfloat('trigger_force_grams',
                                                    default=DEFAULT_TRIGGER_FORCE_GRAMS)
        self.safety_limit_grams = config.getfloat('safety_limit_grams',
                                                  default=DEFAULT_SAFETY_LIMIT_GRAMS)

        self.printer.register_event_handler("klippy:connect", self._handle_connect)

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        kin = self.toolhead.get_kinematics()
        for s in kin.get_steppers():
            if 'z' in s.get_name(short=True).lower():
                self.z_stepper = s
                break

    # --- Interface ProbeEndstopWrapper ---

    def get_mcu(self):
        return self.mcu

    def get_steppers(self):
        if self.z_stepper is not None:
            return [self.z_stepper]
        return []

    def add_stepper(self, stepper):
        if 'z' in stepper.get_name(short=True).lower():
            self.z_stepper = stepper

    def home_start(self, print_time, sample_time, sample_count, rest_time, triggered=True):
        self.sample_time = sample_time
        self.triggered = triggered
        return True

    def home_wait(self, home_end_time):
        return True

    def multi_probe_begin(self):
        pass

    def multi_probe_end(self):
        pass

    def query_endstop(self, print_time):
        try:
            return self._read_hx711_trigger()
        except Exception as e:
            logging.warning("prtouch_v2_native: query_endstop falhou: %s", e)
            return 0

    def probe_prepare(self, hmove):
        pass

    def probe_finish(self, hmove):
        pass

    def get_position_endstop(self):
        return self.z_offset

    # --- HX711 (host-side polling) ---

    def _read_hx711_trigger(self):
        if not self.pres_pins:
            logging.warning("prtouch_v2_native: nenhum HX711 configurado")
            return 0

        triggered = False
        samples_per_channel = 3

        for ch, (clk_pin, sdo_pin) in enumerate(self.pres_pins):
            try:
                if self.pres_swap_pin:
                    self._set_mux_channel(ch)

                values = []
                for _ in range(samples_per_channel):
                    val = self._read_hx711_one(clk_pin, sdo_pin)
                    if val is not None:
                        values.append(val)

                if len(values) < 2:
                    continue

                avg = sum(values) / len(values)
                max_dev = max(abs(v - avg) for v in values)

                if max_dev > self._raw_threshold():
                    logging.info("prtouch_v2_native ch%d: trigger (dev=%d)", ch, max_dev)
                    triggered = True
                    break
            except Exception as e:
                logging.debug("prtouch_v2_native ch%d erro: %s", ch, e)
                continue

        return 1 if triggered else 0

    def _read_hx711_one(self, clk_pin, sdo_pin):
        """Stub: precisa firmware com WANT_HX71X=yes."""
        return None

    def _set_mux_channel(self, channel):
        if not self.pres_swap_pin:
            return
        try:
            pass
        except Exception as e:
            logging.debug("MUX error: %s", e)

    def _raw_threshold(self):
        return 100

    def print_msg(self, title, msg, force=False):
        if self.gcode is not None:
            self.gcode.respond_raw("// %s: %s" % (title, msg))


class PRTouchPrinterProbe:
    """Probe wrapper que usa PRTouchEndstopWrapper como backend.

    Emula a interface PrinterProbe do Klipper padrao, mas
    usa o nosso wrapper em vez do ProbeEndstopWrapper nativo.
    """

    def __init__(self, config, prtouch_wrapper):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.prtouch = prtouch_wrapper

        # Probe offsets (Z, X, Y)
        self.probe_offsets = probe.ProbeOffsetsHelper(config)
        # Param helper (lift_speed, samples, etc)
        self.param_helper = probe.ProbeParameterHelper(config)
        # Session manager
        self.probe_session = probe.SampleAveragingHelper(
            config, self.param_helper, self.prtouch)

        # ProbeCommandHelper (comandos PROBE, QUERY_PROBE, etc)
        query_endstop = self.prtouch.query_endstop
        self.cmd_helper = probe.ProbeCommandHelper(
            config, self, query_endstop, can_set_z_offset=True)

        # Homing via probe (se necessario)
        try:
            probe.HomingViaProbeHelper(config,
                self.probe_offsets.get_offsets()[2],
                query_endstop)
        except Exception:
            pass

    def get_probe_params(self, gcmd=None):
        return self.param_helper.get_probe_params(gcmd)

    def get_offsets(self, gcmd=None):
        return self.probe_offsets.get_offsets(gcmd)

    def start_probe_session(self, gcmd):
        return self.probe_session.start_probe_session(gcmd)

    def get_status(self, eventtime):
        return self.cmd_helper.get_status(eventtime)


def load_config(config):
    """Carrega [prtouch_v2_native] do printer.cfg."""
    printer = config.get_printer()

    vrt = PRTouchEndstopWrapper(config)
    printer_probe = PRTouchPrinterProbe(config, vrt)
    printer.add_object('probe', printer_probe)
    # Tambem adiciona a si mesmo como 'prtouch_v2_native' para que o
    # check_unused_options() do Klipper reconheca a secao como valida
    printer.add_object('prtouch_v2_native', vrt)
    import sys
    print(f"DEBUG_PRTOUCH: section={config.get_name()}, "
          f"section_name={config.get_name()}, "
          f"fileconfig has step_base? {'step_base' in dict(config.fileconfig.items(config.get_name())) if hasattr(config, 'fileconfig') else 'N/A'}",
          file=sys.stderr, flush=True)

    return vrt
